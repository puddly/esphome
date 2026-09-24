import esphome.codegen as cg
from esphome.components.esp32 import (
    VARIANT_ESP32H4,
    VARIANT_ESP32P4,
    VARIANT_ESP32S2,
    VARIANT_ESP32S3,
    VARIANT_ESP32S31,
    add_idf_component,
    add_idf_sdkconfig_option,
    idf_version,
    only_on_variant,
)
import esphome.config_validation as cv
from esphome.const import CONF_DEVICES, CONF_ID, CONF_TYPE
from esphome.core import CORE
from esphome.cpp_generator import MockObj
from esphome.cpp_types import Component
from esphome.types import ConfigType

AUTO_LOAD = ["bytebuffer"]
CODEOWNERS = ["@clydebarrow"]
DEPENDENCIES = ["esp32"]
usb_host_ns = cg.esphome_ns.namespace("usb_host")
USBHost = usb_host_ns.class_("USBHost", Component)
USBClient = usb_host_ns.class_("USBClient", Component)
DOMAIN = "usb_host"
CONF_VID = "vid"
CONF_PID = "pid"
CONF_ENABLE_HUBS = "enable_hubs"
CONF_MAX_TRANSFER_REQUESTS = "max_transfer_requests"
CONF_MAX_PACKET_SIZE = "max_packet_size"
CONF_USB_HOST_ID = "usb_host_id"
CONF_DRIVERS = "drivers"

_request_client_slot = cg.slot_counter("USB_HOST_CLIENT_COUNT")
_request_matcher_slot = cg.slot_counter("USB_HOST_MATCHER_COUNT")


def usb_device_schema(
    cls=USBClient, vid: int | None = None, pid: int | None = None
) -> cv.Schema:
    schema = cv.COMPONENT_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(cls),
            cv.GenerateID(CONF_USB_HOST_ID): cv.use_id(USBHost),
        }
    )
    if vid:
        schema = schema.extend({cv.Optional(CONF_VID, default=vid): cv.hex_uint16_t})
    else:
        schema = schema.extend({cv.Required(CONF_VID): cv.hex_uint16_t})
    if pid:
        schema = schema.extend({cv.Optional(CONF_PID, default=pid): cv.hex_uint16_t})
    else:
        schema = schema.extend({cv.Required(CONF_PID): cv.hex_uint16_t})
    return schema


def _set_max_packet_size(config: dict) -> dict:
    CORE.data.setdefault(DOMAIN, {})[CONF_MAX_PACKET_SIZE] = config[
        CONF_MAX_PACKET_SIZE
    ]
    return config


def get_max_packet_size() -> int:
    return CORE.data.get(DOMAIN, {}).get(CONF_MAX_PACKET_SIZE, 64)


# The drivers a rule can name. Only CDC ACM exists so far; a vendor driver adds a value here.
DRIVER_TYPES = ("CDC_ACM",)

# One row of the driver table: the driver to run for a device with these IDs. Rules are tried
# in order and the first match wins. The line settings come from the client that opens the port.
DRIVER_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_TYPE): cv.one_of(*DRIVER_TYPES, upper=True),
        cv.Required(CONF_VID): cv.hex_uint16_t,
        cv.Required(CONF_PID): cv.hex_uint16_t,
    }
)

CONFIG_SCHEMA = cv.All(
    cv.COMPONENT_SCHEMA.extend(
        {
            cv.GenerateID(): cv.declare_id(USBHost),
            cv.Optional(CONF_ENABLE_HUBS, default=False): cv.boolean,
            cv.Optional(CONF_MAX_TRANSFER_REQUESTS, default=16): cv.int_range(
                min=1, max=32
            ),
            cv.Optional(CONF_MAX_PACKET_SIZE, default=64): cv.one_of(
                64, 128, 256, 512, 1024, int=True
            ),
            cv.Optional(CONF_DEVICES): cv.ensure_list(usb_device_schema()),
            cv.Optional(CONF_DRIVERS, default=[]): cv.ensure_list(DRIVER_SCHEMA),
        }
    ),
    only_on_variant(
        supported=[
            VARIANT_ESP32H4,
            VARIANT_ESP32P4,
            VARIANT_ESP32S2,
            VARIANT_ESP32S3,
            VARIANT_ESP32S31,
        ]
    ),
    _set_max_packet_size,
)


def register_client(host: MockObj, client: MockObj) -> None:
    """Attach a USBClient to its host, matched or not, so a removed device finds it."""
    _request_client_slot()
    cg.add(host.register_client(client))


def register_matcher(host: MockObj, matcher: MockObj) -> None:
    """Add a matcher to the host's dispatch order; call in the order rules should be tried."""
    _request_matcher_slot()
    cg.add(host.register_matcher(matcher))


async def register_usb_client(config: ConfigType) -> MockObj:
    var = cg.new_Pvariable(config[CONF_ID], config[CONF_VID], config[CONF_PID])
    await cg.register_component(var, config)
    host = await cg.get_variable(config[CONF_USB_HOST_ID])
    register_client(host, var)
    register_matcher(host, var)
    return var


async def to_code(config: ConfigType) -> None:
    # IDF 6.0 moved USB host to an external component
    if idf_version() >= cv.Version(6, 0, 0):
        add_idf_component(name="espressif/usb", ref="1.4.1")
    add_idf_sdkconfig_option("CONFIG_USB_HOST_CONTROL_TRANSFER_MAX_SIZE", 1024)
    if config.get(CONF_ENABLE_HUBS):
        add_idf_sdkconfig_option("CONFIG_USB_HOST_HUBS_SUPPORTED", True)

    cg.add_define("USB_HOST_MAX_REQUESTS", config[CONF_MAX_TRANSFER_REQUESTS])
    cg.add_define("USB_HOST_MAX_PACKET_SIZE", config[CONF_MAX_PACKET_SIZE])

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    for device in config.get(CONF_DEVICES) or ():
        await register_usb_client(device)
