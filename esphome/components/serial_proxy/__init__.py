"""
Serial Proxy component for ESPHome.

WARNING: This component is EXPERIMENTAL. The API (both Python configuration
and C++ interfaces) may change at any time without following the normal
breaking changes policy. Use at your own risk.

Once the API is considered stable, this warning will be removed.

Provides a proxy to/from a serial interface on the ESPHome device, allowing
Home Assistant to connect to the serial port and send/receive data to/from
an arbitrary serial device.
"""

from dataclasses import dataclass

from esphome import pins
import esphome.codegen as cg
from esphome.components import uart
from esphome.components.const import (
    CONF_DTR_PIN,
    CONF_MANUFACTURER,
    CONF_RTS_PIN,
    CONF_SERIAL_NUMBER,
)
from esphome.components.usb_host import (
    CONF_INTERFACE_DESCRIPTION,
    CONF_PID,
    CONF_PRODUCT,
    CONF_VID,
)
from esphome.components.usb_uart import is_usb_uart_channel
import esphome.config_validation as cv
from esphome.const import CONF_IDENTITY, CONF_NAME, CONF_UART_ID
from esphome.core import CORE, coroutine_with_priority
from esphome.coroutine import CoroPriority
import esphome.final_validate as fv
from esphome.types import ConfigType

CODEOWNERS = ["@kbx81"]
DEPENDENCIES = ["api", "uart"]

MULTI_CONF = True

serial_proxy_ns = cg.esphome_ns.namespace("serial_proxy")
SerialProxy = serial_proxy_ns.class_("SerialProxy", cg.Component, uart.UARTDevice)
SerialProxyTap = serial_proxy_ns.class_("SerialProxyTap")

api_enums_ns = cg.esphome_ns.namespace("api").namespace("enums")
SerialProxyPortType = api_enums_ns.enum("SerialProxyPortType")
SERIAL_PROXY_PORT_TYPES = SerialProxyPortType.options(
    "TTL", "RS232", "RS485", "USB_SERIAL", prefix="SERIAL_PROXY_PORT_TYPE_"
)

CONF_BCD_DEVICE = "bcd_device"
CONF_INTERFACE_CLASS = "interface_class"
CONF_INTERFACE_NUMBER = "interface_number"
CONF_PORT_NUMBER = "port_number"
CONF_PORT_TYPE = "port_type"

# bInterfaceClass of a vendor-specific interface: only a chip driver bound to one numbers its ports
USB_CLASS_VENDOR_SPEC = 0xFF

DOMAIN = "serial_proxy"


@dataclass
class SerialProxyData:
    count: int = 0


def _get_data() -> SerialProxyData:
    if DOMAIN not in CORE.data:
        CORE.data[DOMAIN] = SerialProxyData()
    return CORE.data[DOMAIN]


def _validate_identity(config: ConfigType) -> ConfigType:
    # A client ignores every USB field of an identity without both IDs
    if CONF_VID not in config and (
        keys := [
            key
            for key in (
                CONF_BCD_DEVICE,
                CONF_INTERFACE_NUMBER,
                CONF_INTERFACE_CLASS,
                CONF_PORT_NUMBER,
            )
            if key in config
        ]
    ):
        raise cv.Invalid(f"{', '.join(keys)} need {CONF_VID} and {CONF_PID}")
    if (
        CONF_PORT_NUMBER in config
        and config.get(CONF_INTERFACE_CLASS) != USB_CLASS_VENDOR_SPEC
    ):
        raise cv.Invalid(
            f"{CONF_PORT_NUMBER} only applies to a vendor-specific interface "
            f"({CONF_INTERFACE_CLASS}: 0xFF); a class driver such as CDC ACM does not number its ports"
        )
    return config


# The USB device a port stands in for, cloned from its descriptors, for a port that has none
# of its own to read. Each string except the interface's may be a lambda, run once in setup(),
# for identifiers that differ per unit.
IDENTITY_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.Optional(CONF_MANUFACTURER): cv.templatable(cv.string),
            cv.Optional(CONF_PRODUCT): cv.templatable(cv.string),
            cv.Optional(CONF_SERIAL_NUMBER): cv.templatable(cv.string),
            cv.Inclusive(CONF_VID, "usb_ids"): cv.hex_uint16_t,
            cv.Inclusive(CONF_PID, "usb_ids"): cv.hex_uint16_t,
            cv.Optional(CONF_BCD_DEVICE): cv.hex_uint16_t,
            cv.Optional(CONF_INTERFACE_NUMBER): cv.uint8_t,
            cv.Optional(CONF_INTERFACE_CLASS): cv.hex_uint8_t,
            cv.Optional(CONF_PORT_NUMBER): cv.uint8_t,
            cv.Optional(CONF_INTERFACE_DESCRIPTION): cv.string_strict,
        }
    ),
    cv.has_at_least_one_key(
        CONF_MANUFACTURER,
        CONF_PRODUCT,
        CONF_SERIAL_NUMBER,
        CONF_VID,
        CONF_INTERFACE_DESCRIPTION,
    ),
    _validate_identity,
)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(SerialProxy),
            cv.Required(CONF_NAME): cv.string_strict,
            cv.Required(CONF_PORT_TYPE): cv.enum(SERIAL_PROXY_PORT_TYPES, upper=True),
            cv.Optional(CONF_RTS_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_DTR_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_IDENTITY): IDENTITY_SCHEMA,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(uart.UART_DEVICE_SCHEMA)
)


def _final_validate(config: ConfigType) -> ConfigType:
    is_usb = is_usb_uart_channel(config[CONF_UART_ID], fv.full_config.get())
    if config[CONF_PORT_TYPE] == "USB_SERIAL" and not is_usb:
        raise cv.Invalid(
            f"{CONF_PORT_TYPE} USB_SERIAL requires {CONF_UART_ID} to be a usb_uart channel"
        )
    if is_usb and CONF_IDENTITY in config:
        raise cv.Invalid(
            f"{CONF_IDENTITY} is read from the USB descriptors on a usb_uart channel; remove it"
        )
    return config


FINAL_VALIDATE_SCHEMA = _final_validate


@coroutine_with_priority(CoroPriority.FINAL)
async def _add_serial_proxy_count_define() -> None:
    """Emit the SERIAL_PROXY_COUNT define once with the final instance count."""
    count = _get_data().count
    if count > 0:
        cg.add_define("SERIAL_PROXY_COUNT", count)


async def to_code(config: ConfigType) -> None:
    var = await uart.new_uart_component(config)
    cg.add(cg.App.register_serial_proxy(var))
    cg.add(var.set_name(config[CONF_NAME]))
    cg.add(var.set_port_type(config[CONF_PORT_TYPE]))
    # port_type names the electrical interface (a USB RS485 adapter is RS485), so every
    # usb_uart channel reports USB identity whatever port type it declares
    if is_usb_uart_channel(config[CONF_UART_ID], CORE.config):
        await cg.add_variable(var.set_usb_channel, config[CONF_UART_ID])
        cg.add_define("USE_SERIAL_PROXY_USB_IDENTITY")
    if uart.is_host_uart(config[CONF_UART_ID], CORE.config):
        await cg.add_variable(var.set_host_uart, config[CONF_UART_ID])
        cg.add_define("USE_SERIAL_PROXY_HOST_UART")
    if (identity := config.get(CONF_IDENTITY)) is not None:
        cg.add_define("USE_SERIAL_PROXY_CONFIGURED_IDENTITY")
        for key, setter in (
            (CONF_MANUFACTURER, var.set_identity_manufacturer),
            (CONF_PRODUCT, var.set_identity_product),
            (CONF_SERIAL_NUMBER, var.set_identity_serial_number),
        ):
            if (value := identity.get(key)) is not None:
                cg.add(setter(await cg.templatable(value, [], cg.std_string)))
        cg.add(
            var.set_identity_usb(
                identity.get(CONF_VID, 0),
                identity.get(CONF_PID, 0),
                identity.get(CONF_BCD_DEVICE, 0),
                identity.get(CONF_INTERFACE_NUMBER, 0),
                identity.get(CONF_INTERFACE_CLASS, 0),
                identity.get(CONF_PORT_NUMBER, 0),
            )
        )
        cg.add_optional(
            var.set_identity_interface_description,
            identity.get(CONF_INTERFACE_DESCRIPTION),
        )
    cg.add_define("USE_SERIAL_PROXY")

    # Track instance count for the FINAL priority define
    data = _get_data()
    if data.count == 0:
        # Schedule the count define job only once (on the first instance)
        CORE.add_job(_add_serial_proxy_count_define)
    data.count += 1

    await cg.add_pin(var.set_rts_pin, config.get(CONF_RTS_PIN))
    await cg.add_pin(var.set_dtr_pin, config.get(CONF_DTR_PIN))
