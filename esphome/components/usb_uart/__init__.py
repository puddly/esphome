from dataclasses import dataclass

import esphome.codegen as cg
from esphome.components import usb_host
from esphome.components.const import CONF_DATA_BITS, CONF_PARITY, CONF_STOP_BITS
from esphome.components.esp32 import VARIANT_ESP32P4, get_esp32_variant
from esphome.components.uart import CONF_DEBUG_PREFIX, CONF_FLUSH_TIMEOUT, UARTComponent
from esphome.components.usb_host import (
    CONF_DRIVERS,
    CONF_PID,
    CONF_VID,
    get_max_packet_size,
    register_usb_client,
    usb_device_schema,
)
import esphome.config_validation as cv
from esphome.const import (
    CONF_BAUD_RATE,
    CONF_BUFFER_SIZE,
    CONF_CHANNELS,
    CONF_DEBUG,
    CONF_DUMMY_RECEIVER,
    CONF_ID,
    CONF_UART_ID,
)
from esphome.core import CORE, ID, coroutine_with_priority
from esphome.coroutine import CoroPriority
from esphome.cpp_generator import MockObj
from esphome.cpp_types import Component
from esphome.types import ConfigType

AUTO_LOAD = ["uart", "usb_host", "bytebuffer"]
CODEOWNERS = ["@clydebarrow"]

DOMAIN = "usb_uart"

usb_uart_ns = cg.esphome_ns.namespace("usb_uart")
USBUartComponent = usb_uart_ns.class_("USBUartComponent", Component)
USBUartChannel = usb_uart_ns.class_("USBUartChannel", UARTComponent)
USBUartTypeCdcAcm = usb_uart_ns.class_("USBUartTypeCdcAcm", USBUartComponent)
USBUartDispatcher = usb_uart_ns.class_("USBUartDispatcher")
USBUartDispatchRule = usb_uart_ns.struct("USBUartDispatchRule")


@dataclass
class UsbUartData:
    max_buffer_size: int = 0
    dispatcher: MockObj | None = None


def _get_data() -> UsbUartData:
    if DOMAIN not in CORE.data:
        CORE.data[DOMAIN] = UsbUartData()
        CORE.add_job(_finalize)
    return CORE.data[DOMAIN]


@coroutine_with_priority(CoroPriority.FINAL)
async def _finalize() -> None:
    data = _get_data()
    # The output chunk pool/queue are compile-time-sized templates shared by all
    # USBUartChannelBase instances, so use the largest buffer_size across every channel
    # of every device and slot. Add one extra slot because LockFreeQueue<T,N> is a ring
    # buffer that wastes one entry.
    output_chunk_count = max(data.max_buffer_size // get_max_packet_size(), 2) + 1
    cg.add_define("USB_UART_OUTPUT_CHUNK_COUNT", output_chunk_count)
    if data.dispatcher is None:
        return
    drivers = CORE.config[usb_host.DOMAIN][CONF_DRIVERS]
    cg.add_define("USB_UART_DISPATCH_RULE_COUNT", len(drivers))
    for rule in drivers:
        cg.add(
            data.dispatcher.add_rule(
                cg.StructInitializer(
                    USBUartDispatchRule,
                    (CONF_VID, rule[CONF_VID]),
                    (CONF_PID, rule[CONF_PID]),
                )
            )
        )


def _note_buffer_size(buffer_size: int) -> None:
    data = _get_data()
    data.max_buffer_size = max(data.max_buffer_size, buffer_size)


_request_slot = cg.slot_counter("USB_UART_SLOT_COUNT")

CONF_SLOT_ID = "slot_id"

# Keys a consumer adds to its own schema for one slot of the host's dispatcher pool
DISPATCH_SLOT_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_SLOT_ID): cv.declare_id(USBUartTypeCdcAcm),
        cv.GenerateID(CONF_UART_ID): cv.declare_id(USBUartChannel),
        cv.Optional(CONF_BUFFER_SIZE, default=256): cv.int_range(min=64, max=8192),
    }
)


async def new_dispatch_slot(host_id: ID, config: ConfigType) -> MockObj:
    """Add one slot to the host's dispatcher pool and return the slot's UART channel."""
    data = _get_data()
    host = await cg.get_variable(host_id)
    if data.dispatcher is None:
        data.dispatcher = cg.new_Pvariable(
            ID("usb_uart_dispatcher", is_declaration=True, type=USBUartDispatcher)
        )
        usb_host.register_matcher(host, data.dispatcher)
    slot = cg.new_Pvariable(config[CONF_SLOT_ID], 0, 0)
    await cg.register_component(slot, {})
    usb_host.register_client(host, slot)
    buffer_size = config[CONF_BUFFER_SIZE]
    channel = cg.new_Pvariable(config[CONF_UART_ID], 0, buffer_size)
    await cg.register_parented(channel, slot)
    cg.add(slot.add_channel(channel))
    _request_slot()
    cg.add(data.dispatcher.add_slot(slot))
    _note_buffer_size(buffer_size)
    return channel


def is_usb_uart_channel(uart_id: ID, full_config: ConfigType) -> bool:
    return any(
        channel[CONF_ID] == uart_id
        for device in full_config.get("usb_uart") or []
        for channel in device[CONF_CHANNELS]
    )


UARTParityOptions = usb_uart_ns.enum("UARTParityOptions")
UART_PARITY_OPTIONS = {
    "NONE": UARTParityOptions.UART_CONFIG_PARITY_NONE,
    "EVEN": UARTParityOptions.UART_CONFIG_PARITY_EVEN,
    "ODD": UARTParityOptions.UART_CONFIG_PARITY_ODD,
    "MARK": UARTParityOptions.UART_CONFIG_PARITY_MARK,
    "SPACE": UARTParityOptions.UART_CONFIG_PARITY_SPACE,
}

UARTStopBitsOptions = usb_uart_ns.enum("UARTStopBitsOptions")
UART_STOP_BITS_OPTIONS = {
    "1": UARTStopBitsOptions.UART_CONFIG_STOP_BITS_1,
    "1.5": UARTStopBitsOptions.UART_CONFIG_STOP_BITS_1_5,
    "2": UARTStopBitsOptions.UART_CONFIG_STOP_BITS_2,
}

DEFAULT_BAUD_RATE = 9600
CONF_CLAIM_COMM_INTERFACE = "claim_comm_interface"


class Type:
    def __init__(
        self,
        name: str,
        vid: int,
        pid: int,
        cls: str | None,
        max_channels: int = 1,
        baud_rate_required: bool = True,
        max_baud: int = 1_000_000,
        has_comm_interface: bool = False,
    ) -> None:
        self.name = name
        cls = cls or name
        self.vid = vid
        self.pid = pid
        self.cls = usb_uart_ns.class_(f"USBUartType{cls}", USBUartComponent)
        self._max_channels = max_channels
        self.baud_rate_required = baud_rate_required
        self.max_baud = max_baud
        # True for types that claim the CDC comm (interrupt) interface; only these
        # accept the claim_comm_interface option.
        self.has_comm_interface = has_comm_interface

    @property
    def max_channels(self) -> int:
        return (
            3
            if (
                CORE.is_esp32
                and get_esp32_variant() != VARIANT_ESP32P4
                and self._max_channels > 3
            )
            else self._max_channels
        )


uart_types = (
    Type(
        "CDC_ACM", 0, 0, "CdcAcm", 1, baud_rate_required=False, has_comm_interface=True
    ),
    Type("CH34X", 0x1A86, 0x55D5, "CH34X", 4, max_baud=2_000_000),
    Type("CH340", 0x1A86, 0x7523, "CH34X", 1, max_baud=2_000_000),
    Type("CP210X", 0x10C4, 0xEA60, "CP210X", 3, max_baud=2_000_000),
    Type(
        "ESP_JTAG",
        0x303A,
        0x1001,
        "CdcAcm",
        1,
        baud_rate_required=False,
        has_comm_interface=True,
    ),
    Type("FT232", 0x0403, 0x6001, "FT23XX", 1, max_baud=3_000_000),
    Type("FT2232", 0x0403, 0x6010, "FT23XX", 2, max_baud=12_000_000),
    Type("FT4232", 0x0403, 0x6011, "FT23XX", 4, max_baud=12_000_000),
    Type("PL2303", 0x067B, 0x2303, "PL2303", 1, max_baud=6_000_000),
    Type("PL2303GB", 0x067B, 0x23B3, "PL2303", 1, max_baud=6_000_000),
    Type("PL2303GC", 0x067B, 0x23A3, "PL2303", 1, max_baud=6_000_000),
    Type("PL2303GE", 0x067B, 0x23E3, "PL2303", 1, max_baud=6_000_000),
    Type("PL2303GL", 0x067B, 0x23D3, "PL2303", 1, max_baud=6_000_000),
    Type("PL2303GS", 0x067B, 0x23F3, "PL2303", 1, max_baud=6_000_000),
    Type("PL2303GT", 0x067B, 0x23C3, "PL2303", 1, max_baud=6_000_000),
    Type(
        "STM32_VCP",
        0x0483,
        0x5740,
        "CdcAcm",
        1,
        baud_rate_required=False,
        has_comm_interface=True,
    ),
)


def channel_schema(type_: "Type") -> cv.Schema:
    schema = cv.Schema(
        {
            cv.Required(CONF_CHANNELS): cv.All(
                cv.ensure_list(
                    cv.Schema(
                        {
                            cv.GenerateID(): cv.declare_id(USBUartChannel),
                            cv.Optional(CONF_BUFFER_SIZE, default=256): cv.int_range(
                                min=64, max=8192
                            ),
                            (
                                cv.Required(CONF_BAUD_RATE)
                                if type_.baud_rate_required
                                else cv.Optional(
                                    CONF_BAUD_RATE, default=DEFAULT_BAUD_RATE
                                )
                            ): cv.int_range(min=300, max=type_.max_baud),
                            cv.Optional(CONF_STOP_BITS, default="1"): cv.enum(
                                UART_STOP_BITS_OPTIONS, upper=True
                            ),
                            cv.Optional(CONF_PARITY, default="NONE"): cv.enum(
                                UART_PARITY_OPTIONS, upper=True
                            ),
                            cv.Optional(CONF_DATA_BITS, default=8): cv.int_range(
                                min=5, max=8
                            ),
                            cv.Optional(CONF_DUMMY_RECEIVER, default=False): cv.boolean,
                            cv.Optional(CONF_DEBUG, default=False): cv.boolean,
                            cv.Optional(CONF_DEBUG_PREFIX, default=""): cv.string,
                            cv.Optional(
                                CONF_FLUSH_TIMEOUT, default="100ms"
                            ): cv.positive_time_period_milliseconds,
                        }
                    )
                ),
                cv.Length(
                    max=type_.max_channels,
                    msg=f"Device type {type_.name} supports a maximum of {type_.max_channels} channels",
                ),
            ),
        }
    )
    if type_.has_comm_interface:
        # The comm (interrupt) interface pins a host hardware channel per device;
        # disable to save one on channel-poor hosts (some devices may need it
        # claimed before enabling data flow).
        schema = schema.extend(
            {cv.Optional(CONF_CLAIM_COMM_INTERFACE, default=True): cv.boolean}
        )
    else:
        schema = schema.extend(
            {
                cv.Optional(CONF_CLAIM_COMM_INTERFACE): cv.invalid(
                    f"'{CONF_CLAIM_COMM_INTERFACE}' is only supported on device types "
                    f"that claim the CDC comm interface; {type_.name} never claims it"
                )
            }
        )
    return schema


CONFIG_SCHEMA = cv.ensure_list(
    cv.typed_schema(
        {
            it.name: usb_device_schema(it.cls, it.vid, it.pid).extend(
                channel_schema(it)
            )
            for it in uart_types
        },
        upper=True,
    )
)


async def to_code(config: list[ConfigType]) -> None:
    # Auto-loaded for the dispatcher's slots, this runs with no devices of its own
    for device in config:
        var = await register_usb_client(device)
        # The C++ default is true; only emit the override
        if not device.get(CONF_CLAIM_COMM_INTERFACE, True):
            cg.add(var.set_claim_comm_interface(False))
        for index, channel in enumerate(device[CONF_CHANNELS]):
            _note_buffer_size(channel[CONF_BUFFER_SIZE])
            chvar = cg.new_Pvariable(channel[CONF_ID], index, channel[CONF_BUFFER_SIZE])
            await cg.register_parented(chvar, var)
            cg.add(chvar.set_stop_bits(channel[CONF_STOP_BITS]))
            cg.add(chvar.set_data_bits(channel[CONF_DATA_BITS]))
            cg.add(chvar.set_parity(channel[CONF_PARITY]))
            cg.add(chvar.set_baud_rate(channel[CONF_BAUD_RATE]))
            cg.add(chvar.set_dummy_receiver(channel[CONF_DUMMY_RECEIVER]))
            cg.add(chvar.set_flush_timeout(channel[CONF_FLUSH_TIMEOUT]))
            cg.add(chvar.set_debug(channel[CONF_DEBUG]))
            if channel[CONF_DEBUG_PREFIX]:
                cg.add(chvar.set_debug_prefix(channel[CONF_DEBUG_PREFIX]))
            cg.add(var.add_channel(chvar))
            if channel[CONF_DEBUG]:
                cg.add_define("USE_UART_DEBUGGER")
