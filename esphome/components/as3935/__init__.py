import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.const import (
    CONF_INDOOR,
    CONF_WATCHDOG_THRESHOLD,
    CONF_NOISE_LEVEL,
    CONF_SPIKE_REJECTION,
    CONF_LIGHTNING_THRESHOLD,
    CONF_MASK_DISTURBER,
    CONF_CAPACITANCE,
)

AUTO_LOAD = ["sensor", "binary_sensor"]
MULTI_CONF = True

CONF_AS3935_ID = "as3935_id"

as3935_ns = cg.esphome_ns.namespace("as3935")
AS3935 = as3935_ns.class_("AS3935Component", cg.Component)

AS3935DeviceMode = as3935_ns.enum("AS3935DeviceMode")
ENUM_AS3935_DEVICE_MODE = {
    "NORMAL": AS3935DeviceMode.AS3935_DEVICE_MODE_NORMAL,
    "NORMAL_AUTOCAL": AS3935DeviceMode.AS3935_DEVICE_MODE_NORMAL_AUTOCAL,
    "LCO_CALIBRATION": AS3935DeviceMode.AS3935_DEVICE_MODE_LCO_CALIBRATION,
    "SRCO_DISPLAY": AS3935DeviceMode.AS3935_DEVICE_MODE_SRCO_DISPLAY,
    "TRCO_DISPLAY": AS3935DeviceMode.AS3935_DEVICE_MODE_TRCO_DISPLAY,
}

CONF_IRQ_PIN = "irq_pin"
CONF_DEVICE_MODE = "device_mode"

AS3935_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(AS3935),
        cv.Required(CONF_IRQ_PIN): pins.gpio_input_pin_schema,
        cv.Optional(CONF_INDOOR, default=True): cv.boolean,
        cv.Optional(CONF_NOISE_LEVEL, default=2): cv.int_range(min=0, max=7),
        cv.Optional(CONF_WATCHDOG_THRESHOLD, default=2): cv.int_range(min=0, max=10),
        cv.Optional(CONF_SPIKE_REJECTION, default=2): cv.int_range(min=0, max=11),
        cv.Optional(CONF_LIGHTNING_THRESHOLD, default=1): cv.one_of(
            1, 5, 9, 16, int=True
        ),
        cv.Optional(CONF_MASK_DISTURBER, default=False): cv.boolean,
        # Device mode (normal, LCO calibration)
        cv.Optional(CONF_DEVICE_MODE, default="NORMAL"): cv.enum(
            ENUM_AS3935_DEVICE_MODE, upper=True
        ),
        cv.Optional(CONF_CAPACITANCE, default=0): cv.int_range(min=0, max=15),
    }
)


async def setup_as3935(var, config):
    await cg.register_component(var, config)

    irq_pin = await cg.gpio_pin_expression(config[CONF_IRQ_PIN])
    cg.add(var.set_irq_pin(irq_pin))
    cg.add(var.set_indoor(config[CONF_INDOOR]))
    cg.add(var.set_noise_level(config[CONF_NOISE_LEVEL]))
    cg.add(var.set_watchdog_threshold(config[CONF_WATCHDOG_THRESHOLD]))
    cg.add(var.set_spike_rejection(config[CONF_SPIKE_REJECTION]))
    cg.add(var.set_lightning_threshold(config[CONF_LIGHTNING_THRESHOLD]))
    cg.add(var.set_mask_disturber(config[CONF_MASK_DISTURBER]))
    cg.add(var.set_device_mode(config[CONF_DEVICE_MODE]))
    cg.add(var.set_capacitance(config[CONF_CAPACITANCE]))
