import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from esphome.const import (
    CONF_DISTANCE,
    CONF_LIGHTNING_ENERGY,
    UNIT_KILOMETER,
    UNIT_EMPTY,
    ICON_SIGNAL_DISTANCE_VARIANT,
    ICON_FLASH,
)
from . import AS3935, CONF_AS3935_ID

DEPENDENCIES = ["as3935"]
CONF_FAULTS_STATE = "faults_state"
ICON_ALERT_CIRCLE = "mdi:alert-circle"

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(CONF_AS3935_ID): cv.use_id(AS3935),
        cv.Optional(CONF_DISTANCE): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOMETER,
            icon=ICON_SIGNAL_DISTANCE_VARIANT,
            accuracy_decimals=0,
        ),
        cv.Optional(CONF_LIGHTNING_ENERGY): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_FLASH,
            accuracy_decimals=0,
        ),
        cv.Optional(CONF_FAULTS_STATE): sensor.sensor_schema(
            unit_of_measurement=UNIT_EMPTY,
            icon=ICON_ALERT_CIRCLE,
            accuracy_decimals=0,
        ),
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    hub = await cg.get_variable(config[CONF_AS3935_ID])

    if CONF_DISTANCE in config:
        conf = config[CONF_DISTANCE]
        distance_sensor = await sensor.new_sensor(conf)
        cg.add(hub.set_distance_sensor(distance_sensor))

    if CONF_LIGHTNING_ENERGY in config:
        conf = config[CONF_LIGHTNING_ENERGY]
        lightning_energy_sensor = await sensor.new_sensor(conf)
        cg.add(hub.set_energy_sensor(lightning_energy_sensor))

    if CONF_FAULTS_STATE in config:
        conf = config[CONF_FAULTS_STATE]
        faults_state_sensor = await sensor.new_sensor(conf)
        cg.add(hub.set_faults_state_sensor(faults_state_sensor))
