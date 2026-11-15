import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import uart, sensor, binary_sensor, cs1258
from esphome.const import (
    CONF_ID,
    UNIT_KILOGRAM,
    DEVICE_CLASS_WEIGHT,
    STATE_CLASS_MEASUREMENT,
    DEVICE_CLASS_CONNECTIVITY,
    ENTITY_CATEGORY_DIAGNOSTIC,
)

DEPENDENCIES = ["uart", "cs1258"]
AUTO_LOAD = ["sensor", "binary_sensor"]

s7_mcu_ns = cg.esphome_ns.namespace("s7_mcu")
S7McuComponent = s7_mcu_ns.class_(
    "S7McuComponent", cg.Component, uart.UARTDevice
)

CONF_WEIGHT_HI = "weight_hi"
CONF_WEIGHT_LO = "weight_lo"
CONF_WEIGHT_DIV = "weight_div"
CONF_WEIGHT_OFFSET = "weight_offset"
CONF_WEIGHT_SENSOR = "weight"
CONF_WEIGHT_RAW_SENSOR = "weight_raw"
CONF_MEASUREMENT_ID_SENSOR = "measurement_id"
CONF_MEASUREMENT_VALID = "measurement_valid"
CONF_MCU_ALIVE = "mcu_alive"
CONF_CS1258_ID = "cs1258_id"

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(S7McuComponent),
            cv.Required(CONF_CS1258_ID): cv.use_id(cs1258.CS1258Component),
            cv.Optional(CONF_WEIGHT_HI, default=0): cv.int_range(min=0, max=30),
            cv.Optional(CONF_WEIGHT_LO, default=1): cv.int_range(min=0, max=30),
            cv.Optional(CONF_WEIGHT_DIV, default=100.0): cv.float_,
            cv.Optional(CONF_WEIGHT_OFFSET, default=0.0): cv.float_,
            cv.Optional(CONF_WEIGHT_SENSOR): sensor.sensor_schema(
                unit_of_measurement=UNIT_KILOGRAM,
                accuracy_decimals=2,
                device_class=DEVICE_CLASS_WEIGHT,
                state_class=STATE_CLASS_MEASUREMENT,
            ),
            cv.Optional(CONF_WEIGHT_RAW_SENSOR): sensor.sensor_schema(
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_MEASUREMENT_ID_SENSOR): sensor.sensor_schema(
                accuracy_decimals=0,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_MEASUREMENT_VALID): binary_sensor.binary_sensor_schema(
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
            cv.Optional(CONF_MCU_ALIVE): binary_sensor.binary_sensor_schema(
                device_class=DEVICE_CLASS_CONNECTIVITY,
                entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
            ),
        }
    )
    .extend(uart.UART_DEVICE_SCHEMA)
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await uart.register_uart_device(var, config)
    cs1258_var = await cg.get_variable(config[CONF_CS1258_ID])
    cg.add(var.set_cs1258_component(cs1258_var))

    cg.add(var.set_weight_hi(config[CONF_WEIGHT_HI]))
    cg.add(var.set_weight_lo(config[CONF_WEIGHT_LO]))
    cg.add(var.set_weight_div(config[CONF_WEIGHT_DIV]))
    cg.add(var.set_weight_offset(config[CONF_WEIGHT_OFFSET]))

    if CONF_WEIGHT_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_WEIGHT_SENSOR])
        cg.add(var.set_weight_sensor(sens))

    if CONF_WEIGHT_RAW_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_WEIGHT_RAW_SENSOR])
        cg.add(var.set_weight_raw_sensor(sens))

    if CONF_MEASUREMENT_ID_SENSOR in config:
        sens = await sensor.new_sensor(config[CONF_MEASUREMENT_ID_SENSOR])
        cg.add(var.set_measurement_id_sensor(sens))

    if CONF_MEASUREMENT_VALID in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_MEASUREMENT_VALID])
        cg.add(var.set_measurement_valid_sensor(bs))

    if CONF_MCU_ALIVE in config:
        bs = await binary_sensor.new_binary_sensor(config[CONF_MCU_ALIVE])
        cg.add(var.set_mcu_alive_sensor(bs))
