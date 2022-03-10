import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import fan, spi
from esphome.const import CONF_ID

CONF_AM_PIN = "am_pin"
CONF_CD_PIN = "cd_pin"
CONF_CE_PIN = "ce_pin"
CONF_DR_PIN = "dr_pin"
CONF_PWR_PIN = "pwr_pin"
CONF_TXEN_PIN = "txen_pin"

DEPENDENCIES = ["spi"]

nrf905_ns = cg.esphome_ns.namespace("nrf905")
nRF905Component = nrf905_ns.class_("nRF905", fan.FanState)

CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(nRF905Component),
            cv.Optional(CONF_CD_PIN): pins.gpio_input_pin_schema,
            cv.Required(CONF_CE_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_PWR_PIN): pins.gpio_output_pin_schema,
            cv.Required(CONF_TXEN_PIN): pins.gpio_output_pin_schema,
            cv.Optional(CONF_AM_PIN): pins.gpio_input_pin_schema,
            cv.Optional(CONF_DR_PIN): pins.gpio_input_pin_schema,
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(spi.spi_device_schema(cs_pin_required=True))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    if CONF_AM_PIN in config:
        data = await cg.gpio_pin_expression(config[CONF_AM_PIN])
        cg.add(var.set_am_pin(data))
    if CONF_CD_PIN in config:
        data = await cg.gpio_pin_expression(config[CONF_CD_PIN])
        cg.add(var.set_cd_pin(data))
    data = await cg.gpio_pin_expression(config[CONF_CE_PIN])
    cg.add(var.set_ce_pin(data))
    if CONF_DR_PIN in config:
        data = await cg.gpio_pin_expression(config[CONF_DR_PIN])
        cg.add(var.set_dr_pin(data))
    data = await cg.gpio_pin_expression(config[CONF_PWR_PIN])
    cg.add(var.set_pwr_pin(data))
    data = await cg.gpio_pin_expression(config[CONF_TXEN_PIN])
    cg.add(var.set_txen_pin(data))
