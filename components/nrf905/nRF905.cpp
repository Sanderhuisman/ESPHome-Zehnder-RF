#include "nRF905.h"
#include "esphome/core/log.h"

#include <string.h>

#define CHECK_REG_WRITE true

namespace esphome {
namespace nrf905 {

static const char *TAG = "nRF905";

nRF905::nRF905(void) {}

void nRF905::setup() {
  Config config;

  ESP_LOGD(TAG, "Start nRF905 init");

  this->spi_setup();
  if (this->_gpio_pin_am != NULL) {
    this->_gpio_pin_am->setup();
  }
  if (this->_gpio_pin_cd != NULL) {
    this->_gpio_pin_cd->setup();
  }
  this->_gpio_pin_ce->setup();
  if (this->_gpio_pin_dr != NULL) {
    this->_gpio_pin_dr->setup();
  }
  this->_gpio_pin_pwr->setup();
  this->_gpio_pin_txen->setup();

  this->setMode(PowerDown);

  this->readConfigRegisters();

  this->_config.band = true;
  this->_config.channel = 118;

  // CRC 16
  this->_config.crc_enable = true;
  this->_config.crc_bits = 16;

  // TX power 10
  this->_config.tx_power = 10;

  // RX power normal
  this->_config.rx_power = PowerNormal;

  this->_config.rx_address = 0x89816EA9;  // ZEHNDER_NETWORK_LINK_ID;
  this->_config.rx_address_width = 4;
  this->_config.rx_payload_width = 16;

  this->_config.tx_address_width = 4;
  this->_config.tx_payload_width = 16;

  this->_config.xtal_frequency = 16000000;  // defaults for now
  this->_config.clkOutFrequency = ClkOut500000;
  this->_config.clkOutEnable = false;

  // Write config back
  this->writeConfigRegisters();
  this->writeTxAddress(0x89816EA9);

  // Return to idle
  this->setMode(Idle);

  ESP_LOGD(TAG, "nRF905 Setup complete");
}

void nRF905::dump_config() {
  ESP_LOGCONFIG(TAG, "Config:");

  LOG_PIN("  CS Pin:", this->cs_);
  if (this->_gpio_pin_am != NULL) {
    LOG_PIN("  AM Pin:", this->_gpio_pin_am);
  }
  if (this->_gpio_pin_dr != NULL) {
    LOG_PIN("  DR Pin:", this->_gpio_pin_dr);
  }
  if (this->_gpio_pin_cd != NULL) {
    LOG_PIN("  CD Pin:", this->_gpio_pin_cd);
  }
  LOG_PIN("  CE Pin:", this->_gpio_pin_ce);
  LOG_PIN("  PWR Pin:", this->_gpio_pin_pwr);
  LOG_PIN("  TXEN Pin:", this->_gpio_pin_txen);
}

void nRF905::loop() {
  static uint8_t lastState = 0x00;
  static bool addrMatch;
  uint8_t buffer[NRF905_MAX_FRAMESIZE];

  uint8_t state = this->readStatus() & ((1 << NRF905_STATUS_DR) | (1 << NRF905_STATUS_AM));
  if (lastState != state) {
    ESP_LOGV(TAG, "State change: 0x%02X -> 0x%02X", lastState, state);
    if (state == ((1 << NRF905_STATUS_DR) | (1 << NRF905_STATUS_AM))) {
      addrMatch = false;

      // Read data
      this->readRxPayload(buffer, NRF905_MAX_FRAMESIZE);
      ESP_LOGV(TAG, "RX Complete: %s", hexArrayToStr(buffer, NRF905_MAX_FRAMESIZE));

      if (this->onRxComplete != NULL) {
        this->onRxComplete(buffer, NRF905_MAX_FRAMESIZE);
      }
    } else if (state == (1 << NRF905_STATUS_DR)) {
      addrMatch = false;

      // ESP_LOGD(TAG, "TX Ready; retransmits: %u", this->retransmitCounter);
      // if (this->retransmitCounter > 0) {
      //   --this->retransmitCounter;
      // } else {
      this->setMode(this->nextMode);

      if (this->onTxReady != NULL) {
        this->onTxReady();
      }
      // }
    } else if (state == (1 << NRF905_STATUS_AM)) {
      addrMatch = true;
      ESP_LOGD(TAG, "Addr match");

      // if (onAddrMatch != NULL)
      //   onAddrMatch(this);
    } else if (state == 0 && addrMatch) {
      addrMatch = false;
      ESP_LOGD(TAG, "Rx Invalid");
      // if (onRxInvalid != NULL)
      //   onRxInvalid(this);
    }

    lastState = state;
  }

  // _drPrev = _drNew;
}

void nRF905::setMode(const Mode mode) {
  // Set power
  switch (mode) {
    case PowerDown:
      this->_gpio_pin_pwr->digital_write(false);
      break;

    default:
      this->_gpio_pin_pwr->digital_write(true);
      break;
  }

  // Set CE
  switch (mode) {
    case Receive:  // fall through
    case Transmit:
      this->_gpio_pin_ce->digital_write(true);
      break;

    default:
      this->_gpio_pin_ce->digital_write(false);
      break;
  }

  // Enable TX
  switch (mode) {
    case Transmit:
      this->_gpio_pin_txen->digital_write(true);
      break;

    default:
      this->_gpio_pin_txen->digital_write(false);
      break;
  }

  this->_mode = mode;
}

void nRF905::updateConfig(Config *config, uint8_t *const pStatus) {
  this->_config = *config;

  this->writeConfigRegisters(pStatus);
}

void nRF905::readConfigRegisters(uint8_t *const pStatus) {
  Mode mode;
  ConfigBuffer buffer;

  // Set mode to idle
  mode = this->_mode;
  this->setMode(Idle);

  // Prepare data
  buffer.command = NRF905_COMMAND_R_CONFIG;
  (void) memset(buffer.data, 0, sizeof(buffer.data));

  // Transfer
  this->spiTransfer((uint8_t *) &buffer, sizeof(ConfigBuffer));
  if (pStatus != NULL) {
    *pStatus = buffer.command;
  }

  // Ccear
  (void) memset(&this->_config, 0, sizeof(Config));
  this->decodeConfigRegisters(&buffer, &this->_config);

  // Restore mode
  this->setMode(mode);
}

void nRF905::writeConfigRegisters(uint8_t *const pStatus) {
  Mode mode;
  ConfigBuffer buffer;
#if CHECK_REG_WRITE
  uint8_t writeData[NRF905_REGISTER_COUNT];
#endif

  mode = this->_mode;
  this->setMode(Idle);

  this->printConfig(&this->_config);

  // Create data
  buffer.command = NRF905_COMMAND_W_CONFIG;
  this->encodeConfigRegisters(&this->_config, &buffer);

  ESP_LOGV(TAG, "Write config data: %s", hexArrayToStr(buffer.data, NRF905_REGISTER_COUNT));
#if CHECK_REG_WRITE
  (void) memcpy(writeData, buffer.data, NRF905_REGISTER_COUNT);
#endif

  this->spiTransfer((uint8_t *) &buffer, sizeof(ConfigBuffer));

#if CHECK_REG_WRITE
  // Check config write by reading config back and compare
  {
    ConfigBuffer bufferRead;

    bufferRead.command = NRF905_COMMAND_R_CONFIG;
    (void) memset(bufferRead.data, 0, NRF905_REGISTER_COUNT);

    this->spiTransfer((uint8_t *) &bufferRead, sizeof(ConfigBuffer));
    if (memcmp((void *) writeData, (void *) bufferRead.data, NRF905_REGISTER_COUNT) != 0) {
      ESP_LOGE(TAG, "Config write failed");
    } else {
      ESP_LOGV(TAG, "Write config OK");
    }
  }
#endif

  if (pStatus != NULL) {
    *pStatus = buffer.command;
  }

  // Restore mode
  this->setMode(mode);
}

void nRF905::writeTxAddress(const uint32_t txAddress, uint8_t *const pStatus) {
  Mode mode;
  AddressBuffer buffer;

  ESP_LOGD(TAG, "Set TX Address: 0x%08X", txAddress);

  mode = this->_mode;
  this->setMode(Idle);

  buffer.command = NRF905_COMMAND_W_TX_ADDRESS;
  buffer.address[3] = (txAddress >> 24) & 0xFF;
  buffer.address[2] = (txAddress >> 16) & 0xFF;
  buffer.address[1] = (txAddress >> 8) & 0xFF;
  buffer.address[0] = (txAddress) &0xFF;

  this->spiTransfer((uint8_t *) &buffer, sizeof(AddressBuffer));

  if (pStatus != NULL) {
    *pStatus = buffer.command;
  }

  // Restore mode
  this->setMode(mode);
}

void nRF905::readTxAddress(uint32_t *pTxAddress, uint8_t *const pStatus) {
  Mode mode;
  AddressBuffer buffer;

  mode = this->_mode;
  this->setMode(Idle);

  buffer.command = NRF905_COMMAND_R_TX_ADDRESS;
  (void) memset(buffer.address, 0, 4);

  this->spiTransfer((uint8_t *) &buffer, sizeof(AddressBuffer));

  *pTxAddress = buffer.address[0];
  *pTxAddress |= (buffer.address[1] << 8);
  *pTxAddress |= (buffer.address[2] << 16);
  *pTxAddress |= (buffer.address[3] << 24);

  ESP_LOGD(TAG, "Got TX Address: 0x%08X", *pTxAddress);

  if (pStatus != NULL) {
    *pStatus = buffer.command;
  }

  this->setMode(mode);
}

void nRF905::readTxPayload(uint8_t *const pData, const uint8_t dataLength, uint8_t *const pStatus) {
  Mode mode;
  Buffer buffer;

  if (pData == NULL) {
    ESP_LOGE(TAG, "Read TX payload data pointer invalid");
    return;
  }
  if (dataLength > NRF905_MAX_FRAMESIZE) {
    ESP_LOGE(TAG, "Read TX payload data length invalid");
    return;
  }

  buffer.command = NRF905_COMMAND_R_TX_PAYLOAD;
  (void) memset(buffer.payload, 0, NRF905_MAX_FRAMESIZE);

  mode = this->_mode;
  this->setMode(Idle);

  this->spiTransfer((uint8_t *) &buffer, sizeof(Buffer));
  (void) memcpy(pData, buffer.payload, dataLength);

  if (pStatus != NULL) {
    *pStatus = buffer.command;
  }

  this->setMode(mode);
}

void nRF905::writeTxPayload(const uint8_t *const pData, const uint8_t dataLength, uint8_t *const pStatus) {
  Mode mode;
  Buffer buffer;

  if (pData == NULL) {
    ESP_LOGE(TAG, "Write data pointer invalid");
    return;
  }
  if (dataLength > NRF905_MAX_FRAMESIZE) {
    ESP_LOGE(TAG, "Write data length invalid");
    return;
  }

  ESP_LOGV(TAG, "Read config data: %s", hexArrayToStr(pData, dataLength));

  // Clear buffer payload
  (void) memset(buffer.payload, 0, NRF905_MAX_FRAMESIZE);

  buffer.command = NRF905_COMMAND_W_TX_PAYLOAD;
  (void) memcpy(buffer.payload, (uint8_t *) pData, dataLength);

  mode = this->_mode;
  this->setMode(Idle);

  this->spiTransfer((uint8_t *) &buffer, sizeof(Buffer));
  if (pStatus != NULL) {
    *pStatus = buffer.command;
  }

  this->setMode(mode);
}

void nRF905::readRxPayload(uint8_t *const pData, const uint8_t dataLength, uint8_t *const pStatus) {
  Buffer buffer;

  if (pData == NULL) {
    ESP_LOGE(TAG, "Read RX data pointer invalid");
    return;
  }
  if (dataLength > NRF905_MAX_FRAMESIZE) {
    ESP_LOGE(TAG, "Read RX data length invalid");
    return;
  }

  buffer.command = NRF905_COMMAND_R_RX_PAYLOAD;
  (void) memset(buffer.payload, 0, NRF905_MAX_FRAMESIZE);

  this->spiTransfer((uint8_t *) &buffer, sizeof(Buffer));

  (void) memcpy(pData, buffer.payload, dataLength);

  // Return status if needed
  if (pStatus != NULL) {
    *pStatus = buffer.command;
  }
}

void nRF905::decodeConfigRegisters(const ConfigBuffer *const pBuffer, Config *const pConfig) {
  pConfig->channel = ((pBuffer->data[1] & 0x01) << 8) | pBuffer->data[0];
  pConfig->band = (pBuffer->data[1] & 0x02) ? true : false;
  pConfig->rx_power = (RxPower)(pBuffer->data[1] & 0x10);
  pConfig->auto_retransmit = (pBuffer->data[1] & 0x20) ? true : false;
  pConfig->rx_address_width = pBuffer->data[2] & 0x07;
  pConfig->tx_address_width = (pBuffer->data[2] >> 4) & 0x07;
  pConfig->rx_payload_width = pBuffer->data[3] & 0x3F;
  pConfig->tx_payload_width = pBuffer->data[4] & 0x3F;
  pConfig->rx_address =
      ((pBuffer->data[8] << 24) | (pBuffer->data[7] << 16) | (pBuffer->data[6] << 8) | pBuffer->data[5]);
  pConfig->clkOutFrequency = (ClkOut)(pBuffer->data[9] & 0x03);
  pConfig->clkOutEnable = (pBuffer->data[9] & 0x04) ? true : false;
  pConfig->xtal_frequency = (((pBuffer->data[9] >> 3) & 0x07) + 1) * 4000000;
  pConfig->crc_enable = (pBuffer->data[9] & 0x40) ? true : false;
  pConfig->crc_bits = (pBuffer->data[9] & 0x80) ? 16 : 8;

  pConfig->frequency = ((422400000 + (pConfig->channel * 100000)) * (pConfig->band ? 2 : 1));  // internal
  switch ((pBuffer->data[1] >> 2) & 0x03) {
    case 0x00:
      pConfig->tx_power = -10;
      break;

    case 0x01:
      pConfig->tx_power = -2;
      break;

    case 0x02:
      pConfig->tx_power = 6;
      break;

    case 0x03:
      pConfig->tx_power = 10;
      break;

    default:
      pConfig->tx_power = 10;
      break;
  }
}

void nRF905::encodeConfigRegisters(const Config *const pConfig, ConfigBuffer *const pBuffer) {
  uint8_t tx_power;

  switch (pConfig->tx_power) {
    case -10:
      tx_power = 0x00;
      break;

    case -2:
      tx_power = 0x04;
      break;

    case 6:
      tx_power = 0x08;
      break;

    case 10:
      tx_power = 0x0C;
      break;

    default:
      tx_power = 0x0C;
      break;
  }

  pBuffer->data[0] = (pConfig->channel & 0xFF);
  pBuffer->data[1] = (pConfig->channel >> 8) & 0x01;
  pBuffer->data[1] |= (pConfig->band ? 0x02 : 0x00);
  pBuffer->data[1] |= tx_power;
  pBuffer->data[1] |= (pConfig->rx_power == PowerReduced ? 0x10 : 0x00);
  pBuffer->data[1] |= (pConfig->auto_retransmit ? 0x20 : 0x00);
  pBuffer->data[2] = (pConfig->rx_address_width & 0x07);
  pBuffer->data[2] |= (pConfig->tx_address_width & 0x07) << 4;
  pBuffer->data[3] = (pConfig->rx_payload_width & 0x3F);
  pBuffer->data[4] = (pConfig->tx_payload_width & 0x3F);
  pBuffer->data[5] = (pConfig->rx_address & 0xFF);
  pBuffer->data[6] = (pConfig->rx_address >> 8) & 0xFF;
  pBuffer->data[7] = (pConfig->rx_address >> 16) & 0xFF;
  pBuffer->data[8] = (pConfig->rx_address >> 24) & 0xFF;
  pBuffer->data[9] = pConfig->clkOutFrequency;  // use enum value
  pBuffer->data[9] |= (pConfig->clkOutEnable ? 0x04 : 0x00);
  pBuffer->data[9] |= ((pConfig->xtal_frequency / 4000000) - 1) << 3;
  pBuffer->data[9] |= (pConfig->crc_enable ? 0x40 : 0x00);
  pBuffer->data[9] |= (pConfig->crc_bits == 8) ? 0x00 : 0x80;
}

void nRF905::printConfig(const Config *const pConfig) {
  uint32_t hz = 0;

  switch (pConfig->clkOutFrequency) {
    case ClkOut500000:
      hz = 500000;
      break;

    case ClkOut1000000:
      hz = 1000000;
      break;

    case ClkOut2000000:
      hz = 2000000;
      break;

    case ClkOut4000000:
      hz = 4000000;
      break;

    default:
      ESP_LOGD(TAG, "Unvalid clock freq");
      break;
  }

  ESP_LOGV(TAG,
           "Config:\r\n"
           "  Channel %u Band %s MHz -> %u\r\n"
           "  Rx Power %s\r\n"
           "  Tx Retransmit %s\r\n"
           "  Rx Address (%u) 0x%08X\r\n"
           "  Rx Payload width %u\r\n"
           "  Tx Address (%u)\r\n"
           "  Tx Payload width %u\r\n"
           "  Clk Out %u\r\n"
           "  XTAL Freq %u\r\n"
           "  CRC %s -> %u\r\n"
           "  TX Power %d dBm",
           pConfig->channel, pConfig->band ? "868" : "434", pConfig->frequency,
           pConfig->rx_power ? "reduced" : "normal", pConfig->auto_retransmit ? "On" : "Off", pConfig->rx_address_width,
           pConfig->rx_address, pConfig->rx_payload_width, pConfig->tx_address_width, pConfig->tx_payload_width, hz,
           pConfig->xtal_frequency, pConfig->crc_enable ? "On" : "Off", pConfig->crc_bits, pConfig->tx_power);
}

bool nRF905::airwayBusy(void) {
  bool busy = false;

  if (this->_gpio_pin_cd != NULL) {
    busy = this->_gpio_pin_cd->digital_read() == true;
  }

  return busy;
}

void nRF905::startTx(const uint32_t retransmit, const Mode nextMode) {
  bool update = false;
  if (this->_mode == PowerDown) {
    this->setMode(Idle);
    delay(3);  // Delay is needed to the radio has time to power-up and see the standby/TX pins pulse
  }

  // Update counters
  // this->retransmitCounter = retransmit;
  this->nextMode = nextMode;

  // Set or clear retransmit flag
  // if ((this->_config.auto_retransmit == false) && (retransmit > 0)) {
  //   this->_config.auto_retransmit = true;
  //   update = true;
  // } else if ((this->_config.auto_retransmit == true) && (retransmit == 0)) {
  this->_config.auto_retransmit = false;
  update = true;
  // }
  if (update == true) {
    this->writeConfigRegisters();
  }

  // Start transmit
  this->setMode(Transmit);
}

uint8_t nRF905::readStatus(void) {
  uint8_t status = 0;

  status = NRF905_COMMAND_NOP;

  this->spiTransfer(&status, 1);

  return status;
}

void nRF905::spiTransfer(uint8_t *const data, const size_t length) {
  this->enable();

  this->transfer_array(data, length);

  this->disable();
}

char *nRF905::hexArrayToStr(const uint8_t *const pData, const size_t dataLength) {
  static char buf[256];
  size_t bufIdx = 0;

  for (size_t i = 0; i < dataLength; ++i) {
    if (i > 0) {
      bufIdx += snprintf(&buf[bufIdx], 256 - bufIdx, " ");
    }
    bufIdx += snprintf(&buf[bufIdx], 256 - bufIdx, "0x%02X", pData[i]);
  }

  return buf;
}

}  // namespace nrf905
}  // namespace esphome
