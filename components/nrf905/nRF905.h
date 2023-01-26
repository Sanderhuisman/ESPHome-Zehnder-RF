#ifndef __COMPONENT_nRF905_H__
#define __COMPONENT_nRF905_H__

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/helpers.h"
#include "esphome/components/spi/spi.h"
#include "nRF905.h"

namespace esphome {
namespace nrf905 {

#define MAX_TRANSMIT_TIME 2000      // TODO figure out what timeout we want
#define CARRIERDETECT_LED_DELAY 20  // On-board LED will light up for 20ms when data is received

/* nRF905 register sizes */
#define NRF905_REGISTER_COUNT 10
#define NRF905_MAX_FRAMESIZE 32

/* nRF905 Instructions */
#define NRF905_COMMAND_NOP 0xFF
#define NRF905_COMMAND_W_CONFIG 0x00
#define NRF905_COMMAND_R_CONFIG 0x10
#define NRF905_COMMAND_W_TX_PAYLOAD 0x20
#define NRF905_COMMAND_R_TX_PAYLOAD 0x21
#define NRF905_COMMAND_W_TX_ADDRESS 0x22
#define NRF905_COMMAND_R_TX_ADDRESS 0x23
#define NRF905_COMMAND_R_RX_PAYLOAD 0x24
#define NRF905_COMMAND_CHANNEL_CONFIG 0x80

// Bit positions
#define NRF905_STATUS_DR 5
#define NRF905_STATUS_AM 7

typedef enum {
  Ok,
  Failure,
} nRF905Cc;

typedef enum { PowerDown, Idle, Receive, Transmit } Mode;

typedef enum {
  ClkOut4000000 = 0x00,
  ClkOut2000000 = 0x01,
  ClkOut1000000 = 0x02,
  ClkOut500000 = 0x03,
} ClkOut;

typedef enum { PowerNormal = 0x00, PowerReduced = 0x01 } RxPower;

typedef struct {
  uint16_t channel;          // nRF905 RF channel
  bool band;                 // nRF905 href_ppl: false=434MHz band, true=868MHZ band
  RxPower rx_power;          // nRF905 Receive power: false=normal, true=reduced
  bool auto_retransmit;      // nRF905 Auto retransmission flag: false=off, true=on
  uint32_t rx_address;       // nRF905 Receive address
  uint8_t rx_address_width;  // nRF905 Receive address size (1-4 bytes)
  uint8_t rx_payload_width;  // nRF905 Receive payload size (1-32 bytes)
  // uint32_t tx_address;       // nRF905 Transmit address
  uint8_t tx_address_width;  // nRF905 Transmit address size (1-4 bytes)
  uint8_t tx_payload_width;  // nRF905 Transmit payload size (1-32 bytes)
  ClkOut clkOutFrequency;    // nRF905 clock out frequency
  bool clkOutEnable;         // nRF905 clock out enabled: false=off, true=on
  uint32_t xtal_frequency;   // nRF905 clock in frequency
  bool crc_enable;           // nRF905 Enable CRC: false=CRC disabled, true=CRC enabled
  uint8_t crc_bits;          // nRF905 CRC size: 8=8bit CRC, 16=16bit CRC
  uint32_t frequency;        // Internal: RF frequency (internal use; not nRF905 register)   --> TODO
  int8_t tx_power;           // nRF905 Transmit power (-10dBm, -2dBm, 6dBm or 10dBm)
} Config;

typedef struct {
  uint8_t command;
  uint8_t data[NRF905_REGISTER_COUNT];
} ConfigBuffer;

typedef struct {
  uint8_t command;
  uint8_t address[4];
} AddressBuffer;

typedef struct {
  uint8_t command;
  uint8_t payload[NRF905_MAX_FRAMESIZE];
} Buffer;

typedef std::function<void(void)> TxReadyCalllback;
typedef std::function<void(const uint8_t *const pBuffer, const uint8_t size)> RxCompleteCallback;

class nRF905 : public Component,
               public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW, spi::CLOCK_PHASE_LEADING,
                                     spi::DATA_RATE_1MHZ> {
 public:
  nRF905();

  void setup() override;

  // float get_setup_priority() const override { return setup_priority::HARDWARE; }
  float get_setup_priority() const override { return setup_priority::AFTER_CONNECTION; }

  void dump_config() override;
  void loop() override;

  void set_am_pin(GPIOPin *const pin) { _gpio_pin_am = pin; }
  void set_cd_pin(GPIOPin *const pin) { _gpio_pin_cd = pin; }
  void set_ce_pin(GPIOPin *const pin) { _gpio_pin_ce = pin; }
  void set_dr_pin(GPIOPin *const pin) { _gpio_pin_dr = pin; }
  void set_pwr_pin(GPIOPin *const pin) { _gpio_pin_pwr = pin; }
  void set_txen_pin(GPIOPin *const pin) { _gpio_pin_txen = pin; }

  void setOnRxComplete(RxCompleteCallback callback) { onRxComplete = callback; }
  void setOnTxReady(TxReadyCalllback callback) { onTxReady = callback; }

  Mode getMode(void) { return this->_mode; };
  void setMode(const Mode mode);

  Config getConfig(void) { return this->_config; }
  void updateConfig(Config *config, uint8_t *const pStatus = NULL);

  void writeTxAddress(const uint32_t txAddress, uint8_t *const pStatus = NULL);
  void readTxAddress(uint32_t *const pTxAddress, uint8_t *const pStatus = NULL);

  void writeTxPayload(const uint8_t *const pData, const uint8_t dataLength, uint8_t *const pStatus = NULL);
  void readTxPayload(uint8_t *const pData, const uint8_t dataLength, uint8_t *const pStatus = NULL);

  bool airwayBusy(void);

  void startTx(const uint32_t retransmit, const Mode nextMode);

  void printConfig(const Config *const pConfig);

 protected:
  void readRxPayload(uint8_t *const pData, const uint8_t dataLength, uint8_t *const pStatus = NULL);

  void readConfigRegisters(uint8_t *const pStatus = NULL);
  void writeConfigRegisters(uint8_t *const pStatus = NULL);

  void decodeConfigRegisters(const ConfigBuffer *const pBuffer, Config *const pConfig);
  void encodeConfigRegisters(const Config *const pConfig, ConfigBuffer *const pBuffer);

  uint8_t readStatus(void);

  void spiTransfer(uint8_t *const data, const size_t length);

  char *hexArrayToStr(const uint8_t *const pData, const size_t dataLength);

  RxCompleteCallback onRxComplete{NULL};

  uint32_t retransmitCounter{0};
  Mode nextMode{PowerDown};
  TxReadyCalllback onTxReady{NULL};

  GPIOPin *_gpio_pin_am{NULL};
  GPIOPin *_gpio_pin_cd{NULL};
  GPIOPin *_gpio_pin_ce{NULL};
  GPIOPin *_gpio_pin_dr{NULL};
  GPIOPin *_gpio_pin_pwr{NULL};
  GPIOPin *_gpio_pin_txen{NULL};

  Mode _mode{PowerDown};

  Config _config;
};

}  // namespace nrf905
}  // namespace esphome

#endif /* __COMPONENT_nRF905_H__ */
