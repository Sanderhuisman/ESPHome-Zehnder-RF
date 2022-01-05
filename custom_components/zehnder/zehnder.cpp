#include "zehnder.h"
#include "esphome/core/log.h"

namespace esphome {
namespace zehnder {

#define MAX_TRANSMIT_TIME 2000
#define MAX_RXBUFFER_SIZE 100

static const char *TAG = "zehnder";

typedef struct {
  uint32_t nrf905_tx_address;  // nRF905 Tx address
  uint32_t fan_network_id;     // Fan (Zehnder/BUVA) network ID
  uint8_t fan_my_device_type;  // Fan (Zehnder/BUVA) device type
  uint8_t fan_my_device_id;    // Fan (Zehnder/BUVA) device ID
  uint8_t fan_main_unit_type;  // Fan (Zehnder/BUVA) main unit type
  uint8_t fan_main_unit_id;    // Fan (Zehnder/BUVA) main unit ID
} NVRAMBuffer;

static NVRAMBuffer config;

// Transmit data
static std::function<void(void)> onTransmitReady = NULL;

typedef enum {
  DiscoverStateStart,                //
  DiscoverStateAvailableForJoining,  //
  DiscoverStateJoinRequest,          //
  DiscoverStateJoinComplete,         //
  DiscoverStateJoinFinish,
  DiscoverStateNrOf  // Keep last
} DiscoverState;

typedef struct {
  DiscoverState state;
  // Payload
  uint8_t payload[FAN_FRAMESIZE];
  uint32_t timeStart;
  uint8_t retries;

  // Discovery config
  uint8_t deviceId;
  uint8_t deviceType;
  uint32_t network_id;
} DiscoveryData;

static DiscoveryData discoveryData = {
    .state = DiscoverStateNrOf,
};

//  FAN_FRAME_SETVOLTAGE = 0x01,  // Set speed (voltage / percentage)
//  FAN_FRAME_SETSPEED = 0x02,    // Set speed (preset)
//  FAN_FRAME_SETTIMER = 0x03,    // Set speed with timer
//  FAN_NETWORK_JOIN_REQUEST = 0x04,
//  FAN_FRAME_SETSPEED_REPLY = 0x05,
//  FAN_NETWORK_JOIN_OPEN = 0x06,
//  FAN_TYPE_FAN_SETTINGS = 0x07,  // Current settings, sent by fan in reply to 0x01, 0x02, 0x10
//  FAN_FRAME_0B = 0x0B,
//  FAN_NETWORK_JOIN_ACK = 0x0C,
//  // FAN_NETWORK_JOIN_FINISH = 0x0D,
//  FAN_TYPE_QUERY_NETWORK = 0x0D,
//  FAN_TYPE_QUERY_DEVICE = 0x10,
//  FAN_FRAME_SETVOLTAGE_REPLY = 0x1D

typedef struct __attribute__((packed)) {
  uint32_t network_id;
  uint32_t reserved[5];
} RfPayloadNetworkJoinOpen;

typedef struct __attribute__((packed)) {
  uint32_t network_id;
  uint32_t reserved[5];
} RfPayloadNetworkJoinRequest;

typedef struct __attribute__((packed)) {
  uint32_t network_id;
  uint32_t reserved[5];
} RfPayloadNetworkJoinAck;

typedef struct __attribute__((packed)) {
  uint8_t speed;
  uint8_t voltage;
  uint8_t timer;
  uint32_t reserved[6];
} RfPayloadFanSettings;

typedef struct __attribute__((packed)) {
  uint8_t rx_type;          // 0x00 RX Type
  uint8_t rx_id;            // 0x01 RX ID
  uint8_t tx_type;          // 0x02 TX Type
  uint8_t tx_id;            // 0x03 TX ID
  uint8_t ttl;              // 0x04 Time-To-Live
  uint8_t command;          // 0x05 Frame type
  uint8_t parameter_count;  // 0x06 Number of parameters

  union {
    uint8_t parameters[9];                           // 0x07 - 0x0F Depends on command
    RfPayloadNetworkJoinRequest networkJoinRequest;  // Command 0x04
    RfPayloadNetworkJoinOpen networkJoinOpen;        // Command 0x06
    RfPayloadFanSettings fanSettings;                // Command 0x07
    RfPayloadNetworkJoinAck networkJoinAck;          // Command 0x0C
  } payload;
} RfFrame;

static bool linked = false;
static uint32_t lastQuery = 0;

ZehnderRF::ZehnderRF(void) {}

void ZehnderRF::setup() {
  config.fan_my_device_type = FAN_TYPE_REMOTE_CONTROL;
  config.fan_my_device_id = 0x04;
  config.fan_main_unit_type = FAN_TYPE_MAIN_UNIT;
  config.fan_main_unit_id = 0X1D;
  config.nrf905_tx_address = 0x89816EA9;

  ESP_LOGCONFIG(TAG, "ZEHNDER '%s':", this->get_name().c_str());

  auto traits = fan::FanTraits(false, true, false, 3);
  traits.set_speed(0);
  traits.set_supported_speed_count(3);
  this->set_traits(traits);

  this->add_on_state_callback([this]() { this->next_update_ = true; });

  this->_rf->setOnTxReady([this](void) {
    std::function<void(void)> transmitReadyCallback;
    ESP_LOGD(TAG, "Tx Ready");
    transmitReadyCallback = onTransmitReady;
    onTransmitReady = NULL;  // Clear callback such that next one can start

    if (transmitReadyCallback != NULL) {
      transmitReadyCallback();
    }
  });

  this->_rf->setOnRxComplete([this](uint8_t *const pData, const uint8_t dataLength) {
    if (linked == false) {
      this->discoveryEvent(DiscoveryEventRxComplete, pData, dataLength);
    } else {
      RfFrame *const pResponse = (RfFrame *) pData;

      if ((pResponse->rx_type = config.fan_my_device_type) && (pResponse->rx_id == config.fan_my_device_id)) {
        switch (pResponse->command) {
          case FAN_TYPE_FAN_SETTINGS:
            ESP_LOGD(TAG, "Received fan settings; speed=0x%02X voltage=%i timer=%i",
                     pResponse->payload.fanSettings.speed, pResponse->payload.fanSettings.voltage,
                     pResponse->payload.fanSettings.timer);
            break;

          default:
            ESP_LOGD(TAG, "Received unexpected frame; type 0x%02X from ID 0x%02X", pResponse->command,
                     pResponse->tx_id);
            break;
        }
      } else {
        ESP_LOGD(TAG, "Received frame from unknown device; type 0x%02X from ID 0x%02X type 0x%02X", pResponse->command,
                 pResponse->tx_id, pResponse->tx_type);
      }
    }
  });
}

void ZehnderRF::dump_config() {
  // ESP_LOGCONFIG(TAG, "ZEHNDER:");

  ESP_LOGCONFIG(TAG, "fan_network_id=%08X", config.fan_network_id);
  ESP_LOGCONFIG(TAG, "fan_my_device_type=%02X", config.fan_my_device_type);
  ESP_LOGCONFIG(TAG, "fan_my_device_id=%02X", config.fan_my_device_id);
  ESP_LOGCONFIG(TAG, "fan_main_unit_type=%02X", config.fan_main_unit_type);
  ESP_LOGCONFIG(TAG, "fan_main_unit_id=%02X", config.fan_main_unit_id);
}

void ZehnderRF::loop() {
  static bool startup = true;

  if (startup && millis() > 15000) {
    // Discovery
    uint8_t deviceId = this->createDeviceID();

    // this->discoveryStart(deviceId, FAN_JOIN_DEFAULT_TIMEOUT);
    startup = false;

    linked = true;
  }

  if (linked == true) {
    if (millis() - lastQuery > 10000) {
      lastQuery = millis();

      uint8_t payload[FAN_FRAMESIZE];
      RfFrame *const pFrame = (RfFrame *) payload;

      (void) memset(payload, 0, FAN_FRAMESIZE);

      pFrame->rx_type = config.fan_main_unit_type;
      pFrame->rx_id = config.fan_main_unit_id;
      pFrame->tx_type = config.fan_my_device_type;
      pFrame->tx_id = config.fan_my_device_id;
      pFrame->ttl = FAN_TTL;
      pFrame->command = FAN_TYPE_QUERY_NETWORK;
      pFrame->parameter_count = 0x00;  // No parameters

      startTransmit(
          payload, [this]() {}, FAN_TX_RETRIES);
    }
  }

  // Discovery Submodule Ish
  this->discoveryEvent(DiscoveryEventLoop);

  if (this->next_update_) {
    ESP_LOGD(TAG, "Something changed! Is it speed? %d %d", this->state, this->speed);
  }
  this->next_update_ = false;
}

uint8_t ZehnderRF::createDeviceID(void) {
  // return random(1, 254);  // Generate random device_id; don't use 0x00 and 0xFF
  return 4;  // TODO
  // TODO: there's a 1 in 255 chance that the generated ID matches the ID of the main unit. Decide how to deal with
  // this (some sort of ping discovery?)
}

void ZehnderRF::discoveryStart(const uint8_t deviceId, const uint32_t timeout) {
  nrf905::Config rfConfig;
  RfFrame *const pFrame = (RfFrame *) discoveryData.payload;

  ESP_LOGD(TAG, "Start discovery with ID %u", deviceId);

  discoveryData.deviceId = deviceId;
  discoveryData.deviceType = FAN_TYPE_REMOTE_CONTROL;

  (void) memset(discoveryData.payload, 0, FAN_FRAMESIZE);

  // Set payload, available for linking
  pFrame->rx_type = 0x04;
  pFrame->rx_id = 0x00;
  pFrame->tx_type = discoveryData.deviceType;
  pFrame->tx_id = discoveryData.deviceId;
  pFrame->ttl = FAN_TTL;
  pFrame->command = FAN_NETWORK_JOIN_ACK;
  pFrame->parameter_count = 0x04;
  pFrame->payload.networkJoinAck.network_id = NETWORK_LINK_ID;

  // Set mode idle
  this->_rf->setMode(nrf905::Idle);

  // Set RX and TX address
  rfConfig = this->_rf->getConfig();
  rfConfig.rx_address = NETWORK_LINK_ID;
  this->_rf->updateConfig(&rfConfig, NULL);
  this->_rf->writeTxAddress(NETWORK_LINK_ID, NULL);

  // Update state
  discoveryData.state = DiscoverStateAvailableForJoining;

  this->startTransmit(discoveryData.payload, NULL, FAN_TX_RETRIES);
  discoveryData.timeStart = millis();
}

void ZehnderRF::discoveryEvent(const DiscoveryEvent eventId, const uint8_t *const pData, const size_t dataLength) {
  nrf905::Config rfConfig;
  const RfFrame *const pFrameRx = (RfFrame *) pData;
  RfFrame *const pFrameTx = (RfFrame *) discoveryData.payload;

  switch (discoveryData.state) {
    case DiscoverStateStart:
      switch (eventId) {
        default:
          break;
      }

      break;

    case DiscoverStateAvailableForJoining:
      switch (eventId) {
        case DiscoveryEventLoop:
          if ((discoveryData.timeStart > 0) &&  // Frame sent
              ((millis() - discoveryData.timeStart) > FAN_REPLY_TIMEOUT)) {
            ESP_LOGD(TAG, "Discovery: join timeout");

            discoveryData.timeStart = 0;

            if (discoveryData.retries > 0) {
              --discoveryData.retries;
              ESP_LOGD(TAG, "No data received, retry (left: %u)", discoveryData.retries);

              this->startTransmit(
                  discoveryData.payload, [this]() { discoveryData.timeStart = millis(); }, FAN_TX_RETRIES);
            } else {
              discoveryData.state = DiscoverStateNrOf;  // TODO restart discovery
            }
          }
          break;

        case DiscoveryEventRxComplete:
          switch (pFrameRx->command) {
            case FAN_NETWORK_JOIN_OPEN:  // Received linking request from main unit
              ESP_LOGD(TAG, "Discovery: Found unit type 0x%02X%s with ID 0x%02X on network 0x%08X", pFrameRx->tx_type,
                       pFrameRx->tx_type == FAN_TYPE_MAIN_UNIT ? " (Main)" : "", pFrameRx->tx_id,
                       pFrameRx->payload.networkJoinOpen.network_id);

              discoveryData.timeStart = 0;  // Disable rx timeout detection

              // Found a main unit, so send a join request
              pFrameTx->rx_type = FAN_TYPE_MAIN_UNIT;        // Set type to main unit
              pFrameTx->rx_id = pFrameRx->tx_id;             // Set ID to the ID of the main unit
              pFrameTx->command = FAN_NETWORK_JOIN_REQUEST;  // Request to connect to network
              pFrameTx->parameter_count = 0x04;
              // Request to connect to the received network ID
              pFrameTx->payload.networkJoinRequest.network_id = pFrameRx->payload.networkJoinOpen.network_id;

              // Store for later
              config.fan_main_unit_type = pFrameRx->tx_type;
              config.fan_main_unit_id = pFrameRx->tx_id;
              config.nrf905_tx_address = pFrameRx->payload.networkJoinOpen.network_id;

              // Update address
              rfConfig = this->_rf->getConfig();
              rfConfig.rx_address = pFrameRx->payload.networkJoinOpen.network_id;
              this->_rf->updateConfig(&rfConfig, NULL);
              this->_rf->writeTxAddress(pFrameRx->payload.networkJoinOpen.network_id, NULL);

              // Send response frame
              discoveryData.retries = 4;
              this->startTransmit(
                  discoveryData.payload, [this]() { discoveryData.timeStart = millis(); }, FAN_TX_RETRIES);

              discoveryData.state = DiscoverStateJoinRequest;  // Next state
              break;

            default:
              ESP_LOGD(TAG, "Discovery: Received unknown frame type 0x%02X from ID 0x%02X on network 0x%08X",
                       pFrameRx->command, pFrameRx->tx_id, pFrameRx->payload.networkJoinOpen.network_id);
              break;
          }
          break;

        default:
          break;
      }
      break;

    case DiscoverStateJoinRequest:
      switch (eventId) {
        case DiscoveryEventLoop:
          if ((discoveryData.timeStart > 0) &&  // Frame sent
              ((millis() - discoveryData.timeStart) > FAN_REPLY_TIMEOUT)) {
            ESP_LOGD(TAG, "Discovery: join timeout");

            discoveryData.timeStart = 0;

            if (discoveryData.retries > 0) {
              --discoveryData.retries;
              ESP_LOGD(TAG, "No data received, retry (left: %u)", discoveryData.retries);

              this->startTransmit(
                  discoveryData.payload, [this]() { discoveryData.timeStart = millis(); }, FAN_TX_RETRIES);
            } else {
              discoveryData.state = DiscoverStateNrOf;  // TODO restart discovery
            }
          }
          break;

        case DiscoveryEventRxComplete:
          ESP_LOGD(TAG, "DiscoverStateJoinRequest");
          switch (pFrameRx->command) {
            case FAN_FRAME_0B:
              if ((pFrameRx->rx_type == discoveryData.deviceType) && (pFrameRx->rx_id == discoveryData.deviceId) &&
                  (pFrameRx->tx_type == config.fan_main_unit_type) && (pFrameRx->tx_id == config.fan_main_unit_id)) {
                ESP_LOGD(TAG, "Discovery: Link successful to unit with ID 0x%02X on network 0x%08X", pFrameRx->tx_id,
                         config.nrf905_tx_address);

                discoveryData.timeStart = 0;  // Disable rx timeout detection

                discoveryData.command = FAN_FRAME_0B;  // 0x0B acknowledgee link successful
                discoveryData.parameter_count = 0x00;  // No parameters
                (void) memset(pFrameTx->payload.parameters, 0, sizeof(pFrameTx->payload.parameters));

                discoveryData.retries = 4;
                startTransmit(
                    discoveryData.payload, [this]() { discoveryData.timeStart = millis(); }, FAN_TX_RETRIES);

                discoveryData.state = DiscoverStateJoinComplete;
              } else {
                ESP_LOGE(TAG, "Discovery: Received unknown link succes from ID 0x%02X on network 0x%08X",
                         pFrameRx->tx_id, config.nrf905_tx_address);
              }
              break;

            default:
              ESP_LOGE(TAG, "Discovery: Received unknown frame type 0x%02X from ID 0x%02X on network 0x%08X",
                       pFrameRx->command, pFrameRx->tx_id, config.nrf905_tx_address);
              break;
          }
          break;

        default:
          break;
      }
      break;

    case DiscoverStateJoinComplete:
      switch (eventId) {
        case DiscoveryEventLoop:
          if ((discoveryData.timeStart > 0) &&  // Frame sent
              ((millis() - discoveryData.timeStart) > FAN_REPLY_TIMEOUT)) {
            ESP_LOGD(TAG, "Discovery: join timeout");

            discoveryData.timeStart = 0;

            if (discoveryData.retries > 0) {
              --discoveryData.retries;
              ESP_LOGD(TAG, "No data received, retry (left: %u)", discoveryData.retries);

              this->startTransmit(
                  discoveryData.payload, [this]() { discoveryData.timeStart = millis(); }, FAN_TX_RETRIES);
            } else {
              discoveryData.state = DiscoverStateNrOf;  // TODO restart discovery
            }
          }
          break;

        case DiscoveryEventRxComplete:
          ESP_LOGD(TAG, "DiscoverStateJoinComplete");
          switch (pFrameRx->command) {
            case FAN_TYPE_QUERY_NETWORK:
              ESP_LOGD(TAG, "Discovery: received network join success 0x0D");

              linked = true;

              //   if ((rxbuffer[i][0x00] == tx_type) && (rxbuffer[i][0x01] == tx_id) && (rxbuffer[i][0x02] == tx_type)
              //   &&
              //       (rxbuffer[i][0x03] == tx_id)) {
              //     if (result != FAN_RESULT_SUCCESS) {
              //       Serial.printf("Debug: 0x0D sanity check: ok\n");
              //       result = FAN_RESULT_SUCCESS;
              //     }
              //   }

              discoveryData.state = DiscoverStateJoinFinish;

              break;

            default:
              ESP_LOGE(TAG, "Discovery: Received unknown frame type 0x%02X from ID 0x%02X on network 0x%08X",
                       pFrameRx->command, pFrameRx->tx_id, config.nrf905_tx_address);
              break;
          }
          break;

        default:
          break;
      }
      break;

    case DiscoverStateJoinFinish:
      break;

    default:
      break;
  }
}

void ZehnderRF::startTransmit(const uint8_t *const pData, const std::function<void(void)> callback,
                              const size_t retries) {
  unsigned long startTime;
  bool busy = true;

  if (onTransmitReady == NULL) {
  } else {
    ESP_LOGE(TAG, "TX still ongoing");
  }

  // Write data to RF
  this->_rf->writeTxPayload(pData, FAN_FRAMESIZE, NULL);  // Use framesize

  busy = true;
  startTime = millis();
  while (((millis() - startTime) < MAX_TRANSMIT_TIME) && busy)  // Collision detection
  {
    if (this->_rf->airwayBusy() == false) {
      busy = false;
      break;
    }
    delay(1);  // Before polling again
  }

  if (busy == false) {
    ESP_LOGD(TAG, "Start TX");
    this->_rf->startTx(FAN_TX_FRAMES, nrf905::Receive);

    onTransmitReady = callback;
  }
}

}  // namespace zehnder
}  // namespace esphome
