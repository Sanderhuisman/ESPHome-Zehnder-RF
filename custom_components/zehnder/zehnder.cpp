#include "zehnder.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"

namespace esphome {
namespace zehnder {

#define MAX_TRANSMIT_TIME 2000

static const char *TAG = "zehnder";

typedef struct {
  uint32_t fan_networkId;      // Fan (Zehnder/BUVA) network ID
  uint8_t fan_my_device_type;  // Fan (Zehnder/BUVA) device type
  uint8_t fan_my_device_id;    // Fan (Zehnder/BUVA) device ID
  uint8_t fan_main_unit_type;  // Fan (Zehnder/BUVA) main unit type
  uint8_t fan_main_unit_id;    // Fan (Zehnder/BUVA) main unit ID
} NVRAMBuffer;

static NVRAMBuffer config;

static uint32_t msgSendTime = 0;
static uint32_t airwayFreeWaitTime = 0;
static int8_t retries = -1;

static uint32_t lastFanQuery = 0;

// typedef enum {
//   DiscoverStateStart,                //
//   DiscoverStateWaitForLinkRequest,   //
//   DiscoverStateWaitForJoinResponse,  //
//   DiscoverStateJoinComplete,         //
//   DiscoverStateNrOf                  // Keep last
// } DiscoverState;

// typedef struct {
//   DiscoverState state;
//   // Payload
//   uint8_t payload[FAN_FRAMESIZE];
//   uint32_t timeStart;
//   int8_t retries;

//   // Discovery config
//   uint8_t deviceId;
//   uint8_t deviceType;
//   uint32_t networkId;
// } DiscoveryData;

// static DiscoveryData discoveryData = {
//     // .state = DiscoverStateNrOf,
// };

static std::function<void(void)> onReceiveTimeout = NULL;

typedef struct __attribute__((packed)) {
  uint32_t networkId;
} RfPayloadNetworkJoinOpen;

typedef struct __attribute__((packed)) {
  uint32_t networkId;
} RfPayloadNetworkJoinRequest;

typedef struct __attribute__((packed)) {
  uint32_t networkId;
} RfPayloadNetworkJoinAck;

typedef struct __attribute__((packed)) {
  uint8_t speed;
  uint8_t voltage;
  uint8_t timer;
} RfPayloadFanSettings;

typedef struct __attribute__((packed)) {
  uint8_t speed;
} RfPayloadFanSetSpeed;

typedef struct __attribute__((packed)) {
  uint8_t speed;
  uint8_t timer;
} RfPayloadFanSetTimer;

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
    RfPayloadFanSetSpeed setSpeed;                   // Command 0x02
    RfPayloadFanSetTimer setTimer;                   // Command 0x03
    RfPayloadNetworkJoinRequest networkJoinRequest;  // Command 0x04
    RfPayloadNetworkJoinOpen networkJoinOpen;        // Command 0x06
    RfPayloadFanSettings fanSettings;                // Command 0x07
    RfPayloadNetworkJoinAck networkJoinAck;          // Command 0x0C
  } payload;
} RfFrame;

typedef enum {
  RfStateIdle,            // Idle state
  RfStateWaitAirwayFree,  // wait for airway free
  RfStateTxBusy,          //
  RfStateRxWait,
} RfState;

static RfState rfState = RfStateIdle;

static uint32_t lastQuery = 0;

ZehnderRF::ZehnderRF(void) {}

void ZehnderRF::setup() {
  config.fan_networkId = 0x89816EA9;
  config.fan_my_device_type = FAN_TYPE_REMOTE_CONTROL;
  config.fan_my_device_id = 0x04;
  config.fan_main_unit_type = FAN_TYPE_MAIN_UNIT;
  config.fan_main_unit_id = 0X1D;

  ESP_LOGCONFIG(TAG, "ZEHNDER '%s':", this->get_name().c_str());

  fan::FanTraits traits = fan::FanTraits(false, true, false, 3);
  traits.set_speed(true);
  traits.set_supported_speed_count(4);
  this->set_traits(traits);

  this->add_on_state_callback([this]() {
    ESP_LOGV(TAG, "Something changed");
    this->next_update_ = true;
  });

  this->rf_->setOnTxReady([this](void) {
    ESP_LOGD(TAG, "Tx Ready");
    if (rfState == RfStateTxBusy) {
      if (retries >= 0) {
        msgSendTime = millis();
        rfState = RfStateRxWait;
      } else {
        rfState = RfStateIdle;
      }
    }
  });

  this->rf_->setOnRxComplete([this](const uint8_t *const pData, const uint8_t dataLength) {
    ESP_LOGV(TAG, "Received frame");
    this->rfHandleReceived(pData, dataLength);
  });
}

void ZehnderRF::dump_config(void) {
  ESP_LOGCONFIG(TAG, "Zehnder Fan config:");
  ESP_LOGCONFIG(TAG, "  Polling interval   %u", this->interval_);
  ESP_LOGCONFIG(TAG, "  Fan networkId      0x%08X", config.fan_networkId);
  ESP_LOGCONFIG(TAG, "  Fan my device type 0x%02X", config.fan_my_device_type);
  ESP_LOGCONFIG(TAG, "  Fan my device id   0x%02X", config.fan_my_device_id);
  ESP_LOGCONFIG(TAG, "  Fan main_unit type 0x%02X", config.fan_main_unit_type);
  ESP_LOGCONFIG(TAG, "  Fan main unit id   0x%02X", config.fan_main_unit_id);
}

void ZehnderRF::loop(void) {
  // Run RF handler
  this->rfHandler();

  switch (this->state_) {
    case StateStartup:
      // Wait until started up
      if (millis() > 15000) {
        // Discovery
        this->state_ = StateStartDiscovery;
      }
      break;

    case StateStartDiscovery:
      // uint8_t deviceId = this->createDeviceID();
      // this->discoveryStart(deviceId, FAN_JOIN_DEFAULT_TIMEOUT);

      // For now just set TX
      nrf905::Config rfConfig;
      rfConfig = this->rf_->getConfig();
      rfConfig.rx_address = config.fan_networkId;
      this->rf_->updateConfig(&rfConfig);
      this->rf_->writeTxAddress(config.fan_networkId);

      this->state_ = StateIdle;
      break;

    case StateIdle:
      if (this->next_update_ == true) {
        this->next_update_ = false;
        // Something changed

        this->state = true;
        this->speed = this->speed > 0 ? this->speed : 0x01;

        // Set speed
        this->setSpeed(this->state ? this->speed : 0x00, 0);

        //
        lastFanQuery = millis();  // Update time
      } else if ((millis() - lastFanQuery) > this->interval_) {
        lastFanQuery = millis();  // Update time
        this->queryDevice();
      }
      break;

    case StateWaitSetSpeedConfirm:
      if (rfState == RfStateIdle) {
        // When done, return to idle
        this->state_ = StateIdle;
      }

    default:
      break;
  }
}

void ZehnderRF::rfHandleReceived(const uint8_t *const pData, const uint8_t dataLength) {
  const RfFrame *const pResponse = (RfFrame *) pData;
  RfFrame *const pTxFrame = (RfFrame *) this->_txFrame;  // frame helper
  nrf905::Config rfConfig;

  ESP_LOGD(TAG, "Current state: 0x%02X", this->state_);
  switch (this->state_) {
    case StateDiscoveryWaitForLinkRequest:
      ESP_LOGD(TAG, "DiscoverStateWaitForLinkRequest");
      switch (pResponse->command) {
        case FAN_NETWORK_JOIN_OPEN:  // Received linking request from main unit
          ESP_LOGD(TAG, "Discovery: Found unit type 0x%02X (%s) with ID 0x%02X on network 0x%08X", pResponse->tx_type,
                   pResponse->tx_type == FAN_TYPE_MAIN_UNIT ? "Main" : "?", pResponse->tx_id,
                   pResponse->payload.networkJoinOpen.networkId);

          this->rfComplete();

          (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);  // Clear frame data

          // Found a main unit, so send a join request
          pTxFrame->rx_type = FAN_TYPE_MAIN_UNIT;  // Set type to main unit
          pTxFrame->rx_id = pResponse->tx_id;      // Set ID to the ID of the main unit
          pTxFrame->tx_type = config.fan_my_device_type;
          pTxFrame->tx_id = config.fan_my_device_id;
          pTxFrame->ttl = FAN_TTL;
          pTxFrame->command = FAN_NETWORK_JOIN_REQUEST;  // Request to connect to network
          pTxFrame->parameter_count = sizeof(RfPayloadNetworkJoinOpen);
          // Request to connect to the received network ID
          pTxFrame->payload.networkJoinRequest.networkId = pResponse->payload.networkJoinOpen.networkId;

          // Store for later
          config.fan_networkId = pResponse->payload.networkJoinOpen.networkId;
          config.fan_main_unit_type = pResponse->tx_type;
          config.fan_main_unit_id = pResponse->tx_id;

          // Update address
          rfConfig = this->rf_->getConfig();
          rfConfig.rx_address = pResponse->payload.networkJoinOpen.networkId;
          this->rf_->updateConfig(&rfConfig, NULL);
          this->rf_->writeTxAddress(pResponse->payload.networkJoinOpen.networkId, NULL);

          // Send response frame
          this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]() {
            ESP_LOGW(TAG, "Query Timeout");
            this->state_ = StateStartDiscovery;
          });

          this->state_ = StateDiscoveryWaitForJoinResponse;
          break;

        default:
          ESP_LOGD(TAG, "Discovery: Received unknown frame type 0x%02X from ID 0x%02X", pResponse->command,
                   pResponse->tx_id);
          break;
      }
      break;

    case StateDiscoveryWaitForJoinResponse:
      ESP_LOGD(TAG, "DiscoverStateWaitForJoinResponse");
      switch (pResponse->command) {
        case FAN_FRAME_0B:
          if ((pResponse->rx_type == config.fan_my_device_type) && (pResponse->rx_id == config.fan_my_device_id) &&
              (pResponse->tx_type == config.fan_main_unit_type) && (pResponse->tx_id == config.fan_main_unit_id)) {
            ESP_LOGD(TAG, "Discovery: Link successful to unit with ID 0x%02X on network 0x%08X", pResponse->tx_id,
                     config.fan_networkId);

            this->rfComplete();

            (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);  // Clear frame data

            pTxFrame->rx_type = FAN_TYPE_MAIN_UNIT;  // Set type to main unit
            pTxFrame->rx_id = pResponse->tx_id;      // Set ID to the ID of the main unit
            pTxFrame->tx_type = config.fan_my_device_type;
            pTxFrame->tx_id = config.fan_my_device_id;
            pTxFrame->ttl = FAN_TTL;
            pTxFrame->command = FAN_FRAME_0B;  // 0x0B acknowledgee link successful
            pTxFrame->parameter_count = 0x00;  // No parameters

            // Send response frame
            this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]() {
              ESP_LOGW(TAG, "Query Timeout");
              this->state_ = StateStartDiscovery;
            });

            this->state_ = StateDiscoveryJoinComplete;
          } else {
            ESP_LOGE(TAG, "Discovery: Received unknown link succes from ID 0x%02X on network 0x%08X", pResponse->tx_id,
                     config.fan_networkId);
          }
          break;

        default:
          ESP_LOGE(TAG, "Discovery: Received unknown frame type 0x%02X from ID 0x%02X", pResponse->command,
                   pResponse->tx_id);
          break;
      }
      break;

    case StateDiscoveryJoinComplete:
      ESP_LOGD(TAG, "StateDiscoveryJoinComplete");
      switch (pResponse->command) {
        case FAN_TYPE_QUERY_NETWORK:
          if ((pResponse->rx_type == config.fan_main_unit_type) && (pResponse->rx_id == config.fan_main_unit_id) &&
              (pResponse->tx_type == config.fan_main_unit_type) && (pResponse->tx_id == config.fan_main_unit_id)) {
            ESP_LOGD(TAG, "Discovery: received network join success 0x0D");

            this->rfComplete();

            this->state_ = StateIdle;
          } else {
            ESP_LOGW(TAG, "Unexpected frame join reponse from Type 0x%02X ID 0x%02X", pResponse->tx_type,
                     pResponse->tx_id);
          }
          break;

        default:
          ESP_LOGE(TAG, "Discovery: Received unknown frame type 0x%02X from ID 0x%02X on network 0x%08X",
                   pResponse->command, pResponse->tx_id, config.fan_networkId);
          break;
      }
      break;

    case StateWaitQueryResponse:
      if ((pResponse->rx_type == config.fan_my_device_type) &&  // If type
          (pResponse->rx_id == config.fan_my_device_id)) {      // and id match, it is for us
        switch (pResponse->command) {
          case FAN_TYPE_FAN_SETTINGS:
            ESP_LOGD(TAG, "Received fan settings; speed: 0x%02X voltage: %i timer: %i",
                     pResponse->payload.fanSettings.speed, pResponse->payload.fanSettings.voltage,
                     pResponse->payload.fanSettings.timer);

            this->rfComplete();

            this->state_ = StateIdle;
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
      break;

    case StateWaitSetSpeedResponse:
      if ((pResponse->rx_type == config.fan_my_device_type) &&  // If type
          (pResponse->rx_id == config.fan_my_device_id)) {      // and id match, it is for us
        switch (pResponse->command) {
          case FAN_TYPE_FAN_SETTINGS:
            ESP_LOGD(TAG, "Received fan settings; speed: 0x%02X voltage: %i timer: %i",
                     pResponse->payload.fanSettings.speed, pResponse->payload.fanSettings.voltage,
                     pResponse->payload.fanSettings.timer);
            this->rfComplete();

            this->rfComplete();

            (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);  // Clear frame data

            pTxFrame->rx_type = config.fan_main_unit_type;  // Set type to main unit
            pTxFrame->rx_id = config.fan_main_unit_id;      // Set ID to the ID of the main unit
            pTxFrame->tx_type = config.fan_my_device_type;
            pTxFrame->tx_id = config.fan_my_device_id;
            pTxFrame->ttl = FAN_TTL;
            pTxFrame->command = FAN_FRAME_SETSPEED_REPLY;  // 0x0B acknowledgee link successful
            pTxFrame->parameter_count = 0x03;              // 3 parameters
            pTxFrame->payload.parameters[0] = 0x54;
            pTxFrame->payload.parameters[1] = 0x03;
            pTxFrame->payload.parameters[2] = 0x20;

            // Send response frame
            this->startTransmit(this->_txFrame, -1, NULL);

            this->state_ = StateWaitSetSpeedConfirm;
            break;

          case FAN_FRAME_SETSPEED_REPLY:
          case FAN_FRAME_SETVOLTAGE_REPLY:
            // this->rfComplete();

            // this->state_ = StateIdle;
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
      break;

    default:
      ESP_LOGD(TAG, "Received frame from unknown device in unknown state; type 0x%02X from ID 0x%02X type 0x%02X",
               pResponse->command, pResponse->tx_id, pResponse->tx_type);
      break;
  }
}

static uint8_t minmax(const uint8_t value, const uint8_t min, const uint8_t max) {
  if (value <= min) {
    return min;
  } else if (value >= max) {
    return max;
  } else {
    return value;
  }
}

uint8_t ZehnderRF::createDeviceID(void) {
  uint8_t random = (uint8_t) random_uint32();
  // Generate random device_id; don't use 0x00 and 0xFF

  // TODO: there's a 1 in 255 chance that the generated ID matches the ID of the main unit. Decide how to deal
  // withthis (some sort of ping discovery?)

  return minmax(random, 1, 0xFE);
}

void ZehnderRF::queryDevice(void) {
  RfFrame *const pFrame = (RfFrame *) this->_txFrame;  // frame helper

  ESP_LOGD(TAG, "Query device");

  // Clear frame data
  (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);

  // Build frame
  pFrame->rx_type = config.fan_main_unit_type;
  pFrame->rx_id = config.fan_main_unit_id;
  pFrame->tx_type = config.fan_my_device_type;
  pFrame->tx_id = config.fan_my_device_id;
  pFrame->ttl = FAN_TTL;
  pFrame->command = FAN_TYPE_QUERY_DEVICE;
  pFrame->parameter_count = 0x00;  // No parameters

  this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]() {
    ESP_LOGW(TAG, "Query Timeout");
    this->state_ = StateIdle;
  });

  this->state_ = StateWaitQueryResponse;
}

void ZehnderRF::setSpeed(const uint8_t speed, const uint8_t timer) {
  RfFrame *const pFrame = (RfFrame *) this->_txFrame;  // frame helper

  ESP_LOGD(TAG, "Set speed: 0x%02X", speed);

  (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);  // Clear frame data

  // Build frame
  pFrame->rx_type = config.fan_main_unit_type;
  pFrame->rx_id = config.fan_main_unit_id;
  pFrame->tx_type = config.fan_my_device_type;
  pFrame->tx_id = config.fan_my_device_id;
  pFrame->ttl = FAN_TTL;

  if (timer == 0) {
    pFrame->command = FAN_FRAME_SETSPEED;
    pFrame->parameter_count = sizeof(RfPayloadFanSetSpeed);
    pFrame->payload.setSpeed.speed = speed;
  } else {
    pFrame->command = FAN_FRAME_SETTIMER;
    pFrame->parameter_count = sizeof(RfPayloadFanSetTimer);
    pFrame->payload.setTimer.speed = speed;
    pFrame->payload.setTimer.timer = timer;
  }

  this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]() {
    ESP_LOGW(TAG, "Set speed timeout");
    this->state_ = StateIdle;
  });

  this->state_ = StateWaitSetSpeedResponse;
}

// void ZehnderRF::updateSpeed(const uint8_t speed) {
//   // auto call = this->make_call();
//   // call.set_state(speed);
//   // call.perform();
// }

void ZehnderRF::discoveryStart(const uint8_t deviceId) {
  RfFrame *const pFrame = (RfFrame *) this->_txFrame;  // frame helper
  nrf905::Config rfConfig;

  ESP_LOGD(TAG, "Start discovery with ID %u", deviceId);

  config.fan_my_device_type = FAN_TYPE_REMOTE_CONTROL;
  config.fan_my_device_id = deviceId;

  // Build frame
  (void) memset(this->_txFrame, 0, FAN_FRAMESIZE);  // Clear frame data

  // Set payload, available for linking
  pFrame->rx_type = 0x04;
  pFrame->rx_id = 0x00;
  pFrame->tx_type = config.fan_my_device_type;
  pFrame->tx_id = config.fan_my_device_id;
  pFrame->ttl = FAN_TTL;
  pFrame->command = FAN_NETWORK_JOIN_ACK;
  pFrame->parameter_count = sizeof(RfPayloadNetworkJoinAck);
  pFrame->payload.networkJoinAck.networkId = NETWORK_LINK_ID;

  // Set RX and TX address
  rfConfig = this->rf_->getConfig();
  rfConfig.rx_address = NETWORK_LINK_ID;
  this->rf_->updateConfig(&rfConfig, NULL);
  this->rf_->writeTxAddress(NETWORK_LINK_ID, NULL);

  this->startTransmit(this->_txFrame, FAN_TX_RETRIES, [this]() {
    ESP_LOGW(TAG, "Start discovery timeout");
    this->state_ = StateStartDiscovery;
  });

  // Update state
  this->state_ = StateDiscoveryWaitForLinkRequest;
}

Result ZehnderRF::startTransmit(const uint8_t *const pData, const int8_t rxRetries,
                                const std::function<void(void)> callback) {
  Result result = ResultOk;
  unsigned long startTime;
  bool busy = true;

  if (rfState != RfStateIdle) {
    ESP_LOGW(TAG, "TX still ongoing");
    result = ResultBusy;
  } else {
    onReceiveTimeout = callback;
    retries = rxRetries;

    // Write data to RF
    // if (pData != NULL) {  // If frame given, load it in the nRF. Else use previous TX payload
    // ESP_LOGD(TAG, "Write payload");
    this->rf_->writeTxPayload(pData, FAN_FRAMESIZE);  // Use framesize
    // }

    rfState = RfStateWaitAirwayFree;
    airwayFreeWaitTime = millis();
  }

  return result;
}

void ZehnderRF::rfComplete(void) {
  retries = -1;  // Disable retries
  rfState = RfStateIdle;
}

void ZehnderRF::rfHandler(void) {
  switch (rfState) {
    case RfStateIdle:
      break;

    case RfStateWaitAirwayFree:
      if ((millis() - airwayFreeWaitTime) > 5000) {
        ESP_LOGW(TAG, "Airway too busy, giving up");
        rfState = RfStateIdle;

        if (onReceiveTimeout != NULL) {
          onReceiveTimeout();
        }
      } else if (this->rf_->airwayBusy() == false) {
        ESP_LOGD(TAG, "Start TX");
        this->rf_->startTx(FAN_TX_FRAMES, nrf905::Receive);  // After transmit, wait for response

        rfState = RfStateTxBusy;
      }
      break;

    case RfStateTxBusy:
      break;

    case RfStateRxWait:
      if ((retries >= 0) && ((millis() - msgSendTime) > FAN_REPLY_TIMEOUT)) {
        ESP_LOGD(TAG, "Receive timeout");

        if (retries > 0) {
          --retries;
          ESP_LOGD(TAG, "No data received, retry again (left: %u)", retries);

          rfState = RfStateWaitAirwayFree;
          airwayFreeWaitTime = millis();
        } else if (retries == 0) {
          // Oh oh, ran out of options

          ESP_LOGD(TAG, "No messages received, giving up now...");
          if (onReceiveTimeout != NULL) {
            onReceiveTimeout();
          }

          // Back to idle
          rfState = RfStateIdle;
        }
      }
      break;

    default:
      break;
  }
}

}  // namespace zehnder
}  // namespace esphome
