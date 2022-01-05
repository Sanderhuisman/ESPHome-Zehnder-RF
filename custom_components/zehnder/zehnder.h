#ifndef __COMPONENT_ZEHNDER_H__
#define __COMPONENT_ZEHNDER_H__

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/fan/fan_state.h"
#include "esphome/components/nrf905/nRF905.h"

namespace esphome {
namespace zehnder {

#define FAN_FRAMESIZE 16        // Each frame consists of 16 bytes
#define FAN_TX_FRAMES 4         // Retransmit every transmitted frame 4 times
#define FAN_TX_RETRIES 10       // Retry transmission 10 times if no reply is received
#define FAN_TTL 250             // 0xFA, default time-to-live for a frame
#define FAN_REPLY_TIMEOUT 1000  // Wait 500ms for receiving a reply when doing a network scan

/* Fan device types */
enum {
  FAN_TYPE_BROADCAST = 0x00,       // Broadcast to all devices
  FAN_TYPE_MAIN_UNIT = 0x01,       // Fans
  FAN_TYPE_REMOTE_CONTROL = 0x03,  // Remote controls
  FAN_TYPE_CO2_SENSOR = 0x18
};  // CO2 sensors

/* Fan commands */
enum {
  FAN_FRAME_SETVOLTAGE = 0x01,  // Set speed (voltage / percentage)
  FAN_FRAME_SETSPEED = 0x02,    // Set speed (preset)
  FAN_FRAME_SETTIMER = 0x03,    // Set speed with timer
  FAN_NETWORK_JOIN_REQUEST = 0x04,
  FAN_FRAME_SETSPEED_REPLY = 0x05,
  FAN_NETWORK_JOIN_OPEN = 0x06,
  FAN_TYPE_FAN_SETTINGS = 0x07,  // Current settings, sent by fan in reply to 0x01, 0x02, 0x10
  FAN_FRAME_0B = 0x0B,
  FAN_NETWORK_JOIN_ACK = 0x0C,
  //	FAN_NETWORK_JOIN_FINISH		= 0x0D,
  FAN_TYPE_QUERY_NETWORK = 0x0D,
  FAN_TYPE_QUERY_DEVICE = 0x10,
  FAN_FRAME_SETVOLTAGE_REPLY = 0x1D
};

/* Fan speed presets */
enum {
  FAN_SPEED_AUTO = 0x00,    // Off:      0% or  0.0 volt
  FAN_SPEED_LOW = 0x01,     // Low:     30% or  3.0 volt
  FAN_SPEED_MEDIUM = 0x02,  // Medium:  50% or  5.0 volt
  FAN_SPEED_HIGH = 0x03,    // High:    90% or  9.0 volt
  FAN_SPEED_MAX = 0x04
};  // Max:    100% or 10.0 volt

/* Internal result codes */
enum {
  FAN_RESULT_SUCCESS = 0x00,  // Success
  FAN_ERROR_NOT_FOUND,        // Remote device not found
  FAN_ERROR_NOT_COMPLETED,    // Join operation wasn't completed
  FAN_ERROR_TX_FAILED,        // Transmission failed
  FAN_ERROR_NO_REPLY,         // Remote device did not reply
  FAN_ERROR_NO_ACKNOWLEDGE,   // Remote device did not acknowledge
  FAN_ERROR_CONFIG_FAILED = 0xFF
};  // Failed to store nRF905 configuration

/* nRF905 configuration profiles */
enum {
  FAN_PROFILE_NOT_CONFIGURED = 0x00,  // nRF905 is not configured
  FAN_PROFILE_ZEHNDER,                // nRF905 configured for Zehnder fans
  FAN_PROFILE_BUVA,                   // nRF905 configured for BUVA fans
  FAN_PROFILE_DEFAULT,                // nRF905 configured with factory default settings
  FAN_PROFILE_CUSTOM
};  // nRF905 configured with custom settings

#define NETWORK_LINK_ID 0xA55A5AA5
const uint32_t network_default_id = 0xE7E7E7E7;
const uint32_t FAN_JOIN_DEFAULT_TIMEOUT = 10000;

typedef enum {
  DiscoveryEventLoop,        //
  DiscoveryEventRxComplete,  //
  DiscoveryEventNrOf         //
} DiscoveryEvent;

class ZehnderRF : public fan::FanState {
 public:
  ZehnderRF();

  void setup() override;

  void dump_config() override;
  void loop() override;

  int get_speed_count() { return 3; }

  float get_setup_priority() const override { return setup_priority::DATA; }

  void set_rf(nrf905::nRF905 *const pRf) { _rf = pRf; }

 protected:
  void discoveryStart(const uint8_t device_id, const uint32_t timeout);
  void discoveryEvent(const DiscoveryEvent eventId, const uint8_t *const pData = NULL, const size_t dataLength = 0);

  void startTransmit(const uint8_t *const pData, const std::function<void(void)> callback, const size_t retries);
  uint8_t createDeviceID(void);

  bool next_update_{true};
  nrf905::nRF905 *_rf;
};

}  // namespace zehnder
}  // namespace esphome

#endif /* __COMPONENT_ZEHNDER_H__ */
