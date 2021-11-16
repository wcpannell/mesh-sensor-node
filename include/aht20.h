/// This module presents a high level interface to the AHT20 sensor.
///
/// To use the sensor first call the aht_init method to initialize
/// and calibrate the device. After calibration there are two methods to
/// reading the sensor, blocking and asynchronously.
///
/// To do a blocking measurement, issue the aht_blocking_measure command. This
/// command will have the sensor take a measurement, wait for approximately
/// 80ms while the sensor measures and prepares the data, read the data from
/// the sensor, and return the sensor data.
///
/// To do an asynchronous measurement...
/// This functionality is not yet implemented.
///
/// TODO: provide a wrapper that provides callbacks to a statemachine that
/// handles the aht_async_measure, aht_is_busy, and aht_async_read functions.
/// The methodology is to start a measurement by calling the aht_async_measure
/// command. The measurement takes approximately 80 milliseconds. The status of
/// the measurement is checked by calling the aht_is_busy command. Finally,
/// once the measurement is ready, the data can be read by calling the
/// aht_async_read command. The end state of this should be to update a global
/// sensor state variable that is shared between the IRQ callbacks and the main
/// code thread.
#ifndef _AHT20
#define _AHT20

#include <device.h>

enum aht_error { AHT_NO_ERROR = 0, AHT_BAD_CRC, AHT_NO_RESPONSE };

struct aht_status {
  bool busy;
  bool calibrated;
};

struct aht_data {
  struct aht_status status;
  int16_t temperature; // 0.01C
  uint16_t humidity;   // 0.01% Relative Humidity
  uint8_t crc;
};

/// Initialize sensor
enum aht_error aht_init(const struct device *i2c_dev);

/// Check if measurement in progress
bool aht_is_busy(const struct device *i2c_dev);

enum aht_error aht_blocking_measure(const struct device *i2c_dev,
                                    struct aht_data *data);

// enum aht_error aht_async_measure(const struct device *i2c_dev);
//
// /// Read the result of the measurement
// ///
// /// Caller must verify the status field of aht_data. If busy is set then
// /// values are stale
// enum aht_error aht_async_read(const struct device *i2c_dev,
//                              struct aht_data *data);
#endif
