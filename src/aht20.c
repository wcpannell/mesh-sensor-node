#include "aht20.h"
#include <device.h>
#include <drivers/i2c.h>
#include <sys/crc.h>
#include <zephyr.h>

// Module internal-only stuff

#define AHT_ADDR 0x38

// polynomial is x8 + x5 + x4 + 1 = 0b0011_0001
#define AHT_CRC_POLY 0x31

// IV is 0xff
#define AHT_CRC_IV 0xff

/// Reset the AHT20 Sensor
static void reset(const struct device *i2c_dev) {
  uint8_t reset_msg[1] = {0xBA};
  if (i2c_write(i2c_dev, reset_msg, 1, AHT_ADDR)) {
    printk("Error Sending AHT Reset command\n");
  }
}

/// Calibrate the AHT20 Sensor
static void calibrate(const struct device *i2c_dev) {
  uint8_t msg[3] = {0xbe, 0x08, 0};
  if (i2c_write(i2c_dev, msg, 3, AHT_ADDR)) {
    printk("Error Sending AHT Calibrate command\n");
  }
}

/// Return the current AHT Status
static void translate_status(uint8_t status_byte, struct aht_status *status) {
  status->busy = status_byte & (1 << 7);
  status->calibrated = status_byte & (1 << 3);
}

// External facing methods

/// Initialize sensor
enum aht_error aht_init(const struct device *i2c_dev) {
  // Wait 100ms after powering, per AHT20 datasheet
  k_msleep(100);

  reset(i2c_dev);
  calibrate(i2c_dev);
  return AHT_NO_ERROR;
}

/// Check if measurement in progress
bool aht_is_busy(const struct device *i2c_dev) {
  uint8_t data[1];
  int ret;
  struct aht_status status;

  ret = i2c_read(i2c_dev, data, 1, AHT_ADDR);

  if (ret != 0) {
    printk("I2C: AHT20: Error reading status in aht_is_busy. Error (%d)\n",
           ret);
    return true; // try again
  }

  translate_status(data[0], &status);
  return status.busy;
}

enum aht_error aht_blocking_measure(const struct device *i2c_dev,
                                    struct aht_data *data) {
  uint8_t start_msg[3] = {0xAC, 0x33, 0x00};
  uint8_t data_msg[7];
  uint32_t raw_temp, raw_humidity;
  int retval;

  // Start measurement
  retval = i2c_write(i2c_dev, start_msg, 3, AHT_ADDR);
  if (retval != 0) {
    printk("I2C: Error sending AHT20 start command. code (%d)\n", retval);
    return AHT_NO_RESPONSE;
  }

  // AHT20 takes at least 80ms to measure, per datasheet.
  // It's nice to share.
  k_msleep(80);

  // Time's up, data should be ready. Poll device until it's ready.
  // Assume device is busy.
  data->status.busy = true;
  while (data->status.busy) {
    i2c_read(i2c_dev, data_msg, 7, AHT_ADDR);
    translate_status(data_msg[0], &(data->status));
  }

  // got new data, check crc.
  // per AHT20 datasheet
  uint8_t crc = crc8(data_msg, 6, AHT_CRC_POLY, AHT_CRC_IV, false);
  if (crc != data_msg[6]) {
    printk("AHT: CRC Error, got %d, expected %d\n", data_msg[6], crc);
    return AHT_BAD_CRC;
  }
  data->crc = data_msg[6];

  // Temperature Conversion, aht_data units are 0.01degC, -50C -> 150C
  // counts * 200 / (2**20) = 0.000190734863281 counts / degC
  // = 5242.88degC/count = counts * 100 / 524288degC
  // = counts * 100 * 100 / 524288 (0.01degC)
  // = counts * (2000 / 8) * 5 / 65536 (0.01degC) << minimal loss, no div
  //
  // measurement offset 50C => 5000 (0.01degC)
  // TODO: Learn to trust the FPU.
  raw_temp = data_msg[5] + (data_msg[4] << 8) + ((data_msg[3] & 0xF) << 16);
  raw_temp *= 2000;         // at 2**20-1, uses most of 32bits
  raw_temp = raw_temp >> 3; // divide by 8
  raw_temp *= 5;
  raw_temp = raw_temp >> 16;
  raw_temp -= 5000;
  data->temperature = (int16_t)raw_temp;

  // Humidity Conversion, aht_data units are 0.01%rh, 0-100%
  // counts * 100 / (2**20) => 10485.76 percent/count
  // counts * 10000 / 1048576 = counts * 5000 / 524288
  // can use same methodology as above
  raw_humidity = (data_msg[3] + (data_msg[2] << 8) + (data_msg[1] << 16)) >> 4;
  raw_humidity *= 1000;
  raw_humidity = raw_humidity >> 3;
  raw_humidity *= 5;
  raw_humidity = raw_humidity >> 16;
  data->humidity = (uint16_t)raw_humidity;

  return AHT_NO_ERROR;
}

enum aht_error aht_async_measure(const struct device *i2c_dev) {
  printk("aht_async_measure is not implemented!\n");
  return AHT_NO_RESPONSE;
}

/// Read the result of the measurement
///
/// Caller must verify the status field of aht_data. If busy is set then
/// values are stale
enum aht_error aht_async_read(const struct device *i2c_dev,
                              struct aht_data *data) {
  printk("aht_async_read is not implemented!\n");
  return AHT_NO_RESPONSE;
}
