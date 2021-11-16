#include "model_handler.h"
#include "aht20.h"
#include "bluetooth/mesh/access.h"
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh/models.h>
#include <dk_buttons_and_leds.h>

// local "globals"
static const struct device *i2c0_dev; // stores sensor bus
static struct aht_data data;          // stores sensor values
static struct sensor_value temp_val = {0};
static struct sensor_value humid_val = {0};

// Sensor Server
// =============

// getter to read latest Temp values into mesh model
// TODO: Convert AHT20 to sensor?
static int temp_get(struct bt_mesh_sensor *sensor, struct bt_mesh_msg_ctx *ctx,
                    struct sensor_value *rsp) {
  // load sensor data into model value
  rsp->val1 = temp_val.val1;
  rsp->val2 = temp_val.val2;
  return 0;
}

static struct bt_mesh_sensor_descriptor temp_desc = {
    .tolerance = // Sensor Tolerance
    {
        .positive = {0, 500000}, // + 0.5°C
        .negative = {0, 500000}, // - 0.5°C
    },
    .sampling_type = BT_MESH_SENSOR_SAMPLING_INSTANTANEOUS, // One value
    .period = 0,          // One value, no period.
    .update_interval = 0, // One value, no update interval
};

// Temperature Sensor Model
static struct bt_mesh_sensor amb_temp = {
    .type = &bt_mesh_sensor_precise_present_amb_temp, // Property ID
    .get = temp_get,                                  // Value getter
    .descriptor = &temp_desc,                         // Sensor Descriptor
};

// Callback to read latest Humidity values into mesh model
static int humid_get(struct bt_mesh_sensor *sensor, struct bt_mesh_msg_ctx *ctx,
                     struct sensor_value *rsp) {
  // load sensor data into model value
  rsp->val1 = humid_val.val1;
  rsp->val2 = humid_val.val2;
  return 0;
}

static struct bt_mesh_sensor_descriptor humid_desc = {
    .tolerance = // Sensor Tolerance
    {
        .positive = {3, 0}, // + 3%
        .negative = {3, 0}, // - 3%
    },
    .sampling_type = BT_MESH_SENSOR_SAMPLING_INSTANTANEOUS, // One value
    .period = 0,          // One value, no period.
    .update_interval = 0, // One value, no update interval
};

// Humidity Sensor Model
static struct bt_mesh_sensor amb_humidity = {
    .type = &bt_mesh_sensor_present_amb_rel_humidity, // Property ID
    .get = humid_get,                                 // Value getter
    .descriptor = &humid_desc,                        // Sensor Descriptor
};

// Sensor Server Model
static struct bt_mesh_sensor *const sensors[] = {
    &amb_temp,
    &amb_humidity,
};

static struct bt_mesh_sensor_srv sensor_srv =
    BT_MESH_SENSOR_SRV_INIT(sensors, ARRAY_SIZE(sensors));

// Health Server
// =============

// Zephyr Job handle for blinking lights when need to draw attention
static struct k_work_delayable attention_blink_work;

// Attention state variable
static bool attention;

// Zephyr job to draw attention
// TODO: Should we reschedule attention_blink_work or work that's passed in?
static void attention_blink(struct k_work *work) {
  if (attention) {
    dk_set_leds(BIT(1) | BIT(2));
    k_work_reschedule(&attention_blink_work, K_MSEC(100));
  } else {
    dk_set_leds(DK_NO_LEDS_MSK);
  }
}

// set attention, schedule work
static void attention_on(struct bt_mesh_model *mod) {
  attention = true;
  k_work_reschedule(&attention_blink_work, K_NO_WAIT);
}

// clear attention
static void attention_off(struct bt_mesh_model *mod) {
  attention = false;
  // no reschedule
}

// Health Server Callback datastructure
static const struct bt_mesh_health_srv_cb health_srv_cb = {
    .attn_on = attention_on,
    .attn_off = attention_off,
};

// Health Server
static struct bt_mesh_health_srv health_srv = {
    .cb = &health_srv_cb,
};

BT_MESH_HEALTH_PUB_DEFINE(health_pub, 0);

static struct bt_mesh_elem elements[] = {
    BT_MESH_ELEM(
        1,
        BT_MESH_MODEL_LIST(BT_MESH_MODEL_CFG_SRV,
                           BT_MESH_MODEL_HEALTH_SRV(&health_srv, &health_pub),
                           BT_MESH_MODEL_SENSOR_SRV(&sensor_srv)),
        BT_MESH_MODEL_NONE),
};

static const struct bt_mesh_comp comp = {
    .cid = CONFIG_BT_COMPANY_ID,
    .elem = elements,
    .elem_count = ARRAY_SIZE(elements),
};

static struct k_work_delayable ambient_sensor_work;

static struct bt_mesh_msg_ctx pub_ctx = {.net_idx = 0,
                                         .app_idx = 0,
                                         .addr = 0xFFFF, // broadcast publish
                                         .send_ttl = BT_MESH_TTL_DEFAULT};

static void ambient_sensor_get(struct k_work *work) {
  // get fresh value from sensor
  enum aht_error aht_err = aht_blocking_measure(i2c0_dev, &data);
  if (aht_err != AHT_NO_ERROR) {
    printk("Error while performing AHT20 measure! (err %d)", aht_err);
  } else {
    printk("New sensor data! Temp: %d, Humid: %d, cal: %d, busy: %d\n",
           data.temperature, data.humidity, data.status.calibrated,
           data.status.busy);
    temp_val.val1 = data.temperature / 100;
    temp_val.val2 = (data.temperature % 100) * 10000;
    humid_val.val1 = data.humidity / 100;
    humid_val.val2 = (data.humidity % 100) * 10000;
  }

  // forced publication. Desired?
  printk("Publishing Temp. retval %d\n",
         bt_mesh_sensor_srv_pub(&sensor_srv, &pub_ctx, &amb_temp, &temp_val));
  // bt_mesh_sensor_srv_sample(&sensor_srv, &amb_temp));
  // printk("slept for %d\n", 10000 - k_msleep(10000));
  printk(
      "Publishing Humdity. reval %d\n",
      bt_mesh_sensor_srv_pub(&sensor_srv, &pub_ctx, &amb_humidity, &humid_val));
  // bt_mesh_sensor_srv_sample(&sensor_srv, &amb_humidity));

  k_work_reschedule(&ambient_sensor_work, K_MSEC(30 * 1000));
}

/// Initialize the BT Mesh node.
/// @returns node composition to pass into bt_mesh_init in main
const struct bt_mesh_comp *model_handler_init(void) {
  printk("handler called\n");
  i2c0_dev = device_get_binding("I2C_0");
  if (!i2c0_dev) {
    printk("Unable to get I2C bus!\n");
  }
  enum aht_error aht_err = aht_init(i2c0_dev);
  if (aht_err != AHT_NO_ERROR) {
    printk("Error Initializing AHT20! (err %d)\n", aht_err);
  }

  k_work_init_delayable(&attention_blink_work, attention_blink);
  k_work_init_delayable(&ambient_sensor_work, ambient_sensor_get);
  k_work_reschedule(&ambient_sensor_work, K_NO_WAIT);
  return &comp;
}
