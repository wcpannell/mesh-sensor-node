#include "model_handler.h"
#include <bluetooth/bluetooth.h>
#include <bluetooth/mesh/dk_prov.h> // investigate
#include <bluetooth/mesh/models.h>
#include <device.h>
#include <dk_buttons_and_leds.h>
#include <drivers/hwinfo.h>
#include <drivers/i2c.h>
#include <errno.h>
#include <settings/settings.h>
#include <sys/printk.h> //Kernel printing
#include <zephyr.h>

// static int no_outputnum(bt_mesh_output_action_t action, uint32_t number) {
//   return -ENOTSUP;
// }
// static int no_outputstr(const char *string) {
//   return -ENOTSUP;
// }
//
// static uint8_t dev_uuid[16];
//
// static const struct bt_mesh_prov prov = {
//   .uuid = dev_uuid,
//   .output_actions = BT_MESH_BLINK,
//   .output_number = output_number

// const struct bt_mesh_prov *provision_init(void) {
//  hwinfo_get_device_id(dev_uuid, sizeof(dev_uuid));
//
// Liberated from NCS samples/bluetooth/mesh/sensor_server
static void bt_ready(int err) {
  if (err) {
    printk("Bluetooth init failed (err %d)\n", err);
    return;
  }

  printk("Bluetooth initialized\n");

  // intializes dev board leds & buttons
  dk_leds_init();
  dk_buttons_init(NULL);

  printk("Initializing...");

  err = bt_mesh_init(bt_mesh_dk_prov_init(), model_handler_init());
  if (err) {
    printk("Initializing mesh failed (err %d)\n", err);
    return;
  }

  if (IS_ENABLED(CONFIG_SETTINGS)) {
    settings_load();
  }

  /* This will be a no-op if settings_load() loaded provisioning info */
  bt_mesh_prov_enable(BT_MESH_PROV_ADV | BT_MESH_PROV_GATT);

  printk("Mesh initialized\n");
}

void main(void) {
  int err = bt_enable(bt_ready);
  if (err) {
    printk("BT init failed at bt_enable. (err %d)\n", err);
  }
}
