#include "stdio.h"
#include "stdbool.h"
#include "esp_private/usb_phy.h"

static usb_phy_handle_t phy_hdl;

bool usb_init(void) {
  // Configure USB PHY
  usb_phy_config_t phy_conf = {
    .controller = USB_PHY_CTRL_OTG,
    .target = USB_PHY_TARGET_INT,

    // maybe we can use USB_OTG_MODE_DEFAULT and switch using dwc2 driver
#if CFG_TUD_ENABLED
    .otg_mode = USB_OTG_MODE_DEVICE,
#elif CFG_TUH_ENABLED
    .otg_mode = USB_OTG_MODE_HOST,
#endif
    // https://github.com/hathach/tinyusb/issues/2943#issuecomment-2601888322
    // Set speed to undefined (auto-detect) to avoid timinng/racing issue with S3 with host such as macOS
    .otg_speed = USB_PHY_SPEED_UNDEFINED,
  };

  ESP_ERROR_CHECK(usb_new_phy(&phy_conf, &phy_hdl));

  return true;
}

void board_init(void) {
  usb_init();
}
