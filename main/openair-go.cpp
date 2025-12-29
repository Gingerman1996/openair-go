#include <cstdio>
#include <inttypes.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include "driver/spi_master.h"
#include "driver/i2c_master.h"
#include "esp_attr.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_sleep.h"
#include "esp_system.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_types.h"
#include "i2cdev.h"
#include "sht4x.h"

#include "config.h"
#include "esp_lcd_panel_ssd1681.h"

static const char *kTag = "openair";
static constexpr i2c_port_t kI2CPort = I2C_NUM_0;
static constexpr int kI2CClkHz = 400000;
static constexpr int kEpaperSpiHz = 1'000'000;

static sht4x_t s_sht4x;
static i2c_master_bus_handle_t s_i2c_bus = nullptr;
static esp_lcd_panel_io_handle_t s_epaper_io = nullptr;
static esp_lcd_panel_handle_t s_epaper_panel = nullptr;
RTC_DATA_ATTR static uint32_t s_wakeup_count = 0;

static const char *wakeup_cause_to_str(esp_sleep_wakeup_cause_t cause);
static const char *reset_reason_to_str(esp_reset_reason_t reason);
static esp_err_t init_gpio(void);
static esp_err_t init_i2c_bus(void);
static esp_err_t init_sht4x(void);
static esp_err_t init_spi_epaper(void);
static void log_reset_reason(void);
static void log_wakeup_reason(esp_sleep_wakeup_cause_t cause);
static void log_boot_info(esp_sleep_wakeup_cause_t cause, bool woke_from_deepsleep);

// Application entrypoint ----------------------------------------------------
extern "C" void app_main(void) {
  const esp_sleep_wakeup_cause_t wakeup_reason = esp_sleep_get_wakeup_cause();
  const bool woke_from_deepsleep = (wakeup_reason != ESP_SLEEP_WAKEUP_UNDEFINED);

  if (woke_from_deepsleep) {
    ++s_wakeup_count;
  }

  if (init_gpio() != ESP_OK) {
    ESP_LOGW(kTag, "GPIO init failed; continuing");
  }
  if (init_i2c_bus() != ESP_OK) {
    ESP_LOGW(kTag, "I2C init failed; continuing");
  } else if (init_sht4x() != ESP_OK) {
    ESP_LOGW(kTag, "SHT4x init failed; continuing");
  }
  if (init_spi_epaper() != ESP_OK) {
    ESP_LOGW(kTag, "SSD1681 init failed; continuing");
  }

  log_boot_info(wakeup_reason, woke_from_deepsleep);
}

// Helper utilities ----------------------------------------------------------
static const char *wakeup_cause_to_str(esp_sleep_wakeup_cause_t cause) {
  switch (cause) {
  case ESP_SLEEP_WAKEUP_UNDEFINED:
    return "UNDEFINED";
  case ESP_SLEEP_WAKEUP_EXT0:
    return "EXT0";
  case ESP_SLEEP_WAKEUP_EXT1:
    return "EXT1";
  case ESP_SLEEP_WAKEUP_TIMER:
    return "TIMER";
  case ESP_SLEEP_WAKEUP_TOUCHPAD:
    return "TOUCHPAD";
  case ESP_SLEEP_WAKEUP_ULP:
    return "ULP";
  case ESP_SLEEP_WAKEUP_GPIO:
    return "GPIO";
  case ESP_SLEEP_WAKEUP_UART:
    return "UART";
  case ESP_SLEEP_WAKEUP_WIFI:
    return "WIFI";
  case ESP_SLEEP_WAKEUP_COCPU:
    return "COCPU";
  default:
    return "OTHER";
  }
}

static const char *reset_reason_to_str(esp_reset_reason_t reason) {
  switch (reason) {
  case ESP_RST_UNKNOWN:
    return "UNKNOWN";
  case ESP_RST_POWERON:
    return "POWERON";
  case ESP_RST_EXT:
    return "EXT";
  case ESP_RST_SW:
    return "SW";
  case ESP_RST_PANIC:
    return "PANIC";
  case ESP_RST_INT_WDT:
    return "INT_WDT";
  case ESP_RST_TASK_WDT:
    return "TASK_WDT";
  case ESP_RST_WDT:
    return "WDT";
  case ESP_RST_DEEPSLEEP:
    return "DEEPSLEEP";
  case ESP_RST_BROWNOUT:
    return "BROWNOUT";
  case ESP_RST_SDIO:
    return "SDIO";
  default:
    return "OTHER";
  }
}

static void log_reset_reason(void) {
  const esp_reset_reason_t reason = esp_reset_reason();
  ESP_LOGI(kTag, "Reset reason: %s (%d)", reset_reason_to_str(reason), static_cast<int>(reason));
}

static void log_wakeup_reason(esp_sleep_wakeup_cause_t cause) {
  ESP_LOGI(kTag, "Wakeup cause: %s (%d)", wakeup_cause_to_str(cause), static_cast<int>(cause));
}

static esp_err_t init_gpio(void) {
  // ePaper control pins to safe defaults
  const uint64_t output_mask =
      (1ULL << EPAPER_DC_GPIO) | (1ULL << EPAPER_RES_GPIO) | (1ULL << EPAPER_CS_GPIO);

  gpio_config_t out_cfg = {};
  out_cfg.mode = GPIO_MODE_OUTPUT;
  out_cfg.intr_type = GPIO_INTR_DISABLE;
  out_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  out_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
  out_cfg.pin_bit_mask = output_mask;

  ESP_RETURN_ON_ERROR(gpio_config(&out_cfg), kTag, "gpio config (epaper outs)");

  // Busy pin as input
  gpio_config_t in_cfg = {};
  in_cfg.mode = GPIO_MODE_INPUT;
  in_cfg.intr_type = GPIO_INTR_DISABLE;
  in_cfg.pull_up_en = GPIO_PULLUP_DISABLE;
  in_cfg.pull_down_en = GPIO_PULLDOWN_ENABLE;
  in_cfg.pin_bit_mask = (1ULL << EPAPER_BUSY_GPIO);
  ESP_RETURN_ON_ERROR(gpio_config(&in_cfg), kTag, "gpio config (epaper busy)");

  // Default levels
  gpio_set_level(EPAPER_CS_GPIO, 1);  // deselect
  gpio_set_level(EPAPER_RES_GPIO, 1); // idle high for active-low reset
  gpio_set_level(EPAPER_DC_GPIO, 0);

  return ESP_OK;
}

static esp_err_t init_i2c_bus(void) {
  i2c_master_bus_config_t bus_cfg = {};
  bus_cfg.i2c_port = kI2CPort;
  bus_cfg.sda_io_num = I2C_SDA_GPIO;
  bus_cfg.scl_io_num = I2C_SCL_GPIO;
  bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
  bus_cfg.glitch_ignore_cnt = 7;
  bus_cfg.intr_priority = 0;
  bus_cfg.trans_queue_depth = 4;
  bus_cfg.flags.enable_internal_pullup = true;
  bus_cfg.flags.allow_pd = false;

  esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(kTag, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
    return err;
  }

  err = i2cdev_init();
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "i2cdev_init failed: %s", esp_err_to_name(err));
    return err;
  }

  ESP_LOGI(kTag, "I2C ready on port %d (SDA %d, SCL %d @ %d Hz, pullups on)",
           static_cast<int>(kI2CPort), static_cast<int>(I2C_SDA_GPIO),
           static_cast<int>(I2C_SCL_GPIO), kI2CClkHz);
  return ESP_OK;
}

static esp_err_t init_sht4x(void) {
  esp_err_t err = sht4x_init_desc(&s_sht4x, kI2CPort, I2C_SDA_GPIO, I2C_SCL_GPIO);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "sht4x_init_desc failed: %s", esp_err_to_name(err));
    return err;
  }

  s_sht4x.i2c_dev.cfg.master.clk_speed = kI2CClkHz;

  err = sht4x_init(&s_sht4x);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "sht4x_init failed: %s", esp_err_to_name(err));
    return err;
  }

  ESP_LOGI(kTag, "SHT4x initialized on I2C%d (addr 0x%02x)", static_cast<int>(kI2CPort),
           SHT4X_I2C_ADDRESS);
  return ESP_OK;
}

static esp_err_t init_spi_epaper(void) {
  spi_bus_config_t bus_cfg = {};
  bus_cfg.mosi_io_num = EPAPER_MOSI_GPIO;
  bus_cfg.miso_io_num = EPAPER_MISO_GPIO;
  bus_cfg.sclk_io_num = EPAPER_SCLK_GPIO;
  bus_cfg.quadwp_io_num = -1;
  bus_cfg.quadhd_io_num = -1;
  bus_cfg.max_transfer_sz = 4096;

  esp_err_t err = spi_bus_initialize(SPI2_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(kTag, "spi_bus_initialize failed: %s", esp_err_to_name(err));
    return err;
  }

  esp_lcd_panel_io_spi_config_t io_cfg = {};
  io_cfg.cs_gpio_num = EPAPER_CS_GPIO;
  io_cfg.dc_gpio_num = EPAPER_DC_GPIO;
  io_cfg.spi_mode = 0;
  io_cfg.pclk_hz = kEpaperSpiHz;
  io_cfg.trans_queue_depth = 4;
  io_cfg.lcd_cmd_bits = 8;
  io_cfg.lcd_param_bits = 8;
  io_cfg.on_color_trans_done = nullptr;

  err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_cfg, &s_epaper_io);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "esp_lcd_new_panel_io_spi failed: %s", esp_err_to_name(err));
    return err;
  }

  esp_lcd_ssd1681_config_t vendor_cfg = {};
  vendor_cfg.busy_gpio_num = EPAPER_BUSY_GPIO;
  vendor_cfg.non_copy_mode = false;

  esp_lcd_panel_dev_config_t panel_cfg = {};
  panel_cfg.reset_gpio_num = EPAPER_RES_GPIO;
  panel_cfg.color_space = ESP_LCD_COLOR_SPACE_MONOCHROME;
  panel_cfg.bits_per_pixel = 1;
  panel_cfg.vendor_config = &vendor_cfg;
  panel_cfg.flags.reset_active_high = false;

  // ISR service may already exist; ignore double-install
  err = gpio_install_isr_service(0);
  if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
    ESP_LOGE(kTag, "gpio_install_isr_service failed: %s", esp_err_to_name(err));
    return err;
  }

  err = esp_lcd_new_panel_ssd1681(s_epaper_io, &panel_cfg, &s_epaper_panel);
  if (err != ESP_OK) {
    ESP_LOGE(kTag, "esp_lcd_new_panel_ssd1681 failed: %s", esp_err_to_name(err));
    return err;
  }

  ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(s_epaper_panel), kTag, "panel reset");
  vTaskDelay(pdMS_TO_TICKS(20));
  ESP_RETURN_ON_ERROR(esp_lcd_panel_init(s_epaper_panel), kTag, "panel init");
  ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(s_epaper_panel, true), kTag, "panel on");

  ESP_LOGI(kTag, "SSD1681 panel ready (SPI2 @ %d Hz)", kEpaperSpiHz);
  return ESP_OK;
}

static void log_boot_info(esp_sleep_wakeup_cause_t cause, bool woke_from_deepsleep) {
  vTaskDelay(pdMS_TO_TICKS(1000));

  ESP_LOGI(kTag, "Wakeup count: %" PRIu32, s_wakeup_count);
  log_reset_reason();
  log_wakeup_reason(cause);

  if (!woke_from_deepsleep) {
    ESP_LOGI(kTag, "Booted from fresh start (not deep sleep)");
  }
}
