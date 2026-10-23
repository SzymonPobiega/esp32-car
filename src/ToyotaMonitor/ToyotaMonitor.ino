#pragma GCC push_options
#pragma GCC optimize("O3")

#include <Arduino.h>
#include "lvgl.h"
#include "driver/i2c_master.h"
#include "pins_config.h"
#include "src/lcd/st7701_lcd.h"
#include "src/touch/gt911_touch.h"
#include "gauge_ui.hpp"
#include "tpms_data.hpp"
#include "modem_link.hpp"
#include "attitude_ui.hpp"
#include "bms_ui.hpp"
#include "link_indicator.hpp"
#include "src/pitch.c"
#include "src/roll.c"

// Keep these somewhere global if you need to update them later
static AttitudeUI pitch_ui;
static AttitudeUI roll_ui;

extern const lv_img_dsc_t pitch_img;
extern const lv_img_dsc_t roll_img;

bsp_lcd_handles_t lcd_panels;

st7701_lcd lcd = st7701_lcd(LCD_RST);
gt911_touch touch = gt911_touch(TP_I2C_SDA, TP_I2C_SCL, TP_RST, TP_INT);

static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf;
static lv_color_t *buf1;


static bool lvgl_port_flush_dpi_panel_ready_callback(esp_lcd_panel_handle_t panel_io, esp_lcd_dpi_panel_event_data_t *edata, void *user_ctx)
{
    lv_disp_drv_t *disp_drv = (lv_disp_drv_t *)user_ctx;
    assert(disp_drv != NULL);
    // lv_disp_flush_ready(disp_drv);
    lv_disp_flush_ready(disp_drv);

    // if (disp_ctx->trans_size && disp_ctx->trans_sem) {
    //     xSemaphoreGiveFromISR(disp_ctx->trans_sem, &taskAwake);
    // }

    return false;
}


void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  const int offsetx1 = area->x1;
  const int offsetx2 = area->x2;
  const int offsety1 = area->y1;
  const int offsety2 = area->y2;
  lcd.lcd_draw_bitmap(offsetx1, offsety1, offsetx2 + 1, offsety2 + 1, &color_p->full);
  // lv_disp_flush_ready(disp); // 告诉lvgl刷新完成
}

void my_touchpad_read(lv_indev_drv_t *indev_driver, lv_indev_data_t *data)
{
  bool touched;
  uint16_t touchX, touchY;

  touched = touch.getTouch(&touchX, &touchY);
  // touchX = 800 - touchX;

  if (!touched)
  {
    data->state = LV_INDEV_STATE_REL;
  }
  else
  {
    data->state = LV_INDEV_STATE_PR;

    // 设置坐标
    data->point.x = touchX;
    data->point.y = touchY;
    Serial.printf("x=%d,y=%d \r\n",touchX,touchY);
  }
}

static void lvgl_port_update_callback(lv_disp_drv_t *drv)
{
    switch (drv->rotated) {
    case LV_DISP_ROT_NONE:
        touch.set_rotation(0);
        break;
    case LV_DISP_ROT_90:
        touch.set_rotation(1);
        break;
    case LV_DISP_ROT_180:
        touch.set_rotation(2);
        break;
    case LV_DISP_ROT_270:
        touch.set_rotation(3);
        break;
    }
}

void setup()
{
  Serial.begin(115200);
  Serial.println("ESP32P4 MIPI DSI LVGL");

  i2c_master_bus_handle_t i2c_handle = NULL;

  i2c_master_bus_config_t i2c_bus_conf = {
      .i2c_port = I2C_NUM_1,
      .sda_io_num = (gpio_num_t)TP_I2C_SDA,
      .scl_io_num = (gpio_num_t)TP_I2C_SCL,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .intr_priority = 0,
      .trans_queue_depth = 0,
      .flags = {
          .enable_internal_pullup = 1,
      },
  };
  i2c_new_master_bus(&i2c_bus_conf, &i2c_handle);
  
  lcd.begin();
  touch.begin();

  lcd.get_handle(&lcd_panels);
  

  lv_init();
  size_t buffer_size = sizeof(int16_t) * LCD_H_RES * LCD_V_RES;
  // buf = (int32_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
  // buf1 = (int32_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
  buf = (lv_color_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
  buf1 = (lv_color_t *)heap_caps_malloc(buffer_size, MALLOC_CAP_SPIRAM);
  assert(buf);
  assert(buf1);

  lv_disp_draw_buf_init(&draw_buf, buf, buf1, LCD_H_RES * LCD_V_RES);

  static lv_disp_drv_t disp_drv;
  /*Initialize the display*/
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = LCD_H_RES;
  disp_drv.ver_res = LCD_V_RES;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  disp_drv.full_refresh = false;
  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touchpad_read;
  lv_indev_drv_register(&indev_drv);

  esp_lcd_dpi_panel_event_callbacks_t cbs = {0};
  cbs.on_color_trans_done = lvgl_port_flush_dpi_panel_ready_callback;
   /* Register done callback */
  esp_lcd_dpi_panel_register_event_callbacks(lcd_panels.panel, &cbs, &disp_drv);

  // lv_disp_set_rotation(NULL, 0);
  Serial.println("start demo");

  // Get the (now valid) default display & its active screen
  lv_disp_t *disp = lv_disp_get_default();
  lv_obj_t  *scr  = lv_disp_get_scr_act(disp);

  int disp_w = lv_disp_get_hor_res(disp);
  int disp_h = lv_disp_get_ver_res(disp);

  // Screen background
  lv_obj_set_style_bg_color(scr, lv_color_make(155, 155, 155), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  /* BMS bar at the bottom, outside the tabview so it shows on every tab */
  const int bms_bar_h = 56;
  lv_obj_t *status_bar = create_bms_bar(scr, disp_w, bms_bar_h);

  // TPMS connector pings every 5 s, so 15 s without one means the link is down
  create_link_indicator(status_bar, "TPMS", &tpms::last_update, 15000);

  /* Create the tabview as top-level */
  const int tab_bar_h = 60;
  lv_obj_t * tv = lv_tabview_create(lv_scr_act(), LV_DIR_TOP, tab_bar_h);
  lv_obj_set_size(tv, disp_w, disp_h - bms_bar_h);

  /* Create tabs */
  lv_obj_t * tab1 = lv_tabview_add_tab(tv, "Gauges");
  lv_obj_t * tab2 = lv_tabview_add_tab(tv, "Tab2");
  lv_obj_t * tab3 = lv_tabview_add_tab(tv, "Tab3");

  lv_obj_t * content = lv_tabview_get_content(tv);
  lv_obj_set_style_pad_all(content, 0, 0);
  lv_obj_set_style_border_width(content, 0, 0);

  /* Remove padding/border from the tab page itself */
  lv_obj_set_style_pad_all(tab1, 0, 0);
  lv_obj_set_style_border_width(tab1, 0, 0);
  lv_obj_set_style_radius(tab1, 0, 0);

  lv_obj_t* cont = lv_obj_create(tab1);

  lv_obj_clear_flag(tab1, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_set_size(cont, lv_pct(100), lv_pct(100));
  lv_obj_set_style_pad_all(cont, 0, 0);
  lv_obj_set_style_border_width(cont, 0, 0);
  lv_obj_set_style_bg_opa(cont, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

  static lv_coord_t col_dsc[] = {
      LV_GRID_FR(1), LV_GRID_FR(1),
      LV_GRID_TEMPLATE_LAST
  };
  // Two equal rows filling the space between the tab bar and the BMS bar,
  // so the gauges are centered vertically
  static lv_coord_t row_dsc[] = {
      LV_GRID_FR(1),   // Row 0: top gauges
      LV_GRID_FR(1),   // Row 1: bottom gauges
      LV_GRID_TEMPLATE_LAST
  };
  lv_obj_set_grid_dsc_array(cont, col_dsc, row_dsc);

  int cell_w = disp_w / 2;
  int cell_h = disp_h / 3;
  int s = (cell_w < cell_h ? cell_w : cell_h) * 9 / 10;

  // Create 4 gauges and place them in the grid
  gauges.clear();
  gauges.reserve(4);
  for (int r = 0; r < 2; ++r) {
      for (int c = 0; c < 2; ++c) {

          // One container per cell
          lv_obj_t *cell = lv_obj_create(cont);
          lv_obj_set_style_bg_opa(cell, LV_OPA_TRANSP, 0);  // transparent
          lv_obj_remove_style_all(cell);
          lv_obj_set_size(cell, LV_SIZE_CONTENT, LV_SIZE_CONTENT);

          lv_obj_set_grid_cell(
              cell,
              LV_GRID_ALIGN_CENTER, c, 1,
              LV_GRID_ALIGN_CENTER, r, 1);

          GaugeUI g = create_pressure_gauge(cell, s);
          
          gauges.push_back(g);
      }
  }

  /* Remove padding/border from the tab page itself */
  lv_obj_set_style_pad_all(tab2, 0, 0);
  lv_obj_set_style_border_width(tab2, 0, 0);
  lv_obj_set_style_radius(tab2, 0, 0);

  lv_obj_t* cont2 = lv_obj_create(tab2);

  lv_obj_clear_flag(tab2, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(cont2, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_set_size(cont2, lv_pct(100), lv_pct(100));
  lv_obj_set_style_pad_all(cont2, 0, 0);
  lv_obj_set_style_border_width(cont2, 0, 0);
  lv_obj_set_style_bg_opa(cont2, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(cont2, LV_OBJ_FLAG_SCROLLABLE);

  setup_tab2(cont2, disp_w, disp_h);

  init_gauge_timer();
  modem::begin();
}

void setup_tab2(lv_obj_t* cont, int disp_w, int disp_h)
{
  static lv_coord_t col_dsc[] = {
    LV_GRID_FR(1), LV_GRID_FR(1),
    LV_GRID_TEMPLATE_LAST
  };
  // Two equal rows, so the indicators are centered vertically
  static lv_coord_t row_dsc[] = {
      LV_GRID_FR(1),   // Row 0: top (Pitch)
      LV_GRID_FR(1),   // Row 1: bottom (Roll)
      LV_GRID_TEMPLATE_LAST
  };
  lv_obj_set_grid_dsc_array(cont, col_dsc, row_dsc);

  // Match your size logic
  int cell_w = disp_w / 2;
  int cell_h = disp_h / 3;
  int s = (cell_w < cell_h ? cell_w : cell_h) * 9 / 10;

  // ---- Pitch cell (row 0, span 2 columns) ----
  lv_obj_t *cell_pitch = lv_obj_create(cont);
  lv_obj_remove_style_all(cell_pitch);
  lv_obj_set_style_bg_opa(cell_pitch, LV_OPA_TRANSP, 0);
  lv_obj_set_size(cell_pitch, LV_SIZE_CONTENT, LV_SIZE_CONTENT);

  lv_obj_set_grid_cell(
      cell_pitch,
      LV_GRID_ALIGN_CENTER, 0, 2,
      LV_GRID_ALIGN_CENTER, 0, 1
  );

  int s_att = (int)(s * 1.3f);

  // Keep the indicators inside their grid row (half the height, minus the row gap)
  lv_obj_update_layout(cont);
  int row_h = (lv_obj_get_content_height(cont) - lv_obj_get_style_pad_row(cont, 0)) / 2;
  if (s_att > row_h) s_att = row_h;

  pitch_ui = create_attitude_indicator(cell_pitch, s_att);
  attitude_set_center_image(&pitch_ui, &pitch_img);

  // ---- Roll cell (row 1, span 2 columns) ----
  lv_obj_t *cell_roll = lv_obj_create(cont);
  lv_obj_remove_style_all(cell_roll);
  lv_obj_set_style_bg_opa(cell_roll, LV_OPA_TRANSP, 0);
  lv_obj_set_size(cell_roll, LV_SIZE_CONTENT, LV_SIZE_CONTENT);

  lv_obj_set_grid_cell(
      cell_roll,
      LV_GRID_ALIGN_CENTER, 0, 2,
      LV_GRID_ALIGN_CENTER, 1, 1
  );

  roll_ui = create_attitude_indicator(cell_roll, s_att);
  attitude_set_center_image(&roll_ui, &roll_img);

  //lv_obj_add_flag(pitch_ui.car, LV_OBJ_FLAG_HIDDEN);
  //lv_obj_add_flag(roll_ui.car,  LV_OBJ_FLAG_HIDDEN);

  attitude_set_value(&pitch_ui,  20.0f);   // +20°
  attitude_set_value(&roll_ui,  -15.0f);   // -15°
}

void handleModemMessage(const modem::Message& m)
{
  int16_t type;
  if (m.len < sizeof(type)) return;
  memcpy(&type, m.data, sizeof(type));

  switch (type) {
  case Contracts::TYPE_TPMS: {
    Contracts::TpmsPacket in;
    if (m.len != sizeof(in)) return;
    memcpy(&in, m.data, sizeof(in));

    tpms::TpmsPacket p = { in.sequence, in.sensorId, in.pressure, in.temp };
    tpms::handleTpms(p);
    if (p.sensorId == tpms::kNoSensor) {
      Serial.printf("TPMS connector ping (no sensor received yet) rssi=%d\n", m.rssi);
      break;
    }
    Serial.print("seq="); Serial.print(p.sequence);
    Serial.print(" id="); Serial.print(p.sensorId);
    Serial.print(" pressure="); Serial.print(p.pressure);
    Serial.print(" temp="); Serial.print(p.temp);
    Serial.print(" rssi="); Serial.println(m.rssi);
    break;
  }

  case Contracts::TYPE_GYRO_ANGLE: {
    Contracts::GyroAnglePacket g;
    if (m.len != sizeof(g)) return;
    memcpy(&g, m.data, sizeof(g));

    attitude_set_value(&pitch_ui, g.pitch_deg / 100.0f);
    attitude_set_value(&roll_ui, g.roll_deg / 100.0f);
    break;
  }

  case Contracts::TYPE_BMS: {
    Contracts::BmsPacket b;
    if (m.len != sizeof(b)) return;
    memcpy(&b, m.data, sizeof(b));

    bms_bar_update(b);
    Serial.printf("BMS: %.2f V %.2f A soc=%d%% remaining=%.2f Ah temp=%.1f C\n",
                  b.volts / 100.0f, b.amps / 100.0f, b.soc_percent,
                  b.remaining / 100.0f, b.temp / 10.0f);
    break;
  }

  default:
    Serial.printf("ESP-NOW: unknown type %d (%u bytes) from %02X:%02X:%02X:%02X:%02X:%02X\n",
                  type, m.len, m.srcMac[0], m.srcMac[1], m.srcMac[2],
                  m.srcMac[3], m.srcMac[4], m.srcMac[5]);
    break;
  }
}

void loop()
{
  static uint32_t last = millis();
  uint32_t now = millis();
  lv_tick_inc(now - last);
  last = now;

  lv_timer_handler();

  modem::Message m;
  while (modem::poll(m)) {
    handleModemMessage(m);
  }
  delay(5);
}