#include <Arduino.h>
#include "lvgl.h"
#include <esp_display_panel.hpp>
#include <WiFi.h>
#include <esp_mac.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include "gauge_ui.hpp"
#include "tpms_data.hpp"
#include "attitude_ui.hpp"
#include "compas_ui.hpp"
#include "pitch.c"
#include "roll.c"
#include "gyro_data.hpp"
#include "camera_ui.hpp"
#include "espnow_retry.hpp"
#include "energy_ui.hpp"

using namespace esp_panel::board;
using namespace esp_panel::drivers;
using namespace tpms;
using namespace gyro;
using namespace CamperUI;

static portMUX_TYPE g_bmsMux = portMUX_INITIALIZER_UNLOCKED;
static Contracts::BmsPacket g_pendingBms = {};
static bool g_bmsPending = false;

static portMUX_TYPE g_lightsMux = portMUX_INITIALIZER_UNLOCKED;
static Contracts::WorkLightsStatusPacket g_pendingWorkLights = {};
static bool g_workLightsPending = false;
static Contracts::LightsStatusPacket g_pendingLights = {};
static bool g_lightsPending = false;

static void onLightChanged(CamperUI::LightGroup group, uint8_t idx, bool on)
{
  Serial.printf("Light group=%u idx=%u -> %s\n",
                (unsigned)group, (unsigned)idx, on ? "ON" : "OFF");

  esp_err_t res;
  if (group == CamperUI::LightGroup::Interior) {
    Contracts::LightsSwitchPacket pkt;
    pkt.type = Contracts::TYPE_LIGHT_SWITCH;
    pkt.zone = (int16_t)idx + 1;
    pkt.state = (int16_t)on;
    res = espnow_send_cached(Vehicle::SHELLY_BRIDGE_MAC, (uint8_t*)&pkt, sizeof(pkt));
  } else {
    Contracts::WorkLightsSwitchPacket pkt;
    pkt.type = Contracts::TYPE_WORKLIGHT_SWITCH;
    pkt.input = (int16_t)idx + 1;
    pkt.state = (int16_t)on;
    res = espnow_send_cached(Vehicle::LILYGO_WORKLIGHT_MAC, (uint8_t*)&pkt, sizeof(pkt));
  }

  if (res != ESP_OK) {
    Serial.print("ESP-NOW send: ERR ");
    Serial.println(res);
  }
}

#if LV_COLOR_DEPTH != 16
#error "Set LV_COLOR_DEPTH to 16 (RGB565) in lv_conf.h"
#endif

// Keep these somewhere global if you need to update them later
static AttitudeUI pitch_ui;
static AttitudeUI roll_ui;
static compass_strip_t g_compass;

extern const lv_img_dsc_t pitch_img;
extern const lv_img_dsc_t roll_img;

Board *board = nullptr;
Touch *touch = nullptr; 
LCD *lcd = nullptr;

// ---- LVGL glue ----
static lv_disp_draw_buf_t draw_buf;
static lv_color_t *buf1 = nullptr;
static lv_color_t *buf2 = nullptr;
static int lcd_w = 0, lcd_h = 0;

static void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  int32_t x = area->x1;
  int32_t y = area->y1;
  int32_t w = area->x2 - area->x1 + 1;
  int32_t h = area->y2 - area->y1 + 1;
  if (w <= 0 || h <= 0) {
    lv_disp_flush_ready(disp);
    return;
  }

  lcd->drawBitmap(x, y, w, h, reinterpret_cast<const uint8_t *>(color_p));
  lv_disp_flush_ready(disp);
}

static void my_touch_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
  if (!touch) {
    data->state = LV_INDEV_STATE_RELEASED;
    return;
  }

  // Read up to 1 point (you can read more if you want gestures)
  esp_panel::drivers::TouchPoint p;
  int n = touch->readPoints(&p, 1, 0);   // (points, max_points, timeout_ms)

  if (n > 0) {
    data->state = LV_INDEV_STATE_PRESSED;

    // --- coordinate transform for your LVGL SW rotation (ROT_90) ---
    // physical: (p.x, p.y) in LCD coordinates (lcd_w x lcd_h)
    // logical:  rotated 90° clockwise
    int x = p.x;
    int y = p.y;

    data->point.x = x;
    data->point.y = y;

  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

void setup() {
  Serial.begin(115200);
  Serial.println("Volvo monitor");

  board = new Board();
  assert(board->begin());

  lcd = board->getLCD();
  if (!lcd) {
    Serial0.println("LCD is not available");
    while (true) { delay(1000); }
  }

  touch = board->getTouch();

  if (auto bl = board->getBacklight()) {
    bl->on();
    bl->setBrightness(50);
  }

  lcd_w = lcd->getFrameWidth();
  lcd_h = lcd->getFrameHeight();
  Serial0.printf("LCD: %dx%d\n", lcd_w, lcd_h);

  lv_init();

  // Two line-buffers (40 lines each)
  const int lines = 40;
  size_t buf_pixels = lcd_w * lines;
  buf1 = (lv_color_t *)heap_caps_malloc(buf_pixels * sizeof(lv_color_t), MALLOC_CAP_8BIT);
  buf2 = (lv_color_t *)heap_caps_malloc(buf_pixels * sizeof(lv_color_t), MALLOC_CAP_8BIT);
  assert(buf1 && buf2);

  lv_disp_draw_buf_init(&draw_buf, buf1, buf2, buf_pixels);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = lcd_w;
  disp_drv.ver_res = lcd_h;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;

  disp_drv.sw_rotate = 1;
  disp_drv.rotated = LV_DISP_ROT_180;

  lv_disp_drv_register(&disp_drv);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = my_touch_read;
  lv_indev_drv_register(&indev_drv);
    
  // Get the (now valid) default display & its active screen
  lv_disp_t *disp = lv_disp_get_default();
  lv_obj_t *scr = lv_disp_get_scr_act(disp);

  int disp_w = lv_disp_get_hor_res(disp);
  int disp_h = lv_disp_get_ver_res(disp);

  // Screen background
  lv_obj_set_style_bg_color(scr, lv_color_make(155, 155, 155), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

  /* Create the tabview as top-level */
  lv_obj_t *tv = lv_tabview_create(lv_scr_act(), LV_DIR_RIGHT, 60);  // 50 = tab bar height
  lv_obj_set_size(tv, disp_w, disp_h);

  /* Create tabs */
  lv_obj_t *tab1 = lv_tabview_add_tab(tv, "Tires");
  lv_obj_t *tab2 = lv_tabview_add_tab(tv, "Gyro");
  lv_obj_t *tab3 = lv_tabview_add_tab(tv, "Power");
  lv_obj_t *tab4 = lv_tabview_add_tab(tv, "Camera");

  lv_obj_t *content = lv_tabview_get_content(tv);
  lv_obj_set_style_pad_all(content, 0, 0);
  lv_obj_set_style_border_width(content, 0, 0);

  /* Remove padding/border from the tab page itself */
  lv_obj_set_style_pad_all(tab1, 0, 0);
  lv_obj_set_style_border_width(tab1, 0, 0);
  lv_obj_set_style_radius(tab1, 0, 0);

    // ---- Tab1 content container (same as you already do) ----
  lv_obj_t *cont = lv_obj_create(tab1);

  lv_obj_clear_flag(tab1, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(cont, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_set_size(cont, lv_pct(100), lv_pct(100));
    // --- spacing ---
  const int outer = 12;   // outer margin around the whole grid
  const int gap   = 12;   // gap between cells
  const int inset = 10;   // padding inside each cell (room for labels)

  lv_obj_set_style_pad_all(cont, outer, 0);
  lv_obj_set_style_pad_row(cont, gap, 0);
  lv_obj_set_style_pad_column(cont, gap, 0);

  // 3 columns, 2 rows
  static lv_coord_t col_dsc[] = {
    LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_FR(1),
    LV_GRID_TEMPLATE_LAST
  };
  static lv_coord_t row_dsc[] = {
    LV_GRID_FR(1), LV_GRID_FR(1),
    LV_GRID_TEMPLATE_LAST
  };
  lv_obj_set_grid_dsc_array(cont, col_dsc, row_dsc);

  // Compute usable cell size AFTER outer margin + gaps
  int usable_w = disp_w - 2 * outer - 2 * gap;  // 3 cols => 2 internal gaps
  int usable_h = disp_h - 2 * outer - 1 * gap;  // 2 rows => 1 internal gap

  int cell_w = usable_w / 3;
  int cell_h = usable_h / 2;

  // Leave room around the gauge for your temp labels
  int s = (min(cell_w, cell_h) - 2 * inset);
  s = (s * 90) / 100;     // shrink a bit more (85%) to be safe

  // Create 6 gauges: 2 rows x 3 columns
  gauges.clear();
  gauges.reserve(6);

  for (int c = 0; c < 3; ++c) {
    for (int r = 0; r < 2; ++r) {
    // One container per cell
      lv_obj_t *cell = lv_obj_create(cont);
      lv_obj_remove_style_all(cell);

      lv_obj_set_style_bg_opa(cell, LV_OPA_TRANSP, 0);
      lv_obj_set_style_border_width(cell, 0, 0);
      lv_obj_set_style_pad_all(cell, inset, 0);          // <-- key
      lv_obj_clear_flag(cell, LV_OBJ_FLAG_SCROLLABLE);

      lv_obj_set_size(cell, lv_pct(100), lv_pct(100));
      lv_obj_set_grid_cell(cell,
        LV_GRID_ALIGN_STRETCH, c, 1,
        LV_GRID_ALIGN_STRETCH, r, 1);

      GaugeUI g = create_pressure_gauge(cell, s);
      gauges.push_back(g);
    }
  }

  /* Remove padding/border from the tab page itself */
  lv_obj_set_style_pad_all(tab2, 0, 0);
  lv_obj_set_style_border_width(tab2, 0, 0);
  lv_obj_set_style_radius(tab2, 0, 0);

  lv_obj_t *cont2 = lv_obj_create(tab2);

  lv_obj_clear_flag(tab2, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(cont2, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_set_size(cont2, lv_pct(100), lv_pct(100));
  lv_obj_set_style_pad_all(cont2, 0, 0);
  lv_obj_set_style_border_width(cont2, 0, 0);
  lv_obj_set_style_bg_opa(cont2, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(cont2, LV_OBJ_FLAG_SCROLLABLE);

  setup_gyro_tab(cont2, disp_w, disp_h);

  //Tab3 - energy

  lv_obj_set_style_pad_all(tab3, 0, 0);
  lv_obj_set_style_border_width(tab3, 0, 0);
  lv_obj_set_style_radius(tab3, 0, 0);

  lv_obj_t *cont3 = lv_obj_create(tab3);
  lv_obj_clear_flag(tab3, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(cont3, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_set_size(cont3, lv_pct(100), lv_pct(100));
  lv_obj_set_style_pad_all(cont3, 0, 0);
  lv_obj_set_style_border_width(cont3, 0, 0);
  lv_obj_set_style_bg_opa(cont3, LV_OPA_TRANSP, 0);

  CamperUI::Callbacks camper_cb = {};
  camper_cb.onLightChanged = onLightChanged;
  // camper_cb.onAllOff = ...;
  // camper_cb.onActuatorPressed = ...;
  // camper_cb.onActuatorReleased = ...;

  lv_obj_update_layout(tv);
  lv_obj_update_layout(cont3);

  lv_coord_t tab3_w = lv_obj_get_content_width(cont3);
  lv_coord_t tab3_h = lv_obj_get_content_height(cont3);

  CamperUI::init(cont3, tab3_w, tab3_h, camper_cb);

    /* Remove padding/border from the tab page itself */
  lv_obj_set_style_pad_all(tab4, 0, 0);
  lv_obj_set_style_border_width(tab4, 0, 0);
  lv_obj_set_style_radius(tab4, 0, 0);

  lv_obj_t *cont4 = lv_obj_create(tab4);
  lv_obj_clear_flag(tab4, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_clear_flag(cont4, LV_OBJ_FLAG_SCROLLABLE);

  lv_obj_set_size(cont4, lv_pct(100), lv_pct(100));
  lv_obj_set_style_pad_all(cont4, 0, 0);
  lv_obj_set_style_border_width(cont4, 0, 0);
  lv_obj_set_style_bg_opa(cont4, LV_OPA_TRANSP, 0);

  setup_camera_tab(cont4, disp_w, disp_h);
  camera_ui_set_callback(on_camera_selected);

  init_gauge_timer();
  init_timer();
  
  if (!initEspNow())
  {
    Serial0.printf("ESP-NOW init failed"); 
  }

  if (!espnow_retry_init()) {
    Serial.println("Retry layer init failed");
  }

  on_camera_selected(0);

  Serial0.printf("Setup complete"); 
}

void on_camera_selected(uint8_t id)
{
  Serial.printf("Camera selected: %d\n", id);

  switch(id)
  {
    case 0: Serial.println("Rear high"); break;
    case 1: Serial.println("Rear low"); break;
    case 2: Serial.println("Front high"); break;
    case 3: Serial.println("Front low"); break;
  }

  Contracts::CameraSelectPacket pkt;
  pkt.type = Contracts::TYPE_CAMERA_SELECT;
  pkt.input  = (int16_t)id+1;

  esp_err_t res = espnow_send_cached(Vehicle::LILYGO_CAMERA_MAC, (uint8_t*)&pkt, sizeof(pkt));
  if (res != ESP_OK) {
    Serial.print("ESP-NOW send: ");
    Serial.println(res == ESP_OK ? "OK" : String("ERR ") + res);
  }
          
}

static void anim_all_cb(lv_timer_t *t)
{
  attitude_set_value(&pitch_ui, gyro::pitch_deg);
  attitude_set_value(&roll_ui, gyro::roll_deg);
  compass_strip_set_heading(&g_compass, gyro::yaw_deg);
  compass_strip_set_altitude(&g_compass, gyro::height);
}

void init_timer()
{
    // one place to create the timer
    lv_timer_create(anim_all_cb, 100, nullptr);
}

void setup_gyro_tab(lv_obj_t *cont, int disp_w, int disp_h)
{
  // IMPORTANT: enable grid layout
  lv_obj_set_layout(cont, LV_LAYOUT_GRID);

  // Optional: remove padding/gaps so you get full use of the screen
  lv_obj_set_style_pad_all(cont, 0, 0);
  lv_obj_set_style_pad_row(cont, 0, 0);
  lv_obj_set_style_pad_column(cont, 0, 0);

  static lv_coord_t col_dsc[] = {
    LV_GRID_FR(1), LV_GRID_FR(1),
    LV_GRID_TEMPLATE_LAST
  };

  // 3/4 top, 1/4 bottom
  static lv_coord_t row_dsc[] = {
    LV_GRID_FR(3),   // row 0: pitch + roll
    LV_GRID_FR(1),   // row 1: controls
    LV_GRID_TEMPLATE_LAST
  };

  lv_obj_set_grid_dsc_array(cont, col_dsc, row_dsc);

  // ---- Panel containers that actually fill their grid cells ----
  lv_obj_t *cell_pitch = lv_obj_create(cont);
  lv_obj_remove_style_all(cell_pitch);
  lv_obj_set_style_bg_opa(cell_pitch, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(cell_pitch, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(cell_pitch, lv_pct(100), lv_pct(100));
  lv_obj_set_grid_cell(cell_pitch,
                       LV_GRID_ALIGN_STRETCH, 0, 1,
                       LV_GRID_ALIGN_STRETCH, 0, 1);

  lv_obj_t *cell_roll = lv_obj_create(cont);
  lv_obj_remove_style_all(cell_roll);
  lv_obj_set_style_bg_opa(cell_roll, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(cell_roll, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(cell_roll, lv_pct(100), lv_pct(100));
  lv_obj_set_grid_cell(cell_roll,
                       LV_GRID_ALIGN_STRETCH, 1, 1,
                       LV_GRID_ALIGN_STRETCH, 0, 1);

  // ---- Compute size so each indicator fits in HALF the screen width ----
  // Your indicator root width is W = 2 * size_px, height = size_px.
  // Each panel is ~disp_w/2 wide. So we need 2*size_px <= panel_w * margin.
  int panel_w = disp_w / 2;
  int panel_h = (disp_h * 3) / 4;

  // leave a little margin (10%)
  int max_root_w = (panel_w * 90) / 100;
  int max_root_h = (panel_h * 90) / 100;

  int size_px = max_root_w / 2;          // because root W = 2*size_px
  if(size_px > max_root_h) size_px = max_root_h;

  // ---- Create indicators inside each panel ----
  pitch_ui = create_attitude_indicator(cell_pitch, size_px);
  attitude_set_center_image(&pitch_ui, &pitch_img);

  roll_ui = create_attitude_indicator(cell_roll, size_px);
  attitude_set_center_image(&roll_ui, &roll_img);

  // Compass
  // ---- Controls area (bottom row) spanning both columns ----
  lv_obj_t *controls = lv_obj_create(cont);
  lv_obj_remove_style_all(controls);
  lv_obj_set_style_bg_opa(controls, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(controls, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(controls, lv_pct(100), lv_pct(100));
  lv_obj_set_grid_cell(controls,
                      LV_GRID_ALIGN_STRETCH, 0, 2,
                      LV_GRID_ALIGN_STRETCH, 1, 1);

  // Make controls a 2-row grid:
  // row 0 = compass (150px fixed)
  // row 1 = your existing 2-cell controls (fills remaining space)
  lv_obj_set_layout(controls, LV_LAYOUT_GRID);
  static lv_coord_t ctrl_cols[] = { LV_GRID_FR(1), LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
  static lv_coord_t ctrl_rows[] = { 150, LV_GRID_FR(1), LV_GRID_TEMPLATE_LAST };
  lv_obj_set_grid_dsc_array(controls, ctrl_cols, ctrl_rows);

  // Optional: remove padding/gaps inside controls so compass fits cleanly
  lv_obj_set_style_pad_all(controls, 0, 0);
  lv_obj_set_style_pad_row(controls, 0, 0);
  lv_obj_set_style_pad_column(controls, 0, 0);

  // ---- Compass cell spanning both columns (row 0) ----
  lv_obj_t *cell_compass = lv_obj_create(controls);
  lv_obj_remove_style_all(cell_compass);
  lv_obj_set_style_bg_opa(cell_compass, LV_OPA_TRANSP, 0);
  lv_obj_clear_flag(cell_compass, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_set_size(cell_compass, lv_pct(100), lv_pct(100));
  lv_obj_set_grid_cell(cell_compass,
                      LV_GRID_ALIGN_STRETCH, 0, 2,   // span 2 columns
                      LV_GRID_ALIGN_STRETCH, 0, 1);  // row 0

  // after creating controls + cell_compass and setting grid cells:
  lv_obj_update_layout(cont);
  lv_obj_update_layout(controls);
  lv_obj_update_layout(cell_compass);

  lv_coord_t cw = lv_obj_get_width(cell_compass);
  lv_coord_t ch = lv_obj_get_height(cell_compass);

  compass_strip_create(&g_compass, cell_compass, 0, 0, cw, ch);
}

void onEspNowRecv(const esp_now_recv_info_t *info,
                  const uint8_t *data, int len)
{
    if (len < 1) {
      return;
    }
    uint8_t type = data[0];
    Serial0.printf("ESP-NOW packet received. Type: %d Length: %d\n", type, len);
    if (type == Contracts::TYPE_GYRO_ANGLE && len == (int)sizeof(Contracts::GyroAnglePacket))
    {
      Contracts::GyroAnglePacket pkt;
      memcpy(&pkt, data, sizeof(pkt));
      processGyroAngle(pkt);
      return;
    }
    if (type == Contracts::TYPE_GYRO_HEIGHT && len == (int)sizeof(Contracts::GyroHeightPacket))
    {
      Contracts::GyroHeightPacket pkt;
      memcpy(&pkt, data, sizeof(pkt));
      processGyroHeight(pkt);
      return;
    }
    if (type == Contracts::TYPE_TPMS && len == (int)sizeof(Contracts::TpmsPacket))
    {
      uint32_t now = millis();
      Contracts::TpmsPacket pkt;
      memcpy(&pkt, data, sizeof(pkt));
      processTpms(pkt);
      last_update = now;
      return;
    }
    if (type == Contracts::TYPE_BMS && len == (int)sizeof(Contracts::BmsPacket))
    {
      Contracts::BmsPacket pkt;
      memcpy(&pkt, data, sizeof(pkt));

      // ESP-NOW callbacks run on the Wi-Fi task. LVGL must only be touched
      // from the task that calls lv_timer_handler().
      portENTER_CRITICAL(&g_bmsMux);
      g_pendingBms = pkt;
      g_bmsPending = true;
      portEXIT_CRITICAL(&g_bmsMux);
      //setDischargeCurrentA(67.0f);

      // Serial.printf("BMS: SOC=%u, Remaining=%d Ah, Charge=%d A\n",
      //             (unsigned)pkt.soc_percent,
      //             pkt.remaining,
      //             pkt.amps);

      return;
    }
    if (type == Contracts::TYPE_WORKLIGHT_STATUS && len == (int)sizeof(Contracts::WorkLightsStatusPacket))
    {
      Contracts::WorkLightsStatusPacket pkt;
      memcpy(&pkt, data, sizeof(pkt));

      portENTER_CRITICAL(&g_lightsMux);
      g_pendingWorkLights = pkt;
      g_workLightsPending = true;
      portEXIT_CRITICAL(&g_lightsMux);
      return;
    }
    if (type == Contracts::TYPE_LIGHT_STATUS && len == (int)sizeof(Contracts::LightsStatusPacket))
    {
      Contracts::LightsStatusPacket pkt;
      memcpy(&pkt, data, sizeof(pkt));

      portENTER_CRITICAL(&g_lightsMux);
      g_pendingLights = pkt;
      g_lightsPending = true;
      portEXIT_CRITICAL(&g_lightsMux);
      return;
    }
}

void onEspNowSent(const wifi_tx_info_t* tx_info, esp_now_send_status_t status) {
    Serial.print("ESP-NOW send status: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "SUCCESS" : "FAIL");
}

bool initEspNow()
{
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    uint8_t staMac[6];
    esp_err_t err = esp_read_mac(staMac, ESP_MAC_WIFI_STA);
    if (err != ESP_OK) {
        Serial0.printf("esp_read_mac failed: %d\n", err);
        return false;
    }

    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
             staMac[0], staMac[1], staMac[2],
             staMac[3], staMac[4], staMac[5]);

    Serial0.print("ESP-NOW (STA) MAC: ");
    Serial0.println(macStr);

    if (esp_now_init() != ESP_OK) {
        Serial0.println("ESP-NOW init failed!");
        return false;
    }

    esp_now_register_recv_cb(onEspNowRecv);
    esp_now_register_send_cb(onEspNowSent);

    // Add peer (your ESP32-S3)
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, Vehicle::LILYGO_CAMERA_MAC, 6);
    peerInfo.channel = 0;   // 0 = current Wi-Fi channel
    peerInfo.encrypt = false;

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add ESP-NOW peer!");
        while (true) { delay(1000); }
    }

    memcpy(peerInfo.peer_addr, Vehicle::LILYGO_WORKLIGHT_MAC, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add work lights peer!");
    }

    memcpy(peerInfo.peer_addr, Vehicle::SHELLY_BRIDGE_MAC, 6);
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add Shelly bridge peer!");
    }

    start_time  = millis();
    last_update = start_time;
    return true;
}

void loop() {
  static uint32_t last = millis();
  uint32_t now = millis();
  lv_tick_inc(now - last);
  last = now;

  Contracts::BmsPacket bms = {};
  bool updateBms = false;
  portENTER_CRITICAL(&g_bmsMux);
  if (g_bmsPending) {
    bms = g_pendingBms;
    g_bmsPending = false;
    updateBms = true;
  }
  portEXIT_CRITICAL(&g_bmsMux);

  if (updateBms) {
    setSocPercent(bms.soc_percent);
    setRemainingAh(bms.remaining / 100.0f);
    setChargeCurrentA(bms.amps / 100.0f);
  }

  Contracts::WorkLightsStatusPacket workLights = {};
  Contracts::LightsStatusPacket lights = {};
  bool updateWorkLights = false;
  bool updateLights = false;
  portENTER_CRITICAL(&g_lightsMux);
  if (g_workLightsPending) {
    workLights = g_pendingWorkLights;
    g_workLightsPending = false;
    updateWorkLights = true;
  }
  if (g_lightsPending) {
    lights = g_pendingLights;
    g_lightsPending = false;
    updateLights = true;
  }
  portEXIT_CRITICAL(&g_lightsMux);

  if (updateWorkLights) {
    setLightState(LightGroup::Exterior, 0, workLights.relay1);
    setLightState(LightGroup::Exterior, 1, workLights.relay2);
    setLightState(LightGroup::Exterior, 2, workLights.relay3);
    setLightState(LightGroup::Exterior, 3, workLights.relay4);
  }

  if (updateLights) {
    setLightState(LightGroup::Interior, 0, lights.zone1);
    setLightState(LightGroup::Interior, 1, lights.zone2);
    setLightState(LightGroup::Interior, 2, lights.zone3);
    setLightState(LightGroup::Interior, 3, lights.zone4);
  }

  lv_timer_handler();
  espnow_retry_poll();
  delay(5);
}
