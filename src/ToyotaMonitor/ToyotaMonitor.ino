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
#include "obd_ui.hpp"
#include "fo_theme.hpp"
#include "sound.hpp"
#include "diag_ui.hpp"
#include "heater_ui.hpp"

// millis() of the last gyro packet (0 = never)
static volatile uint32_t gyro_last_update = 0;

// Pages: TYRES, GYRO, TRANS, HEAT (rotated in round-robin mode), then
// DIAGNOSTICS behind the cog button
static const int kPages = 5;
static const int kRotatePages = 4;
static const int DIAG_PAGE = 4;
static lv_obj_t *pages[kPages];

// Tab bar buttons; a button can hold several pages (tap again to switch)
static const int kButtons = 5;
static const int kPageButton[kPages] = { 0, 1, 2, 3, 4 };
static lv_obj_t *tab_buttons[kButtons];
static lv_obj_t *tab_lamps[kButtons];   // none for the cog button

static void build_ui();
static void show_page(int index);
static void start_diagnostics();

// Layout check: uncomment to start on a page (0 TYRES, 1 GYRO, 2 TRANS, 3 HEAT)
// with made-up readings, refreshed every second
//#define UI_DEMO_PAGE 2

// Screen recording (for an animated GIF of the startup diagnostics): uncomment to record the
// screen from boot into PSRAM, then read it out over USB with 'R' and 'F'
//#define UI_RECORD
#ifdef UI_RECORD
static void rec_flush(const lv_area_t *area, const lv_color_t *color_p);
#endif

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
#ifdef UI_RECORD
  rec_flush(area, color_p);
#endif
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

#ifdef UI_DEMO_PAGE
// Made-up readings for checking the layout without the connectors
static void ui_demo_timer_cb(lv_timer_t *t)
{
  static uint32_t seq = 1000000;
  static const uint32_t pressure[] = { 245000, 242000, 260000, 258000 };   // Pa
  static const int16_t temp[] = { 2150, 2200, 2050, 2000 };              // centi-degrees
  for (int i = 0; i < 4; i++) {
    tpms::TpmsPacket p = { ++seq, Vehicle::TPMS_SENSORS[i], pressure[i], temp[i] };
    tpms::handleTpms(p);
  }

  gyro_last_update = millis();
  attitude_update(4.0f, -2.0f);

  Contracts::ObdPacket o = { Contracts::TYPE_OBD, Contracts::OBD_STATUS_OK, 4, 1, 92, 108, 186, 100, 85 };
  obd_ui_update(o);


  Contracts::HeaterPacket h = { Contracts::TYPE_HEATER, Contracts::HEATER_LINK_OK, Contracts::HEATER_STATE_RUNNING,
                                Contracts::HEATER_MODE_LEVEL, 2, 120, 1280, 140, 0, 0, 7 * 60, 1,
                                Contracts::HEATER_CLOCK_NONE };
  heater_ui_update(h);

  Contracts::BmsPacket b = { Contracts::TYPE_BMS, 1330, 1200, 87, 19000, 180 };
  bms_bar_update(b);
}

static void start_ui_demo(int page)
{
  show_page(page);
  ui_demo_timer_cb(nullptr);
  lv_timer_create(ui_demo_timer_cb, 1000, nullptr);
}
#endif

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

  build_ui();
#ifdef UI_RECORD
  start_recording();
#endif

  init_gauge_timer();
  modem::begin();
  sound::begin();

#ifdef UI_DEMO_PAGE
  start_ui_demo(UI_DEMO_PAGE);
#else
  // Start on the diagnostics page; round-robin starts once everything checks out
  start_diagnostics();
#endif
}

// Round-robin: rotate the tabs automatically while the button is on
static const uint32_t AUTO_ROTATE_MS = 30000;
static int current_page = 0;
static bool auto_rotate = false;
static lv_obj_t *auto_button;
static lv_timer_t *auto_timer;

static void show_page(int index)
{
  current_page = index;
  // Leaving the diagnostics page stops a run
  if (index != DIAG_PAGE) diag_stop();

  for (int i = 0; i < kPages; i++) {
    if (i == index) lv_obj_clear_flag(pages[i], LV_OBJ_FLAG_HIDDEN);
    else lv_obj_add_flag(pages[i], LV_OBJ_FLAG_HIDDEN);
  }

  for (int i = 0; i < kButtons; i++) {
    bool active = i == kPageButton[index];
    if (active) lv_obj_add_state(tab_buttons[i], LV_STATE_CHECKED);
    else lv_obj_clear_state(tab_buttons[i], LV_STATE_CHECKED);
    if (!tab_lamps[i]) continue;
    if (active) lv_obj_add_state(tab_lamps[i], LV_STATE_CHECKED);
    else lv_obj_clear_state(tab_lamps[i], LV_STATE_CHECKED);
    // The active lamp glows yellow instead of red while rotating
    if (auto_rotate) lv_obj_add_state(tab_lamps[i], LV_STATE_USER_1);
    else lv_obj_clear_state(tab_lamps[i], LV_STATE_USER_1);
  }
}

static void set_auto_rotate(bool on)
{
  auto_rotate = on;
  if (on) {
    lv_obj_add_state(auto_button, LV_STATE_CHECKED);
    lv_timer_reset(auto_timer);
    lv_timer_resume(auto_timer);
  } else {
    lv_obj_clear_state(auto_button, LV_STATE_CHECKED);
    lv_timer_pause(auto_timer);
  }
}

static void tab_clicked_cb(lv_event_t *e)
{
  int button = (int)(intptr_t)lv_event_get_user_data(e);
  // First page of the button, or its next page when it's already showing
  int page = -1;
  for (int i = 0; i < kPages; i++) {
    if (kPageButton[i] != button) continue;
    if (page < 0) page = i;
    if (i > current_page && kPageButton[current_page] == button) {
      page = i;
      break;
    }
  }
  // A manual pick ends the rotation and stays on that tab
  set_auto_rotate(false);
  show_page(page);
}

static void auto_timer_cb(lv_timer_t *t)
{
  show_page(current_page >= kRotatePages - 1 ? 0 : current_page + 1);
}

static void auto_clicked_cb(lv_event_t *e)
{
  set_auto_rotate(!auto_rotate);
  // Diagnostics isn't part of the rotation
  show_page(auto_rotate && current_page == DIAG_PAGE ? 0 : current_page);
}

// All checks passed (and announced by the diagnostics page): start rotating the tabs
static void diagnostics_passed()
{
  set_auto_rotate(true);
  show_page(0);
}

static void start_diagnostics()
{
  set_auto_rotate(false);
  show_page(DIAG_PAGE);
  diag_start(diagnostics_passed);
}

static void diag_clicked_cb(lv_event_t *e)
{
  start_diagnostics();
}

// Swiping left / right steps through the rotated tabs (not diagnostics). Like a tab
// tap it's a manual pick, so it ends the rotation.
static void gesture_cb(lv_event_t *e)
{
  lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_get_act());
  if (dir != LV_DIR_LEFT && dir != LV_DIR_RIGHT) return;
  // A swipe that started on a button must not also click it
  lv_indev_wait_release(lv_indev_get_act());

  int next;
  if (current_page >= kRotatePages) next = dir == LV_DIR_LEFT ? 0 : kRotatePages - 1;
  else next = (current_page + (dir == LV_DIR_LEFT ? 1 : kRotatePages - 1)) % kRotatePages;
  set_auto_rotate(false);
  show_page(next);
}

// Four text tabs and two icon buttons share the 480 px bar, so it's tight:
// TRANS needs 2 + 12 + 5 + 62 + 2 = 83 px of the ~83 px each tab gets
static const lv_coord_t NAV_PAD = 8;
static const lv_coord_t NAV_GAP = 6;
static const lv_coord_t TAB_LAMP_SIZE = 12;
static const lv_coord_t TAB_LAMP_GAP = 5;
static const lv_coord_t ICON_BUTTON_W = 50;

static lv_style_t style_tab, style_tab_on, style_tab_pressed, style_lamp, style_lamp_on, style_lamp_auto,
                  style_auto_on;

static void init_tab_styles()
{
  lv_style_init(&style_tab);
  lv_style_set_radius(&style_tab, 6);
  lv_style_set_bg_opa(&style_tab, LV_OPA_COVER);
  lv_style_set_bg_color(&style_tab, fo::hex(0x5a4834));
  lv_style_set_bg_grad_color(&style_tab, fo::hex(0x3a2e22));
  lv_style_set_bg_grad_dir(&style_tab, LV_GRAD_DIR_VER);
  lv_style_set_border_color(&style_tab, fo::hex(0x1a130c));
  lv_style_set_border_width(&style_tab, 2);
  lv_style_set_text_font(&style_tab, &fo_stencil_19);
  lv_style_set_text_color(&style_tab, fo::hex(fo::ENGRAVED));
  lv_style_set_pad_column(&style_tab, TAB_LAMP_GAP);

  lv_style_init(&style_tab_on);
  lv_style_set_bg_color(&style_tab_on, fo::hex(0x2f251b));
  lv_style_set_bg_grad_color(&style_tab_on, fo::hex(0x40332a));
  lv_style_set_text_color(&style_tab_on, fo::hex(fo::ENGRAVED_HI));

  lv_style_init(&style_tab_pressed);
  lv_style_set_bg_color(&style_tab_pressed, fo::hex(0x2a2017));
  lv_style_set_bg_grad_color(&style_tab_pressed, fo::hex(0x2a2017));

  lv_style_init(&style_lamp);
  lv_style_set_size(&style_lamp, TAB_LAMP_SIZE);
  lv_style_set_radius(&style_lamp, LV_RADIUS_CIRCLE);
  lv_style_set_bg_opa(&style_lamp, LV_OPA_COVER);
  lv_style_set_bg_color(&style_lamp, fo::hex(0x5a2a22));
  lv_style_set_bg_grad_color(&style_lamp, fo::hex(0x2a100c));
  lv_style_set_bg_grad_dir(&style_lamp, LV_GRAD_DIR_VER);
  lv_style_set_border_color(&style_lamp, fo::hex(0x1a130c));
  lv_style_set_border_width(&style_lamp, 2);

  lv_style_init(&style_lamp_on);
  lv_style_set_bg_color(&style_lamp_on, fo::hex(0xff6a4a));
  lv_style_set_bg_grad_color(&style_lamp_on, fo::hex(0xa01208));
  lv_style_set_shadow_color(&style_lamp_on, fo::hex(0xff4a2a));
  lv_style_set_shadow_width(&style_lamp_on, 12);
  lv_style_set_shadow_spread(&style_lamp_on, 1);

  lv_style_init(&style_lamp_auto);
  lv_style_set_bg_color(&style_lamp_auto, fo::hex(0xfff09a));
  lv_style_set_bg_grad_color(&style_lamp_auto, fo::hex(0xc89a08));
  lv_style_set_shadow_color(&style_lamp_auto, fo::hex(0xffd23a));

  lv_style_init(&style_auto_on);
  lv_style_set_text_color(&style_auto_on, fo::hex(0xffd23a));
}

static lv_obj_t *create_nav_button(lv_obj_t *nav, lv_event_cb_t cb, void *user_data)
{
  lv_obj_t *btn = lv_btn_create(nav);
  lv_obj_remove_style_all(btn);
  lv_obj_add_style(btn, &style_tab, 0);
  lv_obj_add_style(btn, &style_tab_on, LV_STATE_CHECKED);
  lv_obj_add_style(btn, &style_tab_pressed, LV_STATE_PRESSED);
  lv_obj_set_height(btn, 66);
  lv_obj_set_flex_flow(btn, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(btn, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  lv_obj_add_event_cb(btn, cb, LV_EVENT_CLICKED, user_data);
  return btn;
}

// Stencilled metal tab button with a lamp that lights on the active tab
// (red, or yellow while the tabs rotate)
static void create_tab_button(lv_obj_t *nav, int index, const char *name)
{
  lv_obj_t *btn = create_nav_button(nav, tab_clicked_cb, (void *)(intptr_t)index);
  lv_obj_set_flex_grow(btn, 1);

  lv_obj_t *lamp = fo::box(btn);
  lv_obj_add_style(lamp, &style_lamp, 0);
  lv_obj_add_style(lamp, &style_lamp_on, LV_STATE_CHECKED);
  lv_obj_add_style(lamp, &style_lamp_auto, LV_STATE_CHECKED | LV_STATE_USER_1);

  lv_obj_t *label = lv_label_create(btn);
  lv_label_set_text(label, name);

  tab_buttons[index] = btn;
  tab_lamps[index] = lamp;
}

static lv_obj_t *create_icon_button(lv_obj_t *nav, lv_event_cb_t cb, const char *symbol)
{
  lv_obj_t *btn = create_nav_button(nav, cb, nullptr);
  lv_obj_set_width(btn, ICON_BUTTON_W);
  lv_obj_t *icon = lv_label_create(btn);
  lv_obj_set_style_text_font(icon, &lv_font_montserrat_36, 0);
  lv_label_set_text(icon, symbol);
  return btn;
}

// Square cog button that opens the diagnostics page and runs the checks
static void create_diag_button(lv_obj_t *nav)
{
  tab_buttons[kPageButton[DIAG_PAGE]] = create_icon_button(nav, diag_clicked_cb, LV_SYMBOL_SETTINGS);
  tab_lamps[kPageButton[DIAG_PAGE]] = nullptr;
}

// Square round-robin button with a loop icon, yellow while on
static void create_auto_button(lv_obj_t *nav)
{
  auto_button = create_icon_button(nav, auto_clicked_cb, LV_SYMBOL_LOOP);
  lv_obj_add_style(auto_button, &style_auto_on, LV_STATE_CHECKED);

  auto_timer = lv_timer_create(auto_timer_cb, AUTO_ROTATE_MS, nullptr);
  lv_timer_pause(auto_timer);
}

// Fallout 2 style screen: metal tab bar, CRT with the pages, metal status bar
static void build_ui()
{
  fo::init();
  init_tab_styles();

  lv_obj_t *scr = lv_scr_act();
  lv_obj_set_style_bg_color(scr, fo::hex(fo::FRAME_DARK), 0);
  lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
  lv_obj_clear_flag(scr, LV_OBJ_FLAG_SCROLLABLE);
  // The pages hold no clickable widgets, so a press on them lands on the screen
  lv_obj_add_event_cb(scr, gesture_cb, LV_EVENT_GESTURE, nullptr);

  // ---- Tab bar ----
  lv_obj_t *nav = fo::metal_panel(scr, 0, 0, fo::SCREEN_W, fo::NAV_H);
  lv_obj_set_style_pad_hor(nav, NAV_PAD, 0);
  lv_obj_set_style_pad_column(nav, NAV_GAP, 0);
  lv_obj_set_flex_flow(nav, LV_FLEX_FLOW_ROW);
  lv_obj_set_flex_align(nav, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
  create_tab_button(nav, 0, "TYRES");
  create_tab_button(nav, 1, "GYRO");
  create_tab_button(nav, 2, "TRANS");
  create_tab_button(nav, 3, "HEAT");
  create_diag_button(nav);
  create_auto_button(nav);

  // ---- CRT with the pages ----
  lv_obj_t *main_panel = fo::metal_panel(scr, 0, fo::NAV_H, fo::SCREEN_W, fo::MAIN_H);
  fo::add_rivets(main_panel);
  lv_obj_t *glass = fo::crt(main_panel);

  static const lv_coord_t page_pad[kPages] = { 10, 10, 14, 14, 14 };
  for (int i = 0; i < kPages; i++) {
    pages[i] = fo::box(glass);
    lv_obj_set_size(pages[i], lv_pct(100), lv_pct(100));
    lv_obj_set_style_pad_all(pages[i], page_pad[i], 0);
  }
  create_tyres_page(pages[0]);
  create_level_page(pages[1], &gyro_last_update);
  create_obd_tab(pages[2]);
  create_heater_tab(pages[3]);
  create_diag_page(pages[DIAG_PAGE], &gyro_last_update);

  // ---- Status bar: battery counters and connector lamps ----
  lv_obj_t *status_bar = create_bms_bar(scr, fo::SCREEN_W, fo::FOOTER_H);

  // Three rows keep the lamps narrow enough for the counter captions
  static lv_coord_t link_col_dsc[] = { LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_TEMPLATE_LAST };
  static lv_coord_t link_row_dsc[] = { LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_CONTENT, LV_GRID_TEMPLATE_LAST };
  lv_obj_t *links = fo::box(status_bar);
  lv_obj_set_size(links, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
  lv_obj_set_grid_dsc_array(links, link_col_dsc, link_row_dsc);
  lv_obj_set_style_pad_row(links, 4, 0);
  lv_obj_set_style_pad_column(links, 10, 0);
  lv_obj_set_style_pad_hor(links, 10, 0);
  lv_obj_set_style_pad_ver(links, 3, 0);
  lv_obj_set_style_bg_color(links, fo::hex(0x2a2017), 0);
  lv_obj_set_style_bg_opa(links, LV_OPA_COVER, 0);
  lv_obj_set_style_border_color(links, fo::hex(fo::EDGE), 0);
  lv_obj_set_style_border_width(links, 2, 0);
  lv_obj_set_style_radius(links, 6, 0);

  // TPMS connector pings every 5 s, so 15 s without one means the link is down
  lv_obj_t *link = create_link_indicator(links, "TPMS", &tpms::last_update, 15000);
  lv_obj_set_grid_cell(link, LV_GRID_ALIGN_START, 0, 1, LV_GRID_ALIGN_CENTER, 0, 1);
  // OBD connector sends at least every second, but BLE scanning can pause it for several seconds
  link = create_link_indicator(links, "OBD", &obd_last_update, 15000);
  lv_obj_set_grid_cell(link, LV_GRID_ALIGN_START, 1, 1, LV_GRID_ALIGN_CENTER, 0, 1);
  // BMS and gyro connectors send every second
  link = create_link_indicator(links, "BMS", &bms_last_update, 10000);
  lv_obj_set_grid_cell(link, LV_GRID_ALIGN_START, 0, 1, LV_GRID_ALIGN_CENTER, 1, 1);
  link = create_link_indicator(links, "GYRO", &gyro_last_update, 10000);
  lv_obj_set_grid_cell(link, LV_GRID_ALIGN_START, 1, 1, LV_GRID_ALIGN_CENTER, 1, 1);
  // Heater connector sends every second, but scanning for the heater pauses it for ~5 s
  link = create_link_indicator(links, "HEAT", &heater_last_update, 15000);
  lv_obj_set_grid_cell(link, LV_GRID_ALIGN_START, 0, 1, LV_GRID_ALIGN_CENTER, 2, 1);

  show_page(0);
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

    gyro_last_update = millis();
    attitude_update(g.pitch_deg / 100.0f, g.roll_deg / 100.0f);
    break;
  }

  case Contracts::TYPE_OBD: {
    Contracts::ObdPacket o;
    if (m.len != sizeof(o)) return;
    memcpy(&o, m.data, sizeof(o));

    obd_ui_update(o);
    diag_obd_update(o);
    break;
  }

  case Contracts::TYPE_DTC: {
    Contracts::DtcPacket d;
    if (m.len != sizeof(d)) return;
    memcpy(&d, m.data, sizeof(d));

    diag_dtc_update(d);
    Serial.printf("DTC scan %u: mil=%d engine=%d/%d gearbox=%d/%d abs=%d\n", d.scan, d.mil,
                  d.stored[Contracts::DTC_ENGINE], d.pending[Contracts::DTC_ENGINE],
                  d.stored[Contracts::DTC_GEARBOX], d.pending[Contracts::DTC_GEARBOX],
                  d.stored[Contracts::DTC_ABS]);
    break;
  }

  case Contracts::TYPE_HEATER: {
    Contracts::HeaterPacket h;
    if (m.len != sizeof(h)) return;
    memcpy(&h, m.data, sizeof(h));

    heater_ui_update(h);
    diag_heater_update(h);
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

// Layout checks: sending 'S' over USB serial replies "SNAP <w> <h> <bytes>\n" followed
// by the screen as raw RGB565 (little endian)
static void send_snapshot()
{
  static uint8_t *snap = nullptr;
  uint32_t size = lv_snapshot_buf_size_needed(lv_scr_act(), LV_IMG_CF_TRUE_COLOR);
  if (!snap) snap = (uint8_t *)heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
  if (!snap) return;

  lv_img_dsc_t dsc;
  if (lv_snapshot_take_to_buf(lv_scr_act(), LV_IMG_CF_TRUE_COLOR, &dsc, snap, size) != LV_RES_OK) {
    Serial.println("SNAP failed");
    return;
  }
  Serial.printf("SNAP %d %d %u\n", dsc.header.w, dsc.header.h, (unsigned)dsc.data_size);
  // USB CDC drops what doesn't fit its buffer: send in chunks and let each drain
  uint32_t sent = 0, start = millis();
  while (sent < dsc.data_size && millis() - start < 30000) {
    size_t n = dsc.data_size - sent < 512 ? dsc.data_size - sent : 512;
    size_t w = Serial.write(snap + sent, n);
    Serial.flush();
    sent += w;
    if (w == 0) delay(2);
  }
}

#ifdef UI_RECORD
// Mirrors what the display driver draws, and every REC_INTERVAL_MS stores the rows that
// changed since the previous frame. 'R' stops recording and replies
// "REC <w> <h> <frames> <avg ms per frame>\n". 'F' + frame index (2 bytes LE) replies
// "FRAME <i> <ms> <len> <sum>\n" + len bytes: [rows:2] then rows x [y:2][row pixels],
// sum = byte sum, so the reader can retry a bad one.
static const uint32_t REC_INTERVAL_MS = 125;   // the diagnostics cursor blinks every 250 ms
static const uint32_t REC_MAX_FRAMES = 1200;
static const size_t REC_RESERVE = 4 * 1024 * 1024;   // PSRAM left for everything else
static const lv_coord_t REC_W = LCD_H_RES, REC_H = LCD_V_RES;
static const size_t REC_ROW = REC_W * sizeof(lv_color_t);

static uint8_t *rec_buf, *rec_prev, *rec_screen;
static size_t rec_cap, rec_len;
static uint32_t rec_offset[REC_MAX_FRAMES], rec_ms[REC_MAX_FRAMES];
static uint32_t rec_frames, rec_cost_total;
static bool rec_on;

// Called from the flush callback with the area LVGL just drew
static void rec_flush(const lv_area_t *area, const lv_color_t *color_p)
{
  if (!rec_on) return;
  size_t w = (area->x2 - area->x1 + 1) * sizeof(lv_color_t);
  for (int y = area->y1; y <= area->y2; y++, color_p += area->x2 - area->x1 + 1) {
    memcpy(rec_screen + y * REC_ROW + area->x1 * sizeof(lv_color_t), color_p, w);
  }
}

static void rec_put16(uint16_t v)
{
  rec_buf[rec_len++] = v & 0xFF;
  rec_buf[rec_len++] = v >> 8;
}

static void rec_timer_cb(lv_timer_t *)
{
  if (!rec_on) return;
  uint32_t t0 = millis();
  uint16_t changed = 0;
  for (int y = 0; y < REC_H; y++) {
    if (rec_frames == 0 || memcmp(rec_screen + y * REC_ROW, rec_prev + y * REC_ROW, REC_ROW) != 0) changed++;
  }
  if (rec_frames >= REC_MAX_FRAMES || rec_len + 2 + changed * (2 + REC_ROW) > rec_cap) {
    rec_on = false;
    return;
  }
  rec_offset[rec_frames] = rec_len;
  rec_ms[rec_frames] = t0;
  rec_put16(changed);
  for (int y = 0; y < REC_H; y++) {
    uint8_t *row = rec_screen + y * REC_ROW;
    if (rec_frames != 0 && memcmp(row, rec_prev + y * REC_ROW, REC_ROW) == 0) continue;
    rec_put16(y);
    memcpy(rec_buf + rec_len, row, REC_ROW);
    memcpy(rec_prev + y * REC_ROW, row, REC_ROW);
    rec_len += REC_ROW;
  }
  rec_frames++;
  rec_cost_total += millis() - t0;
}

static void start_recording()
{
  rec_prev = (uint8_t *)heap_caps_calloc(REC_H, REC_ROW, MALLOC_CAP_SPIRAM);
  rec_screen = (uint8_t *)heap_caps_calloc(REC_H, REC_ROW, MALLOC_CAP_SPIRAM);
  size_t largest = heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
  rec_cap = largest > REC_RESERVE ? largest - REC_RESERVE : 0;
  rec_buf = rec_cap ? (uint8_t *)heap_caps_malloc(rec_cap, MALLOC_CAP_SPIRAM) : nullptr;
  if (!rec_prev || !rec_screen || !rec_buf) {
    Serial.println("REC: no memory");
    return;
  }
  Serial.printf("REC: %u bytes\n", (unsigned)rec_cap);
  rec_on = true;
  lv_timer_create(rec_timer_cb, REC_INTERVAL_MS, nullptr);
}

static void send_rec_frame(uint32_t i)
{
  if (i >= rec_frames) {
    Serial.println("FRAME none");
    return;
  }
  uint32_t end = i + 1 < rec_frames ? rec_offset[i + 1] : rec_len;
  const uint8_t *data = rec_buf + rec_offset[i];
  uint32_t len = end - rec_offset[i], sum = 0;
  for (uint32_t k = 0; k < len; k++) sum += data[k];
  Serial.printf("FRAME %u %u %u %u\n", (unsigned)i, (unsigned)(rec_ms[i] - rec_ms[0]),
                (unsigned)len, (unsigned)sum);
  uint32_t sent = 0, start = millis();
  while (sent < len && millis() - start < 10000) {
    // Long bursts lose bytes now and then (the reader retries) and can crash the CDC driver
    size_t n = len - sent < 256 ? len - sent : 256;
    size_t w = Serial.write(data + sent, n);
    Serial.flush();
    sent += w;
    delay(w == 0 ? 2 : 1);
  }
}
#endif

void loop()
{
  static uint32_t last = millis();
  uint32_t now = millis();
  lv_tick_inc(now - last);
  last = now;

  lv_timer_handler();

  // 'S' screenshot, '1'..'5' show a page (for screenshots of every page),
  // 'P' plays the startup announcement (speaker test)
  while (Serial.available()) {
    int c = Serial.read();
    if (c == 'S') send_snapshot();
#ifdef UI_RECORD
    else if (c == 'R') {
      rec_on = false;
      Serial.printf("REC %d %d %u %u reset=%d\n", REC_W, REC_H, (unsigned)rec_frames,
                    (unsigned)(rec_frames ? rec_cost_total / rec_frames : 0), (int)esp_reset_reason());
    }
    else if (c == 'F') {
      uint8_t idx[2];
      if (Serial.readBytes(idx, 2) == 2) send_rec_frame(idx[0] | (idx[1] << 8));
    }
#endif
    else if (c == 'P') {
      static const Sound *const startup[] = {
        &snd_reactor_online, &snd_sensors_online, &snd_all_systems_nominal, nullptr
      };
      sound::play(startup);
    }
    else if (c >= '1' && c < '1' + kPages) {
      set_auto_rotate(false);
      show_page(c - '1');
    }
  }

  modem::Message m;
  while (modem::poll(m)) {
    handleModemMessage(m);
  }
  delay(5);
}