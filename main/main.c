/* ============================================================
   BỘ SẠC MPPT ESP32-C5 — BẢN ESP-IDF THUẦN (KHÔNG DÙNG ARDUINO)
   ------------------------------------------------------------
   Port lại từ bản Arduino (.ino), giữ nguyên:
     - Thuật toán MPPT Perturb & Observe
     - Bảo vệ điện áp (CV theo hóa học pin)
     - Bảo vệ dòng 3 mức (Imax*1.05 ghì dòng, Imax*1.10 >1ms dừng)
     - SOC coulomb-counting + tự hiệu chỉnh khi pin đầy
     - Đẩy/đọc dữ liệu qua Firebase Realtime Database (REST API)
   Tạm BỎ QUA phần LCD I2C (giảm độ phức tạp lúc port sang IDF).

   *** BẢN SỬA LỖI TLS -0x2700 (MBEDTLS_ERR_X509_CERT_VERIFY_FAILED) ***
   - Không nhúng 1 root CA đơn lẻ nữa (GTS Root R4 KHÔNG khớp chuỗi của
     firebasedatabase.app -> verify fail). Thay bằng certificate bundle
     của ESP-IDF (esp_crt_bundle_attach), chứa sẵn toàn bộ root Google.
   - Chỉ gọi Firebase khi NTP đã đồng bộ (TLS cần giờ đúng để check hạn CA).
   ============================================================ */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <sys/time.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_netif_sntp.h"
#include "esp_timer.h"
#include "esp_log.h"
#include "esp_crt_bundle.h"   // <-- bundle CA của ESP-IDF (thay cho cert nhúng tay)
#include "esp_http_client.h"
#include "esp_https_ota.h"    // <-- OTA qua HTTPS
#include "esp_ota_ops.h"      // <-- rollback / xác nhận firmware
#include "esp_app_desc.h"     // <-- đọc mô tả app đang chạy (version/ngày build) để chẩn đoán OTA
#include "esp_system.h"       // <-- esp_reset_reason(): biết lần reset trước là do crash hay chủ động
#include "esp_heap_caps.h"

#include "nvs_flash.h"
#include "nvs.h"
#include "esp_mac.h"             // đọc MAC address làm Device ID
#include "esp_http_server.h"     // HTTP server cho WiFi Manager AP mode

#include "driver/ledc.h"
#include "driver/gpio.h"        // đọc nút BOOT
#include "esp_adc/adc_oneshot.h"

#include "cJSON.h"

static const char *TAG = "MPPT";

// ============================================================
//   DEVICE ID — dùng MAC address WiFi làm định danh duy nhất
//   Mỗi board có device_id riêng, tất cả path Firebase đều
//   có prefix /devices/{device_id}/ để tách biệt dữ liệu
//   giữa các khách hàng độc lập.
// ============================================================
static char g_device_id[13];    // 12 ký tự hex + null, vd: "d0cf13e52c78"
static char g_dev_path_buf[192]; // buffer cho hàm dev_path()

// Ghép /devices/{device_id} vào trước bất kỳ subpath nào
// Lưu ý: KHÔNG thread-safe nếu gọi đồng thời từ nhiều task.
// Trong code này mọi Firebase call đều từ app_main task nên an toàn.
static const char *dev_path(const char *subpath) {
    snprintf(g_dev_path_buf, sizeof(g_dev_path_buf),
             "/devices/%s%s", g_device_id, subpath);
    return g_dev_path_buf;
}

// Đọc MAC address WiFi STA -> lưu vào g_device_id (gọi sau wifi_init)
static void device_id_init(void) {
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(g_device_id, sizeof(g_device_id),
             "%02x%02x%02x%02x%02x%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "=== DEVICE ID: %s ===", g_device_id);
    ESP_LOGI(TAG, "URL web: https://<your-app>.web.app?device=%s", g_device_id);
}

// ============================================================
//   0. WIFI + FIREBASE — ĐIỀN THÔNG TIN CỦA BẠN
// ============================================================
#define WIFI_SSID     "LEXUANKHOA"
#define WIFI_PASSWORD "12345678"

// ---- WiFi Manager: tên AP phát khi cần cấu hình lại WiFi ----
// Người dùng kết nối vào AP này, mở 192.168.4.1 để nhập WiFi mới
#define WIFI_AP_SSID  "Bo sac MPPT_Cap Nhat WiFi"   // ASCII tương đương (ESP-IDF không encode UTF-8 trong SSID dễ)
#define WIFI_AP_PASS  ""                             // AP mở, không cần mật khẩu
#define NVS_WIFI_NS   "wifi_cfg"                     // namespace NVS lưu credentials
#define NVS_KEY_SSID  "ssid"
#define NVS_KEY_PASS  "pass"
#define FIREBASE_HOST "https://battery-solar-system-default-rtdb.asia-southeast1.firebasedatabase.app"

// ---- OTA: phiên bản firmware hiện tại (tăng số này mỗi lần phát hành bản mới) ----
#define FIRMWARE_VERSION 2

// Link firmware mặc định (đóng cứng trong code) — dùng làm giá trị khởi tạo cho những thiết bị
// CHƯA từng được Admin cấu hình link OTA riêng trên Firebase. Admin vẫn có thể đổi link này
// bất cứ lúc nào từ trang quản trị (ghi đè /mppt/ota/url trên Firebase), không cần build lại code.
// DÙNG "raw.githubusercontent.com" (KHÔNG dùng .../releases/download/...): link Releases phải qua
// bước chuyển hướng phức tạp, dễ gây lỗi khi OTA — link raw tải trực tiếp, ổn định.
// QUAN TRỌNG: PHẢI trỏ tới file "_app.bin" (chỉ chứa app, do idf.py build tạo TRƯỚC khi gộp),
// TUYỆT ĐỐI KHÔNG dùng file "_full.bin" (đã gộp bootloader+partition+app, chỉ dành cho nạp
// USB ở offset 0x0) — dùng nhầm file gộp làm OTA sẽ luôn báo lỗi "Mismatch chip id... found
// 65535" vì phần đầu file gộp là vùng đệm trống (0xFF), không phải app image thật.
#define OTA_DEFAULT_URL "https://raw.githubusercontent.com/mppt1codevn88-del/firmware_bomppt/main/firmware/firmware_v1_app.bin"

// ============================================================
//   1. SƠ ĐỒ CHÂN — giống bản Arduino cho ESP32-C5 Mini
// ============================================================
#define PIN_VPV_ADC   0     // GPIO0 — ADC1_CH0
#define PIN_IPV_ADC   1     // GPIO1 — ADC1_CH1
#define PIN_VBAT_ADC  4     // GPIO4 — ADC1_CH2
#define PIN_IBAT_ADC  3     // GPIO3 — ADC1_CH3
#define PIN_PWM_OUT   6     // GPIO6 — PWM (LEDC)
#define PIN_EXT_RESET_OUT  2  // GPIO2

// ---- LED báo trạng thái (LED đơn, chỉ bật/tắt) + nút BOOT ----
#define PIN_LED_STATUS  23    // GPIO23 — LED trạng thái đơn (ON/OFF), không đổi màu được
#define PIN_BOOT_BTN    28    // GPIO28 — chân strapping boot-mode thật sự trên ESP32-C5 (GPIO9 chỉ đúng trên ESP32-C3!)
                              // Nếu đo thực tế nút BOOT trên board của bạn nối vào chân khác (GPIO2/3/7/25/26/27),
                              // chỉ cần đổi số này.
#define BOOT_HOLD_MS    5000  // giữ 5 giây để kích hoạt reconnect WiFi

#define PWM_FREQ_HZ   78000
#define PWM_RES_BITS  LEDC_TIMER_10_BIT
#define PWM_MAX       ((1 << 10) - 1)

// ============================================================
//   2. NGƯỠNG BẢO VỆ PV (cố định, chỉnh theo tấm pin thực tế)
// ============================================================
#define PV_U_WARN    215.0f
#define PV_U_DANGER  235.0f
#define PV_U_MAX     250.0f
#define PV_I_WARN    12.0f
#define PV_I_DANGER  13.0f
#define PV_I_MAX     1500.0f
#define PV_P_WARN    2430.0f
#define PV_P_DANGER  2650.0f
#define PV_P_MAX     2700.0f

#define CLAMP_TARGET_A   4000.0f
#define ENDURANCE_US     1000UL

// Ngưỡng phát hiện "không có pin cắm vào" — dưới mức này (dù hệ 12/24/36/48V)
// coi như pin đã bị tháo ra, phải tắt sạc ngay để an toàn.
// LƯU Ý: khi SIM_MODE=1, điện áp pin mô phỏng không bao giờ xuống dưới 40V
// nên tính năng này sẽ không tự kích hoạt lúc test mô phỏng — chỉ có ý nghĩa
// khi chạy với cảm biến thật (SIM_MODE=0).
#define VBAT_PRESENT_MIN 5.0f

// ============================================================
//   3. TRẠNG THÁI HỆ THỐNG
// ============================================================
typedef enum { ST_OK=0, ST_STARTUP, ST_CC, ST_ERR_VOLT, ST_ERR_CURR, ST_WARN_CURR, ST_OFF } sys_status_t;
static volatile sys_status_t g_status = ST_STARTUP;


static const char* statusText(sys_status_t s) {
    switch (s) {
        case ST_OK:        return "OK";
        case ST_STARTUP:   return "STARTING UP...";
        case ST_CC:        return "CURRENT CLAMPING";
        case ST_ERR_VOLT:  return "ERROR VOLTAGE";
        case ST_ERR_CURR:  return "ERROR CURRENT";
        case ST_WARN_CURR: return "CURRENT LIMIT";
        case ST_OFF:        return "OFF";
        default:           return "UNKNOWN";
    }
}

// ---- Bật/tắt sạc + phát hiện pin đã lắp hay chưa (đồng bộ qua Firebase, lưu NVS) ----
static volatile bool g_charge_enable = false;   // false = OFF (mặc định an toàn, phải bấm Start mới sạc)
static volatile bool g_batt_present  = false;   // tính lại mỗi vòng lặp dựa trên VbatF
static volatile bool g_charge_just_completed = false; // cờ 1-lần: vừa tự tắt do sạc đầy (SOC=100%), chờ push lên Firebase

// ---- Cấu hình người dùng (đồng bộ qua Firebase, lưu NVS) ----
static volatile int   g_chem = 3;         // 0=Lead-Acid,1=Li-ion,2=LiFePO4,3=Auto
static volatile int   g_volt_sel = 0;     // 0=Auto,12/24/36/48
static volatile int   g_imax = 50;        // A
static volatile float g_capacity_ah = 100.0f;

// ---- Biến hiển thị / trạng thái (MPPT_Task ghi, main loop đọc) ----
static volatile float d_Ppv=0, d_Pbat=0, d_Eff=0;
static volatile float d_Vbat=0, d_Ibat=0, d_Vpv=0, d_Ipv=0;
static volatile float d_SOC=50.0f;
static volatile float d_Vbat_target=14.4f, d_Vbat_emg=15.0f;
static volatile float d_Ibat_cc=50.0f, d_Ibat_trip=55.0f;
static volatile bool  g_reset_request = false;

static void nvs_save_offline_history(void);  // forward declare — định nghĩa đầy đủ ở dưới, cần gọi sớm từ request_external_reset()
static void fb_client_reset(void);  // forward declare — reset client HTTP dùng chung khi WiFi kết nối lại

// ============================================================
//   CƠ CHẾ YÊU CẦU RESET NGOÀI QUA GPIO2
//   - Mặc định LOW (mọi thứ ổn định).
//   - Khi phát hiện 1 trong 2 lỗi dưới đây -> log rõ nguyên nhân -> set GPIO2 HIGH 1 lần.
//   - GPIO2 GIỮ NGUYÊN HIGH (không tự về LOW) cho tới khi mạch ngoài thực sự reset ESP32.
//     Sau khi reset, lúc boot lại GPIO sẽ được cấu hình LOW lại từ đầu -> không cần timer.
//   - 2 nguyên nhân được theo dõi TÁCH BIỆT để log rõ ràng:
//       a) NTP không đồng bộ được (thường do UDP port 123 bị chặn/mạng chập chờn lúc boot)
//       b) Giao tiếp Firebase bị "treo" — lỗi liên tục >= 10 GIÂY THỰC (không phải đếm số lần,
//          vì mỗi request lỗi có thể mất từ vài ms tới 8000ms tuỳ tình huống)
// ============================================================
static volatile bool     g_reset_triggered = false;   // đã yêu cầu reset ngoài hay chưa (chặn log/trigger lặp lại)
static volatile uint64_t g_fail_start_time = 0;        // mốc thời gian (us) của lần lỗi Firebase ĐẦU TIÊN trong chuỗi lỗi hiện tại
static volatile uint64_t g_reset_trigger_time = 0;    // Mốc thời gian (us) bắt đầu dựng xung HIGH
static volatile uint64_t g_low_start_time = 0;        // Mốc thời gian (us) bắt đầu hạ chân xuống LOW
static volatile bool     g_pulse_completed = false;   // Đánh dấu đã hạ xung về LOW hay chưa
#define NET_HANG_US   10000000ULL   // 10 giây treo Firebase liên tục -> coi là lỗi
#define LOW_PERIOD_US 10000000ULL                     // 10 giây chờ ở mức LOW trước khi kích HIGH tiếp



// Gọi khi phát hiện 1 trong 2 điều kiện lỗi ở trên. Idempotent: chỉ log + set GPIO1 lần.
static void request_external_reset(const char *reason) {
    if (g_reset_triggered) return;   // đã yêu cầu rồi, tránh log/trigger lặp lại trong lúc chờ mạch ngoài reset
    g_reset_triggered = true;
    g_pulse_completed = false;
    g_reset_trigger_time = esp_timer_get_time();
    ESP_LOGE("BẢO VỆ", "YEU CAU RESET NGOAI - Nguyen nhan: %s", reason);
    ESP_LOGW("BẢO VỆ", "Bat dau dung xung HIGH tren GPIO2...");
    // Flush ngay RAM ring buffer xuống Flash TRƯỚC khi mạch ngoài reset ESP32,
    // vì reset sẽ xoá sạch RAM -> nếu không flush kịp sẽ mất toàn bộ dữ liệu đang chờ upload.
    nvs_save_offline_history();
    gpio_set_level(PIN_EXT_RESET_OUT, 1);
}

// ============================================================
//   KHOÁ 10% RAM — KHÔNG BAO GIỜ ĐỤNG TỚI
//   ------------------------------------------------------------
//   Ngay lúc khởi động, đo tổng RAM còn trống rồi cấp phát luôn 10% số đó
//   và giữ nguyên suốt vòng đời chương trình — KHÔNG BAO GIỜ free() khối
//   này trong lúc chạy bình thường. Nhờ vậy toàn bộ code còn lại (Firebase,
//   TLS, JSON, OTA...) chỉ còn tối đa 90% RAM để hoạt động, luôn chừa lại
//   1 khoảng trống an toàn tuyệt đối, không phụ thuộc vào việc code có
//   quản lý bộ nhớ tốt hay không.
// ============================================================
#define RAM_RESERVE_PERCENT   10   // % RAM khoá lại vĩnh viễn, không bao giờ dùng
static void  *g_ram_reserve      = NULL;
static size_t g_ram_reserve_size = 0;

static void ram_reserve_init(void) {
    // Đo RAM trống NGAY DÒNG ĐẦU TIÊN của app_main (trước khi module nào khác
    // kịp cấp phát) để con số 10% phản ánh đúng tổng RAM khả dụng thực tế.
    size_t total_free_now = esp_get_free_heap_size();
    g_ram_reserve_size = (total_free_now * RAM_RESERVE_PERCENT) / 100;
    g_ram_reserve = heap_caps_malloc(g_ram_reserve_size, MALLOC_CAP_8BIT);
    if (g_ram_reserve) {
        ESP_LOGI(TAG, "Da khoa %u bytes (%d%% cua %u bytes RAM luc boot) - KHONG BAO GIO dung toi.",
                 (unsigned)g_ram_reserve_size, RAM_RESERVE_PERCENT, (unsigned)total_free_now);
    } else {
        ESP_LOGW(TAG, "Khong the khoa RAM du tru (RAM da qua it ngay tu dau)!");
    }
}

// ---- SOC coulomb-counting ----
static volatile float soc_anchor_pct = -1.0f;
static volatile float soc_ah_accum   = 0.0f;

// ---- Thống kê nạp/xả theo ngày ----
#define STAT_DAYS 366
static float statCharge[STAT_DAYS];
static float statDischarge[STAT_DAYS];
static int   statYear = 0;
// ============================================================
//   CẤU HÌNH BỘ ĐỆM OFFLINE (RING BUFFER - STORE & FORWARD)
// ============================================================
typedef struct {
    time_t ts;
    float vpv;
    float ipv;
    float ppv;
    float vbat;
    float ibat;
    float pbat;
    float soc;
    float eff;
} hist_point_t;

#define MAX_OFFLINE_POINTS 240  // 240 điểm = 1 tiếng rớt mạng ở tần suất 15s/điểm

typedef struct {
    hist_point_t data[MAX_OFFLINE_POINTS];
    int head;  // Vị trí ghi tiếp theo
    int tail;  // Vị trí đọc tiếp theo
    int count; // Số điểm đang lưu trong đệm
} ring_buffer_t;

static ring_buffer_t g_offline_buf = { .head = 0, .tail = 0, .count = 0 };
static unsigned long last_flash_write_time = 0;
static bool is_uploading_offline = false; // Cờ chặn tạo nhiều task đồng bộ trùng lặp
// ============================================================
//   4. NVS — LƯU/ĐỌC CẤU HÌNH
// ============================================================
static nvs_handle_t g_nvs;
static void nvs_load_offline_history(void);
static bool wifi_creds_load(char *ssid_out, char *pass_out, size_t len);
static void nvs_load_all(void) {
    nvs_open("mppt", NVS_READWRITE, &g_nvs);
    int32_t v;
    if (nvs_get_i32(g_nvs, "chem", &v) == ESP_OK) g_chem = v; else g_chem = 3;
    if (nvs_get_i32(g_nvs, "volt", &v) == ESP_OK) g_volt_sel = v; else g_volt_sel = 0;
    if (nvs_get_i32(g_nvs, "imax", &v) == ESP_OK) g_imax = v; else g_imax = 50;
    if (nvs_get_i32(g_nvs, "chgEn", &v) == ESP_OK) g_charge_enable = (v!=0); else g_charge_enable = false;
    size_t sz = sizeof(float);
    float f;
    if (nvs_get_blob(g_nvs, "capAh", &f, &sz) == ESP_OK) g_capacity_ah = f; else g_capacity_ah = 100.0f;
    sz = sizeof(float);
    if (nvs_get_blob(g_nvs, "socAnchor", &f, &sz) == ESP_OK) soc_anchor_pct = f; else soc_anchor_pct = -1.0f;
    if (nvs_get_i32(g_nvs, "stY", &v) == ESP_OK) statYear = v; else statYear = 0;
    sz = sizeof(statCharge);
    nvs_get_blob(g_nvs, "stC", statCharge, &sz);
    sz = sizeof(statDischarge);
    nvs_get_blob(g_nvs, "stD", statDischarge, &sz);
    nvs_load_offline_history();
}

static void nvs_save_config(void) {
    nvs_set_i32(g_nvs, "chem", g_chem);
    nvs_set_i32(g_nvs, "volt", g_volt_sel);
    nvs_set_i32(g_nvs, "imax", g_imax);
    nvs_set_i32(g_nvs, "chgEn", g_charge_enable ? 1 : 0);
    nvs_set_blob(g_nvs, "capAh", (const void*)&g_capacity_ah, sizeof(float));
    nvs_commit(g_nvs);
}

static void nvs_save_stats(void) {
    nvs_set_i32(g_nvs, "stY", statYear);
    nvs_set_blob(g_nvs, "stC", statCharge, sizeof(statCharge));
    nvs_set_blob(g_nvs, "stD", statDischarge, sizeof(statDischarge));
    nvs_commit(g_nvs);
}

static void nvs_save_soc_anchor(void) {
    nvs_set_blob(g_nvs, "socAnchor", (const void*)&soc_anchor_pct, sizeof(float));
    nvs_commit(g_nvs);
}
// Backup Ring Buffer từ RAM xuống Flash
static void nvs_save_offline_history(void) {
    if (g_offline_buf.count == 0) return;
    nvs_set_blob(g_nvs, "offRingBuf", &g_offline_buf, sizeof(ring_buffer_t));
    nvs_commit(g_nvs);
    ESP_LOGW("OFFLINE", "Da backup %d diem vao Flash NVS.", g_offline_buf.count);
}

// Tải Ring Buffer từ Flash lên RAM khi khởi động
static void nvs_load_offline_history(void) {
    size_t sz = sizeof(ring_buffer_t);
    if (nvs_get_blob(g_nvs, "offRingBuf", &g_offline_buf, &sz) == ESP_OK) {
        ESP_LOGW("OFFLINE", "Nap lai %d diem offline tu Flash.", g_offline_buf.count);
    } else {
        g_offline_buf.head = 0; g_offline_buf.tail = 0; g_offline_buf.count = 0;
    }
}

// Xóa Flash sau khi đã đồng bộ Firebase xong
static void nvs_clear_offline_history(void) {
    nvs_erase_key(g_nvs, "offRingBuf");
    nvs_commit(g_nvs);
    g_offline_buf.head = 0; g_offline_buf.tail = 0; g_offline_buf.count = 0;
    ESP_LOGI("OFFLINE", "Da xoa sach bo nho dem tren Flash.");
}
// ============================================================
//   5. WIFI STATION
// ============================================================
static volatile bool g_wifi_connected = false;

static void wifi_event_handler(void* arg, esp_event_base_t base, int32_t id, void* data) {
    if (base == WIFI_EVENT && id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (base == WIFI_EVENT && id == WIFI_EVENT_STA_DISCONNECTED) {
        g_wifi_connected = false;
        wifi_event_sta_disconnected_t *d = (wifi_event_sta_disconnected_t*)data;
        ESP_LOGW(TAG, "WiFi rot ket noi, ly do = %d", d ? d->reason : -1);
        fb_client_reset();  // socket/TLS session cũ chắc chắn đã chết cùng WiFi -> dọn sạch, tránh dùng lại
        esp_wifi_connect();
    } else if (base == IP_EVENT && id == IP_EVENT_STA_GOT_IP) {
        g_wifi_connected = true;
        ESP_LOGI(TAG, "WiFi da ket noi, co IP.");
    }
}

static void wifi_init(void) {
    esp_netif_init();
    esp_event_loop_create_default();
    esp_netif_create_default_wifi_sta();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL);

    // Ưu tiên load credentials từ NVS (do WiFi Manager lưu sau khi cấu hình).
    // Nếu NVS chưa có (lần đầu boot), dùng credentials hardcoded.
    char nvs_ssid[33] = {0}, nvs_pass[65] = {0};
    bool has_nvs_creds = wifi_creds_load(nvs_ssid, nvs_pass, 64);

    wifi_config_t wc = { 0 };
    if (has_nvs_creds) {
        strncpy((char*)wc.sta.ssid, nvs_ssid, sizeof(wc.sta.ssid));
        strncpy((char*)wc.sta.password, nvs_pass, sizeof(wc.sta.password));
        ESP_LOGI(TAG, "WiFi: dung credentials tu NVS (ssid=%s)", nvs_ssid);
    } else {
        strncpy((char*)wc.sta.ssid, WIFI_SSID, sizeof(wc.sta.ssid));
        strncpy((char*)wc.sta.password, WIFI_PASSWORD, sizeof(wc.sta.password));
        ESP_LOGI(TAG, "WiFi: NVS chua co credentials, dung default (ssid=%s)", WIFI_SSID);
    }
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(WIFI_IF_STA, &wc);
    esp_wifi_start();
    esp_wifi_set_ps(WIFI_PS_NONE);
}

// ============================================================
//   5b. LED BÁO TRẠNG THÁI (LED đơn ON/OFF, GPIO23) + NÚT BOOT RECONNECT WIFI
//   ------------------------------------------------------------
//   LED không đổi màu được, nên trạng thái được mã hoá bằng SỐ LẦN CHỚP
//   trong một nhóm, sau đó nghỉ rồi lặp lại nhóm đó:
//     - LED_WIFI_LOST : chớp 1 lần → lặp lại (mất WiFi)
//     - LED_NTP_LOST  : chớp 2 lần → lặp lại (có WiFi nhưng chưa đồng bộ NTP)
//     - LED_UPDATING  : chớp 3 lần → lặp lại (đang cập nhật firmware OTA)
//     - LED_AP_MODE   : chớp 4 lần → lặp lại (đang ở chế độ Reset WiFi / AP mode)
//     - LED_ONLINE    : sáng liên tục (đã kết nối WiFi, hệ thống ổn định)
//   Giữa mỗi lần chớp trong 1 nhóm, đèn TẮT 2 giây để mắt người dễ đếm số lần.
// ============================================================

// Trạng thái hiển thị LED
typedef enum {
    LED_WIFI_LOST = 0,    // chớp 1 lần: mất WiFi
    LED_NTP_LOST,         // chớp 2 lần: có WiFi nhưng chưa đồng bộ NTP
    LED_UPDATING,         // chớp 3 lần: đang cập nhật firmware OTA
    LED_AP_MODE,          // chớp 4 lần: đang ở chế độ Reset WiFi (AP mode)
    LED_ONLINE            // sáng liên tục: WiFi + Firebase OK, ổn định
} led_mode_t;

static volatile led_mode_t g_led_mode = LED_WIFI_LOST;
static volatile bool g_force_reconnect = false;
static volatile bool g_ota_in_progress = false;
// Kết quả THẬT của lần gọi esp_ota_mark_app_valid_cancel_rollback() lúc boot,
// để đưa vào node chẩn đoán /mppt/ota/build — không đoán suông, đọc lại đúng
// esp_err_t trả về và trạng thái phân vùng SAU khi gọi, xem có chuyển sang
// VALID thật hay không.
static esp_err_t g_ota_mark_err = ESP_ERR_NOT_FINISHED; // NOT_FINISHED = "chưa từng chạy hàm này"
static esp_ota_img_states_t g_ota_state_after_mark = ESP_OTA_IMG_UNDEFINED;
// CÓ THỰC SỰ phải xác nhận hay không. Phân biệt 2 chuyện rất dễ nhầm:
//   - wasPending = true  : ảnh này VỪA ĐƯỢC OTA, cần xác nhận -> con số
//                          markValidWorked mới có ý nghĩa.
//   - wasPending = false : ảnh này nạp bằng USB (luôn VALID sẵn) -> lúc này
//                          markValidWorked=true KHÔNG chứng minh được điều gì
//                          về ảnh vừa OTA cả. Bản trước thiếu cờ này nên nhìn
//                          "markValidWorked: true" rất dễ tưởng nhầm là ảnh mới
//                          đã tự xác nhận thành công.
static bool g_ota_was_pending_at_boot = false;
// Version ĐÍCH của lần cài đang diễn ra (bản sắp thay thế). 0 = không cài gì.
static int g_ota_target_ver = 0;
static volatile bool g_start_ap_mode   = false;   // true khi admin/boot yêu cầu reset WiFi

// ---- Cực LED: đổi thành 0 nếu LED của bạn sáng khi GPIO ở mức CAO (active-high) ----
// LED xanh dương của bạn sáng khi GPIO ở mức THẤP -> để 1 (active-low)
#define LED_ACTIVE_LOW   1
#if LED_ACTIVE_LOW
    #define LED_ON_LVL   0
    #define LED_OFF_LVL  1
#else
    #define LED_ON_LVL   1
    #define LED_OFF_LVL  0
#endif

#define LED_BLINK_ON_MS    150     // độ dài LED sáng trong mỗi lần chớp
#define LED_BLINK_OFF_MS   1000    // độ dài LED tắt giữa mỗi lần chớp (2s, dễ đếm)
#define LED_GROUP_PAUSE_MS 10000    // nghỉ thêm sau khi chớp đủ số lần, trước khi lặp lại nhóm

static void led_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_LED_STATUS),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
    gpio_set_level(PIN_LED_STATUS, LED_OFF_LVL);
}

// Chớp LED "n" lần (sáng 150ms / tắt 1s giữa mỗi lần), rồi nghỉ thêm "pause_ms"
// trước khi kết thúc nhóm. Kiểm tra g_led_mode thường xuyên để thoát sớm và
// phản hồi ngay nếu trạng thái đổi giữa chừng.
static bool led_blink_group(int n, uint32_t pause_ms, led_mode_t mode_when_called) {
    for (int i = 0; i < n; i++) {
        if (g_led_mode != mode_when_called) return false;
        gpio_set_level(PIN_LED_STATUS, LED_ON_LVL);
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_ON_MS));
        if (g_led_mode != mode_when_called) return false;
        gpio_set_level(PIN_LED_STATUS, LED_OFF_LVL);
        vTaskDelay(pdMS_TO_TICKS(LED_BLINK_OFF_MS));
    }
    uint32_t waited = 0;
    while (waited < pause_ms) {
        if (g_led_mode != mode_when_called) return false;
        vTaskDelay(pdMS_TO_TICKS(100));
        waited += 100;
    }
    return true;
}

// Task riêng điều khiển LED theo trạng thái (không chặn vòng lặp chính)
static void led_task(void *arg) {
    for (;;) {
        switch (g_led_mode) {
            case LED_ONLINE:  // sáng liên tục — đã kết nối, hệ thống ổn định
                gpio_set_level(PIN_LED_STATUS, LED_ON_LVL);
                vTaskDelay(pdMS_TO_TICKS(300));
                break;
            case LED_WIFI_LOST:  // chớp 1 lần — mất WiFi
                led_blink_group(1, LED_GROUP_PAUSE_MS, LED_WIFI_LOST);
                break;
            case LED_NTP_LOST:  // chớp 2 lần — có WiFi nhưng chưa đồng bộ NTP
                led_blink_group(2, LED_GROUP_PAUSE_MS, LED_NTP_LOST);
                break;
            case LED_UPDATING:  // chớp 3 lần — đang cập nhật firmware OTA
                led_blink_group(3, LED_GROUP_PAUSE_MS, LED_UPDATING);
                break;
            case LED_AP_MODE:  // chớp 4 lần — đang ở chế độ Reset WiFi (AP mode)
                led_blink_group(4, LED_GROUP_PAUSE_MS, LED_AP_MODE);
                break;
        }
    }
}

// ---- Nút BOOT: giữ 5s để buộc reconnect WiFi ----
static void boot_btn_init(void) {
    gpio_config_t io = {
        .pin_bit_mask = (1ULL << PIN_BOOT_BTN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,   // nhấn = kéo xuống LOW
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io);
}

// Task theo dõi nút BOOT:
// - Nhấn ngắn (< 1s): không làm gì
// - Giữ 5 giây: vào AP mode (WiFi Manager) để cấu hình lại WiFi
static void boot_btn_task(void *arg) {
    unsigned long press_start = 0;
    unsigned long last_log = 0;
    bool counted = false;
    for (;;) {
        int level = gpio_get_level(PIN_BOOT_BTN);  // 0 = đang nhấn
        if (level == 0) {
            unsigned long nowms = (unsigned long)(esp_timer_get_time()/1000ULL);
            if (press_start == 0) {
                press_start = nowms;
                counted = false;
                last_log = nowms;
                ESP_LOGI(TAG, "Nut BOOT: phat hien nhan xuong (GPIO%d = LOW)...", PIN_BOOT_BTN);
            } else if (!counted && (nowms - last_log >= 1000)) {
                last_log = nowms;
                ESP_LOGI(TAG, "Nut BOOT: dang giu... %lu ms / %d ms", nowms - press_start, BOOT_HOLD_MS);
            }
            if (!counted && (nowms - press_start >= BOOT_HOLD_MS)) {
                counted = true;
                g_start_ap_mode = true;   // vào AP mode để cấu hình WiFi mới
                ESP_LOGW(TAG, "Nut BOOT giu 5s -> Vao WiFi Manager AP Mode!");
            }
        } else {
            if (press_start != 0) {
                ESP_LOGI(TAG, "Nut BOOT: da tha (GPIO%d = HIGH)", PIN_BOOT_BTN);
            }
            press_start = 0;
            counted = false;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Thực hiện reconnect WiFi (dùng khi wifiReconnect flag)
static void do_wifi_reconnect(void) {
    ESP_LOGW(TAG, "Dang ngat va ket noi lai WiFi...");
    g_led_mode = LED_WIFI_LOST;  // coi như mất WiFi trong lúc đang thử kết nối lại
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_wifi_connect();
}

// ============================================================
//   5d. WIFI MANAGER — AP MODE + HTTP CONFIG SERVER
//   Khi admin bấm "Reset WiFi" hoặc người dùng giữ BOOT 5 giây:
//   1. Xóa credentials NVS
//   2. Chuyển sang AP mode, phát mạng "Bo sac MPPT_Cap Nhat WiFi"
//   3. HTTP server phục vụ trang cấu hình tại 192.168.4.1
//   4. Người dùng nhập SSID/Pass mới → lưu NVS → restart
// ============================================================

// --- NVS helpers ---
static void wifi_creds_save(const char *ssid, const char *pass) {
    nvs_handle_t h;
    if (nvs_open(NVS_WIFI_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_set_str(h, NVS_KEY_SSID, ssid);
        nvs_set_str(h, NVS_KEY_PASS, pass);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "WiFi credentials saved: ssid=%s", ssid);
    }
}

static bool wifi_creds_load(char *ssid_out, char *pass_out, size_t len) {
    nvs_handle_t h;
    bool ok = false;
    if (nvs_open(NVS_WIFI_NS, NVS_READONLY, &h) == ESP_OK) {
        size_t s = len;
        bool got_ssid = (nvs_get_str(h, NVS_KEY_SSID, ssid_out, &s) == ESP_OK && strlen(ssid_out) > 0);
        s = len;
        bool got_pass = (nvs_get_str(h, NVS_KEY_PASS, pass_out, &s) == ESP_OK);
        ok = got_ssid && got_pass;
        nvs_close(h);
    }
    return ok;
}

static void wifi_creds_erase(void) {
    nvs_handle_t h;
    if (nvs_open(NVS_WIFI_NS, NVS_READWRITE, &h) == ESP_OK) {
        nvs_erase_key(h, NVS_KEY_SSID);
        nvs_erase_key(h, NVS_KEY_PASS);
        nvs_commit(h);
        nvs_close(h);
        ESP_LOGW(TAG, "WiFi credentials erased from NVS");
    }
}

// --- HTML trang cấu hình WiFi (nhúng vào firmware) ---
static const char WIFI_CFG_HTML[] =
"<!DOCTYPE html>"
"<html lang='vi'><head><meta charset='UTF-8'>"
"<meta name='viewport' content='width=device-width,initial-scale=1'>"
"<title>Cap Nhat WiFi - Bo Sac MPPT</title>"
"<style>"
"*{box-sizing:border-box;margin:0;padding:0}"
"body{font-family:sans-serif;background:#0f172a;color:#e2e8f0;"
"min-height:100vh;display:flex;align-items:center;justify-content:center;padding:16px}"
".card{background:#1e293b;border:1px solid #334155;border-radius:16px;padding:32px 28px;"
"max-width:400px;width:100%;box-shadow:0 8px 32px rgba(0,0,0,.5)}"
".logo{font-size:40px;text-align:center;margin-bottom:12px}"
"h2{font-size:18px;font-weight:700;text-align:center;margin-bottom:6px}"
".sub{font-size:13px;color:#64748b;text-align:center;margin-bottom:24px;line-height:1.5}"
"label{display:block;font-size:12px;font-weight:600;color:#64748b;margin-bottom:5px;"
"text-transform:uppercase;letter-spacing:.4px}"
"input{width:100%;padding:11px 14px;background:#0f172a;border:1.5px solid #334155;"
"border-radius:8px;color:#e2e8f0;font-size:14px;margin-bottom:14px}"
"input:focus{outline:none;border-color:#6366f1}"
".btn{width:100%;padding:13px;background:linear-gradient(135deg,#6366f1,#8b5cf6);"
"color:#fff;border:none;border-radius:10px;font-size:15px;font-weight:700;cursor:pointer}"
".note{font-size:12px;color:#64748b;text-align:center;margin-top:14px;line-height:1.5}"
"</style></head>"
"<body><div class='card'>"
"<div class='logo'>📡</div>"
"<h2>Cập nhật WiFi — Bộ sạc MPPT</h2>"
"<div class='sub'>Nhập thông tin mạng WiFi mới để bộ sạc kết nối lại.</div>"
"<form method='POST' action='/save'>"
"<label>Tên WiFi (SSID)</label>"
"<input name='ssid' placeholder='Tên mạng WiFi...' required maxlength='32'>"
"<label>Mật khẩu WiFi</label>"
"<input name='pass' type='password' placeholder='Để trống nếu không có mật khẩu' maxlength='64'>"
"<button class='btn' type='submit'>💾 Lưu và kết nối lại</button>"
"</form>"
"<div class='note'>⚙️ Sau khi nhấn Lưu, bộ sạc sẽ tự khởi động lại<br>và kết nối vào WiFi mới trong vài giây.</div>"
"</div></body></html>";

static const char WIFI_CFG_OK_HTML[] =
"<!DOCTYPE html><html lang='vi'><head><meta charset='UTF-8'>"
"<title>Da luu WiFi</title>"
"<style>body{font-family:sans-serif;background:#0f172a;color:#e2e8f0;"
"display:flex;align-items:center;justify-content:center;min-height:100vh;padding:16px}"
".card{background:#1e293b;border-radius:16px;padding:32px 28px;max-width:360px;"
"width:100%;text-align:center}"
".icon{font-size:56px;margin-bottom:16px}"
"h2{color:#22c55e;margin-bottom:10px}"
"p{color:#64748b;font-size:14px;line-height:1.6}"
"</style></head>"
"<body><div class='card'>"
"<div class='icon'>✅</div>"
"<h2>Đã lưu thành công!</h2>"
"<p>Bộ sạc đang khởi động lại và kết nối vào WiFi mới.<br><br>"
"Vui lòng kết nối lại điện thoại vào mạng WiFi nhà bạn<br>và mở ứng dụng MPPT như bình thường.</p>"
"</div></body></html>";

// --- HTTP handlers ---

// Phục vụ trang cấu hình WiFi chính
static esp_err_t http_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html; charset=UTF-8");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache, no-store, must-revalidate");
    httpd_resp_send(req, WIFI_CFG_HTML, HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

// Captive portal redirect — dành cho iOS và Android
// iOS probe /hotspot-detect.html, Android probe /generate_204
// Trả 302 redirect về trang cấu hình → iOS/Android tự mở popup captive portal
static esp_err_t http_captive_redirect(httpd_req_t *req) {
    httpd_resp_set_status(req, "302 Found");
    httpd_resp_set_hdr(req, "Location", "http://192.168.4.1/");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_send(req, NULL, 0);
    return ESP_OK;
}

static esp_err_t http_post_handler(httpd_req_t *req) {
    char body[200] = {0};
    int len = req->content_len < (int)sizeof(body)-1 ? req->content_len : (int)sizeof(body)-1;
    httpd_req_recv(req, body, len);

    // Parse ssid= và pass= từ URL-encoded form
    char ssid[33] = {0}, pass[65] = {0};
    // Tìm ssid=...
    char *p = strstr(body, "ssid=");
    if (p) {
        p += 5;
        char *end = strchr(p, '&'); if (!end) end = p + strlen(p);
        int n = end - p < 32 ? end - p : 32;
        strncpy(ssid, p, n);
    }
    // Tìm pass=...
    p = strstr(body, "pass=");
    if (p) {
        p += 5;
        char *end = strchr(p, '&'); if (!end) end = p + strlen(p);
        int n = end - p < 64 ? end - p : 64;
        strncpy(pass, p, n);
    }

    // URL decode đơn giản: thay '+' thành ' ' và %XX
    for (int i = 0; ssid[i]; i++) { if (ssid[i]=='+') ssid[i]=' '; }
    for (int i = 0; pass[i]; i++) { if (pass[i]=='+') pass[i]=' '; }

    ESP_LOGI(TAG, "WiFi Manager: SSID='%s'", ssid);

    // Lưu credentials và gửi trang xác nhận
    httpd_resp_set_type(req, "text/html; charset=UTF-8");
    if (strlen(ssid) > 0) {
        wifi_creds_save(ssid, pass);
        httpd_resp_send(req, WIFI_CFG_OK_HTML, HTTPD_RESP_USE_STRLEN);
        // Restart sau 2 giây để trả response về browser trước
        vTaskDelay(pdMS_TO_TICKS(2000));
        esp_restart();
    } else {
        httpd_resp_send(req, WIFI_CFG_HTML, HTTPD_RESP_USE_STRLEN);
    }
    return ESP_OK;
}

// --- Khởi động AP mode + HTTP server ---
static void start_wifi_manager(void) {
    ESP_LOGW(TAG, "=== WIFI MANAGER: Khoi dong AP Mode ===");
    g_led_mode = LED_AP_MODE;
    g_ota_in_progress = true;  // dừng sạc an toàn trong lúc cấu hình

    // Dừng STA mode
    esp_wifi_disconnect();
    vTaskDelay(pdMS_TO_TICKS(500));
    esp_wifi_stop();
    vTaskDelay(pdMS_TO_TICKS(300));

    // Tạo AP netif (BẮT BUỘC — nếu thiếu, AP khởi động nhưng không phát được mạng WiFi)
    esp_netif_create_default_wifi_ap();

    esp_wifi_set_mode(WIFI_MODE_AP);

    wifi_config_t ap_cfg = {
        .ap = {
            .ssid_len = 0,
            .channel  = 6,
            .authmode = WIFI_AUTH_OPEN,   // mở, không cần mật khẩu
            .max_connection = 4,
        }
    };
    strncpy((char*)ap_cfg.ap.ssid, WIFI_AP_SSID, sizeof(ap_cfg.ap.ssid)-1);

    esp_wifi_set_config(WIFI_IF_AP, &ap_cfg);
    esp_wifi_start();
    ESP_LOGI(TAG, "AP Mode ON: SSID='%s', IP=192.168.4.1", WIFI_AP_SSID);

    // Khởi động HTTP server
    httpd_config_t hcfg = HTTPD_DEFAULT_CONFIG();
    hcfg.server_port      = 80;
    hcfg.max_uri_handlers = 12;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &hcfg) == ESP_OK) {
        // ── Trang cấu hình chính ──
        httpd_uri_t h_root = { .uri="/",     .method=HTTP_GET,  .handler=http_get_handler,      .user_ctx=NULL };
        httpd_uri_t h_save = { .uri="/save", .method=HTTP_POST, .handler=http_post_handler,     .user_ctx=NULL };
        httpd_register_uri_handler(server, &h_root);
        httpd_register_uri_handler(server, &h_save);

        // ── Captive portal iOS ──
        // Khi iPhone kết nối AP, nó probe /hotspot-detect.html
        // ESP32 trả 302 → iOS hiện popup "Đăng nhập vào mạng" → tap vào tự mở trang cấu hình
        httpd_uri_t h_ios1 = { .uri="/hotspot-detect.html",        .method=HTTP_GET, .handler=http_captive_redirect, .user_ctx=NULL };
        httpd_uri_t h_ios2 = { .uri="/library/test/success.html",  .method=HTTP_GET, .handler=http_captive_redirect, .user_ctx=NULL };
        httpd_uri_t h_ios3 = { .uri="/bag",                        .method=HTTP_GET, .handler=http_captive_redirect, .user_ctx=NULL };
        httpd_register_uri_handler(server, &h_ios1);
        httpd_register_uri_handler(server, &h_ios2);
        httpd_register_uri_handler(server, &h_ios3);

        // ── Captive portal Android ──
        httpd_uri_t h_and1 = { .uri="/generate_204",      .method=HTTP_GET, .handler=http_captive_redirect, .user_ctx=NULL };
        httpd_uri_t h_and2 = { .uri="/connecttest.txt",   .method=HTTP_GET, .handler=http_captive_redirect, .user_ctx=NULL };
        httpd_uri_t h_and3 = { .uri="/ncsi.txt",          .method=HTTP_GET, .handler=http_captive_redirect, .user_ctx=NULL };
        httpd_register_uri_handler(server, &h_and1);
        httpd_register_uri_handler(server, &h_and2);
        httpd_register_uri_handler(server, &h_and3);

        ESP_LOGI(TAG, "HTTP server started → Captive portal ON (iOS + Android)");
    } else {
        ESP_LOGE(TAG, "Khoi dong HTTP server that bai!");
    }

    // Giữ AP mode, đợi người dùng cấu hình (vòng lặp vô hạn)
    for (;;) { vTaskDelay(pdMS_TO_TICKS(5000)); }
}

// ============================================================
//   5c. OTA — CẬP NHẬT FIRMWARE QUA HTTPS (từ Firebase Storage)
// ============================================================
// Khai báo trước hàm fb_put (định nghĩa ở phần Firebase bên dưới)
static void fb_put(const char *path, const char *json);

// Báo trạng thái OTA ngược lên Firebase để web hiển thị
// LƯU Ý VỀ Ý NGHĨA CỦA fwVer (chỗ này rất hay bị hiểu nhầm):
//   fwVer = phiên bản của FIRMWARE ĐANG CHẠY VÀ ĐANG GHI DÒNG BÁO CÁO NÀY.
// Toàn bộ quá trình tải + ghi flash đều do BẢN CŨ thực hiện (bản mới lúc đó
// mới chỉ nằm im trong flash, chưa chạy). Nên trong suốt lúc cập nhật, kể cả
// khi state="success", fwVer VẪN PHẢI LÀ 1 — đó là hành vi ĐÚNG, không phải lỗi.
// Nó chỉ đổi thành 2 ở lần boot SAU, khi bản mới thực sự chạy và tự báo cáo.
// Thêm targetVer để nhìn phát biết ngay "đang cài lên bản mấy".
static void ota_report(const char *state, int progress) {
    char j[192];
    snprintf(j, sizeof(j),
             "{\"state\":\"%s\",\"progress\":%d,\"fwVer\":%d,"
             "\"runningVer\":%d,\"targetVer\":%d}",
             state, progress, FIRMWARE_VERSION, FIRMWARE_VERSION, g_ota_target_ver);
    fb_put(dev_path("/mppt/ota/status"), j);
}

// Reset CỨNG bằng mạch ngoài (GPIO2) để có một lần bật nguồn thật sự.
// Trình tự đúng bằng máy trạng thái xung reset ở vòng lặp chính: giữ HIGH 3s,
// hạ LOW để kích sườn xuống, rồi chờ mạch ngoài cắt nguồn.
// Nếu bo không gắn mạch reset ngoài (hoặc mạch không tác động) thì sau 12 giây
// vẫn còn sống -> quay về esp_restart() dự phòng để chắc chắn vào firmware mới.
static void ota_hard_reset_after_update(void) {
    ESP_LOGW(TAG, "Kich mach reset NGOAI de vao firmware moi bang mot lan bat nguon that su...");
    gpio_set_level(PIN_EXT_RESET_OUT, 1);
    vTaskDelay(pdMS_TO_TICKS(3000));
    gpio_set_level(PIN_EXT_RESET_OUT, 0);   // sườn xuống = lệnh cắt nguồn
    ESP_LOGW(TAG, "Da ha GPIO2 ve LOW, cho mach ngoai cat nguon...");
    vTaskDelay(pdMS_TO_TICKS(12000));
    ESP_LOGE(TAG, "Mach reset ngoai KHONG tac dong sau 12s -> dung esp_restart() du phong");
    esp_restart();
}

// Thực hiện OTA từ 1 URL .bin. Trả về true nếu thành công (sẽ reboot ngay sau đó).
static bool do_ota_update(const char *url) {
    // CHỐNG CACHE CDN: raw.githubusercontent.com giữ bản cũ trong bộ nhớ đệm
    // tới ~5 phút sau khi push file mới. Vừa upload xong mà OTA ngay thì rất dễ
    // tải trúng bản CŨ -> cài "thành công" nhưng version không đổi.
    // Thêm tham số ?t=<giây> làm khoá cache khác nhau mỗi lần -> luôn lấy bản mới.
    char url_nc[440];
    if (strchr(url, '?') == NULL) {
        snprintf(url_nc, sizeof(url_nc), "%s?t=%ld", url, (long)time(NULL));
        url = url_nc;
    }
    ESP_LOGW(TAG, "=== BAT DAU OTA tu URL: %s ===", url);

    // 1) Tạm dừng sạc để an toàn: bật cờ, chờ mppt_task nhả PWM về 0
    g_ota_in_progress = true;
    g_led_mode = LED_UPDATING;
    vTaskDelay(pdMS_TO_TICKS(500));   // cho mppt_task 1 nhịp để hạ PWM
    ota_report("downloading", 0);

    esp_http_client_config_t http_cfg = {
        .url = url,
        .crt_bundle_attach = esp_crt_bundle_attach,  // dùng bundle CA như phần Firebase
        .timeout_ms = 20000,
        .keep_alive_enable = true,

        // TĂNG buffer_size (2048 -> 8192): link .bin từ GitHub Releases luôn
        // CHUYỂN HƯỚNG sang objects.githubusercontent.com kèm dòng "Location"
        // rất dài (chữ ký AWS). buffer nhỏ làm dòng đó bị cắt cụt -> tải nhầm
        // trang lỗi -> lỗi "Mismatch chip id... found 65535" (đọc trúng vùng
        // flash trống). Dùng link raw.githubusercontent.com (không redirect)
        // thì an toàn hơn nữa, nhưng vẫn để buffer lớn cho chắc.
        .buffer_size = 8192,
        .buffer_size_tx = 1024,  // Tăng bộ đệm gửi Tx lên 1KB
    };
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };

    esp_https_ota_handle_t ota_handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_cfg, &ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_begin that bai: %s", esp_err_to_name(err));
        ota_report("failed", 0);
        g_ota_in_progress = false;
        return false;
    }

    // ★ 1b) ĐỌC "CHỨNG MINH THƯ" CỦA ẢNH SẮP CÀI, TRƯỚC KHI GHI ĐÈ.
    // esp_https_ota_get_img_desc() lấy được esp_app_desc_t nằm ở đầu file .bin
    // đang tải (tên project, chuỗi version, NGÀY + GIỜ BIÊN DỊCH).
    // Mục đích: nếu file trên GitHub thực chất VẪN LÀ BẢN CŨ (quên build lại,
    // quên push, hoặc CDN raw.githubusercontent.com còn trả bản cache cũ ~5
    // phút), thì ngày giờ biên dịch sẽ TRÙNG KHÍT với bản đang chạy. Khi đó
    // OTA vẫn "thành công" nhưng version đứng yên — đúng hiện tượng khó hiểu
    // nhất. Bắt được ở đây thì báo lỗi rõ ràng thay vì cài rồi ngơ ngác.
    {
        esp_app_desc_t nd;
        if (esp_https_ota_get_img_desc(ota_handle, &nd) == ESP_OK) {
            const esp_app_desc_t *cd = esp_app_get_description();
            static char ji[300];
            snprintf(ji, sizeof(ji), "{\"appVer\":\"%s\",\"built\":\"%s %s\"}",
                     nd.version, nd.date, nd.time);
            fb_put(dev_path("/mppt/ota/incoming"), ji);

            // Lưu RIÊNG ngày-giờ-biên-dịch của ảnh sắp cài để lúc boot lại so
            // sánh NGUYÊN VĂN với ảnh đang chạy — xem mục "KẾT LUẬN LẦN CÀI
            // TRƯỚC" ở app_main để biết vì sao cần tách riêng field này.
            static char jpb[64];
            snprintf(jpb, sizeof(jpb), "\"%s %s\"", nd.date, nd.time);
            fb_put(dev_path("/mppt/ota/pendingBuilt"), jpb);
            ESP_LOGW(TAG, "Anh firmware SAP CAI: ver=%s built=%s %s", nd.version, nd.date, nd.time);
            ESP_LOGW(TAG, "Anh firmware DANG CHAY: ver=%s built=%s %s",
                     cd ? cd->version : "?", cd ? cd->date : "?", cd ? cd->time : "?");

            if (cd && strcmp(cd->date, nd.date) == 0 && strcmp(cd->time, nd.time) == 0) {
                ESP_LOGE(TAG, "File .bin tren server GIONG HET ban dang chay "
                              "(cung ngay gio bien dich) -> HUY OTA, khong cai lai vo ich.");
                esp_https_ota_abort(ota_handle);
                fb_put(dev_path("/mppt/ota/lastResult"), "\"same_build\"");
                fb_put(dev_path("/mppt/ota/pendingVersion"), "0");
                ota_report("failed", 0);
                g_ota_in_progress = false;
                return false;
            }
        }
    }

    // 2) Tải và ghi từng khối; báo tiến độ %
    int total = esp_https_ota_get_image_size(ota_handle);
    int last_pct = -1;
    while (1) {
        err = esp_https_ota_perform(ota_handle);
        if (err != ESP_ERR_HTTPS_OTA_IN_PROGRESS) break;
        int done = esp_https_ota_get_image_len_read(ota_handle);
        if (total > 0) {
            int pct = (int)(100.0 * done / total);
            if (pct != last_pct && pct % 10 == 0) {   // báo mỗi 10%
                last_pct = pct;
                ESP_LOGI(TAG, "OTA tien do: %d%%", pct);
                ota_report("downloading", pct);
            }
        }
    }

    // 3) Kiểm tra đã nhận đủ dữ liệu chưa
    if (!esp_https_ota_is_complete_data_received(ota_handle)) {
        ESP_LOGE(TAG, "OTA: du lieu nhan khong day du");
        esp_https_ota_abort(ota_handle);
        ota_report("failed", 0);
        g_ota_in_progress = false;
        return false;
    }

    err = esp_https_ota_finish(ota_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_https_ota_finish that bai: %s", esp_err_to_name(err));
        ota_report("failed", 0);
        g_ota_in_progress = false;
        return false;
    }

    ESP_LOGW(TAG, "=== OTA THANH CONG - khoi dong lai vao firmware moi ===");
    ota_report("success", 100);
    vTaskDelay(pdMS_TO_TICKS(1000));   // cho lệnh report kịp gửi lên Firebase

    // ★★★ KHÔNG DÙNG esp_restart() NỮA ★★★
    //
    // Manh mối quyết định: CÙNG file .bin đó, nạp thẳng qua USB thì chạy tốt,
    // nhưng vào bằng OTA thì chết (resetReason=7, INT_WDT). Hai đường đó khác
    // nhau đúng một điểm về phần cứng: nạp USB kết thúc bằng một lần BẬT NGUỒN
    // THẬT SỰ (power-on reset, toàn bộ chip kể cả khối RTC/LP về mặc định),
    // còn esp_restart() chỉ là RESET MỀM (rst:0xc SW_CPU) — khối RTC/LP, một số
    // thanh ghi và trạng thái ngoại vi vẫn giữ nguyên từ phiên trước.
    //
    // Ở đây phiên trước vừa chạy WiFi + TLS + ghi flash liên tục rồi mới restart,
    // nên trạng thái để lại càng "bẩn". Vì vậy: dùng chính mạch reset ngoài sẵn
    // có (GPIO2) để CẮT NGUỒN thật, tạo ra lần khởi động sạch y hệt khi nạp USB.
    ota_hard_reset_after_update();
    return true;  // không bao giờ tới đây
}

// ============================================================
//  ★★★ SỬA LỖI GỐC: "OTA báo thành công nhưng máy vẫn chạy bản cũ" ★★★
//
//  ESP-IDF bật CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE thì ảnh firmware vừa OTA
//  xong sẽ khởi động ở trạng thái ESP_OTA_IMG_PENDING_VERIFY. Nếu thiết bị
//  KHỞI ĐỘNG LẠI LẦN NỮA trước khi gọi esp_ota_mark_app_valid_cancel_rollback(),
//  bootloader coi bản mới là hỏng và TỰ QUAY LUI (rollback) về bản cũ VĨNH VIỄN.
//
//  Bản cũ chỉ gọi hàm xác nhận này SAU KHI: có WiFi + có NTP + chạy tiếp 15 giây.
//  Mà ngay trong app_main, nếu WiFi không lên trong 15s HOẶC NTP hỏng 3 lần thì
//  request_external_reset() sẽ đá GPIO2 -> mạch ngoài cắt nguồn -> reboot.
//  Reboot đó xảy ra TRƯỚC lúc xác nhận => rollback về v1. Vòng lặp: cài v2 xong,
//  báo "success", khởi động lại, rớt về v1, Firebase ghi currentVersion=1,
//  status.fwVer=1 — ĐÚNG như hiện tượng đang thấy trên bo 3.
//
//  Cách sửa: xác nhận NGAY ở đầu app_main (thao tác cục bộ trên flash, không cần
//  mạng). Firmware đã tự bảo vệ bằng cơ chế reset ngoài + watchdog riêng rồi,
//  không cần dựa vào rollback của bootloader.
// ============================================================
// ============================================================
//  HỘP ĐEN CHẨN ĐOÁN KHỞI ĐỘNG ("breadcrumb" lưu trong NVS)
// ============================================================
//  Vì sao cần: khi ảnh vừa OTA chết lúc khởi động, cổng USB-Serial-JTAG rớt
//  theo (ClearCommError) nên KHÔNG THỂ đọc được log của nó — mà đó lại đúng là
//  đoạn log cần nhất. Không phải ai cũng có sẵn mạch USB-TTL để cắm vào UART0.
//
//  Cách làm: mỗi mốc khởi động, firmware ghi 1 con số vào NVS (tồn tại qua mọi
//  kiểu reset, kể cả rollback). Ảnh nào boot lên sau — kể cả ảnh CŨ sau khi
//  bootloader quay lui — đều đọc lại được dấu chân cuối cùng mà ảnh trước để
//  lại, rồi đẩy lên Firebase. Nhìn con số đó là biết ảnh mới chết ở BƯỚC NÀO.
//
//  Ghi kèm ngày-giờ-biên-dịch của ảnh ĐÃ GHI dấu chân, để phân biệt chắc chắn
//  dấu chân đó do ảnh MỚI hay ảnh CŨ để lại (2 ảnh khác build khác nhau).
//
//  Chi phí flash: ~9 lần ghi NVS mỗi lần khởi động, không đáng kể.
// ============================================================
#define DIAG_NS "otadiag"
typedef enum {
    BOOT_STEP_NVS_READY      = 1,  // đã qua nvs_flash_init + nvs_load_all
    BOOT_STEP_MARK_VALID     = 2,  // đã qua ota_mark_valid_now
    BOOT_STEP_PERIPH         = 3,  // đã qua adc_setup + pwm_setup
    BOOT_STEP_TASKS          = 4,  // đã tạo xong led/bootbtn/mppt task
    BOOT_STEP_WIFI_INIT      = 5,  // đã qua wifi_init + device_id_init
    BOOT_STEP_WIFI_UP        = 6,  // WiFi đã kết nối
    BOOT_STEP_NTP_OK         = 7,  // NTP đồng bộ xong
    BOOT_STEP_FB_REPORTED    = 8,  // đã đẩy được dữ liệu lên Firebase
    BOOT_STEP_MAIN_LOOP      = 9,  // đã vào vòng lặp chính, coi như sống ổn
} boot_step_t;

static nvs_handle_t g_diag_h = 0;
// Dấu chân của LẦN BOOT TRƯỚC (đọc ra trước khi ghi đè bằng dấu chân của mình)
static int  g_prev_step = 0;
static int  g_prev_fw   = 0;
static char g_prev_built[40] = "-";

static void diag_mark(boot_step_t step) {
    if (!g_diag_h) return;
    nvs_set_u8(g_diag_h, "step", (uint8_t)step);
    nvs_commit(g_diag_h);
}

// Gọi NGAY sau nvs_flash_init(): đọc dấu chân cũ rồi đặt dấu chân mới.
static void diag_begin(void) {
    if (nvs_open(DIAG_NS, NVS_READWRITE, &g_diag_h) != ESP_OK) {
        g_diag_h = 0;
        return;
    }
    uint8_t st = 0, fw = 0;
    if (nvs_get_u8(g_diag_h, "step", &st) == ESP_OK) g_prev_step = st;
    if (nvs_get_u8(g_diag_h, "fw",   &fw) == ESP_OK) g_prev_fw   = fw;
    size_t len = sizeof(g_prev_built);
    if (nvs_get_str(g_diag_h, "built", g_prev_built, &len) != ESP_OK) {
        strcpy(g_prev_built, "-");
    }

    // Ghi "chứng minh thư" của ảnh ĐANG chạy để lần sau biết dấu chân của ai
    const esp_app_desc_t *d = esp_app_get_description();
    char mine[40];
    snprintf(mine, sizeof(mine), "%s %s", d ? d->date : "?", d ? d->time : "?");
    nvs_set_str(g_diag_h, "built", mine);
    nvs_set_u8(g_diag_h, "fw", (uint8_t)FIRMWARE_VERSION);
    nvs_set_u8(g_diag_h, "step", (uint8_t)BOOT_STEP_NVS_READY);
    nvs_commit(g_diag_h);

    ESP_LOGW(TAG, "HOP DEN: lan boot truoc dung o BUOC %d (fw v%d, built %s)",
             g_prev_step, g_prev_fw, g_prev_built);
}

static void ota_mark_valid_now(void) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (!running) return;
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(running, &st) != ESP_OK) return;
    if (st == ESP_OTA_IMG_PENDING_VERIFY) {
        g_ota_was_pending_at_boot = true;
        g_ota_mark_err = esp_ota_mark_app_valid_cancel_rollback();
        // ĐỌC LẠI trạng thái NGAY SAU khi gọi — không tin suông vào esp_err_t,
        // vì (theo đúng lỗi đang gặp) có khả năng lệnh gọi "coi như chạy" mà
        // trạng thái KHÔNG THỰC SỰ chuyển sang VALID (ví dụ do sdkconfig của
        // ảnh vừa OTA không bật CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE giống ảnh
        // đang biên dịch ở đây, hoặc do phân vùng otadata bị bootloader ghi đè
        // lại ngay sau đó). Đọc lại là cách DUY NHẤT biết chắc có ăn hay không.
        esp_ota_get_state_partition(running, &g_ota_state_after_mark);
        ESP_LOGW(TAG, "Firmware moi (partition %s) -> XAC NHAN HOP LE ngay luc boot: %s "
                      "(trang thai sau khi xac nhan = %d, %s)",
                 running->label, esp_err_to_name(g_ota_mark_err),
                 (int)g_ota_state_after_mark,
                 g_ota_state_after_mark == ESP_OTA_IMG_VALID ? "DA THANH VALID - AN TOAN"
                                                              : "VAN CHUA VALID - VAN CO NGUY CO ROLLBACK");
    } else {
        g_ota_mark_err = ESP_OK; // không ở trạng thái pending -> không cần mark, coi như "ổn"
        g_ota_state_after_mark = st;
        ESP_LOGI(TAG, "Partition dang chay: %s (img_state=%d)", running->label, (int)st);
    }
}

// Đẩy "chứng minh thư" của firmware đang chạy lên Firebase để CHẨN ĐOÁN:
//   part     : ota_0 / ota_1 / factory  -> biết OTA đã đổi phân vùng chưa
//   built    : ngày+giờ biên dịch        -> biết file .bin có đúng bản mới không
//   imgState : 1=PENDING_VERIFY, 2=VALID, 4=ABORTED(đã bị rollback)
// Chỉ cần nhìn node này là biết ngay "máy có từng chạy v2 chưa" hay "file .bin
// trên GitHub thực chất vẫn là bản cũ".
static void ota_report_build_info(void) {
    const esp_partition_t *run = esp_ota_get_running_partition();
    const esp_app_desc_t *d = esp_app_get_description();
    esp_ota_img_states_t st = ESP_OTA_IMG_UNDEFINED;
    if (run) esp_ota_get_state_partition(run, &st);
#ifdef CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE
    const int rb = 1;
#else
    const int rb = 0;
#endif
    // Đọc luôn "chứng minh thư" của ảnh nằm ở PHÂN VÙNG CÒN LẠI (chính là ảnh
    // vừa OTA về). So ngày giờ biên dịch của nó với ảnh đang chạy là biết ngay
    // file .bin trên GitHub có đúng bản mới hay không.
    esp_app_desc_t od;
    bool has_other = false;
    const esp_partition_t *other = esp_ota_get_next_update_partition(NULL);
    if (other && esp_ota_get_partition_description(other, &od) == ESP_OK) has_other = true;
    // Trạng thái của ảnh nằm ở phân vùng kia. Nếu = 4 (ESP_OTA_IMG_ABORTED) thì
    // chính bootloader đã TỪ CHỐI ảnh đó và quay lui — bằng chứng trực tiếp,
    // không cần suy đoán.
    esp_ota_img_states_t ost = ESP_OTA_IMG_UNDEFINED;
    if (other) esp_ota_get_state_partition(other, &ost);
    // Lý do của lần reset gần nhất: 6=PANIC(code lỗi/crash), 7=INT_WDT,
    // 8=TASK_WDT, 9=WDT, 11=BROWNOUT(sụt áp), 3=SW(esp_restart), 1=POWERON.
    // Nếu ảnh mới vừa boot lên đã crash thì giá trị này chỉ thẳng ra nguyên nhân.
    const int rst = (int)esp_reset_reason();
    char other_built[48];
    snprintf(other_built, sizeof(other_built), "%s %s",
             has_other ? od.date : "-", has_other ? od.time : "");

    static char j[760];
    snprintf(j, sizeof(j),
             "{\"fwVer\":%d,\"part\":\"%s\",\"appVer\":\"%s\",\"built\":\"%s %s\","
             "\"imgState\":%d,\"rollbackEnabled\":%d,\"idf\":\"%s\","
             "\"otherPart\":\"%s\",\"otherVer\":\"%s\",\"otherBuilt\":\"%s\","
             "\"markValidErr\":\"%s\",\"markValidWorked\":%s,"
             "\"wasPendingAtBoot\":%s,\"otherState\":%d,\"resetReason\":%d}",
             FIRMWARE_VERSION,
             run ? run->label : "?",
             d ? d->version : "?",
             d ? d->date : "?", d ? d->time : "?",
             (int)st, rb,
             d ? d->idf_ver : "?",
             other ? other->label : "?",
             has_other ? od.version : "-",
             other_built,
             esp_err_to_name(g_ota_mark_err),
             g_ota_state_after_mark == ESP_OTA_IMG_VALID ? "true" : "false",
             g_ota_was_pending_at_boot ? "true" : "false",
             (int)ost, rst);
    fb_put(dev_path("/mppt/ota/build"), j);
}

// Xác nhận firmware mới chạy tốt (chống rollback). Gọi sau khi hệ thống ổn định.
// (Giữ lại làm lớp dự phòng — thực tế đã được xác nhận từ đầu app_main.)
static void ota_mark_valid_if_pending(void) {
    const esp_partition_t *running = esp_ota_get_running_partition();
    esp_ota_img_states_t st;
    if (esp_ota_get_state_partition(running, &st) == ESP_OK) {
        if (st == ESP_OTA_IMG_PENDING_VERIFY) {
            ESP_LOGI(TAG, "Firmware moi chay on dinh -> xac nhan hop le");
            esp_ota_mark_app_valid_cancel_rollback();
            
            // ── THÊM DÒNG NÀY ĐỂ BÁO CÁO VERSION MỚI LÊN WEB ──
            char jv[64];
            snprintf(jv, sizeof(jv), "%d", FIRMWARE_VERSION); 
            fb_put(dev_path("/mppt/ota/currentVersion"), jv); // Cập nhật v2 lên database
            fb_put(dev_path("/mppt/ota/status/fwVer"), jv);   // Đồng bộ luôn vào status cho web
        }
    }
}

// ============================================================
//   6. NTP (giờ Việt Nam UTC+7)
// ============================================================
static volatile bool g_ntp_ok = false;

static void ntp_sync_cb(struct timeval *tv) { g_ntp_ok = true; }

static void ntp_init(const char *server) {
    esp_sntp_config_t cfg = ESP_NETIF_SNTP_DEFAULT_CONFIG(server);
    cfg.sync_cb = ntp_sync_cb;
    esp_netif_sntp_init(&cfg);
    setenv("TZ", "ICT-7", 1);  // Việt Nam UTC+7
    tzset();
}

// ============================================================
//   7. ADC — 4 kênh (VPV, IPV, VBAT, IBAT)
// ============================================================
static adc_oneshot_unit_handle_t g_adc;

static void adc_setup(void) {
    adc_oneshot_unit_init_cfg_t init_cfg = { .unit_id = ADC_UNIT_1 };
    esp_err_t e = adc_oneshot_new_unit(&init_cfg, &g_adc);
    ESP_LOGI(TAG, "adc_oneshot_new_unit: %s", esp_err_to_name(e));
    adc_oneshot_chan_cfg_t ch_cfg = { .atten = ADC_ATTEN_DB_12, .bitwidth = ADC_BITWIDTH_DEFAULT };
    e = adc_oneshot_config_channel(g_adc, PIN_VPV_ADC,  &ch_cfg);
    ESP_LOGI(TAG, "config channel VPV(%d): %s", PIN_VPV_ADC, esp_err_to_name(e));
    e = adc_oneshot_config_channel(g_adc, PIN_IPV_ADC,  &ch_cfg);
    ESP_LOGI(TAG, "config channel IPV(%d): %s", PIN_IPV_ADC, esp_err_to_name(e));
    e = adc_oneshot_config_channel(g_adc, PIN_VBAT_ADC, &ch_cfg);
    ESP_LOGI(TAG, "config channel VBAT(%d): %s", PIN_VBAT_ADC, esp_err_to_name(e));
    e = adc_oneshot_config_channel(g_adc, PIN_IBAT_ADC, &ch_cfg);
    ESP_LOGI(TAG, "config channel IBAT(%d): %s", PIN_IBAT_ADC, esp_err_to_name(e));
}

// Đọc ADC ra Volt (xấp xỉ tuyến tính 0..3.3V trên thang 12-bit — KHÔNG hiệu
// chuẩn factory như analogReadMilliVolts() bên Arduino, độ chính xác thấp hơn)
static float adc_read_volts(int ch) {
    int raw = 0;
    adc_oneshot_read(g_adc, ch, &raw);
    return (raw / 4095.0f) * 3.3f;
}

// ============================================================
//   8. PWM (LEDC) — ngõ ra điều khiển Buck converter
// ============================================================
static void pwm_setup(void) {
    ledc_timer_config_t tcfg = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = PWM_RES_BITS,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };
    ledc_timer_config(&tcfg);
    ledc_channel_config_t ccfg = {
        .gpio_num = PIN_PWM_OUT,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0
    };
    ledc_channel_config(&ccfg);
}

static void pwm_write(uint32_t duty) {
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static inline float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ============================================================
//   SIM MODE — tạo số liệu ẢO để test dashboard khi CHƯA có phần cứng thật
//   Đặt SIM_MODE=0 để dùng lại cảm biến thật (ADC) như cũ.
//
//   Thông số hệ: PV 225V-12A (~2.7kWp) · Pin Li-ion 48V (cấu hình g_capacity_ah,
//   ví dụ 56.25Ah qua trang cấu hình). Kịch bản lặp lại mỗi 2 phút (120s) — rút ngắn
//   từ 10 phút xuống để đảm bảo chạy đủ trọn 1 vòng sạc->xả TRƯỚC khi thiết bị có thể
//   tự restart do phân mảnh RAM (quan sát thực tế ~360s/lần trên log serial):
//     0  - 30s  : Sạc nhanh — nắng tốt, Vpv/Ipv tiến dần lên gần định mức
//     30 - 60s  : Mây che — Ipv/Vpv sụt & dao động mạnh, sạc chậm lại
//     60 - 90s  : Trời tối — PV tắt dần, pin bắt đầu chuyển sang xả
//     90 - 120s : Ban đêm — tiếp tục xả, dòng xả tăng dần
//   Toàn bộ giá trị có nhiễu ngẫu nhiên nhẹ quanh đường kịch bản để giống
//   dữ liệu thật. SOC/trạng thái vẫn được tính bởi thuật toán coulomb-
//   counting & bảo vệ điện áp gốc (không đụng vào), chỉ thay đầu vào cảm biến.
// ============================================================
#define SIM_MODE 1

#if SIM_MODE
static inline float sim_randf(float lo, float hi) {
    return lo + (hi - lo) * ((float)rand() / (float)RAND_MAX);
}

// vbatSim là trạng thái "điện áp pin" tiến hoá dần theo thời gian (không nhảy cóc),
// mô phỏng quán tính thật của accu khi sạc/xả.
// Mô phỏng kịch bản "vừa lắp pin Li-ion 48V vào, SOC hiện tại 50%":
//   - vbatSim khởi động ở ~48.0V (mức nghỉ điển hình của pack Li-ion 48V ở khoảng 50% SOC).
//   - Ép soc_anchor_pct = 50% ngay lần chạy đầu tiên (ghi đè giá trị cũ trong NVS từ các lần test trước).
//   Nhớ vào khung "CẤU HÌNH SẠC PIN" chọn Loại hóa học = Li-ion, Cấp điện áp = Hệ 48V,
//   rồi bấm CẬP NHẬT + BẮT ĐẦU để thấy mô phỏng chạy đúng.
static void sim_generate(float *Vpv, float *Ipv, float *Vbat, float *Ibat) {
    static float t0 = -1.0f;
    static float vbatSim = 48.0f;

    float now = (float)(esp_timer_get_time() / 1000000ULL);
    if (t0 < 0) {
        t0 = now;
        srand((unsigned)now);
        soc_anchor_pct = 50.0f;
        soc_ah_accum = 0.0f;
    }
    float t = fmodf(now - t0, 120.0f);   // lặp lại chu kỳ 2 phút để test liên tục

    float pv_v, pv_i, ibat;

    if (t < 30.0f) {                                    // ---- Sạc nhanh (nắng tốt) ----
        float k = t / 30.0f;
        pv_v   = 195.0f + 25.0f * k + sim_randf(-3.0f, 3.0f);      // tiến dần lên ~220V (gần 225V định mức)
        pv_i   = 8.5f   + 3.2f  * k + sim_randf(-0.4f, 0.4f);      // tiến dần lên ~11.7A (gần 12A định mức)
        ibat   = (pv_v * pv_i * 0.92f) / fmaxf(vbatSim, 30.0f) + sim_randf(-0.5f, 0.5f);
        vbatSim += 0.014f;                                         // pin tăng áp đều khi sạc mạnh
    } else if (t < 60.0f) {                             // ---- Mây che, sạc chậm lại ----
        float k = (t - 30.0f) / 30.0f;
        pv_v   = 218.0f - 55.0f * k + sim_randf(-10.0f, 10.0f);    // MPP tụt & dao động do mây trôi qua
        pv_i   = 11.5f  - 8.0f  * k + sim_randf(-1.2f, 1.2f);
        pv_i   = fmaxf(pv_i, 1.2f);
        ibat   = (pv_v * pv_i * 0.9f) / fmaxf(vbatSim, 30.0f) + sim_randf(-0.4f, 0.4f);
        vbatSim += 0.0045f;                                        // vẫn tăng nhưng chậm hơn nhiều
    } else if (t < 90.0f) {                             // ---- Trời tối, bắt đầu xả ----
        float k = (t - 60.0f) / 30.0f;
        pv_v   = fmaxf(6.0f - 5.0f * k, 0.3f) + sim_randf(0.0f, 1.5f);   // PV tắt dần về gần 0
        pv_i   = fmaxf(1.0f * (1.0f - k), 0.0f) + sim_randf(0.0f, 0.15f);
        ibat   = -(6.0f + 8.0f * k) + sim_randf(-1.0f, 1.0f);            // pin chuyển sang xả (âm)
        vbatSim -= 0.0175f;
    } else {                                            // ---- Ban đêm, tiếp tục xả ----
        float k = (t - 90.0f) / 30.0f;
        pv_v   = sim_randf(0.0f, 1.0f);
        pv_i   = 0.0f;
        ibat   = -(14.0f + 7.0f * k) + sim_randf(-1.5f, 1.5f);           // dòng xả tăng dần về khuya
        vbatSim -= 0.0275f;
    }

    vbatSim = clampf(vbatSim, 40.0f, 53.5f);

    // Chặn trần dòng pin đúng theo giới hạn thiết kế thực tế (công suất PV tối đa ~2.7kW / điện áp hệ):
    //   12V -> 200A · 24V -> 112A · 36V -> ~75A (nội suy) · 48V -> 56.25A
    float capCur;
    if      (vbatSim < 17.0f) capCur = 200.0f;
    else if (vbatSim < 32.0f) capCur = 112.0f;
    else if (vbatSim < 42.0f) capCur = 75.0f;
    else                       capCur = 56.25f;
    ibat = clampf(ibat, -capCur, capCur);

    *Vpv  = clampf(pv_v, 0.0f, 230.0f);   // tấm PV định mức 225V-12A, cho dư chút headroom nhiễu
    *Ipv  = clampf(pv_i, 0.0f, 13.0f);
    *Vbat = vbatSim;
    *Ibat = ibat;
}
#endif

// ============================================================
//   9. TASK MPPT — thuật toán P&O + bảo vệ + SOC (giữ nguyên logic .ino)
// ============================================================
static void mppt_task(void *arg) {
    float Vf=0, If=0, VbatF=12.0f, IbatF=0, PpvF=0, PbatF=0;
    float PpvF_old=0, Vf_old=0, Dold=0.02f;
    long startup_timer = 0;
    int latched_volt_sel = 12;
    int cnt = 0;
    float trip_state = 0;
    unsigned long over_start_us = 0;
    const float alpha = 0.1f;
    const float Dmax=0.95f, Dmin=0.02f;
    const int N_mppt_sample = 20;
    const float step_mppt = 0.001f;
    const float dD_max=0.008f, dD_min=0.001f;
    const long STARTUP_LOOPS = 2000;

    static unsigned long last_coulomb_ms = 0;
    static unsigned long full_hold_start = 0;

    for (;;) {
        // ---- TẠM DỪNG SẠC KHI ĐANG OTA (an toàn) ----
        // Trong lúc cập nhật firmware, hạ PWM về 0 để ngừng buck converter,
        // tránh sạc pin khi không có thuật toán bảo vệ giám sát.
        if (g_ota_in_progress) {
            pwm_write(0);
            g_status = ST_STARTUP;
            vTaskDelay(pdMS_TO_TICKS(200));
            continue;
        }

        if (g_reset_request) {
            g_reset_request = false;
            trip_state = 0; over_start_us = 0; startup_timer = 0; Dold = 0.02f;
        }

        // ---- Đọc cảm biến thật, HOẶC tạo số liệu ảo để test dashboard (xem SIM_MODE ở trên) ----
#if SIM_MODE
        float Vpv, Ipv_sim, Vbat, Ibat_sim;
        sim_generate(&Vpv, &Ipv_sim, &Vbat, &Ibat_sim);
#else
        float Vbat_adc = adc_read_volts(PIN_VBAT_ADC);
        float Vpv_adc  = adc_read_volts(PIN_VPV_ADC);
        float Ipv_adc  = adc_read_volts(PIN_IPV_ADC);
        float Ibat_adc = adc_read_volts(PIN_IBAT_ADC);

        float Vbat = Vbat_adc / (12000.0f / (200000.0f + 12000.0f));
        float Vpv  = Vpv_adc  / (2380.0f  / (200000.0f + 2380.0f));
#endif

        if (startup_timer < 10) VbatF = Vbat;
        else VbatF += alpha * (Vbat - VbatF);
        startup_timer++;
        bool is_starting_up = (startup_timer < STARTUP_LOOPS);

        // Phát hiện pin đã bị tháo ra: điện áp đọc được quá thấp cho bất kỳ hệ 12/24/36/48V nào.
        g_batt_present = (VbatF >= VBAT_PRESENT_MIN);

        int internal_volt_sel = g_volt_sel;
        if (g_volt_sel == 0) {
            if (is_starting_up) {
                if      (VbatF < 17.0f) latched_volt_sel = 12;
                else if (VbatF < 32.0f) latched_volt_sel = 24;
                else if (VbatF < 42.0f) latched_volt_sel = 36;
                else                    latched_volt_sel = 48;
            }
            internal_volt_sel = latched_volt_sel;
        }
        int internal_chem = (g_chem == 3) ? 0 : g_chem;

        float Vnom = (internal_volt_sel==24)?24.0f:(internal_volt_sel==36)?36.0f:(internal_volt_sel==48)?48.0f:12.0f;
        float G_opamp = (Vnom==24)?3.0f:(Vnom==36)?4.0f:(Vnom==48)?5.0f:2.0f;
        (void)G_opamp; // chỉ dùng ở nhánh đọc ADC thật; giữ dòng này để tránh cảnh báo unused khi SIM_MODE=1

#if SIM_MODE
        float Ibat = Ibat_sim;
        float Ipv  = Ipv_sim;
#else
        float Ibat = Ibat_adc / (0.001f * G_opamp);
        float Ipv  = Ipv_adc * 10.0f;
#endif

        if (startup_timer < 10) {
            IbatF=Ibat; Vf=Vpv; If=Ipv; PpvF=Vf*If; PbatF=VbatF*IbatF;
        } else {
            IbatF += alpha*(Ibat-IbatF);
            Vf    += alpha*(Vpv-Vf);
            If    += alpha*(Ipv-If);
            PpvF  += alpha*(Vf*If - PpvF);
            PbatF += alpha*(VbatF*IbatF - PbatF);
        }

        d_Ppv=PpvF; d_Pbat=PbatF; d_Vbat=VbatF; d_Ibat=IbatF; d_Vpv=Vf; d_Ipv=If;
        d_Eff = (PpvF>1.0f) ? clampf((PbatF/PpvF)*100.0f,0,100) : 0.0f;

        // ---- Ngưỡng CV theo hóa học (đã sửa Li-ion về 1.05 = an toàn) ----
        float Vbat_target = (internal_chem==2)? Vnom*1.2167f : (internal_chem==1)? Vnom*1.0500f : Vnom*1.2000f;
        float Vbat_emergency = Vbat_target + (Vnom/12.0f)*0.3f;

        float Imax = (float)g_imax;
        float I_trip = Imax*1.10f;
        float I_warn = Imax*1.05f;
        float Ipow = 2700.0f / Vnom;
        float Ibat_cc = (Ipow < Imax) ? Ipow : Imax;

        d_Vbat_target=Vbat_target; d_Vbat_emg=Vbat_emergency; d_Ibat_cc=Ibat_cc; d_Ibat_trip=I_trip;

        // ---- SOC coulomb-counting ----
        unsigned long now_ms = (unsigned long)(esp_timer_get_time()/1000ULL);
        if (last_coulomb_ms==0) last_coulomb_ms = now_ms;
        float dt_h = (now_ms - last_coulomb_ms)/3600000.0f;
        last_coulomb_ms = now_ms;
        if (soc_anchor_pct < 0.0f) {
            float k0 = (internal_chem==0)?0.980f:(internal_chem==1)?0.900f:0.850f;
            float Vempty0 = Vnom*k0;
            soc_anchor_pct = clampf((VbatF-Vempty0)/(Vbat_target-Vempty0)*100.0f,0,100);
            soc_ah_accum = 0;
        }
        float capAh = (g_capacity_ah>0.1f)?g_capacity_ah:100.0f;
        soc_ah_accum += IbatF*dt_h;
        float socNow = clampf(soc_anchor_pct + (soc_ah_accum/capAh)*100.0f,0,100);
        bool nearFull = (!is_starting_up) && (VbatF>=Vbat_target*0.995f) && (IbatF>0.01f) && (IbatF<capAh*0.03f);
        if (nearFull) {
            if (full_hold_start==0) full_hold_start = now_ms;
            if (now_ms - full_hold_start > 60000UL && soc_anchor_pct < 99.5f) {
                soc_anchor_pct=100; soc_ah_accum=0; socNow=100;
                // Pin đã sạc đầy (SOC=100%) — tự động TẮT sạc để an toàn + báo cho người dùng qua web.
                g_charge_enable = false;
                g_charge_just_completed = true;
                nvs_save_config();
            }
        } else full_hold_start = 0;
        d_SOC = socNow;

        // ---- OFF: chưa bật sạc (chưa bấm Start / đã bấm Stop) HOẶC không phát hiện có pin ----
        if (!g_charge_enable || !g_batt_present) {
            Dold = 0;
            cnt = N_mppt_sample;
            g_status = ST_OFF;
            // Xoá sạch trạng thái bảo vệ cũ để khi bật sạc lại (lắp pin vào + bấm Start),
            // hệ thống khởi động lại từ đầu, không bị dính lỗi/clamp từ trước lúc tắt.
            trip_state = 0;
            over_start_us = 0;
            full_hold_start = 0;
        } else {

        // ---- Bảo vệ ----
        cnt++;
        bool overTrip = (IbatF > I_trip);
        bool overWarn = (IbatF > I_warn);
        if (trip_state==0 && !is_starting_up) {
            if (VbatF >= Vbat_emergency) { trip_state=1; g_status=ST_ERR_VOLT; }
            else if (overTrip) {
                if (over_start_us==0) over_start_us = (unsigned long)esp_timer_get_time();
                if (((unsigned long)esp_timer_get_time() - over_start_us) > ENDURANCE_US) { trip_state=2; g_status=ST_ERR_CURR; }
                else g_status = ST_WARN_CURR;
            } else {
                over_start_us = 0;
                if (overWarn) g_status = ST_WARN_CURR;
            }
        }

        // ---- Điều khiển ----
        if (trip_state > 0) { Dold=0; cnt=N_mppt_sample; }
        else if (is_starting_up) { Dold=0.02f; g_status=ST_STARTUP; }
        else if (overWarn) {
            float clampT = (CLAMP_TARGET_A < Imax) ? CLAMP_TARGET_A : Imax;
            if (IbatF > clampT) Dold -= 0.03f; else Dold += dD_min;
            Dold = clampf(Dold, Dmin, Dmax);
            g_status = ST_WARN_CURR;
        } else {
            g_status = (IbatF>=Ibat_cc) ? ST_CC : ST_OK;
            if (cnt >= N_mppt_sample) {
                cnt = 0;
                if (VbatF >= Vbat_target) Dold -= dD_max;
                else if (IbatF >= Ibat_cc) {
                    float I_error = IbatF - Ibat_cc;
                    float step_cc = fminf(dD_max, fmaxf(dD_min, I_error*0.0005f));
                    Dold -= step_cc;
                } else {
                    float dP = PpvF - PpvF_old, dV = Vf - Vf_old;
                    if (fabsf(dP) > 1.0f) {
                        if (dP>0) Dold += (dV>0)?-step_mppt:step_mppt;
                        else      Dold += (dV>0)? step_mppt:-step_mppt;
                    } else if (PpvF < 500.0f) Dold += step_mppt;
                    PpvF_old=PpvF; Vf_old=Vf;
                }
                Dold = clampf(Dold, Dmin, Dmax);
            }
        }
        } // end else (charge enable && battery present)

        pwm_write((uint32_t)(Dold * PWM_MAX));
        vTaskDelay(1);   // nghỉ đúng 1 tick thật (khác pdMS_TO_TICKS(1), tránh làm tròn về 0)
    }
}

// ============================================================
//   10. FIREBASE — REST API qua esp_http_client (TLS: certificate bundle)
//   SỬA QUAN TRỌNG: dùng lại 1 client duy nhất (keep-alive) thay vì tạo/hủy
//   TLS session mới cho MỖI request (trước đây mỗi giây tạo/hủy ~5-8 phiên
//   TLS riêng biệt) — đây là nguyên nhân chính gây phân mảnh heap dẫn tới
//   restart định kỳ mỗi ~6 phút quan sát được trên log thực tế.
// ============================================================
typedef struct { char *buf; int len; int cap; } http_resp_t;

// Con trỏ tới buffer nhận response của request ĐANG chạy (chỉ có 1 request
// chạy tại 1 thời điểm vì mọi lời gọi fb_* đều tuần tự trong cùng 1 task).
static http_resp_t *g_fb_active_resp = NULL;

static esp_err_t http_evt_handler(esp_http_client_event_t *evt) {
    if (evt->event_id == HTTP_EVENT_ON_DATA) {
        http_resp_t *r = g_fb_active_resp;
        if (r && r->buf && evt->data_len > 0) {
            int room = r->cap - r->len - 1;
            int n = (evt->data_len < room) ? evt->data_len : room;
            if (n > 0) { memcpy(r->buf + r->len, evt->data, n); r->len += n; r->buf[r->len]=0; }
        }
    }
    return ESP_OK;
}

// Client HTTP dùng chung cho toàn bộ request tới Firebase — chỉ tạo 1 lần.
static esp_http_client_handle_t g_fb_client = NULL;

static esp_http_client_handle_t fb_client(void) {
    if (!g_fb_client) {
        esp_http_client_config_t http_cfg = {
            .url = FIREBASE_HOST,   // sẽ bị ghi đè bằng esp_http_client_set_url() mỗi request
            .crt_bundle_attach = esp_crt_bundle_attach,
            .event_handler = http_evt_handler,
            .timeout_ms = 8000,
            .keep_alive_enable = true,
            .keep_alive_idle = 15,
            .keep_alive_interval = 5,
            .keep_alive_count = 3,
        };
        g_fb_client = esp_http_client_init(&http_cfg);
    }
    return g_fb_client;
}

// Gọi khi WiFi mất/kết nối lại để đảm bảo không dùng phải 1 socket/TLS session
// đã chết từ phiên WiFi cũ — buộc tạo lại client sạch cho phiên kết nối mới.
static void fb_client_reset(void) {
    if (g_fb_client) {
        esp_http_client_cleanup(g_fb_client);
        g_fb_client = NULL;
    }
}

static bool fb_request(const char *path, esp_http_client_method_t method, const char *body, char *resp_buf, int resp_cap) {
    char url[256];
    snprintf(url, sizeof(url), "%s%s.json", FIREBASE_HOST, path);
    http_resp_t r = { resp_buf, 0, resp_cap };
    if (resp_buf) resp_buf[0] = 0;

    g_fb_active_resp = &r;
    esp_http_client_handle_t cl = fb_client();
    esp_http_client_set_url(cl, url);
    esp_http_client_set_method(cl, method);
    if (body) {
        esp_http_client_set_header(cl, "Content-Type", "application/json");
        esp_http_client_set_post_field(cl, body, strlen(body));
    } else {
        // Xoá sạch body/Content-Length còn sót lại từ 1 request PUT/POST trước đó
        // (vì dùng lại cùng 1 client) — tránh gửi nhầm body cũ vào request GET.
        esp_http_client_set_post_field(cl, NULL, 0);
    }

    esp_err_t err = esp_http_client_perform(cl);
    int code = esp_http_client_get_status_code(cl);
    g_fb_active_resp = NULL;
    // KHÔNG gọi esp_http_client_cleanup(cl) nữa — giữ lại client + kết nối keep-alive
    // để tái sử dụng cho request tiếp theo, tránh bắt tay TLS lại từ đầu mỗi lần.
    bool ok = (err == ESP_OK && code == 200);
    if (err != ESP_OK) {
        // Kết nối/socket có thể đã hỏng — dọn để lần gọi sau tự tạo phiên mới sạch,
        // tránh client bị kẹt vĩnh viễn ở trạng thái lỗi.
        fb_client_reset();
    }

    if (ok) {
        // 1 request thành công -> chuỗi lỗi (nếu có) coi như chấm dứt.
        // KHÔNG đụng vào GPIO2 ở đây: nếu đã yêu cầu reset ngoài (g_reset_triggered), phải giữ
        // nguyên HIGH cho tới khi mạch ngoài thực sự reset ESP32 — 1 request thành công lẻ tẻ
        // giữa chừng không có nghĩa là hệ thống đã thực sự ổn định trở lại.
        g_fail_start_time = 0;
    } else {
        uint64_t now = esp_timer_get_time();
        if (g_fail_start_time == 0) g_fail_start_time = now;  // đánh dấu mốc bắt đầu chuỗi lỗi

        // Treo giao tiếp Firebase LIÊN TỤC >= 10 giây (đo theo thời gian thực, không phải đếm
        // số lần lỗi — vì mỗi request lỗi có thể mất từ vài ms tới 8000ms tuỳ tình huống).
        if (!g_reset_triggered && (now - g_fail_start_time) >= NET_HANG_US) {
            char reason[96];
            snprintf(reason, sizeof(reason), "Treo giao tiep Firebase lien tuc >= 10s (path cuoi: %s)", path);
            request_external_reset(reason);
        }
    }

    ESP_LOGI(TAG, "FB %s %s -> err=%s http=%d %s",
        (method==HTTP_METHOD_PUT)?"PUT":(method==HTTP_METHOD_POST)?"POST":"GET",
        path, esp_err_to_name(err), code, ok?"OK":"THAT BAI");
        
    return ok; // Dòng này giờ đã nằm đúng trong hàm
}

static void fb_put(const char *path, const char *json)  { fb_request(path, HTTP_METHOD_PUT,  json, NULL, 0); }
static void fb_push(const char *path, const char *json) { fb_request(path, HTTP_METHOD_POST, json, NULL, 0); }
static bool fb_get(const char *path, char *out, int cap) { return fb_request(path, HTTP_METHOD_GET, NULL, out, cap); }

// Đẩy /mppt/current — gọi mỗi ~1 giây
static void push_current(void) {
    static char j[950];
    time_t ts = time(NULL);
    snprintf(j, sizeof(j),
        "{\"ts\":%ld,\"st\":\"%s\",\"soc\":%.1f,\"eff\":%.1f,\"imax\":%.1f,"
        "\"chargeEnable\":%s,\"battPresent\":%s,"
        "\"pv\":{\"u\":%.1f,\"i\":%.2f,\"p\":%.1f},"
        "\"bat\":{\"u\":%.2f,\"i\":%.2f,\"p\":%.1f},"
        "\"th\":{"
          "\"pv\":{\"uw\":%.1f,\"ud\":%.1f,\"um\":%.1f,\"iw\":%.1f,\"id\":%.1f,\"im\":%.1f,\"pw\":%.0f,\"pd\":%.0f,\"pm\":%.0f},"
          "\"bat\":{\"uw\":%.2f,\"ud\":%.2f,\"um\":%.2f,\"iw\":%.1f,\"id\":%.1f,\"im\":%.1f,\"pw\":%.0f,\"pd\":%.0f,\"pm\":%.0f}"
        "}}",
        (long)ts, statusText(g_status), d_SOC, d_Eff, (float)g_imax,
        g_charge_enable ? "true" : "false", g_batt_present ? "true" : "false",
        d_Vpv, d_Ipv, d_Ppv,
        d_Vbat, d_Ibat, d_Pbat,
        PV_U_WARN, PV_U_DANGER, PV_U_MAX, PV_I_WARN, PV_I_DANGER, PV_I_MAX, PV_P_WARN, PV_P_DANGER, PV_P_MAX,
        d_Vbat_target, d_Vbat_emg, d_Vbat_emg*1.05f, (float)g_imax*1.05f, (float)g_imax*1.10f, (float)g_imax*1.10f*1.1f,
        PV_P_WARN, PV_P_MAX, PV_P_MAX*1.05f);
    fb_put(dev_path("/mppt/current"), j);
}

// Đẩy 1 điểm lịch sử — gọi mỗi ~5 giây
static void push_hist_point(void) {
    char j[200];
    snprintf(j, sizeof(j), "[%ld,%.1f,%.2f,%.1f,%.2f,%.2f,%.1f,%.1f,%.1f]",
        (long)time(NULL), d_Vpv, d_Ipv, d_Ppv, d_Vbat, d_Ibat, d_Pbat, d_SOC, d_Eff);
    fb_push(dev_path("/mppt/hist"), j);
}
// Thêm điểm vào Ring Buffer (Tự động ghi đè vòng tròn nếu đầy)
static void offline_buf_push(time_t ts, float vpv, float ipv, float ppv, float vbat, float ibat, float pbat, float soc, float eff) {
    g_offline_buf.data[g_offline_buf.head] = (hist_point_t){ ts, vpv, ipv, ppv, vbat, ibat, pbat, soc, eff };
    g_offline_buf.head = (g_offline_buf.head + 1) % MAX_OFFLINE_POINTS;
    if (g_offline_buf.count < MAX_OFFLINE_POINTS) {
        g_offline_buf.count++;
    } else {
        // Nếu đầy, đẩy tail lên 1 nấc (chấp nhận đè mất điểm cũ nhất để chứa điểm mới nhất)
        g_offline_buf.tail = (g_offline_buf.tail + 1) % MAX_OFFLINE_POINTS;
        ESP_LOGE("OFFLINE", "Bo dem day! Da ghi de diem cu nhat.");
    }
}

// Lấy điểm cũ nhất ra khỏi Ring Buffer
static bool offline_buf_pop(hist_point_t *p) {
    if (g_offline_buf.count == 0) return false;
    *p = g_offline_buf.data[g_offline_buf.tail];
    g_offline_buf.tail = (g_offline_buf.tail + 1) % MAX_OFFLINE_POINTS;
    g_offline_buf.count--;
    return true;
}

// Task phụ để đồng bộ dữ liệu ngầm (Chạy đa luồng, KHÔNG làm kẹt tiến trình sạc)
static void upload_offline_task(void *arg) {
    is_uploading_offline = true;
    ESP_LOGW("OFFLINE", "Bat dau dong bo %d diem len Firebase...", g_offline_buf.count);
    int success_count = 0;
    char j[200];
    hist_point_t p;
    char offline_hist_path[192];
    snprintf(offline_hist_path, sizeof(offline_hist_path), "/devices/%s/mppt/hist", g_device_id);

    while (g_offline_buf.count > 0 && g_wifi_connected && g_ntp_ok) {
        // Lấy tạm 1 điểm ở vị trí tail để thử gửi
        p = g_offline_buf.data[g_offline_buf.tail];
        
        snprintf(j, sizeof(j), "[%ld,%.1f,%.2f,%.1f,%.2f,%.2f,%.1f,%.1f,%.1f]",
                 (long)p.ts, p.vpv, p.ipv, p.ppv, p.vbat, p.ibat, p.pbat, p.soc, p.eff);
                 
        // Nếu lệnh POST gửi lên Firebase thành công (mã 200 OK)
        if (fb_request(offline_hist_path, HTTP_METHOD_POST, j, NULL, 0)) {
            offline_buf_pop(&p); // Chắc chắn thành công mới xóa điểm đó khỏi RAM
            success_count++;
            vTaskDelay(pdMS_TO_TICKS(100)); // Giãn cách 0.1s chống nghẽn RAM
        } else {
            ESP_LOGE("OFFLINE", "Loi mang giua chung, dung dong bo. Con lai %d diem.", g_offline_buf.count);
            nvs_save_offline_history(); // Lập tức backup phần còn lại xuống Flash
            break;
        }
    }
    
    if (g_offline_buf.count == 0 && success_count > 0) {
        ESP_LOGI("OFFLINE", "✅ Dong bo hoan tat %d diem.", success_count);
        nvs_clear_offline_history(); // Dọn dẹp sạch sẽ Flash
    }
    
    is_uploading_offline = false;
    vTaskDelete(NULL); // Hủy Task sau khi hoàn thành nhiệm vụ
}
// Đẩy thống kê ngày hiện tại — gọi mỗi ~5 phút
static void push_daily_today(void) {
    if (statYear == 0) return;
    time_t now = time(NULL);
    struct tm tmv; localtime_r(&now, &tmv);
    int doy = tmv.tm_yday;
    if (doy < 0 || doy >= STAT_DAYS) return;
    char subpath[64], j[80];
    snprintf(subpath, sizeof(subpath), "/mppt/daily/%d/%d", statYear, doy);
    snprintf(j, sizeof(j), "{\"ch\":%.3f,\"dis\":%.3f}", statCharge[doy], statDischarge[doy]);
    fb_put(dev_path(subpath), j);
}

// Đọc cấu hình (chem/volt/imax/capacity) — gọi mỗi ~3 giây
static void pull_config(void) {
    static char buf[400];
    if (!fb_get(dev_path("/mppt/config"), buf, sizeof(buf))) return;
    if (strlen(buf) < 3 || strstr(buf, "null")) return;
    cJSON *root = cJSON_Parse(buf);
    if (!root) return;
    bool changed = false;
    cJSON *it;
    if ((it = cJSON_GetObjectItem(root, "chem")) && cJSON_IsNumber(it)) {
        int v = it->valueint; if (v != g_chem) { g_chem = v; changed = true; }
    }
    if ((it = cJSON_GetObjectItem(root, "volt")) && cJSON_IsNumber(it)) {
        int v = it->valueint; if (v != g_volt_sel) { g_volt_sel = v; changed = true; }
    }
    if ((it = cJSON_GetObjectItem(root, "imax")) && cJSON_IsNumber(it)) {
        int v = it->valueint; if (v != g_imax) { g_imax = v; changed = true; }
    }
    if ((it = cJSON_GetObjectItem(root, "capacity")) && cJSON_IsNumber(it)) {
        float v = (float)it->valuedouble; if (fabsf(v - g_capacity_ah) > 0.01f) { g_capacity_ah = v; changed = true; }
    }
    cJSON_Delete(root);
    if (changed) nvs_save_config();
}

// Đọc lệnh Start/Stop sạc từ web — gọi mỗi ~3 giây
static void pull_charge_enable(void) {
    static char buf[64];
    if (!fb_get(dev_path("/mppt/control/chargeEnable"), buf, sizeof(buf))) return;
    if (strlen(buf) < 3) return;   // rỗng/null -> chưa có giá trị, giữ nguyên mặc định NVS
    bool v = (strncmp(buf, "true", 4) == 0);
    if (v != g_charge_enable) {
        g_charge_enable = v;
        nvs_save_config();
        ESP_LOGI(TAG, "chargeEnable tu web: %s", v ? "BAT (Start)" : "TAT (Stop)");
    }
}

// Gọi 1 lần duy nhất khi vừa tự tắt do sạc đầy (SOC=100%) — báo cho web biết.
static void notify_charge_complete(void) {
    fb_put(dev_path("/mppt/control/chargeEnable"), "false");
    fb_put(dev_path("/mppt/notify/chargeComplete"), "true");
    ESP_LOGI(TAG, "Da sac day (SOC=100%%) -> tu tat + bao web.");
}

// Đọc lệnh reset — gọi mỗi ~3 giây
static void pull_control(void) {
    static char buf[64];
    if (!fb_get(dev_path("/mppt/control/resetRequest"), buf, sizeof(buf))) return;
    if (strncmp(buf, "true", 4) == 0) {
        g_reset_request = true;
        fb_put(dev_path("/mppt/control/resetRequest"), "false");
    }
}

// Đọc lệnh RESET NGOÀI (GPIO2) do Admin gửi thủ công — KHÁC với resetRequest (chỉ xoá lỗi
// phần mềm). Lệnh này kích hoạt đúng cơ chế phần cứng qua request_external_reset() —
// dựng xung HIGH 3s trên GPIO2 để mạch ngoài reset vật lý thiết bị.
static void pull_hw_reset(void) {
    static char buf[64];
    if (!fb_get(dev_path("/mppt/control/hwReset"), buf, sizeof(buf))) return;
    if (strncmp(buf, "true", 4) == 0) {
        fb_put(dev_path("/mppt/control/hwReset"), "false");
        request_external_reset("Yeu cau thu cong tu Admin (nut Reset Ngoai)");
    }
}

// Đọc cờ reconnect WiFi từ web — gọi mỗi ~3 giây (chỉ chạy khi còn thấy Firebase)
static void pull_wifi_reconnect(void) {
    static char buf[64];
    if (!fb_get(dev_path("/mppt/control/wifiReconnect"), buf, sizeof(buf))) return;
    if (strncmp(buf, "true", 4) == 0) {
        fb_put(dev_path("/mppt/control/wifiReconnect"), "false");
        g_force_reconnect = true;
    }
}

// Đọc lệnh Reset WiFi từ Admin — xóa credentials và vào AP Mode cấu hình lại
static void pull_wifi_reset(void) {
    static char buf[64];
    if (!fb_get(dev_path("/mppt/control/wifiReset"), buf, sizeof(buf))) return;
    if (strncmp(buf, "true", 4) == 0) {
        fb_put(dev_path("/mppt/control/wifiReset"), "false"); // xóa cờ ngay trước khi mất mạng
        ESP_LOGW(TAG, "WIFI RESET yeu cau boi Admin -> xoa credentials NVS, vao AP Mode!");
        wifi_creds_erase();     // xóa credentials cũ trong NVS
        g_start_ap_mode = true; // app_main sẽ khởi động WiFi Manager
    }
}

// Đọc lệnh OTA từ web — gọi mỗi ~5 giây.
// Chỉ chạy 1 lần ngay sau khi WiFi+NTP lên lần đầu: nếu thiết bị CHƯA từng có /mppt/ota/url
// trên Firebase (thiết bị mới, chưa được Admin cấu hình gì), tự điền link mặc định đóng cứng
// trong code (OTA_DEFAULT_URL) vào đó — command để false nên KHÔNG tự OTA ngay, chỉ "để sẵn"
// cho Admin/người dùng bấm cập nhật sau. Nếu Firebase đã có url (do Admin từng đổi) thì bỏ qua,
// không bao giờ ghi đè lựa chọn của Admin.
static void ota_ensure_default_url(void) {
    static char buf[400];
    if (!fb_get(dev_path("/mppt/ota/url"), buf, sizeof(buf))) return;
    if (strlen(buf) >= 10) return;   // đã có url thật -> không đụng vào
    // Ghi từng field riêng lẻ (KHÔNG PUT đè cả node /mppt/ota) để không xoá mất
    // currentVersion/status đã được ghi ngay trước đó lúc boot.
    char jver[16], jurl[420];
    snprintf(jver, sizeof(jver), "%d", FIRMWARE_VERSION);
    snprintf(jurl, sizeof(jurl), "\"%s\"", OTA_DEFAULT_URL);
    fb_put(dev_path("/mppt/ota/url"), jurl);
    fb_put(dev_path("/mppt/ota/latestVersion"), jver);
    fb_put(dev_path("/mppt/ota/command"), "false");
    ESP_LOGI(TAG, "Chua co link OTA rieng -> da dien link mac dinh trong code.");
}

// Cấu trúc Firebase mong đợi tại /mppt/ota:
//   { "latestVersion": 2, "url": "<link .bin>", "command": true/false }
// Khi command==true VÀ latestVersion >= FIRMWARE_VERSION -> tiến hành OTA.
// Dùng ">=" (thay vì ">") để hỗ trợ CẢ 2 trường hợp:
//   1) Nâng cấp lên bản MỚI HƠN (latestVersion > FIRMWARE_VERSION).
//   2) Cài lại ĐÚNG bản đang chạy (latestVersion == FIRMWARE_VERSION) — dùng
//      cho "Code Backup - Cài lại qua WiFi" để khắc phục lỗi phần mềm, không
//      đổi số phiên bản.
static void pull_ota(void) {
    // buf phải ĐỦ LỚN để chứa TOÀN BỘ node /mppt/ota dạng JSON, gồm:
    //   command, currentVersion, latestVersion, url (link GitHub ~80-100 ký
    //   tự), và cả object status {state, progress, fwVer}.
    // Trước đây buf[512] quá nhỏ: sau khi thêm tính năng cấu hình link qua
    // Admin (URL dài hơn + đủ field), tổng JSON vượt 512 byte -> bị CẮT CỤT
    // -> cJSON_Parse trả về NULL -> hàm return sớm, OTA KHÔNG BAO GIỜ chạy.
    // Đây chính là lý do "thêm link qua Admin thì cả web lẫn app đều không
    // cập nhật được". Tăng lên 2048 byte cho dư dả.
    static char buf[2048];
    if (!fb_get(dev_path("/mppt/ota"), buf, sizeof(buf))) return;
    if (strlen(buf) < 3 || strstr(buf, "null")) return;

    cJSON *root = cJSON_Parse(buf);
    if (!root) return;

    cJSON *cmd = cJSON_GetObjectItem(root, "command");
    cJSON *ver = cJSON_GetObjectItem(root, "latestVersion");
    cJSON *url = cJSON_GetObjectItem(root, "url");

    bool want_update = cmd && cJSON_IsBool(cmd) && cJSON_IsTrue(cmd);
    int latest = (ver && cJSON_IsNumber(ver)) ? ver->valueint : 0;

    if (want_update && latest >= FIRMWARE_VERSION && url && cJSON_IsString(url)) {
        // Xóa cờ command ngay để tránh lặp lại OTA sau khi reboot
        fb_put(dev_path("/mppt/ota/command"), "false");
        // Ghi lại "bản đang định cài" để sau khi reboot còn biết mà đối chiếu
        // (phát hiện rollback) — xem phần kết luận trong app_main.
        char jpend[16];
        snprintf(jpend, sizeof(jpend), "%d", latest);
        g_ota_target_ver = latest;   // để status/targetVer hiện đúng bản đang cài
        fb_put(dev_path("/mppt/ota/pendingVersion"), jpend);
        fb_put(dev_path("/mppt/ota/lastResult"), "\"installing\"");
        char url_copy[400];
        strncpy(url_copy, url->valuestring, sizeof(url_copy)-1);
        url_copy[sizeof(url_copy)-1] = 0;
        cJSON_Delete(root);
        // Thực hiện OTA (hàm này sẽ reboot nếu thành công)
        do_ota_update(url_copy);
        return;
    }
    // Có lệnh nhưng version yêu cầu THẤP HƠN bản đang chạy (vd Admin lỡ đặt v1
    // trong khi máy đã ở v2): không cài, nhưng phải hạ cờ, nếu không cờ command
    // treo mãi ở true và mỗi 30s lại vào đây một lần.
    if (want_update && latest < FIRMWARE_VERSION) {
        ESP_LOGW(TAG, "Bo qua OTA: latestVersion=%d < FIRMWARE_VERSION=%d", latest, FIRMWARE_VERSION);
        fb_put(dev_path("/mppt/ota/command"), "false");
        fb_put(dev_path("/mppt/ota/lastResult"), "\"skipped_older\"");
    }
    cJSON_Delete(root);
}

// ============================================================
//   11. APP MAIN
// ============================================================
void app_main(void) {
    // ★★★ DÒNG ĐẦU TIÊN TUYỆT ĐỐI. Chỉ đụng vào phân vùng otadata, không cần
    //     NVS/RAM/WiFi gì cả. Đặt trước cả ram_reserve_init()/nvs_flash_init()
    //     để dù các bước khởi tạo đó có lỗi & reboot thì bản firmware vừa OTA
    //     VẪN đã được xác nhận hợp lệ, không bị bootloader quay lui.
    ota_mark_valid_now();

    ram_reserve_init();         // KHOÁ 10% RAM ngay dòng đầu tiên — giữ nguyên suốt vòng đời chương trình
    nvs_flash_init();
    diag_begin();               // HỘP ĐEN: đọc dấu chân lần boot trước + đặt dấu chân mới
    nvs_load_all();
    diag_mark(BOOT_STEP_MARK_VALID);
    adc_setup();
    pwm_setup();
    diag_mark(BOOT_STEP_PERIPH);

    // Khởi tạo đèn báo trạng thái + nút BOOT
    led_init();
    boot_btn_init();
    gpio_reset_pin(PIN_EXT_RESET_OUT);
    gpio_set_direction(PIN_EXT_RESET_OUT, GPIO_MODE_OUTPUT);
    gpio_set_level(PIN_EXT_RESET_OUT, 0);   // Mặc định ổn định = LOW, đảm bảo đúng trạng thái ngay từ lúc boot
    xTaskCreate(led_task, "led", 3072, NULL, 3, NULL);
    xTaskCreate(boot_btn_task, "bootbtn", 2560, NULL, 4, NULL);

    xTaskCreatePinnedToCore(mppt_task, "mppt", 8192, NULL, 5, NULL, 0);
    diag_mark(BOOT_STEP_TASKS);

    wifi_init();
    device_id_init();   // đọc MAC sau khi WiFi stack đã khởi tạo
    diag_mark(BOOT_STEP_WIFI_INIT);
    ESP_LOGI(TAG, "Dang cho WiFi ket noi...");
    int wait = 0;
    while (!g_wifi_connected && wait < 30) { vTaskDelay(pdMS_TO_TICKS(500)); wait++; }
    if (g_wifi_connected) diag_mark(BOOT_STEP_WIFI_UP);

    if (!g_wifi_connected) {
        // Không có WiFi thì chắc chắn không thể đồng bộ NTP hay nói chuyện với Firebase được —
        // coi như cùng loại lỗi "không đồng bộ được với web", yêu cầu reset ngoài ngay.
        request_external_reset("WiFi khong ket noi duoc sau 15s luc boot");
    }

    if (g_wifi_connected) {
        const char *ntp_servers[] = { "time.google.com", "pool.ntp.org", "vn.pool.ntp.org" };
        for (int i = 0; i < 3 && !g_ntp_ok; i++) {
            ESP_LOGI(TAG, "Dang thu dong bo NTP voi server: %s (cho toi 8s)...", ntp_servers[i]);
            ntp_init(ntp_servers[i]);
            esp_err_t ntp_err = esp_netif_sntp_sync_wait(pdMS_TO_TICKS(8000));
            ESP_LOGI(TAG, "Ket qua server %s: %s", ntp_servers[i], esp_err_to_name(ntp_err));
            if (ntp_err == ESP_OK) { g_ntp_ok = true; diag_mark(BOOT_STEP_NTP_OK); break; }
            esp_netif_sntp_deinit();
        }
        if (!g_ntp_ok) {
            ESP_LOGW(TAG, "Ca 3 server NTP deu that bai - kha nang mang dang chan UDP port 123.");
            // Không có cơ chế thử lại NTP nào khác trong suốt vòng đời chương trình -> nếu không
            // yêu cầu reset ở đây, thiết bị sẽ treo vĩnh viễn ở trạng thái "chưa NTP".
            request_external_reset("Khong dong bo duoc NTP sau 3 lan thu luc boot");
        }
    }

    // Đẩy version firmware hiện tại lên Firebase để web so sánh & biết OTA đã xong
    if (g_wifi_connected && g_ntp_ok) {
        char jv[64];
        snprintf(jv, sizeof(jv), "%d", FIRMWARE_VERSION);
        fb_put(dev_path("/mppt/ota/currentVersion"), jv);
        ota_report("idle", 0);
        ota_report_build_info();

        // HỘP ĐEN: đẩy dấu chân của LẦN BOOT TRƯỚC lên Firebase. Nếu ảnh vừa
        // OTA chết lúc khởi động, đây là cách DUY NHẤT (không cần mạch USB-TTL)
        // để biết nó chết ở bước nào:
        //   1=xong NVS  2=xong xac nhan OTA  3=xong ADC/PWM  4=xong tao task
        //   5=xong wifi_init  6=WiFi len  7=NTP xong  8=day duoc Firebase
        //   9=vao vong lap chinh (song on)
        // prevBuilt cho biết dấu chân đó do ẢNH NÀO để lại (so với otherBuilt).
        {
            static char jd[240];
            snprintf(jd, sizeof(jd),
                     "{\"prevStep\":%d,\"prevFw\":%d,\"prevBuilt\":\"%s\",\"resetReason\":%d}",
                     g_prev_step, g_prev_fw, g_prev_built, (int)esp_reset_reason());
            fb_put(dev_path("/mppt/ota/lastBoot"), jd);
        }
        diag_mark(BOOT_STEP_FB_REPORTED);

        // ============================================================
        // ---- KẾT LUẬN LẦN CÀI TRƯỚC (bản sửa lần 2) ----
        // ============================================================
        // BẢN CŨ chỉ so #define FIRMWARE_VERSION (số tay, DỄ QUÊN TĂNG khi
        // build) với pendingVersion -> hễ số thấp hơn là gán luôn nhãn
        // "rollback", dù có 2 khả năng hoàn toàn khác nhau đứng sau con số đó:
        //   (1) ROLLBACK THẬT: bootloader đã quay lui, code cũ đang chạy.
        //   (2) CHỈ LÀ QUÊN TĂNG SỐ: code MỚI vẫn đang chạy đúng (đổi phân
        //       vùng thành công), nhưng người viết code quên sửa
        //       #define FIRMWARE_VERSION trước khi build, nên con số báo sai.
        // Hai trường hợp cần 2 cách xử lý hoàn toàn khác nhau, nên PHẢI phân
        // biệt được — không thể chỉ dựa vào con số tay để kết luận.
        //
        // CÁCH PHÂN BIỆT: so NGUYÊN VĂN ngày-giờ-biên-dịch (built) của ảnh
        // ĐANG CHẠY (do trình biên dịch tự nhúng, không ai gõ tay được) với
        // built của ảnh đã lưu ở pendingBuilt NGAY TRƯỚC lúc OTA:
        //   - built KHỚP  -> code mới CHẮC CHẮN đang chạy thật.
        //   - built KHÔNG khớp -> code đang chạy KHÔNG PHẢI ảnh vừa tải ->
        //     ROLLBACK THẬT.
        static char pvbuf[32];
        int pending = 0;
        if (fb_get(dev_path("/mppt/ota/pendingVersion"), pvbuf, sizeof(pvbuf))) {
            pending = atoi(pvbuf);
        }
        if (pending > 0) {
            static char pendingBuiltBuf[64];
            bool haveExpected =
                fb_get(dev_path("/mppt/ota/pendingBuilt"), pendingBuiltBuf, sizeof(pendingBuiltBuf))
                && strlen(pendingBuiltBuf) > 2;

            const esp_app_desc_t *cur = esp_app_get_description();
            char curBuiltQ[72]; // cùng định dạng có ngoặc kép như lúc PUT ở pendingBuilt
            snprintf(curBuiltQ, sizeof(curBuiltQ), "\"%s %s\"",
                      cur ? cur->date : "?", cur ? cur->time : "?");

            bool codeMatches = haveExpected && strcmp(curBuiltQ, pendingBuiltBuf) == 0;

            if (codeMatches && FIRMWARE_VERSION >= pending) {
                fb_put(dev_path("/mppt/ota/lastResult"), "\"ok\"");
                ESP_LOGW(TAG, "OTA: da chay dung ban v%d (built khop pendingBuilt)", FIRMWARE_VERSION);
            } else if (codeMatches) {
                // Code MỚI đang chạy thật (ngày giờ biên dịch khớp file vừa
                // tải) nhưng FIRMWARE_VERSION vẫn < pending -> KHÔNG PHẢI
                // rollback, chỉ là quên tăng số trong code trước khi build.
                fb_put(dev_path("/mppt/ota/lastResult"), "\"version_macro_forgotten\"");
                ESP_LOGE(TAG, "OTA: code MOI dang chay THAT (built khop) nhung "
                              "FIRMWARE_VERSION van la %d (< %d) -> QUEN TANG SO "
                              "TRONG CODE, KHONG PHAI rollback.", FIRMWARE_VERSION, pending);
            } else {
                fb_put(dev_path("/mppt/ota/lastResult"), "\"rollback\"");
                ESP_LOGE(TAG, "OTA: yeu cau v%d nhung dang chay v%d VA built KHONG KHOP "
                              "ban vua tai -> DA BI BOOTLOADER ROLLBACK THAT SU",
                         pending, FIRMWARE_VERSION);
            }
            fb_put(dev_path("/mppt/ota/pendingVersion"), "0");
        }

        ota_ensure_default_url();
    }

    unsigned long last_push_cur=0, last_push_hist=0, last_stat_push=0;
    unsigned long last_pull_cfg=0, last_pull_ctrl=0, last_stat_save=0, last_energy=0, last_beat=0;
    unsigned long last_pull_ota=0;
    unsigned long boot_ok_time=0;  // mốc thời gian để xác nhận firmware chạy ổn định (rollback)

    diag_mark(BOOT_STEP_MAIN_LOOP);

    for (;;) {
        uint64_t current_time = esp_timer_get_time();

        // --- MÁY TRẠNG THÁI TUẦN HOÀN XUNG RESET NGOÀI ---
        if (g_reset_triggered) {
            if (!g_pulse_completed) {
                // Đang giữ mức HIGH. Kiểm tra xem đã đủ 3 giây chưa
                if (current_time - g_reset_trigger_time >= 3000000ULL) { 
                    // Tạm dừng các tác vụ nặng trước khi dập LOW để điện áp ổn định
                    vTaskDelay(pdMS_TO_TICKS(50)); 
                    
                    gpio_set_level(PIN_EXT_RESET_OUT, 0); // Kéo mạnh về LOW để kích sườn xuống
                    g_pulse_completed = true;
                    g_low_start_time = current_time;      
                    ESP_LOGE("BẢO VỆ", "Da ha GPIO2 ve LOW. Kich hoat bo reset ngoai!");
                }
            } 
            else {
                // Trạng thái giữ mức LOW chờ reset (10 giây)
                if (current_time - g_low_start_time >= LOW_PERIOD_US) {
                    if (!g_wifi_connected || !g_ntp_ok) {
                        ESP_LOGW("BẢO VỆ", "Van mat mang/NTP! Lap lai chu ky: Kich HIGH GPIO2...");
                        g_pulse_completed = false;
                        g_reset_trigger_time = current_time; 
                        gpio_set_level(PIN_EXT_RESET_OUT, 1); 
                    } 
                    else {
                        g_reset_triggered = false;
                        g_pulse_completed = false;
                        g_fail_start_time = 0;
                        ESP_LOGI("BẢO VỆ", "Mang/NTP da khoi phuc!");
                    }
                }
            }
        }

        // --- CƠ CHẾ KIỂM TRA ĐỊNH KỲ KHI ĐANG CHẠY ---
        // Nếu NTP đã từng OK nhưng đột nhiên mất kết nối WiFi/Firebase quá lâu khi đang chạy, 
        // nó sẽ kích hoạt request_external_reset và rơi vào vòng tuần hoàn ở trên.
        if (g_wifi_connected && g_ntp_ok) {
            // Khi mạng bình thường, liên tục dập chân reset về LOW để an toàn[cite: 4]
            if (!g_reset_triggered) {
                gpio_set_level(PIN_EXT_RESET_OUT, 0);
            }
        }
        uint32_t free_ram = esp_get_free_heap_size();
        uint32_t max_block = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
        // Lưu ý: KHÔNG còn poll/timer nào để tự đưa GPIO2 về LOW ở đây nữa. Theo đúng cơ chế
        // mới, GPIO2 giữ nguyên HIGH cho tới khi mạch ngoài THỰC SỰ reset ESP32 — lúc boot lại,
        // gpio_set_level(PIN_EXT_RESET_OUT, 0) ở phần khởi tạo phía trên sẽ tự đưa nó về LOW.

        // Bảo vệ RAM: Chống phân mảnh và tràn bộ nhớ (free_ram/max_block ở đây đã LOẠI TRỪ
        // sẵn phần 10% bị khoá ở ram_reserve_init() — vì khối đó không bao giờ được free()).
        //
        // QUAN TRỌNG: restart này có thể xảy ra BẤT CỨ LÚC NÀO (kể cả giữa lúc đang xả),
        // và có thể xảy ra SỚM HƠN chu kỳ lưu NVS định kỳ (10 phút ở dưới) — nên phải
        // lưu statCharge/statDischarge/SOC xuống NVS ngay tại đây trước khi restart,
        // nếu không sẽ mất trắng phần kWh đã tích lũy nhưng chưa kịp lưu.
        if (max_block < 25600) {
            ESP_LOGE("BẢO VỆ", "Phan manh RAM! Khoi lon nhat chi con %lu bytes (Tong: %lu). Luu du lieu roi RESTART...", max_block, free_ram);
            nvs_save_stats();
            nvs_save_soc_anchor();
            vTaskDelay(pdMS_TO_TICKS(1000)); 
            esp_restart(); 
        }
        if (free_ram < 51200) {
            ESP_LOGW("BẢO VỆ", "RAM qua thap (%lu bytes)! Luu du lieu roi tu dong restart...", free_ram);
            nvs_save_stats();
            nvs_save_soc_anchor();
            vTaskDelay(pdMS_TO_TICKS(500));
            esp_restart(); 
        }
        
        unsigned long now = (unsigned long)(esp_timer_get_time()/1000ULL);

        // Tích lũy năng lượng nạp/xả theo ngày (cần NTP để biết đúng ngày)
        if (now - last_energy >= 2000) {
            unsigned long dtms = (last_energy==0) ? 2000 : (now-last_energy);
            last_energy = now;
            if (g_ntp_ok) {
                time_t t = time(NULL);
                struct tm tmv; localtime_r(&t, &tmv);
                int yr = tmv.tm_year+1900, doy = tmv.tm_yday;
                if (statYear != yr) { statYear=yr; memset(statCharge,0,sizeof(statCharge)); memset(statDischarge,0,sizeof(statDischarge)); }
                if (doy>=0 && doy<STAT_DAYS) {
                    float dt_h = dtms/3600000.0f;
                    // QUAN TRỌNG: d_Pbat tính từ VbatF*IbatF nên đơn vị là WATT (giống d_Ppv,
                    // chart Công suất phải nhân scale 0.001 để ra kW) — PHẢI đổi ra kW ở đây
                    // trước khi nhân giờ, nếu không statCharge/statDischarge sẽ SAI 1000 LẦN
                    // (thực chất ra Wh chứ không phải kWh như tên field "ch"/"dis" thể hiện).
                    float p_kw = d_Pbat / 1000.0f;
                    if (p_kw>0) statCharge[doy]+=p_kw*dt_h; else statDischarge[doy]+=(-p_kw)*dt_h;
                    // ---- DEBUG TẠM: in ra mỗi lần tích lũy để xác minh statDischarge có tăng không ----
                    ESP_LOGI("STAT_DEBUG", "doy=%d p=%.3fkW dt_h=%.6fh -> ch[doy]=%.4f dis[doy]=%.4f",
                             doy, p_kw, dt_h, statCharge[doy], statDischarge[doy]);
                }
            }
        }
        
        if (now - last_stat_save > 600000UL) {
            last_stat_save = now;
            nvs_save_stats();
            soc_anchor_pct = d_SOC; soc_ah_accum = 0;
            nvs_save_soc_anchor();
        }

        // Kiểm tra yêu cầu vào WiFi Manager AP Mode (từ admin hoặc nút BOOT)
        if (g_start_ap_mode) {
            g_start_ap_mode = false;
            start_wifi_manager(); // hàm này chặn vô hạn, ESP32 sẽ restart sau khi user lưu WiFi mới
        }

        // Cập nhật đèn báo trạng thái theo tình trạng WiFi / NTP hiện tại
        if (g_wifi_connected && g_ntp_ok)      g_led_mode = LED_ONLINE;
        else if (g_wifi_connected)             g_led_mode = LED_NTP_LOST;
        else                                    g_led_mode = LED_WIFI_LOST;

        // ------------------------------------------------------------
        // LỊCH SỬ DỮ LIỆU (15s): ĐẨY TRỰC TIẾP HOẶC STORE & FORWARD
        // ------------------------------------------------------------
        if (now - last_push_hist > 15000) {
            last_push_hist = now;
            
            if (g_wifi_connected && g_ntp_ok) {
                // Có mạng -> Gửi trực tiếp
                push_hist_point();
                
                // Đồng bộ ngầm lên Firebase nếu có hàng tồn trong RAM
                if (g_offline_buf.count > 0 && !is_uploading_offline) {
                    xTaskCreate(upload_offline_task, "upload_off", 4096, NULL, 4, NULL);
                }
            } 
            else {
                // Mất mạng HOẶC chưa có NTP -> vẫn phải đẩy vào Ring Buffer RAM, KHÔNG được bỏ qua.
                // Trước đây điều kiện này là "else if (g_ntp_ok)" nên nếu NTP chưa từng đồng bộ được
                // (không phải do mất mạng) thì rơi vào khoảng trống, dữ liệu bị bỏ hoàn toàn mỗi 15s.
                // Lưu ý: nếu NTP chưa đồng bộ, time(NULL) sẽ cho epoch time gần 1970 (sai) — điểm dữ
                // liệu này sẽ có ngày/giờ không chính xác cho tới khi NTP đồng bộ được, nhưng ít nhất
                // KHÔNG bị mất giá trị đo (V/I/P/SOC).
                offline_buf_push(time(NULL), d_Vpv, d_Ipv, d_Ppv, d_Vbat, d_Ibat, d_Pbat, d_SOC, d_Eff);
                ESP_LOGW("OFFLINE", "Luu RAM: %d/%d diem. (WiFi:%s NTP:%s)",
                    g_offline_buf.count, MAX_OFFLINE_POINTS,
                    g_wifi_connected?"OK":"MAT", g_ntp_ok?"OK":"CHUA");
                
                // Ghi backup xuống Flash định kỳ mỗi 30 phút rớt mạng (1.800.000 ms)
                if (now - last_flash_write_time > 1800000UL) {
                    last_flash_write_time = now;
                    nvs_save_offline_history();
                }
            }
        }

        // Xử lý yêu cầu reconnect (từ nút BOOT hoặc từ web)
        if (g_force_reconnect) {
            g_force_reconnect = false;
            do_wifi_reconnect();
            // chờ tối đa 10s xem có kết nối lại được không
            int w = 0;
            while (!g_wifi_connected && w < 20) { vTaskDelay(pdMS_TO_TICKS(500)); w++; }
            g_led_mode = g_wifi_connected ? LED_ONLINE : LED_WIFI_LOST;
        }

        // *** CHỐT CHẶN GIAO TIẾP FIREBASE (Chỉ chạy khi có mạng & NTP chuẩn) ***
        if (g_wifi_connected && g_ntp_ok) {
            if (now - last_push_cur   > 3000)   { last_push_cur=now;   push_current(); }
            if (now - last_pull_cfg   > 10000)  { last_pull_cfg=now;   pull_config(); }
            if (now - last_pull_ctrl  > 5000)   { last_pull_ctrl=now;  pull_control(); pull_wifi_reconnect(); pull_wifi_reset(); pull_charge_enable(); pull_hw_reset(); }
            if (g_charge_just_completed) { g_charge_just_completed = false; notify_charge_complete(); }
            if (now - last_pull_ota   > 30000)  { last_pull_ota=now;   pull_ota(); }
            if (now - last_stat_push  > 300000UL) { last_stat_push=now; push_daily_today(); }

            // Xác nhận firmware hợp lệ (chống rollback) sau khi chạy mượt 15s
            if (boot_ok_time == 0) boot_ok_time = now;
            else if (now - boot_ok_time > 15000) {
                ota_mark_valid_if_pending();
                boot_ok_time = 0xFFFFFFFF;
            }
        }

        // In log nhịp đập mỗi 2s để theo dõi trạng thái hệ thống
        if (now - last_beat > 2000) {
            last_beat = now;
            ESP_LOGI(TAG, "[%lus] WiFi:%s NTP:%s PV %.1fV/%.2fA Bat %.2fV/%.2fA SOC:%.0f%% Eff:%.1f%% %s",
                now/1000, g_wifi_connected?"OK":"MAT", g_ntp_ok?"OK":"CHUA", d_Vpv, d_Ipv, d_Vbat, d_Ibat, d_SOC, d_Eff, statusText(g_status));
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}