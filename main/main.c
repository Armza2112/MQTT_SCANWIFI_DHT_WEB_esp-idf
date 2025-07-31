#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include <string.h>
#include "esp_http_server.h"
#include "esp_mac.h"
#include "mqtt_client.h"
#include "esp_netif.h"
#include "DHT.h"
#include "driver/gpio.h"
#include <math.h>
#include <time.h>
#include "esp_sntp.h"
#include "cJSON.h"
#include "esp_spiffs.h"

// define Var
#define WIFI_SSID "ESP32"
#define WIFI_PASS "12345678"
#define MAXIMUM_AP 20
#define HISTORY_SIZE 100
#define MAX_STA_CONN 4
#define WIFI_CHANNEL 1
#define WIFI_CONNECTED_BIT BIT0

// define GPIO
#define DHT_GPIO GPIO_NUM_13

// declare function
void wifi_connect(const char *ssid, const char *password);
void wifi_scan_task(void *pvParameters);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);
void add_sensor_reading(float temp, float hum, time_t timestamp);
esp_err_t restart_handle(httpd_req_t *req);
esp_err_t connect_post_handler(httpd_req_t *req);
esp_err_t connect_post_handler(httpd_req_t *req);

// TAG
static const char *TAG_SCAN = "wifi_scan";
static const char *TAG_AP = "wifi_softap_web";
static const char *TAG_INFO = "INFO";
static const char *TAG_RE = "Reconnect";
static const char *TAG_MQTT = "MQTT";
static const char *TAG_DHT22 = "DHT";
static const char *TAG_SPIFFS = "SPIFFS";

// declare var
esp_mqtt_client_handle_t mqtt_client = NULL;
wifi_ap_record_t scanned_aps[MAXIMUM_AP];
uint16_t scanned_ap_count = 0;
TaskHandle_t main_task_handle = NULL;
static EventGroupHandle_t wifi_event_group;
float temp_history[HISTORY_SIZE] = {NAN};
float hum_history[HISTORY_SIZE] = {NAN};
time_t time_history[HISTORY_SIZE] = {0};
int history_index = 0;
static bool user_disconnect = false;

/* Spiffs */
void init_spiffs()
{
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = true};

    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK)
    {
        ESP_LOGE("SPIFFS", "Failed to mount or format filesystem");
    }
    else
    {
        ESP_LOGI("SPIFFS", "SPIFFS mounted successfully");
    }
}
/* Spiffs */

/* Sync time */
void initialize_sntp(void)
{
    ESP_LOGI("SNTP", "Initializing SNTP");
    sntp_setoperatingmode(SNTP_OPMODE_POLL);
    sntp_setservername(0, "pool.ntp.org");
    sntp_init();
}

void wait_for_time_sync(void)
{
    time_t now = 0;
    struct tm timeinfo = {0};
    int retry = 0;
    const int retry_count = 10;

    while (timeinfo.tm_year < (2020 - 1900) && ++retry < retry_count)
    {
        ESP_LOGI("SNTP", "Waiting for system time to be set... (%d/%d)", retry, retry_count);
        vTaskDelay(pdMS_TO_TICKS(2000));
        time(&now);
        localtime_r(&now, &timeinfo);
    }

    if (retry == retry_count)
    {
        ESP_LOGW("SNTP", "Failed to sync time with NTP");
    }
    else
    {
        ESP_LOGI("SNTP", "Time synchronized: %s", asctime(&timeinfo));
    }
}
/* Sync time */
/* WIFI Setup */
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT)
    {
        switch (event_id)
        {
        case WIFI_EVENT_AP_STACONNECTED:
        {
            wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
            ESP_LOGI(TAG_AP, "station " MACSTR " join, AID=%d",
                     MAC2STR(event->mac), event->aid);
            break;
        }
        case WIFI_EVENT_AP_STADISCONNECTED:
        {
            wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
            ESP_LOGI(TAG_AP, "station " MACSTR " leave, AID=%d",
                     MAC2STR(event->mac), event->aid);
            break;
        }
        case WIFI_EVENT_STA_DISCONNECTED:
        {
            xEventGroupClearBits(wifi_event_group, WIFI_CONNECTED_BIT);
            char ssid[33] = {0}, password[65] = {0};
            size_t ssid_len = sizeof(ssid), pass_len = sizeof(password);
            nvs_handle_t nvs_handle;
            if (user_disconnect)
            {
                ESP_LOGI("WIFI", "User disconnect");
                user_disconnect = false;
                
            }
            else
            {
                if (nvs_open("wifi_creds", NVS_READONLY, &nvs_handle) == ESP_OK)
                {
                    if (nvs_get_str(nvs_handle, "ssid", ssid, &ssid_len) == ESP_OK &&
                        nvs_get_str(nvs_handle, "password", password, &pass_len) == ESP_OK)
                    {
                        wifi_config_t wifi_config = {};
                        strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
                        strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));
                        esp_wifi_set_config(WIFI_IF_STA, &wifi_config);
                        esp_wifi_connect();
                        ESP_LOGI("WIFI", "Reconnecting to saved SSID: %s", ssid);
                        vTaskDelay(pdMS_TO_TICKS(60000));
                    }
                    else
                    {
                        esp_wifi_set_mode(WIFI_MODE_APSTA);
                        esp_wifi_start();
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        xTaskCreate(wifi_scan_task, "wifi_scan_task", 8192, NULL, 5, NULL);
                    }
                    nvs_close(nvs_handle);
                }
                else
                {
                    esp_wifi_set_mode(WIFI_MODE_APSTA);
                    esp_wifi_start();
                    vTaskDelay(pdMS_TO_TICKS(1000));
                    xTaskCreate(wifi_scan_task, "wifi_scan_task", 8192, NULL, 5, NULL);
                }
            }
            break;
        }
        }
    }
    else if (event_base == IP_EVENT)
    {
        if (event_id == IP_EVENT_STA_GOT_IP)
        {
            ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
            ESP_LOGI("WIFI", "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
            xEventGroupSetBits(wifi_event_group, WIFI_CONNECTED_BIT);
        }
    }
}
void wifi_connect(const char *ssid, const char *password)
{
    wifi_config_t wifi_sta_config = {0};

    strncpy((char *)wifi_sta_config.sta.ssid, ssid, sizeof(wifi_sta_config.sta.ssid));
    strncpy((char *)wifi_sta_config.sta.password, password, sizeof(wifi_sta_config.sta.password));

    ESP_LOGI(TAG_AP, "Connecting to SSID: %s", ssid);

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_sta_config));

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_ERROR_CHECK(esp_wifi_connect());
}

static const char *auth_mode_type(wifi_auth_mode_t auth_mode)
{
    switch (auth_mode)
    {
    case WIFI_AUTH_OPEN:
        return "OPEN";
    case WIFI_AUTH_WEP:
        return "WEP";
    case WIFI_AUTH_WPA_PSK:
        return "WPA PSK";
    case WIFI_AUTH_WPA2_PSK:
        return "WPA2 PSK";
    case WIFI_AUTH_WPA_WPA2_PSK:
        return "WPA WPA2 PSK";
    case WIFI_AUTH_WPA3_PSK:
        return "WPA3 PSK";
    case WIFI_AUTH_WPA2_WPA3_PSK:
        return "WPA2 WPA3 PSK";
    case WIFI_AUTH_WAPI_PSK:
        return "WAPI PSK";
    default:
        return "UNKNOWN";
    }
}

void wifi_init_apsta(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    esp_netif_create_default_wifi_sta();
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));

    wifi_config_t wifi_ap_config = {
        .ap = {
            .ssid = WIFI_SSID,
            .ssid_len = strlen(WIFI_SSID),
            .channel = WIFI_CHANNEL,
            .password = WIFI_PASS,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };
    if (strlen(WIFI_PASS) == 0)
    {
        wifi_ap_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_APSTA));

    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_ap_config));

    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG_AP, "SoftAP started. SSID:%s, password:%s", WIFI_SSID, WIFI_PASS);
}

void wifi_scan_task(void *pvParameters)
{
    while (1)
    {
        wifi_ap_record_t connected_info;
        esp_err_t connected = esp_wifi_sta_get_ap_info(&connected_info);
        if (connected == ESP_OK)
        {
            ESP_LOGI(TAG_SCAN, "Connected to SSID: %s. Stop scanning.", (char *)connected_info.ssid);
            vTaskDelete(NULL);
            return;
        }
        wifi_mode_t mode;
        esp_err_t err_mode = esp_wifi_get_mode(&mode);
        if (err_mode != ESP_OK || mode == WIFI_MODE_NULL)
        {
            ESP_LOGE(TAG_SCAN, "Wi-Fi not initialized. Abort scan.");
            vTaskDelete(NULL);
            return;
        }

        wifi_scan_config_t scan_config = {
            .ssid = NULL,
            .bssid = NULL,
            .channel = 0,
            .show_hidden = true};

        ESP_LOGI(TAG_SCAN, "Starting scan...");
        esp_err_t err_scan = esp_wifi_scan_start(&scan_config, true);
        if (err_scan != ESP_OK)
        {
            ESP_LOGE(TAG_SCAN, "Scan start failed: %s", esp_err_to_name(err_scan));
            vTaskDelete(NULL);
            return;
        }

        uint16_t ap_num = MAXIMUM_AP;
        wifi_ap_record_t ap_records[MAXIMUM_AP];
        memset(ap_records, 0, sizeof(ap_records));

        esp_err_t err_get = esp_wifi_scan_get_ap_records(&ap_num, ap_records);
        if (err_get != ESP_OK)
        {
            ESP_LOGE(TAG_SCAN, "Failed to get AP records: %s", esp_err_to_name(err_get));
            vTaskDelete(NULL);
            return;
        }
        ESP_LOGI(TAG_SCAN, "Found %d APs", ap_num);
        vTaskDelay(pdMS_TO_TICKS(5000));
        for (int i = 0; i < ap_num; i++)
        {
            char ssid[33] = {0};
            memcpy(ssid, ap_records[i].ssid, sizeof(ap_records[i].ssid));
            ssid[32] = '\0';
            if (strlen(ssid) == 0)
            {
                ESP_LOGI(TAG_SCAN, "AP %d: SSID: <hidden>, Channel: %d, RSSI: %d, Authmode: %s",
                         i, ap_records[i].primary, ap_records[i].rssi,
                         auth_mode_type(ap_records[i].authmode));
            }
            else
            {
                ESP_LOGI(TAG_SCAN, "AP %d: SSID: %s, Channel: %d, RSSI: %d, Authmode: %s",
                         i, ssid, ap_records[i].primary, ap_records[i].rssi,
                         auth_mode_type(ap_records[i].authmode));
            }
        }
        scanned_ap_count = ap_num;
        memcpy(scanned_aps, ap_records, sizeof(wifi_ap_record_t) * ap_num);
        if (main_task_handle != NULL)
        {
            xTaskNotifyGive(main_task_handle);
        }
    }

    vTaskDelete(NULL);
}
void connect_wifi_nvs()
{
    char ssid[33] = {0};
    char password[65] = {0};
    size_t ssid_len = sizeof(ssid);
    size_t pass_len = sizeof(password);

    nvs_handle_t nvs_handle;
    wifi_ap_record_t ap_info;
    esp_err_t ret = esp_wifi_sta_get_ap_info(&ap_info);
    esp_err_t err = nvs_open("wifi_creds", NVS_READONLY, &nvs_handle);
    if (err == ESP_OK)
    {
        if (nvs_get_str(nvs_handle, "ssid", ssid, &ssid_len) == ESP_OK &&
            nvs_get_str(nvs_handle, "password", password, &pass_len) == ESP_OK)
        {

            wifi_connect(ssid, password);
            if (ret == ESP_OK)
            {
                ESP_LOGI(TAG_AP, "Connecting to saved wifi: SSID=%s", ssid);
            }
            else
            {
                ESP_LOGI(TAG_RE, "Can't Connecting: %s", ssid);
            }
        }
        else
        {
            ESP_LOGI(TAG_AP, "No wifi credentials in NVS");
            xTaskCreate(wifi_scan_task, "wifi_scan_task", 8192, NULL, 5, NULL);
        }
        nvs_close(nvs_handle);
    }
    else
    {
        ESP_LOGI(TAG_AP, "Cannot open NVS");
    }
}
/* WIFI Setup */
/* WEB PAGE */
esp_err_t home(httpd_req_t *req)
{
    wifi_config_t sta_config;
    esp_wifi_get_config(WIFI_IF_STA, &sta_config);

    char ssid[33] = {0};
    size_t ssid_len = sizeof(ssid);
    nvs_handle_t nvs_handle;

    if (nvs_open("wifi_creds", NVS_READONLY, &nvs_handle) == ESP_OK)
    {
        if (nvs_get_str(nvs_handle, "ssid", ssid, &ssid_len) == ESP_OK)
        {
        }
        else
        {
            strcpy(ssid, "Not connected");
        }
        nvs_close(nvs_handle);
    }
    else
    {
        strcpy(ssid, "Not connected");
    }

    const char *ssid_display = ssid;
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    ESP_LOGI(TAG_INFO, "MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    char device_id[32];
    snprintf(device_id, sizeof(device_id), "%02X%02X%02X%02X%02X%02X",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    char html[1024];
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
    {
        int8_t rssi = ap_info.rssi;
        ESP_LOGI(TAG_INFO, "Signal strength: %d dBm", rssi);
    }
    snprintf(html, sizeof(html),
             "<!DOCTYPE html><html><head><title>Welcome to my esp32</title></head><body>"
             "<h1>Welcome to my esp32</h1>"
             "<p>Model: IoT6D</p>"
             "<p>Version: 1.01</p>"
             "<p>Wifi: %s</p>"
             "<p>MAC Address: %02X:%02X:%02X:%02X:%02X:%02X</p>"
             "<p>Device id: %02X%02X%02X%02X%02X%02X</p>"
             "<p>Signal wifi%d</p>"
             "<button onclick=\"location.href='/wifimanage'\">Wifi manage</button>"
             "<button onclick=\"location.href='/restart'\">Restart</button>"
             "</body></html>",
             ssid_display, mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], mac[0], mac[1], mac[2], mac[3], mac[4], mac[5], ap_info.rssi);

    return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
}

esp_err_t wifi_manage(httpd_req_t *req)
{
    esp_err_t err;
    wifi_ap_record_t ap_info;

    char ssid[33] = {0};
    char password[65] = {0};
    size_t ssid_len = sizeof(ssid);
    size_t pass_len = sizeof(password);

    nvs_handle_t nvs_handle;

    esp_err_t err_nvs = nvs_open("wifi_creds", NVS_READONLY, &nvs_handle);
    if (err_nvs == ESP_OK)
    {
        if (nvs_get_str(nvs_handle, "ssid", ssid, &ssid_len) == ESP_OK &&
            nvs_get_str(nvs_handle, "password", password, &pass_len) == ESP_OK)
        {
            wifi_config_t sta_config;
            esp_wifi_get_config(WIFI_IF_STA, &sta_config);

            const char *ssid_display = strlen((char *)sta_config.sta.ssid) > 0 ? (char *)sta_config.sta.ssid : "Not connected";

            char html[1024];
            snprintf(html, sizeof(html),
                     "<!DOCTYPE html><html><head><title>Welcome to my esp32</title></head><body>"
                     "<p>Wifi: %s</p>"
                     "<button onclick=\"location.href='/disconnect'\">Dis</button>"
                     "</body></html>",
                     ssid_display);

            return httpd_resp_send(req, html, HTTPD_RESP_USE_STRLEN);
        }
        else
        {
            ESP_LOGI(TAG_AP, "No wifi credentials in NVS");
            const char *html_start =
                "<!DOCTYPE html><html><head><title>WiFi Connect</title></head><body>"
                "<h1>select your wifi</h1>"
                "<form method=\"POST\" action=\"/connect\">"
                "<label for=\"ssid\">SSID:</label>"
                "<select id=\"ssid\" name=\"ssid\">";
            err = httpd_resp_send_chunk(req, html_start, strlen(html_start));
            if (err != ESP_OK)
                return err;

            for (int i = 0; i < scanned_ap_count; i++)
            {
                char ssid[33];
                memcpy(ssid, scanned_aps[i].ssid, sizeof(scanned_aps[i].ssid));
                ssid[32] = '\0';

                char option[128];
                int option_len = snprintf(option, sizeof(option),
                                          "<option value=\"%s\">%s</option>", ssid, ssid);
                err = httpd_resp_send_chunk(req, option, option_len);
                if (err != ESP_OK)
                    return err;
            }

            const char *html_end =
                "</select><br><br>"
                "<label for=\"pass\">Password:</label>"
                "<input type=\"password\" id=\"pass\" name=\"password\" /><br><br>"
                "<input type=\"submit\" value=\"Connect\" />"
                "</form></body></html>";
            err = httpd_resp_send_chunk(req, html_end, strlen(html_end));
            if (err != ESP_OK)
                return err;

            return httpd_resp_send_chunk(req, NULL, 0);
        }
        nvs_close(nvs_handle);
    }
    else
    {
        ESP_LOGI(TAG_AP, "Cannot open NVS");
    }

    const char *html_start =
        "<!DOCTYPE html><html><head><title>WiFi Connect</title></head><body>"
        "<h1>select your wifi</h1>"
        "<form method=\"POST\" action=\"/connect\">"
        "<label for=\"ssid\">SSID:</label>"
        "<select id=\"ssid\" name=\"ssid\">";
    err = httpd_resp_send_chunk(req, html_start, strlen(html_start));
    if (err != ESP_OK)
        return err;

    for (int i = 0; i < scanned_ap_count; i++)
    {
        char ssid[33];
        memcpy(ssid, scanned_aps[i].ssid, sizeof(scanned_aps[i].ssid));
        ssid[32] = '\0';

        char option[128];
        int option_len = snprintf(option, sizeof(option),
                                  "<option value=\"%s\">%s</option>", ssid, ssid);
        err = httpd_resp_send_chunk(req, option, option_len);
        if (err != ESP_OK)
            return err;
    }

    const char *html_end =
        "</select><br><br>"
        "<label for=\"pass\">Password:</label>"
        "<input type=\"password\" id=\"pass\" name=\"password\" /><br><br>"
        "<input type=\"submit\" value=\"Connect\" />"
        "</form></body></html>";
    err = httpd_resp_send_chunk(req, html_end, strlen(html_end));
    if (err != ESP_OK)
        return err;

    return httpd_resp_send_chunk(req, NULL, 0);
}
void disconnect_wifi_task(void *pvParameter)
{
    vTaskDelay(pdMS_TO_TICKS(1000));

    ESP_LOGI(TAG_AP, "Disconnect wifi task running");
    esp_wifi_disconnect();
    user_disconnect = true;
    vTaskDelay(pdMS_TO_TICKS(1000));
    nvs_handle_t nvs_handle;
    if (nvs_open("wifi_creds", NVS_READWRITE, &nvs_handle) == ESP_OK)
    {
        nvs_erase_key(nvs_handle, "ssid");
        nvs_erase_key(nvs_handle, "password");
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    }
    esp_wifi_set_mode(WIFI_MODE_APSTA);
    esp_wifi_start();
    vTaskDelay(pdMS_TO_TICKS(1000));
    xTaskCreate(wifi_scan_task, "wifi_scan_task", 8192, NULL, 5, NULL);
    vTaskDelete(NULL);
}

esp_err_t disconnect_handle(httpd_req_t *req)
{
    ESP_LOGI(TAG_AP, "Disconnect");

    const char *resp = "<html><body><h1>Wi-Fi Disconnected</h1>"
                       "<a href='/'>Back to Home</a></body></html>";

    esp_err_t err = httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG_AP, "httpd_resp_send failed: %s", esp_err_to_name(err));
    }

    xTaskCreate(disconnect_wifi_task, "disconnect_wifi_task", 8192, NULL, 5, NULL);

    return ESP_OK;
}
esp_err_t log_file_handler(httpd_req_t *req)
{
    FILE *f = fopen("/spiffs/log.txt", "r");
    if (!f)
    {
        return httpd_resp_send(req, "Cannot open log.txt", HTTPD_RESP_USE_STRLEN);
    }

    char line[128];
    httpd_resp_sendstr_chunk(req, "<html><body><pre>");

    while (fgets(line, sizeof(line), f))
    {
        httpd_resp_sendstr_chunk(req, line);
    }

    httpd_resp_sendstr_chunk(req, "</pre></body></html>");
    httpd_resp_sendstr_chunk(req, NULL);
    fclose(f);
    return ESP_OK;
}

static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    httpd_handle_t server = NULL;

    if (httpd_start(&server, &config) == ESP_OK)
    {
        httpd_uri_t home_uri = {
            .uri = "/",
            .method = HTTP_GET,
            .handler = home,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &home_uri);

        httpd_uri_t uri_get = {
            .uri = "/wifimanage",
            .method = HTTP_GET,
            .handler = wifi_manage,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &uri_get);

        httpd_uri_t uri_post = {
            .uri = "/connect",
            .method = HTTP_POST,
            .handler = connect_post_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &uri_post);
        httpd_uri_t disconnect_get = {
            .uri = "/disconnect",
            .method = HTTP_GET,
            .handler = disconnect_handle,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &disconnect_get);
        httpd_uri_t restart_get = {
            .uri = "/restart",
            .method = HTTP_GET,
            .handler = restart_handle,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &restart_get);
        httpd_uri_t log_uri = {
            .uri = "/log",
            .method = HTTP_GET,
            .handler = log_file_handler,
            .user_ctx = NULL};
        httpd_register_uri_handler(server, &log_uri);
    }

    return server;
}
esp_err_t connect_post_handler(httpd_req_t *req)
{
    char buf[256];
    int ret, remaining = req->content_len;
    char ssid[33] = {0};
    char password[65] = {0};

    int idx = 0;
    while (remaining > 0 && idx < sizeof(buf) - 1)
    {
        ret = httpd_req_recv(req, buf + idx, remaining > sizeof(buf) - idx - 1 ? sizeof(buf) - idx - 1 : remaining);
        if (ret <= 0)
        {
            return ESP_FAIL;
        }
        remaining -= ret;
        idx += ret;
    }
    buf[idx] = '\0';

    ESP_LOGI(TAG_AP, "Received POST data: %s", buf);

    char *ssid_start = strstr(buf, "ssid=");
    char *pass_start = strstr(buf, "password=");
    if (!ssid_start)
        return ESP_FAIL;

    ssid_start += strlen("ssid=");
    char *ssid_end = strchr(ssid_start, '&');
    if (ssid_end)
    {
        int len = ssid_end - ssid_start;
        if (len > 32)
            len = 32;
        strncpy(ssid, ssid_start, len);
        ssid[len] = '\0';
    }
    else
    {
        strncpy(ssid, ssid_start, 32);
        ssid[32] = '\0';
    }

    if (pass_start)
    {
        pass_start += strlen("password=");
        char *pass_end = strchr(pass_start, '&');
        int len = pass_end ? (pass_end - pass_start) : strlen(pass_start);
        if (len > 64)
            len = 64;
        strncpy(password, pass_start, len);
        password[len] = '\0';
    }

    ESP_LOGI(TAG_AP, "Parsed SSID: %s", ssid);

    wifi_connect(ssid, password);
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, pdFALSE, pdFALSE, pdMS_TO_TICKS(10000));

    if (bits & WIFI_CONNECTED_BIT)
    {
        nvs_handle_t nvs_handle;
        esp_err_t err = nvs_open("wifi_creds", NVS_READWRITE, &nvs_handle);
        if (err == ESP_OK)
        {
            nvs_set_str(nvs_handle, "ssid", ssid);
            nvs_set_str(nvs_handle, "password", password);
            nvs_commit(nvs_handle);
            nvs_close(nvs_handle);
        }
        else
        {
            ESP_LOGI(TAG_AP, "Fail to save in NVS%s", esp_err_to_name(err));
        }
        const char *resp = "<!DOCTYPE html><html><body><h1>connecting WiFi...</h1></body>"
                           "<a href='/'>Back to Home</a></body></html>";
        httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    }
    else
    {
        const char *resp = "<!DOCTYPE html><html><body><h1>Connect fail</h1>"
                           "<p>Wrong password</p>"
                           "<a href='/wifimanage'>Home</a></body></html>";
        return httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
    }

    return ESP_OK;
}
esp_err_t restart_handle(httpd_req_t *req)
{
    const char *resp = "<html><body><h1>Restarting...</h1></body></html>";
    httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);

    vTaskDelay(pdMS_TO_TICKS(1000));

    esp_restart();

    return ESP_OK;
}
/* WEB PAGE */
/* Read Sensor */
void read_dht()
{
    setDHTgpio(DHT_GPIO);
    setenv("TZ", "ICT-7", 1);
    tzset();
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        int ret = readDHT();
        time_t now = time(NULL);
        struct tm timeinfo;
        localtime_r(&now, &timeinfo);
        char timestamp[32];
        strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &timeinfo);
        errorHandler(ret);
        float temp = getTemperature();
        float hum = getHumidity();

        if ((!isnan(temp) && !isnan(hum)) && (temp != 0 && hum != 0))
        {
            char payload[200];
            snprintf(payload, sizeof(payload),
                     "{\"timestamp\": \"%s\", \"temperature\": %.2f, \"humidity\": %.2f}",
                     timestamp, temp, hum);

            ESP_LOGI(TAG_DHT22, "Published payload: %s", payload);

            FILE *f = fopen("/spiffs/log.txt", "a");
            if (f)
            {
                fprintf(f, "{\"time\":\"%s\",\"temp\":%.2f,\"hum\":%.2f}\n", timestamp, temp, hum);
                fclose(f);
                ESP_LOGI(TAG_SPIFFS, "Save to spiffs");
            }

            add_sensor_reading(temp, hum, now);
        }
        else
        {
            ESP_LOGI(TAG_DHT22, "Sensor broken");
        }

        vTaskDelay(pdMS_TO_TICKS(60000));
    }
}
void add_sensor_reading(float temp, float hum, time_t timestamp)
{
    temp_history[history_index] = temp;
    hum_history[history_index] = hum;
    time_history[history_index] = timestamp;

    history_index = (history_index + 1) % HISTORY_SIZE;
}
/* Read Sensor */
/* MQTT */
void mqtt_init()
{
    wifi_event_group = xEventGroupCreate();

    xEventGroupWaitBits(wifi_event_group, WIFI_CONNECTED_BIT, false, true, portMAX_DELAY);
    initialize_sntp();
    wait_for_time_sync();

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://broker.hivemq.com",
    };

    mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
    // xTaskCreate(&mqtt_reconnect, "mqtt_reconnect", 4096, NULL, 5, NULL);
}
// void mqtt_reconnect(void *pvParameters)
// {
//     while (1)
//     {
//         if (!esp_mqtt_client_disconnect(mqtt_client))
//         {
//             ESP_LOGW("MQTT", "Not connected, reconnecting...");
//             esp_mqtt_client_reconnect(mqtt_client);
//         }
//         vTaskDelay(pdMS_TO_TICKS(60000));
//     }
// }
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;

    switch (event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT Connected");
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGI(TAG_MQTT, "MQTT Disconnected");
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG_MQTT, "Message published (msg_id=%d)", event->msg_id);
        break;
    default:
        ESP_LOGI(TAG_MQTT, "Other MQTT event id:%ld", (long)event_id);
        break;
    }
}
void mqtt_publish_task(void *pvParameters)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    while (1)
    {
        int last_index = (history_index - 1 + HISTORY_SIZE) % HISTORY_SIZE;

        if (time_history[last_index] != 0 &&
            !isnan(temp_history[last_index]) &&
            !isnan(hum_history[last_index]))
        {
            char timestamp[32];
            struct tm timeinfo;
            localtime_r(&time_history[last_index], &timeinfo);
            strftime(timestamp, sizeof(timestamp), "%Y-%m-%dT%H:%M:%S", &timeinfo);

            char payload[200];
            snprintf(payload, sizeof(payload),
                     "{\"timestamp\": \"%s\", \"temperature\": %.2f, \"humidity\": %.2f}",
                     timestamp, temp_history[last_index], hum_history[last_index]);

            ESP_LOGI(TAG_MQTT, "Published payload: %s", payload);
            esp_mqtt_client_publish(mqtt_client, "sensor/dht", payload, 0, 1, 0);
        }
        else
        {
            ESP_LOGW("MQTT", "No valid data to publish");
        }

        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(60000)); // ทำให้ตรงรอบ
    }
}

/*MQTT*/

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_INFO);
    esp_rom_gpio_pad_select_gpio(DHT_GPIO);
    ESP_ERROR_CHECK(nvs_flash_init());

    wifi_init_apsta();
    init_spiffs();
    xTaskCreate(&read_dht, "read_dht", 4096, NULL, 5, NULL);
    connect_wifi_nvs();
    start_webserver();
    // MQTT TASK
    mqtt_init();
    xTaskCreate(&mqtt_publish_task, "mqtt_publish_task", 4096, NULL, 5, NULL);
}