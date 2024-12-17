#include <stdio.h>
#include <string.h>
#include <math.h>  
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_err.h"
#include "esp_log.h"
#include "esp_spiffs.h"
#include "mbedtls/md5.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "driver/adc.h"
#include "driver/gpio.h"
#include "soc/gpio_num.h"
#include "mqtt_client.h" // para MQTT
#include "driver/spi_master.h"
#include "cJSON.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "ssd1306.h"

static const char *TAG = "SPIFFS";
static const char *TAG1 = "STA";
static const char *TAG2 = "Eventos_WiFi";
static const char *TAG_RELAY = "Relay";

#define THINGSBOARD_TOKEN "JhSpgw9YY6a5wDhln3x3"
#define THINGSBOARD_RELAY_TOKEN "2j72K2p5Vei2AxNjbX4d" // Token para el dispositivo del relé
#define THINGSBOARD_SERVER "demo.thingsboard.io"
#define THINGSBOARD_PORT 1883 // Para MQTT
static esp_mqtt_client_handle_t mqtt_client;
static esp_mqtt_client_handle_t mqtt_relay_client; // Cliente MQTT para el relé

#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1
// Definir el canal ADC donde está conectada la fotoresistencia
#define ADC_CHANNEL ADC1_CHANNEL_2 // GPIO34
#define ISNS20_CS_PIN 5
#define SPI_SCLK_PIN 18
#define SPI_MISO_PIN 19
#define RELAY_PIN GPIO_NUM_4 // Pin GPIO para el relé
#define PIR_PIN GPIO_NUM_25
#define I2C_SCL 22  // Pin SCL
#define I2C_SDA 21  // Pin SDA
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_MASTER_FREQ_HZ 100000
#define OLED_ADDR 0x3C  // Dirección I2C de la pantalla OLED

SSD1306_t dev;  // Estructura para manejar la pantalla
spi_device_handle_t spi;

static EventGroupHandle_t s_wifi_event_group;
static bool is_ap_mode = false;
static int s_retry_num = 0;

//PROTOPTIPOS DE FUNCIONES
void start_lux_monitoring();
void monitor_pir_and_relay();
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data);
void wifi_init_sta(char* ssid, char* password);
void wifi_init_softap(void);
static void read_wifi_credentials_and_connect(void);
static httpd_handle_t start_webserver(void);
static esp_err_t connect_handler(httpd_req_t *req);
static esp_err_t root_handler(httpd_req_t *req);
void mqtt_event_handler(esp_mqtt_event_handle_t event);
void mqtt_connect();
void mqtt_relay_connect();
void send_to_thingsboard_mqtt(float current);
void send_relay_status_to_thingsboard(bool relay_state);
void handle_relay_command(const char *payload);
void init_relay();
void relay_on();
void relay_off();

// Configuración del SPI para ISNS20

void init_spi() {
    spi_bus_config_t buscfg = {
        .mosi_io_num = -1, // No utilizado
        .miso_io_num = SPI_MISO_PIN,
        .sclk_io_num = SPI_SCLK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 1 * 1000 * 1000, // 1 MHz
        .mode = 0, // Modo SPI 0
        .spics_io_num = ISNS20_CS_PIN,
        .queue_size = 1,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(SPI2_HOST, &devcfg, &spi));
}

// Lectura del sensor ISNS20


float read_current() {
    uint8_t rx_data[2] = {0}; // Para recibir los datos del sensor
    spi_transaction_t trans = {
        .length = 16, // 16 bits
        .rx_buffer = rx_data,
    };
    ESP_ERROR_CHECK(spi_device_transmit(spi, &trans));

    // Procesar los datos recibidos (conversión a corriente)
    uint16_t raw_value = (rx_data[0] << 8) | rx_data[1];

    // Aplicar la fórmula de conversión a corriente (en mA)
    float current_mA = (((float)(raw_value - 2048) * 1000) / 89.95);

    // Convertir a amperios (A) dividiendo entre 1000
    float current_A = current_mA;

    // Obtener el valor absoluto de la corriente
    current_A = fabs(current_A);  // Convierte el valor a su valor absoluto

    return current_A;
}



void mqtt_event_handler(esp_mqtt_event_handle_t event) {
    if (event == NULL) {
        ESP_LOGE(TAG, "Manejador de eventos MQTT: evento nulo");
        return;
    }
    
    switch (event->event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "Conectado a ThingsBoard MQTT");
            esp_mqtt_client_subscribe(mqtt_relay_client, "v1/devices/me/rpc/request/+", 1); // Suscripción para comandos del relé
            break;
        case MQTT_EVENT_DATA:
            ESP_LOGI(TAG, "Mensaje recibido: topic=%.*s, payload=%.*s", 
                     event->topic_len, event->topic, 
                     event->data_len, event->data);
            handle_relay_command(event->data);
            break;
        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "Desconectado de MQTT");
            break;
        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "Error en la conexión MQTT");
            break;
        default:
            break;
    }
}

void mqtt_connect() {
    esp_mqtt_client_config_t mqtt_config = {
        .broker.address.uri = "mqtt://" THINGSBOARD_SERVER,
        .credentials.username = THINGSBOARD_TOKEN,
        .broker.address.port = THINGSBOARD_PORT,

    };

    mqtt_client = esp_mqtt_client_init(&mqtt_config);
    if (mqtt_client == NULL) {
        ESP_LOGE(TAG, "Error al inicializar el cliente MQTT");
        return;
    }

    esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_CONNECTED, mqtt_event_handler, NULL);
    esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_DISCONNECTED, mqtt_event_handler, NULL);
    esp_mqtt_client_register_event(mqtt_client, MQTT_EVENT_ERROR, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_client);
}

void mqtt_relay_connect() {
    esp_mqtt_client_config_t mqtt_config = {
        .broker.address.uri = "mqtt://" THINGSBOARD_SERVER,
        .credentials.username = THINGSBOARD_RELAY_TOKEN,
        .broker.address.port = THINGSBOARD_PORT,
    };

    mqtt_relay_client = esp_mqtt_client_init(&mqtt_config);
    if (mqtt_relay_client == NULL) {
        ESP_LOGE(TAG_RELAY, "Error al inicializar el cliente MQTT para el relé");
        return;
    }

    esp_mqtt_client_register_event(mqtt_relay_client, MQTT_EVENT_CONNECTED, mqtt_event_handler, NULL);
    esp_mqtt_client_start(mqtt_relay_client);
}

void send_relay_status_to_thingsboard(bool relay_state) {
    if (mqtt_relay_client == NULL) {
        ESP_LOGE(TAG_RELAY, "Cliente MQTT para el relé no inicializado.");
        return;
    }

    char payload[64];
    snprintf(payload, sizeof(payload), "{\"relay_status\": \"%s\"}", relay_state ? "on" : "off");
    esp_mqtt_client_publish(mqtt_relay_client, "v1/devices/me/telemetry", payload, 0, 1, 0);
    ESP_LOGI(TAG_RELAY, "Estado del relé enviado: %s", payload);
}

void handle_relay_command(const char *payload) {
    cJSON *json = cJSON_Parse(payload);
    if (json == NULL) {
        ESP_LOGE(TAG_RELAY, "Error al analizar el JSON del comando");
        return;
    }

    cJSON *method = cJSON_GetObjectItem(json, "method");
    cJSON *params = cJSON_GetObjectItem(json, "params");

    if (method) {
        // Manejo del estado inicial del interruptor
        if (strcmp(method->valuestring, "getValue") == 0) {
            bool relay_state = gpio_get_level(RELAY_PIN);
            send_relay_status_to_thingsboard(relay_state);
        }
        // Manejo de comandos para controlar el relé
        else if (strcmp(method->valuestring, "relay_control") == 0 && params) {
            if (strcmp(params->valuestring, "on") == 0) {
                relay_on();
            } else if (strcmp(params->valuestring, "off") == 0) {
                relay_off();
            }
        }
    }

    cJSON_Delete(json);
}

void init_relay() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << RELAY_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(RELAY_PIN, 0); // Apagar relé inicialmente
}

void relay_on() {
    gpio_set_level(RELAY_PIN, 1);
    ESP_LOGI(TAG_RELAY, "Relé encendido");
    send_relay_status_to_thingsboard(true);
}

void relay_off() {
    gpio_set_level(RELAY_PIN, 0);
    ESP_LOGI(TAG_RELAY, "Relé apagado");
    send_relay_status_to_thingsboard(false);
}

void send_to_thingsboard_mqtt(float current) {
    char payload[64];
    // Formatear el payload con el valor de corriente en amperios
    snprintf(payload, sizeof(payload), "{\"current\": %.3f}", current);
    // Publicar el payload a ThingsBoard a través de MQTT
    esp_mqtt_client_publish(mqtt_client, "v1/devices/me/telemetry", payload, 0, 1, 0);
    ESP_LOGI(TAG, "Datos enviados a ThingsBoard (MQTT): %s", payload);
}

// Página HTML que contiene el formulario
const char* html_form = "<form action='/connect' method='get'>"
                        "SSID:<input type='text' name='ssid'><br>"
                        "Password:<input type='password' name='pass'><br>"
                        "<input type='submit' value='Submit'></form>";
void init_pir() {
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIR_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    ESP_LOGI(TAG, "Sensor PIR inicializado.");
}

// Función para iniciar la monitorización de la LDR
void start_lux_monitoring() {
    // Inicializar el SPI
    init_spi();
    i2c_master_init(&dev, I2C_SDA, I2C_SCL, -1);
     ssd1306_init(&dev, 128, 64); // Ajusta el tamaño según tu pantalla
    ssd1306_clear_screen(&dev, false); // Limpia la pantalla al inicio
    init_relay();
    init_pir();
    mqtt_connect(); // Conectar al servidor MQTT
    mqtt_relay_connect(); // Conectar el cliente del relé
    while (1) {
        // Leer el estado del sensor PIR
        int pir_state = gpio_get_level(PIR_PIN);

        // Encender o apagar el relé según el estado del PIR
        if (pir_state) {
            ESP_LOGI(TAG, "Movimiento detectado. Encendiendo el relé.");
            relay_on();
            
        } else {
            ESP_LOGI(TAG, "Sin movimiento. Apagando el relé.");
            relay_off();
        }

        // Leer la corriente y enviarla a ThingsBoard
        float current = read_current();
        ESP_LOGI(TAG, "Corriente medida: %.3f mA", current);
        send_to_thingsboard_mqtt(current);
        
        char buffer[20];
        snprintf(buffer, sizeof(buffer), "current mA: %.3f", current);
        
        // Limpiar la pantalla antes de dibujar nuevo texto
        ssd1306_clear_screen(&dev, false);
        // Dibujar el texto en la pantalla
        ssd1306_display_text(&dev, 0, buffer, strlen(buffer), false);
        // Actualizar el display
        ssd1306_show_buffer(&dev);

        // Esperar 1 segundo antes de la siguiente lectura
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// Inicia el ESP32 en modo Access Point (AP)
void wifi_init_softap(void)
{
    ESP_LOGI(TAG1, "Configuring Access Point...");
    esp_netif_create_default_wifi_ap();
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = "ESP32_AP",            // SSID del ESP32
            .ssid_len = strlen("ESP32_AP"),
            .password = "12345678",        // Contraseña del AP
            .max_connection = 4,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
            .beacon_interval = 100,
        },
    };

    if (strlen((char*)wifi_config.ap.password) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG1, "AP mode started, SSID: ESP32_AP, password: 12345678");


	 // Cambia el estado de AP a "activo"
    is_ap_mode = true;
    // Inicia el servidor web para servir el formulario
    start_webserver();
}

// Maneja las solicitudes GET a la ruta raíz "/"
esp_err_t root_handler(httpd_req_t *req) {
    httpd_resp_send(req, html_form, strlen(html_form));
    return ESP_OK;
}

// Maneja las solicitudes GET del formulario

// Manejador de solicitudes HTTP GET para recibir los parámetros de SSID y contraseña
esp_err_t connect_handler(httpd_req_t *req) {
    char query[128] = {0};       // Buffer para almacenar la cadena de consulta (query string)
    char ssid[32] = {0};         // Buffer para almacenar el SSID
    char pass[64] = {0};         // Buffer para almacenar la contraseña

    // Obtiene la cadena de consulta (query string) de la solicitud
    if (httpd_req_get_url_query_str(req, query, sizeof(query)) == ESP_OK) {
        ESP_LOGI(TAG, "Query string: %s", query);

        // Extrae el parámetro 'ssid' de la cadena de consulta
        if (httpd_query_key_value(query, "ssid", ssid, sizeof(ssid)) == ESP_OK) {
            ESP_LOGI(TAG, "SSID: %s", ssid);
        } else {
            ESP_LOGW(TAG, "No se pudo obtener el SSID");
        }

        // Extrae el parámetro 'pass' de la cadena de consulta
        if (httpd_query_key_value(query, "pass", pass, sizeof(pass)) == ESP_OK) {
            ESP_LOGI(TAG, "Password: %s", pass);
        } else {
            ESP_LOGW(TAG, "No se pudo obtener la contraseña");
        }

        // Comprueba si tanto el SSID como la contraseña tienen valores válidos
        if (strlen(ssid) > 0 && strlen(pass) > 0) {
            // Llama a la función para conectar a la red WiFi con las credenciales proporcionadas
            wifi_init_sta(ssid, pass);
        } else {
            ESP_LOGI("ERROR","SSID o contraseña no proporcionados.");
        }
    } 

    return ESP_OK;
}


// Inicia un servidor web simple
static httpd_handle_t start_webserver(void)
{
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();  // Configuración del servidor HTTP
    config.max_open_sockets = 4; 
    httpd_handle_t server = NULL;

    // Inicia el servidor HTTP
    if (httpd_start(&server, &config) == ESP_OK) {
        
        // URI handler para la ruta raíz "/"
        httpd_uri_t root_uri = {
            .uri = "/",  // Maneja la URL "/"
            .method = HTTP_GET,
            .handler = root_handler,  // Handler que sirve el formulario HTML
            .user_ctx = NULL
        };

        // URI handler para la ruta "/connect"
        httpd_uri_t connect_uri = {
            .uri = "/connect",   // Maneja la URL "/connect"
            .method = HTTP_GET,
            .handler = connect_handler,  // Handler que gestiona la conexión WiFi
            .user_ctx = NULL
        };

        // Registra ambos URI handlers
        httpd_register_uri_handler(server, &root_uri);  // Registrar la ruta "/"
        httpd_register_uri_handler(server, &connect_uri);  // Registrar la ruta "/connect"
    }

    return server;
}



// Manejador de eventos de Wi-Fi
static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                               int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < 5) {
            esp_wifi_connect();  // Reintenta la conexión
            s_retry_num++;
            ESP_LOGI(TAG2, "Retrying to connect to the AP");
        } else {
            ESP_LOGI(TAG2, "Failed to connect after 5 attempts, switching to AP mode");
            wifi_init_softap();  // Cambia al modo AP si fallan los intentos
        }
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG2, "Got IP:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
      
    }
}

// FUNCION PARA CONECTAR EL ESP32 COMO STA
void wifi_init_sta(char* ssid, char* password)
{
    // Si estamos en modo AP, primero detén el AP
    if (is_ap_mode) {
        ESP_LOGI(TAG, "Switching from AP mode to STA mode");
        ESP_ERROR_CHECK(esp_wifi_stop());
        is_ap_mode = false;
    }

    s_wifi_event_group = xEventGroupCreate();

    // Verifica si ya existe la interfaz STA
    esp_netif_t* sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif == NULL) {
        // Solo creamos la interfaz si no existe
        esp_netif_init();
        esp_event_loop_create_default();

        // Crear la interfaz STA explícitamente si no existe
        sta_netif = esp_netif_create_default_wifi_sta();
        if (sta_netif == NULL) {
            ESP_LOGE(TAG, "Failed to create STA netif");
            return;
        }
    }

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_wifi_init(&cfg);

    // Registrar los manejadores de eventos
    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, &instance_any_id);
    esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, &instance_got_ip);

    // Configura el SSID y la contraseña
    wifi_config_t wifi_config = {
        .sta = {
            // El SSID y Password se configuran dinámicamente más adelante
        },
    };
    strncpy((char*)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid) - 1);
    strncpy((char*)wifi_config.sta.password, password, sizeof(wifi_config.sta.password) - 1);

    // Cambia el modo a STA y configura la interfaz STA
    esp_wifi_set_mode(WIFI_MODE_STA);
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_start();

    // Espera a que se conecte a la red o falle
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG1, "connected to SSID:%s password:%s", ssid, password);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG1, "Failed to connect to SSID:%s, password:%s", ssid, password);
    } else {
        ESP_LOGE(TAG1, "UNEXPECTED EVENT");
    }

    // Desregistrar los manejadores de eventos
    esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip);
    esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id);

    // Liberar el grupo de eventos
    vEventGroupDelete(s_wifi_event_group);
}

// FUNCION PARA LEER CREDENCIALES DEL FICHERO wifi.csv
static void read_wifi_credentials_and_connect(void)
{
    ESP_LOGI(TAG, "Reading wifi.csv");

    // Abre el archivo wifi.csv
    FILE* f = fopen("/spiffs/wifi.csv", "r");
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open wifi.csv");
        return;
    }

    char ssid[32] = {0};
	char password[64] = {0};
	char line[128];
	
	if (fgets(line, sizeof(line), f) != NULL) {
	    // Usar sscanf para extraer el SSID y la contraseña
	    sscanf(line, "%31[^,],%63s", ssid, password);
	}
	fclose(f);


    ESP_LOGI(TAG, "SSID: %s, Password: %s", ssid, password);

    // Inicia la conexión WiFi con las credenciales leídas
    wifi_init_sta(ssid, password);
}

void app_main(void)
{
    // Inicialización del NVS antes de usar Wi-Fi
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS necesita ser formateado
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // Inicializar SPIFFS
    ESP_LOGI(TAG, "Initializing SPIFFS");

    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = NULL,
        .max_files = 5,
        .format_if_mount_failed = false
    };

    // Monta SPIFFS
    ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount or format filesystem");
        } else if (ret == ESP_ERR_NOT_FOUND) {
            ESP_LOGE(TAG, "Failed to find SPIFFS partition");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        }
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_spiffs_info(NULL, &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Partition size: total: %d, used: %d", total, used);
    }

    // Inicializar el GPIO del relé
   

    // Lee las credenciales WiFi y conecta
    read_wifi_credentials_and_connect();

    // Conecta el cliente MQTT para el relé
    mqtt_relay_connect();
    ESP_LOGI(TAG, "Cliente MQTT para el relé conectado.");

    // Inicia el monitoreo del sensor y envía datos a ThingsBoard
 
    start_lux_monitoring();

    // Desmonta SPIFFS
    esp_vfs_spiffs_unregister(NULL);
    ESP_LOGI(TAG, "SPIFFS unmounted.");
}


