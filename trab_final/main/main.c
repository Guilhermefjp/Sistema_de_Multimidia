#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_log.h"
#include "esp_nimble_hci.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "sdkconfig.h"
#include "driver/gpio.h"
#include <freertos/semphr.h>
#include <driver/gpio.h>
#include <freertos/queue.h>
#include <ultrasonic.h>
#include "driver/ledc.h"
#include "driver/adc.h"
#include "esp_err.h"
#include <stdlib.h>
#include "driver/adc.h"
#include "driver/ledc.h"
#include "driver/uart.h"


#define TRIGGER_GPIO 18
#define ECHO_GPIO 5
#define BT_IO 0
#define UART_NUM UART_NUM_1
#define GPS_UART_BAUD_RATE 9600
#define GPS_UART_TX_PIN 17
#define GPS_UART_RX_PIN 16
#define BUF_SIZE (4024)

#define BUZZ 1

#define LUM 450

#define MAX_DISTANCE_CM 3 // 5m max

ultrasonic_sensor_t sensor;
QueueHandle_t  fila1,fila2,fila3,fila4,fila5;

char *TAG = "BLE-Server";
uint8_t ble_addr_type;

struct ble_gap_adv_params adv_params;
bool status = false;

void ble_app_advertise(void);
void vTaskLeitura (void *pvparameters);
void vTaskBuzzer (void *pvparameters);
void vTaskLDR (void *pvparameters);
static void vTaskGPS(void *pvParameters);

#define SAMPLE_CNT 32
static const adc1_channel_t adc_channel = ADC_CHANNEL_4;

char lat_value[20] = {0};
char lon_value[20] = {0};
static const char *TAG2 = "GPS_Reader";
void parse_nmea_data(const char *data, double *latitude, double *longitude);
static void parse_lat_lon(const char *nmea_sentence, char *latitude, char *longitude);
//static const char *TAG = "example";
int i = 0;
void extract_lat_long(const char *nmea_message, char *latitude, char *longitude);
double convert_to_decimal(const char *value, char direction, int degree_length);


// Função para converter a latitude e longitude em formato decimal com sinal
double convert_to_decimal(const char *value, char direction, int degree_length) {
    // Separar graus e minutos da latitude/longitude
    double degrees = 0.0;
    double minutes = 0.0;

    // Ajusta a leitura dependendo se é latitude (2 graus) ou longitude (3 graus)
    char degree_part[4] = {0};
    strncpy(degree_part, value, degree_length); // Copia os primeiros caracteres como graus
    degrees = atof(degree_part); // Converte para double
    
    minutes = atof(value + degree_length); // Converte o restante para minutos

    // Converter para decimal
    double decimal = degrees + (minutes / 60.0);

    // Aplicar sinal com base na direção
    if (direction == 'S' || direction == 'W') {
        decimal = -decimal;
    }

    return decimal;
}

void extract_lat_long(const char *nmea_message, char *latitude, char *longitude) {
    
    // Copiando a mensagem para evitar modificar o original
    //char nmea_copy[512];
    //strcpy(nmea_copy, nmea_message);
    

    // Tokenizar a mensagem NMEA por sentenças separadas por '\n'
    char *sentence = strtok(nmea_message, "\n");

    while (sentence != NULL) {
        // Verifica se a sentença é do tipo GPGGA
        if (strstr(sentence, "$GPGGA") != NULL) {
            // Tokenizar a sentença NMEA usando ',' como delimitador
            char *token = strtok(sentence, ",");
            int field_count = 0;

            // Identifica o tipo de sentença e extrai latitude e longitude
            while (token != NULL) {
                field_count++;
                if (field_count == 3) {  // Campo de latitude
                    strcpy(latitude, token);
                } else if (field_count == 4) {  // N/S indicador
                    strcat(latitude, token);  // Anexar N ou S
                } else if (field_count == 5) {  // Campo de longitude
                    strcpy(longitude, token);
                } else if (field_count == 6) {  // E/W indicador
                    strcat(longitude, token);  // Anexar E ou W
                }

                token = strtok(NULL, ",");
            }
            // Saímos do loop após encontrar e processar a sentença GPGGA
            break;
        }
        // Pega a próxima sentença na mensagem
        sentence = strtok(NULL, "\n");
    
    }
    
}

static void init_hw(void)
{
    adc1_config_width(ADC_WIDTH_BIT_10);
    adc1_config_channel_atten(adc_channel, ADC_ATTEN_DB_11);
}

// Write data to ESP32 defined as server
static int device_write(uint16_t conn_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    uint8_t *data = (uint8_t *)ctxt->om->om_data;

    if (data[0] == 1)  
    {
        //ESP_LOGW("BLE", "RECEBA! Valor: %d", data[0]);
        xQueueSendToBack(fila3,&data[0],0);

    }else if (data[0] == 0)
    {
        //ESP_LOGW("BLE", "RECEBA! Valor: %d", data[0]);
        xQueueSendToBack(fila3,&data[0],0);
    }
    else
    {
        ESP_LOGW("BLE", "Erro ao receber mensagem! Valor: %d", data[0]);
    }

    //printf("%d %s\n", strcmp(data, (char *)"LIGHT ON") == 0, data);
    /*
    
     else
     {
        printf("Data from the client: %.*s\n", ctxt->om->om_len, ctxt->om->om_data);
     }
     */
    //memset(data, 0, strlen(data));

    return 0;
}

// Read data from ESP32 defined as server
static int device_read(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    float dado;

    xQueueReceive(fila1, &dado, portMAX_DELAY);
     
    //ESP_LOGW("BLE", "Distancia = %f m", dado);

    os_mbuf_append(ctxt->om, &dado, sizeof(dado)); 

    return 0;
}

float Latitude = -22.951980195698756;
float Longitude = -43.210497932731464;

static int device_read_coordenades(uint16_t con_handle, uint16_t attr_handle, struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    
    xQueueReceive(fila4,&Latitude,0);
    xQueueReceive(fila5,&Longitude,0);

    ESP_LOGW("GPS-BLE","Coordenadas recebidas %f , %f",Latitude,Longitude);

    char coord_string[50];  

     if(Latitude == 0.00000000 || Longitude == 0.00000000)
    {
        Latitude = -22.951980195698756;
        Longitude = -43.210497932731464;

    }

    sprintf(coord_string, "%.15f, %.15f", Latitude, Longitude);  

    os_mbuf_append(ctxt->om, coord_string, strlen(coord_string));  

    ESP_LOGE("GPS-BLE","Coordenadas enviadas %s",coord_string);

    return 0;
}


// Array of pointers to other service definitions
// UUID - Universal Unique Identifier
static const struct ble_gatt_svc_def gatt_svcs[] = {
    {.type = BLE_GATT_SVC_TYPE_PRIMARY,
     .uuid = BLE_UUID16_DECLARE(0xDEED), // Define UUID for device type
     .characteristics = (struct ble_gatt_chr_def[]){
         {.uuid = BLE_UUID16_DECLARE(0xFEF4), // Define UUID for reading
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read},
         {.uuid = BLE_UUID16_DECLARE(0xDEAD), // Define UUID for writing
          .flags = BLE_GATT_CHR_F_WRITE,
          .access_cb = device_write},
         {.uuid = BLE_UUID16_DECLARE(0xDEAF), // Define UUID for reading
          .flags = BLE_GATT_CHR_F_READ,
          .access_cb = device_read_coordenades},
         {0}}},
    {0}};

// BLE event handling
static int ble_gap_event(struct ble_gap_event *event, void *arg)
{
    switch (event->type)
    {
    // Advertise if connected
    case BLE_GAP_EVENT_CONNECT:
        ESP_LOGE("GAP", "BLE GAP EVENT CONNECT %s", event->connect.status == 0 ? "OK!" : "FAILED!");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    // Advertise again after completion of the event
    case BLE_GAP_EVENT_DISCONNECT:
        ESP_LOGE("GAP", "BLE GAP EVENT DISCONNECTED");
        if (event->connect.status != 0)
        {
            ble_app_advertise();
        }
        break;
    case BLE_GAP_EVENT_ADV_COMPLETE:
        ESP_LOGW("GAP", "BLE GAP EVENT");
        ble_app_advertise();
        break;
    default:
        break;
    }
    return 0;
}


// Define the BLE connection
void ble_app_advertise(void)
{
    // GAP - device name definition
    struct ble_hs_adv_fields fields;
    const char *device_name;
    memset(&fields, 0, sizeof(fields));
    device_name = ble_svc_gap_device_name(); // Read the BLE device name    
    fields.name = (uint8_t *)device_name;
    fields.name_len = strlen(device_name);
    fields.name_is_complete = 1;
    ble_gap_adv_set_fields(&fields);

    // GAP - device connectivity definition
    // struct ble_gap_adv_params adv_params;
    memset(&adv_params, 0, sizeof(adv_params));
    // adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
    // adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
    // ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
}

// The application
void ble_app_on_sync(void)
{
    ble_hs_id_infer_auto(0, &ble_addr_type); // Determines the best address type automatically
    ble_app_advertise();                     // Define the BLE connection
}

// The infinite task
void host_task(void *param)
{
    nimble_port_run(); // This function will return only when nimble_port_stop() is executed
}

void connect_ble(void)
{
    nvs_flash_init();                           // 1 - Initialize NVS flash using
    // esp_nimble_hci_and_controller_init();    // 2 - Initialize ESP controller
    nimble_port_init();                        // 3 - Initialize the host stack
    ble_svc_gap_device_name_set("BLE-Server"); // 4 - Initialize NimBLE configuration - server name
    ble_svc_gap_init();                        // 4 - Initialize NimBLE configuration - gap service
    ble_svc_gatt_init();                       // 4 - Initialize NimBLE configuration - gatt service
    ble_gatts_count_cfg(gatt_svcs);            // 4 - Initialize NimBLE configuration - config gatt services
    ble_gatts_add_svcs(gatt_svcs);             // 4 - Initialize NimBLE configuration - queues gatt services.
    ble_hs_cfg.sync_cb = ble_app_on_sync;      // 5 - Initialize application
    nimble_port_freertos_init(host_task);      // 6 - Run the thread
}

void boot_creds_clear(void *param)
{

    // printf("%lld\n", n - m);
    int64_t m = esp_timer_get_time();
    while (1)
    {

        if (!gpio_get_level(GPIO_NUM_0))
        {
            int64_t n = esp_timer_get_time();

            if ((n - m) / 1000 >= 2000)
            {
                ESP_LOGW("BOOT BUTTON:", "Button Pressed FOR 3 SECOND\n");

                adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
                adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
                ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
                status = true;
                vTaskDelay(100);
                m = esp_timer_get_time();
            }
        }
        else
        {
            m = esp_timer_get_time();
        }
        vTaskDelay(10);

        if (status)
        {
            // ESP_LOGI("GAP", "BLE GAP status");
            adv_params.conn_mode = BLE_GAP_CONN_MODE_UND; // connectable or non-connectable
            adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN; // discoverable or non-discoverable
            ble_gap_adv_start(ble_addr_type, NULL, BLE_HS_FOREVER, &adv_params, ble_gap_event, NULL);
        }
    }
}

void app_main() //----------------------------------------------------------------------------------------------------------------------------
{
    //configure perifericos
    sensor.trigger_pin = TRIGGER_GPIO;
    sensor.echo_pin = ECHO_GPIO;
    ultrasonic_init(&sensor);

    init_hw();

        // Configure UART parameters
        uart_config_t uart_config = {
        .baud_rate = GPS_UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    // Set UART parameters
    uart_param_config(UART_NUM, &uart_config);
    // Set UART pins
    uart_set_pin(UART_NUM, GPS_UART_TX_PIN, GPS_UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    // Install UART driver
    uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);

    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);   // boot button
    gpio_set_direction(GPIO_NUM_2, GPIO_MODE_OUTPUT);  // LED do ESP
    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);  // buzzer
    gpio_set_direction(GPIO_NUM_25, GPIO_MODE_OUTPUT); // led ldr
    connect_ble();

    // criação dos objetos
    fila1 = xQueueCreate(1,sizeof(float));
    fila2 = xQueueCreate(1,sizeof(float));
    fila3 = xQueueCreate(1,sizeof(uint8_t));
    fila4 = xQueueCreate(1,sizeof(float));
    fila5 = xQueueCreate(1,sizeof(float));

    // criação das tasks
    xTaskCreate(boot_creds_clear, "boot_creds_clear", 2048, NULL, 1, NULL);
    xTaskCreate(vTaskLeitura,      "LEITURA",         2048, NULL, 2, NULL);
    xTaskCreate(vTaskBuzzer,       "BUZZ",            2048, NULL, 1, NULL);
    xTaskCreate(vTaskLDR,           "LDR",            2048, NULL, 1, NULL);
    xTaskCreate(vTaskGPS,           "GPS",            8192, NULL, 1, NULL);
}

void vTaskLeitura (void *pvparameters)
{   
    float distancia;
    float last_distance=0;
    int cnt=0;
    
    while (1)
    {

        ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distancia);

        //ESP_LOGI("LEITURA","Distancia = %f m",distancia);

        if (distancia < last_distance)
        {
            cnt++;
            if (cnt >= 2)
            {
                last_distance = distancia; 
                cnt = 0;
            }
            //break;
        }
        else
        {
            last_distance = distancia;
            cnt = 0;
        }

        xQueueSendToBack(fila1,&last_distance,0);
        vTaskDelay(pdMS_TO_TICKS(100));
        xQueueSendToBack(fila2,&last_distance,portMAX_DELAY); 
        vTaskDelay(pdMS_TO_TICKS(100));

    }
    
}


void vTaskBuzzer (void *pvparameters)
{
    float dis;

    while(1)
    {
       xQueueReceive(fila2,&dis,portMAX_DELAY);
       //ESP_LOGI("BUZZER","recebeu dado distancia = %f",dis);
       //vTaskDelay(pdMS_TO_TICKS(100));

        if(dis <= BUZZ)
        {
            gpio_set_level(GPIO_NUM_4,1);
            vTaskDelay(pdMS_TO_TICKS(dis*250));
            gpio_set_level(GPIO_NUM_4,0);
            vTaskDelay(pdMS_TO_TICKS(dis*250));
        }
    }
    
}

void vTaskLDR (void *pvparameters)
{
    uint8_t comando = 0;

    while(1)
    {

        xQueueReceive(fila3,&comando,0);
        //ESP_LOGI("LDR","Mensagem recebida: %d",comando);

        uint32_t adc_val = 0;
        for (int i = 0; i < SAMPLE_CNT; ++i)
        {
            adc_val += adc1_get_raw(adc_channel);
        }
        adc_val /= SAMPLE_CNT;

        //ESP_LOGI("LDR","Valor medido : %d",(int)adc_val);
        
        if(comando == 1)
        {
            gpio_set_level(GPIO_NUM_25,1); 
            comando = 1;
        }
        else{
            if(adc_val <= LUM)
            {
            gpio_set_level(GPIO_NUM_25,1); 
            }
            else{
                gpio_set_level(GPIO_NUM_25,0);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

static void vTaskGPS(void *pvParameters) {
    
    uint8_t data[BUF_SIZE];
    int length;
    
    while(1){

        //Começo que fode o código
        // Read data from UART
        length = uart_read_bytes(UART_NUM, data, BUF_SIZE, 100 / portTICK_PERIOD_MS);
        if (length > 0) {
            // Print data to serial monitor
            data[length] = '\0'; // Null-terminate the string
            ESP_LOGE(TAG2, "GPS Data: %s", data);
        
        
            char *nmea_message = (char *)data;


            extract_lat_long(nmea_message, lat_value, lon_value);

            // Extrair os sinais de latitude e longitude
            float latitude = convert_to_decimal(lat_value, lat_value[strlen(lat_value) - 1], 2); // Latitude tem 2 dígitos para graus
            float longitude = convert_to_decimal(lon_value, lon_value[strlen(lon_value) - 1], 3); // Longitude tem 3 dígitos para graus

            // Imprimindo os valores extraídos
            printf("Latitude: %.8f\n", latitude);
            printf("Longitude: %.8f\n", longitude);

                xQueueSendToBack(fila4,&latitude,0);
            xQueueSendToBack(fila5,&longitude,0);

            /////////////////////////////////
                // Definindo os limites do intervalo
            float lat_min = -21.8000;
            float lat_max = -21.7000;
            float lon_min = -43.4000;
            float lon_max = -43.3000;

            // Verifica se as coordenadas estão fora do intervalo
            if (latitude < lat_min || latitude > lat_max || longitude < lon_min || longitude > lon_max) {
                printf("Fora de juiz de fora\n");
            } else {
                printf("Dentro de juiz de fora\n");
            }
            /////////////////////////////////



        vTaskDelay(pdMS_TO_TICKS(1000));
    }
    }
}