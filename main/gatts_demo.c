/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

/****************************************************************************
 *
 * This demo showcases BLE GATT server. It can send adv data, be connected by client.
 * Run the gatt_client demo, the client demo will automatically connect to the gatt_server demo.
 * Client demo will enable gatt_server's notify after connection. The two devices will then exchange
 * data.
 *
 ****************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_bt.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "esp_rom_gpio.h" // Direct GPIO access
#include "soc/rtc_periph.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gatt_common_api.h"

#include "sdkconfig.h"

#include "esp_timer.h"

#define GATTS_TAG "ECEN 215 Lab Kit"

/// Declare the static function
static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);

#define GATTS_SERVICE_UUID_TEST_A 0x00FF
#define GATTS_CHAR_UUID_TEST_A 0xFF01
#define GATTS_DESCR_UUID_TEST_A 0x3333
#define GATTS_NUM_HANDLE_TEST_A 4

static bool oscilloscope_mode = false;
static bool voltmeter_mode = false;
static bool ammeter_mode = false;
static bool ohmmeter_mode = false;

#define TEST_DEVICE_NAME "ECEN 215 Lab Kit"
#define TEST_MANUFACTURER_DATA_LEN 17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024

static uint8_t char1_str[] = {0x11, 0x22, 0x33};
static esp_gatt_char_prop_t a_property = 0;

static esp_attr_value_t gatts_demo_char1_val =
    {
        .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
        .attr_len = sizeof(char1_str),
        .attr_value = char1_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#ifdef CONFIG_SET_RAW_ADV_DATA
static uint8_t raw_adv_data[] = {
    0x02, 0x01, 0x06,       // Length 2, Data Type 1 (Flags), Data 1 (LE General Discoverable Mode, BR/EDR Not Supported)
    0x02, 0x0a, 0xeb,       // Length 2, Data Type 10 (TX power level), Data 2 (-21)
    0x03, 0x03, 0xab, 0xcd, // Length 3, Data Type 3 (Complete 16-bit Service UUIDs), Data 3 (UUID)
};
static uint8_t raw_scan_rsp_data[] = { // Length 15, Data Type 9 (Complete Local Name), Data 1 (ESP_GATTS_DEMO)
    0x0f, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x47, 0x41, 0x54, 0x54, 0x53, 0x5f, 0x44,
    0x45, 0x4d, 0x4f};
#else

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    // first uuid, 16bit, [12],[13] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0xEE,
    0x00,
    0x00,
    0x00,
    // second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb,
    0x34,
    0x9b,
    0x5f,
    0x80,
    0x00,
    0x00,
    0x80,
    0x00,
    0x10,
    0x00,
    0x00,
    0xFF,
    0x00,
    0x00,
    0x00,
};

// The length of adv data must be less than 31 bytes
// static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
// adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, // slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, // slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0,       // TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data = NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

#endif /* CONFIG_SET_RAW_ADV_DATA */

static esp_ble_adv_params_t adv_params = {
    .adv_int_min = 0x20,
    .adv_int_max = 0x40,
    .adv_type = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_NUM 2
#define PROFILE_A_APP_ID 0
#define PROFILE_B_APP_ID 1

int timer_0_cnt = 0;
int timer_1_cnt = 0;

struct gatts_profile_inst
{
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE, /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

typedef struct
{
    uint8_t *prepare_buf;
    int prepare_len;
} prepare_type_env_t;

static prepare_type_env_t a_prepare_write_env;

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);
void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param);

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event)
    {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done == 0)
        {
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        // advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TAG, "Advertising start failed");
        }
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS)
        {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed");
        }
        else
        {
            ESP_LOGI(GATTS_TAG, "Stop adv successfully");
        }
        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
        ESP_LOGI(GATTS_TAG, "update connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                 param->update_conn_params.status,
                 param->update_conn_params.min_int,
                 param->update_conn_params.max_int,
                 param->update_conn_params.conn_int,
                 param->update_conn_params.latency,
                 param->update_conn_params.timeout);
        break;
    case ESP_GAP_BLE_SET_PKT_LENGTH_COMPLETE_EVT:
        ESP_LOGI(GATTS_TAG, "packet length updated: rx = %d, tx = %d, status = %d",
                 param->pkt_data_length_cmpl.params.rx_len,
                 param->pkt_data_length_cmpl.params.tx_len,
                 param->pkt_data_length_cmpl.status);
        break;
    default:
        break;
    }
}

void example_write_event_env(esp_gatt_if_t gatts_if, prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    esp_gatt_status_t status = ESP_GATT_OK;
    if (param->write.need_rsp)
    {
        if (param->write.is_prep)
        {
            if (param->write.offset > PREPARE_BUF_MAX_SIZE)
            {
                status = ESP_GATT_INVALID_OFFSET;
            }
            else if ((param->write.offset + param->write.len) > PREPARE_BUF_MAX_SIZE)
            {
                status = ESP_GATT_INVALID_ATTR_LEN;
            }
            if (status == ESP_GATT_OK && prepare_write_env->prepare_buf == NULL)
            {
                prepare_write_env->prepare_buf = (uint8_t *)malloc(PREPARE_BUF_MAX_SIZE * sizeof(uint8_t));
                prepare_write_env->prepare_len = 0;
                if (prepare_write_env->prepare_buf == NULL)
                {
                    ESP_LOGE(GATTS_TAG, "Gatt_server prep no mem");
                    status = ESP_GATT_NO_RESOURCES;
                }
            }

            esp_gatt_rsp_t *gatt_rsp = (esp_gatt_rsp_t *)malloc(sizeof(esp_gatt_rsp_t));
            if (gatt_rsp)
            {
                gatt_rsp->attr_value.len = param->write.len;
                gatt_rsp->attr_value.handle = param->write.handle;
                gatt_rsp->attr_value.offset = param->write.offset;
                gatt_rsp->attr_value.auth_req = ESP_GATT_AUTH_REQ_NONE;
                memcpy(gatt_rsp->attr_value.value, param->write.value, param->write.len);
                esp_err_t response_err = esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, gatt_rsp);
                if (response_err != ESP_OK)
                {
                    ESP_LOGE(GATTS_TAG, "Send response error\n");
                }
                free(gatt_rsp);
            }
            else
            {
                ESP_LOGE(GATTS_TAG, "malloc failed, no resource to send response error\n");
                status = ESP_GATT_NO_RESOURCES;
            }
            if (status != ESP_GATT_OK)
            {
                return;
            }
            memcpy(prepare_write_env->prepare_buf + param->write.offset,
                   param->write.value,
                   param->write.len);
            prepare_write_env->prepare_len += param->write.len;
        }
        else
        {
            esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, status, NULL);
        }
    }
}

void example_exec_write_event_env(prepare_type_env_t *prepare_write_env, esp_ble_gatts_cb_param_t *param)
{
    if (param->exec_write.exec_write_flag == ESP_GATT_PREP_WRITE_EXEC)
    {
        esp_log_buffer_hex(GATTS_TAG, prepare_write_env->prepare_buf, prepare_write_env->prepare_len);
    }
    else
    {
        ESP_LOGI(GATTS_TAG, "ESP_GATT_PREP_WRITE_CANCEL");
    }
    if (prepare_write_env->prepare_buf)
    {
        free(prepare_write_env->prepare_buf);
        prepare_write_env->prepare_buf = NULL;
    }
    prepare_write_env->prepare_len = 0;
}

// Define a buffer to store the received data
#define MAX_DATA_LEN 512             // Set the maximum data length as needed
uint8_t received_data[MAX_DATA_LEN]; // Buffer to hold the received data
uint16_t received_data_len = 0;      // Variable to store the length of received data

#include <math.h>
#include "esp_check.h"

static const char *TAG = "dac_sine_wave";

/*################################################################Yusuf's Stuff#######################################################*/
#include "driver/dac.h"
#include "esp_task_wdt.h"

#define DAC_CHANNEL DAC_CHANNEL_1

// sine lab 5
#define SAMPLE_RATE 1000 // Higher sample rate for smoother output
#define TABLE_SIZE (SAMPLE_RATE / (SINE_FREQUENCY) * 60)
#define SINE_FREQUENCY 26
#define SINE_AMPLITUDE 80

// sine lab 6
#define SAMPLE_RATE2 1000 // Higher sample rate for smoother output
#define TABLE_SIZE2 (SAMPLE_RATE2 / (SINE_FREQUENCY2) * 60)
#define SINE_FREQUENCY2 1
#define SINE_AMPLITUDE2 80

// sine lab 7
#define SAMPLE_RATE3 1000 // Higher sample rate for smoother output
#define TABLE_SIZE3 (SAMPLE_RATE3 / (SINE_FREQUENCY3) * 60)
#define SINE_FREQUENCY3 26
#define SINE_AMPLITUDE3 40

// square 1
#define SQUARE_WAVE_FREQUENCY 16.42 // Frequency of the square wave in Hz
#define SQUARE_AMPLITUDE 60         // Amplitude of the square wave (0 to 255) 80-.5v amplitude,, 160 -- 1v

// square 2
#define SQUARE_WAVE_FREQUENCY2 25.001f // Frequency of the square wave in Hz
#define SQUARE_AMPLITUDE2 35

// freq of 0.03 -> 4.07 hz
//
#define TRIANGLE_WAVE_FREQUENCY 0.22f // Frequency of the triangle wave in Hz
#define TRI_SAMPLE_RATE 1000          // Sample rate in Hz
#define TRI_AMPLITUDE 35              // Amplitude of the triangle wave (0 to 255)

static uint8_t sine_table[TABLE_SIZE];
static uint8_t sine_table2[TABLE_SIZE2];
static uint8_t sine_table3[TABLE_SIZE3];

void generate_sine_table()
{
    for (int i = 0; i < TABLE_SIZE; i++)
    {
        // sine_table[i] = (uint8_t)(SINE_AMPLITUDE * sin(2 * M_PI * i / TABLE_SIZE) + SINE_AMPLITUDE);
        sine_table[i] = (uint8_t)((SINE_AMPLITUDE / 2) * sin(2 * M_PI * i / TABLE_SIZE) + 128);
    }
}

void generate_sine_table2()
{
    for (int i = 0; i < TABLE_SIZE2; i++)
    {
        sine_table2[i] = (uint8_t)(SINE_AMPLITUDE2 * sin(2 * M_PI * i / TABLE_SIZE2) + SINE_AMPLITUDE2);
    }
}

void generate_sine_table3()
{
    for (int i = 0; i < TABLE_SIZE3; i++)
    {
        sine_table3[i] = (uint8_t)(SINE_AMPLITUDE3 * sin(2 * M_PI * i / TABLE_SIZE3) + SINE_AMPLITUDE3);
    }
}

// void generate_sine_table() {
//     TABLE_SIZE = (SAMPLE_RATE / SIGNAL_FREQUENCY) * 60;  // Recalculate TABLE_SIZE based on current frequency
//     for (int i = 0; i < TABLE_SIZE; i++) {
//         sine_table[i] = (uint8_t)(SINE_AMPLITUDE * sin(2 * M_PI * i / TABLE_SIZE) + SINE_AMPLITUDE);
//     }
// }

void Curvy(void *param)
{
    // Generate sine wave table
    generate_sine_table();

    // Configure DAC
    dac_output_enable(DAC_CHANNEL);

    // Calculate the delay period in ticks to achieve the desired sample rate
    TickType_t delay_ticks = pdMS_TO_TICKS(1000 / SAMPLE_RATE);

    // Infinite loop for generating sine wave
    while (1)
    {
        for (int i = 0; i < TABLE_SIZE; i++)
        {
            // Output the value from the sine wave table to the DAC channel
            dac_output_voltage(DAC_CHANNEL, sine_table[i]);

            // Delay to maintain the desired sample rate
            vTaskDelay(delay_ticks);
        }
    }
}

void Curvy2(void *param)
{
    // Generate sine wave table
    generate_sine_table2();

    // Configure DAC
    dac_output_enable(DAC_CHANNEL);

    // Calculate the delay period in ticks to achieve the desired sample rate
    TickType_t delay_ticks = pdMS_TO_TICKS(1000 / SAMPLE_RATE2);

    // Infinite loop for generating sine wave
    while (1)
    {
        for (int i = 0; i < TABLE_SIZE2; i++)
        {
            // Output the value from the sine wave table to the DAC channel
            dac_output_voltage(DAC_CHANNEL, sine_table2[i]);

            // Delay to maintain the desired sample rate
            vTaskDelay(delay_ticks);
        }
    }
}

void Curvy3(void *param)
{
    // Generate sine wave table
    generate_sine_table3();

    // Configure DAC
    dac_output_enable(DAC_CHANNEL);

    // Calculate the delay period in ticks to achieve the desired sample rate
    TickType_t delay_ticks = pdMS_TO_TICKS(1000 / SAMPLE_RATE3);

    // Infinite loop for generating sine wave
    while (1)
    {
        for (int i = 0; i < TABLE_SIZE3; i++)
        {
            // Output the value from the sine wave table to the DAC channel
            dac_output_voltage(DAC_CHANNEL, sine_table3[i]);

            // Delay to maintain the desired sample rate
            vTaskDelay(delay_ticks);
        }
    }
}

void Square(void *param)
{
    // Configure DAC
    dac_output_enable(DAC_CHANNEL);

    // Calculate the delay period in ticks to achieve the desired frequency
    TickType_t delay_ticks = pdMS_TO_TICKS(1000 / SQUARE_WAVE_FREQUENCY / 2); // Divide by 2 for alternating duty cycle

    // Infinite loop for generating square wave
    while (1)
    {
        // Set DAC output to minimum value (0)
        dac_output_voltage(DAC_CHANNEL, 0);
        vTaskDelay(delay_ticks);

        // Set DAC output to maximum value (defined by AMPLITUDE)
        dac_output_voltage(DAC_CHANNEL, SQUARE_AMPLITUDE);
        vTaskDelay(delay_ticks);
    }
}

void Square2(void *param)
{
    // Configure DAC
    dac_output_enable(DAC_CHANNEL);

    // Calculate the delay period in ticks to achieve the desired frequency
    TickType_t delay_ticks = pdMS_TO_TICKS(1000 / SQUARE_WAVE_FREQUENCY2 / 2); // Divide by 2 for alternating duty cycle

    // Infinite loop for generating square wave
    while (1)
    {
        // Set DAC output to minimum value (0)
        dac_output_voltage(DAC_CHANNEL, 0);
        vTaskDelay(delay_ticks);

        // Set DAC output to maximum value (defined by AMPLITUDE)
        dac_output_voltage(DAC_CHANNEL, SQUARE_AMPLITUDE2);
        vTaskDelay(delay_ticks);
    }
}

void Triangle(void *param)
{
    // Configure DAC
    dac_output_enable(DAC_CHANNEL_1);

    // Calculate the delay period in ticks for the desired sample rate
    TickType_t delay_ticks = (TickType_t)(1000.0 / TRI_SAMPLE_RATE / portTICK_PERIOD_MS);

    // Calculate the increment step as a float for higher precision
    float increment_step = TRI_AMPLITUDE * 4.0f * TRIANGLE_WAVE_FREQUENCY / TRI_SAMPLE_RATE;

    // Initialize triangle wave value
    float triangle_wave_value = 0.0f;

    // Infinite loop for generating triangle wave
    while (1)
    {
        // Update DAC output voltage (convert triangle_wave_value to int)
        dac_output_voltage(DAC_CHANNEL_1, (int)triangle_wave_value);

        // Update triangle wave value
        triangle_wave_value += increment_step;

        // Check if triangle wave value exceeds amplitude
        if (triangle_wave_value >= TRI_AMPLITUDE)
        {
            triangle_wave_value = TRI_AMPLITUDE; // Limit to amplitude
            increment_step = -increment_step;    // Reverse direction
        }
        // Check if triangle wave value becomes negative
        else if (triangle_wave_value <= 0)
        {
            triangle_wave_value = 0;          // Limit to 0
            increment_step = -increment_step; // Reverse direction
        }

        // Delay for the sample rate
        vTaskDelay(delay_ticks);
    }
}

// Define task handles globally
TaskHandle_t waveform_task_handle = NULL;

void stop_current_waveform()
{
    if (waveform_task_handle != NULL)
    {
        vTaskDelete(waveform_task_handle); // Delete the current waveform task
        waveform_task_handle = NULL;
        ESP_LOGI(TAG, "Stopped current waveform task");
    }
}

/*################################################################Yusuf's Stuff#######################################################*/

/*###############################################################Power Supply#########################################################*/

#define GPIO_OUTPUT_PIN 4
#define GPIO_OUTPUT_PIN_ 21
int condition = 0;
int condition_ = 0;

/*################################################################Power Supply######################################################*/

/*############################################Peter's Stuff###################################################################################*/

// 14 bit ADC ADC141S626 used for voltmeter and oscilloscope
#define MAX_MEASUREMENTS 300 // Maximum number of measurements

// Struct hold voltage and time
typedef struct
{
    float voltage;
    float timestamp;
} Measurement;

Measurement measurements[MAX_MEASUREMENTS];
int num_measurements = 0;
int osc_cnt = 0;  // Count what is displayed on the oscilloscope
int curr_cnt = 0; // Current number that is being measured
int64_t start_time = 0;

#define ADC_SPI_HOST HSPI_HOST
#define ADC_CS_GPIO 15   // Chip Select for 14 bit ADC
#define ADC_SCLK_GPIO 14 // Serial Clock for 14 bit ADC
#define ADC_DOUT_GPIO 12 // Data Out for 14 bit ADC

// Start timer
void start_timer()
{
    start_time = xTaskGetTickCount();
}

// Determine time since start_timer()
int64_t stop_timer()
{
    int64_t elapsed_time = xTaskGetTickCount() - start_time;
    return elapsed_time;
};

spi_device_handle_t adc_handle; // SPI handle for 14 bit ADC

void init_spi_osc_volt()
{
    // Configure the SPI bus
    spi_bus_config_t buscfg = {
        .mosi_io_num = -1,            // No MOSI pin
        .miso_io_num = ADC_DOUT_GPIO, // MISO pin to 14 bit ADC data out
        .sclk_io_num = ADC_SCLK_GPIO, // Clock pin connected to 14 bit ADC clock
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 2, // Transferring 2 bytes or 16 bits
    };

    // Configure the SPI device
    spi_device_interface_config_t devcfg = {
        .clock_speed_hz = 4000000,   // 4 MHz. Can be between 0.9 - 4.5 MHz
        .mode = 0,                   // SPI mode 0
        .spics_io_num = ADC_CS_GPIO, // CS pin
        .queue_size = 1,
        .flags = SPI_DEVICE_HALFDUPLEX // 14 bit ADC read only
    };

    // Initialize the SPI bus and attach the device
    ESP_ERROR_CHECK(spi_bus_initialize(ADC_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_ERROR_CHECK(spi_bus_add_device(ADC_SPI_HOST, &devcfg, &adc_handle));
}

int16_t convert_data(uint16_t raw_adc)
{
    // Only use lower 14 bits
    uint16_t masked_adc = raw_adc & 0x3FFF; // 0x3FFF = 0011 1111 1111 1111

    // 2s compliment so check first bit for sign
    if (masked_adc & 0x2000)
    { // 0x2000 = 0010 0000 0000 0000
        // Negative
        return (int16_t)(masked_adc | 0xC000); // 0xC000 = 1100 0000 0000 0000
    }
    else
    {
        // Positive
        return (int16_t)masked_adc;
    }
}

float read_adc()
{
    uint8_t rx_data[2] = {0}; // Buffer store output

    spi_transaction_t trans = {
        .flags = SPI_TRANS_USE_RXDATA, // Internal RX buffer
        .length = 16,                  // Transfer 16 bits
        .rxlength = 16,                // Receive 16 bits
        .tx_buffer = NULL,             // Not transmit
        .rx_buffer = NULL,             // Make rx_buffer null or else error SPI_TRANS_USE_RXDATA
    };

    // Check SPI transaction
    esp_err_t ret = spi_device_transmit(adc_handle, &trans);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI transaction failed 14 bit ADC: %s", esp_err_to_name(ret));
        return 0xFFFF;
    }

    // Convert raw data into voltage value
    uint16_t raw_data = (trans.rx_data[0] << 8) | trans.rx_data[1]; // Combine rx_data buffers to 16-bit value
    int16_t unvolt = convert_data(raw_data);                        // Convert bits from 2s complement
    float convolt = (float)unvolt;
    convolt = (convolt / 8192.0f) * 5.0f; // Reference voltage 5, convert to voltage

    return convolt;
}

// Returns voltage read from voltmeter
float voltmeter()
{
    float voltage = 2 * read_adc(); // Perform scaling to voltage
    // float voltage = (read_adc() - 2.5) * 2;  // Perform scaling to voltage
    return voltage;
}

// Reads voltage and places to measurements struct which oscilloscope reads
void oscilloscope()
{
    // if (timer_1_cnt == 0){
    //     measurements[0].voltage = voltmeter();
    //     measurements[0].timestamp = 0;
    //     timer_1_cnt++;
    // } else {
    //     measurements[0].voltage = voltmeter();
    //     measurements[0].timestamp = (float) (stop_timer()) / configTICK_RATE_HZ;

    // }

    if (timer_1_cnt == 0)
    {
        measurements[curr_cnt].voltage = voltmeter();
        measurements[curr_cnt].timestamp = 0;
        timer_1_cnt++;
    }
    else
    {
        measurements[curr_cnt].voltage = voltmeter();
        measurements[curr_cnt].timestamp = (float)(stop_timer()) / configTICK_RATE_HZ; // in seconds
    }

    curr_cnt = (curr_cnt + 1) % MAX_MEASUREMENTS;
    if (num_measurements < MAX_MEASUREMENTS)
    {
        num_measurements++;
    }
}

/*#########################################################################################################*/

/*#########################################Peter's amm ohm##################################################*/
// 12 bit ADC MCP3201 for ammeter and ohmmeter

#define PIN_OHM_AMM_MISO 19 // Output for 12 bit ADC
#define PIN_OHM_AMM_CLK 18  // Serial Clock for 12 bit ADC
#define PIN_OHM_AM_CS 5     // Clock Select for 12 bit ADC

#define PIN_AMM_SW 27 // Switch for ammeter
#define PIN_OHM_SW 32 // Switch for ohmmeter
#define PIN_MUX 25    // Mux: 0 ohm and 1 amm output

spi_device_handle_t spi_amm_ohm;

void init_spi_amm_ohm()
{
    spi_bus_config_t buscfg_amm_ohm = {
        .miso_io_num = PIN_OHM_AMM_MISO, // 12 bit ADC output
        .mosi_io_num = -1,               // Not used
        .sclk_io_num = PIN_OHM_AMM_CLK,  // 12 bit ADC Serial Clock
        .quadwp_io_num = -1,             // Not used
        .quadhd_io_num = -1,             // Not used
    };

    spi_device_interface_config_t devcfg_amm_ohm = {
        .clock_speed_hz = 1600000,      // 1.6 MHz
        .mode = 0,                      // SPI mode 0
        .spics_io_num = PIN_OHM_AM_CS,  // 12 bit ADC Clock Select
        .queue_size = 1,                // 1 transaction each
        .flags = SPI_DEVICE_HALFDUPLEX, // 14 bit ADC read only
    };

    // Initialize the SPI bus
    ESP_ERROR_CHECK(spi_bus_initialize(VSPI_HOST, &buscfg_amm_ohm, SPI_DMA_CH_AUTO));
    // Attach the MCP3201 to the SPI bus
    ESP_ERROR_CHECK(spi_bus_add_device(VSPI_HOST, &devcfg_amm_ohm, &spi_amm_ohm));

    // set up GPIO
    esp_rom_gpio_pad_select_gpio(PIN_AMM_SW);         // Ammeter
    gpio_set_direction(PIN_AMM_SW, GPIO_MODE_OUTPUT); // Ammeter

    esp_rom_gpio_pad_select_gpio(PIN_OHM_SW);         // Ohmmeter
    gpio_set_direction(PIN_OHM_SW, GPIO_MODE_OUTPUT); // Ohmmeter

    esp_rom_gpio_pad_select_gpio(PIN_MUX);         // Mux
    gpio_set_direction(PIN_MUX, GPIO_MODE_OUTPUT); // Mux
}

uint16_t read_adc_amm_ohm()
{
    uint8_t rx_data[2] = {0};
    spi_transaction_t t = {
        .flags = SPI_TRANS_USE_RXDATA, // Internal RX buffer
        .length = 16,                  // Transfer 16 bits
        .rxlength = 16,                // Receive 16 bits
        .tx_buffer = NULL,             // Not transmit
        .rx_buffer = NULL,             // Make rx_buffer null or else error SPI_TRANS_USE_RXDATA
    };

    // Check SPI transaction
    esp_err_t ret = spi_device_transmit(spi_amm_ohm, &t);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SPI transaction failed 12 bit ADC: %s", esp_err_to_name(ret));
        return 0xFFFF;
    }

    // Pullup for MISO or Output
    gpio_set_direction(PIN_OHM_AMM_MISO, GPIO_MODE_INPUT);
    gpio_set_pull_mode(PIN_OHM_AMM_MISO, GPIO_PULLUP_ONLY);

    // Convert raw data into voltage
    uint16_t raw_data = ((t.rx_data[0] & 0x1F) << 7) | (t.rx_data[1] >> 1); // Turn 16 bits int 12 bits from datasheet
    return raw_data;
}

// Ohmmeter returns resistance.
float ohmmeter()
{
    // Convert raw data from ADC and into resistance

    gpio_set_level(PIN_OHM_SW, 1); // Turn on ohmmeter
    gpio_set_level(PIN_MUX, 0);    // Mux to ohmmeter

    vTaskDelay(pdMS_TO_TICKS(50)); // Delay for 50 ms

    float Vres = (read_adc_amm_ohm() / 4095.0) * 5; // Voltage across resistor

    vTaskDelay(pdMS_TO_TICKS(50)); // Delay for 50 ms

    gpio_set_level(PIN_OHM_SW, 0); // Turn off ohmmeter

    float Itotal = (5.0 - Vres) / 20000; // Calculate current through resistor

    // Make sure if is open circuit or resistance is too high, it isnt infinity
    float ohm_val;
    if (Itotal > 0.0001)
    {
        ohm_val = Vres / Itotal; // Calculte resistance (Vres / Ires)
    }
    else
    {
        ohm_val = 1000000; // Capped at 1 Mohm
    }

    return ohm_val;
};

// Ammeter returns current.
float ammeter()
{
    // Convert raw data from ADC to current

    gpio_set_level(PIN_AMM_SW, 1); // Turn on ammeter
    gpio_set_level(PIN_MUX, 1);    // Mux to ammeter

    vTaskDelay(pdMS_TO_TICKS(50)); // Delay for 50 ms

    float Vout = ((float)read_adc_amm_ohm() * 5.0) / 4095; // Current
    printf("Voltage from ammeter: %3f \n", Vout);

    vTaskDelay(pdMS_TO_TICKS(50)); // Delay for 50 ms

    gpio_set_level(PIN_AMM_SW, 0); // Turn off ohmmeter
    gpio_set_level(PIN_MUX, 0);    // Mux to default or 0

    float amm_val = Vout / (20 * 1); // Calculate current from MAX4080 gain 20 shunt 1 ohm

    return amm_val;
};

/*#########################################################################################################*/

void process_received_data(uint8_t *received_data, uint16_t received_data_len)
{
    // Define the string to compare against
    const char *expected_string = "Square";
    const char *expected_string_ = "Sine";
    const char *expected_string__ = "Tri";
    // Add underscores to change which one is called
    // EX: if (received_data_len == strlen(****expected_string****) && strncmp((const char *)received_data, ****expected_string****, received_data_len) == 0)
    // call specific funcitons like square 1 and Swuare 2

    const char *expected_string_lab4sq = "Lab 4 square";
    const char *expected_string_lab5sine = "Lab 5 sine";
    const char *expected_string_lab6sine = "Lab 6 sine";
    const char *expected_string_lab7sine = "Lab 7 sine";
    const char *expected_string_lab7square = "Lab 7 square";
    const char *expected_string_lab7sinetri = "Lab 7 tri";

    const char *expected_stringy = "Voltmeter";
    const char *expected_stringy_ = "Ammeter";
    const char *expected_stringy__ = "Ohmmeter";
    const char *expected_string_stop = "Off";

    const char *expected_stringyy = "Oscilloscope";
    const char *expected_stringyy_ = "OscData";

    const char *expected_stringyyy = "5V";
    const char *expected_stringyyy_ = "Neg5V";

    // Compare received_data to the expected string
    if (received_data_len == strlen(expected_string_lab4sq) && strncmp((const char *)received_data, expected_string_lab4sq, received_data_len) == 0)
    {
        // Square
        stop_current_waveform(); // Stop any existing task
        printf("Received data matches the string 'Square'\n");
        ESP_LOGI(TAG, "DAC square wave example start");
        ESP_LOGI(TAG, "--------------------------------------");
        xTaskCreate(Square, "square_wave_task", 2048, NULL, 5, &waveform_task_handle);
    }

    else if (received_data_len == strlen(expected_string_lab5sine) && strncmp((const char *)received_data, expected_string_lab5sine, received_data_len) == 0)
    {
        // Sine
        stop_current_waveform(); // Stop any existing task
        printf("Received data matches the string 'Sine'\n");
        // Add your code to handle the matching case
        // Generate sine wave table
        ESP_LOGI(TAG, "DAC sine wave from lab 5 example start");
        ESP_LOGI(TAG, "--------------------------------------");
        xTaskCreate(Curvy, "sine_wave_task", 2048, NULL, 5, &waveform_task_handle);

        // stop_current_waveform();  // Stop any existing task

        // // Update frequency and amplitude for the new waveform
        // SIGNAL_FREQUENCY = 2;
        // SINE_AMPLITUDE = 2;

        // // Recalculate TABLE_SIZE and generate a new sine table
        // generate_sine_table();

        // ESP_LOGI(TAG, "DAC sine wave example start for Lab 4");
        // ESP_LOGI(TAG, "--------------------------------------");
        // xTaskCreate(Curvy, "sine_wave_task", 2048, NULL, 5, &waveform_task_handle);
    }

    else if (received_data_len == strlen(expected_string_lab6sine) && strncmp((const char *)received_data, expected_string_lab6sine, received_data_len) == 0)
    {
        // Triangle
        stop_current_waveform(); // Stop any existing task
        printf("Received data matches the string 'lab 6 sine'\n");
        // Add your code to handle the matching case
        ESP_LOGI(TAG, "DAC Sine lab 6 wave example start");
        ESP_LOGI(TAG, "--------------------------------------");
        xTaskCreate(Curvy2, "sine_wave_task", 2048, NULL, 5, &waveform_task_handle);
    }
    else if (received_data_len == strlen(expected_string_lab7sine) && strncmp((const char *)received_data, expected_string_lab7sine, received_data_len) == 0)
    {
        // Triangle
        stop_current_waveform(); // Stop any existing task
        printf("Received data matches the string 'lab 7 sine'\n");
        // Add your code to handle the matching case
        ESP_LOGI(TAG, "DAC Sine wave from lab 7 example start");
        ESP_LOGI(TAG, "--------------------------------------");
        xTaskCreate(Curvy3, "sine_wave_task", 2048, NULL, 5, &waveform_task_handle);
    }

    else if (received_data_len == strlen(expected_string_lab7square) && strncmp((const char *)received_data, expected_string_lab7square, received_data_len) == 0)
    {
        // Triangle
        stop_current_waveform(); // Stop any existing task
        printf("Received data matches the string 'Tri'\n");
        // Add your code to handle the matching case
        ESP_LOGI(TAG, "DAC square lab 7 wave example start");
        ESP_LOGI(TAG, "--------------------------------------");
        xTaskCreate(Square2, "square_wave_task", 2048, NULL, 5, &waveform_task_handle);
    }
    else if (received_data_len == strlen(expected_string_lab7sinetri) && strncmp((const char *)received_data, expected_string_lab7sinetri, received_data_len) == 0)
    {
        stop_current_waveform(); // Stop any existing task
        ESP_LOGI(TAG, "DAC Triangle wave example start");
        ESP_LOGI(TAG, "--------------------------------------");
        xTaskCreate(Triangle, "Triangle_wave_task", 2048, NULL, 5, &waveform_task_handle);
    }
    else if (received_data_len == strlen(expected_string_stop) && strncmp((const char *)received_data, expected_string_stop, received_data_len) == 0)
    {
        printf("Received data matches the string 'Off'\n");
        stop_current_waveform();
        // Add your code to handle the matching case
    }

    else if (received_data_len == strlen(expected_stringy) && strncmp((const char *)received_data, expected_stringy, received_data_len) == 0)
    {
        printf("Received data matches the string 'Voltmeter'\n");
        // Add your code to handle the matching case
    }
    else if (received_data_len == strlen(expected_stringy_) && strncmp((const char *)received_data, expected_stringy_, received_data_len) == 0)
    {
        printf("Received data matches the string 'Ammeter'\n");
        // Add your code to handle the matching case
    }
    else if (received_data_len == strlen(expected_stringy__) && strncmp((const char *)received_data, expected_stringy__, received_data_len) == 0)
    {
        printf("Received data matches the string 'Ohmmeter'\n");
        // Add your code to handle the matching case
    }
    else if (received_data_len == strlen(expected_stringyy) && strncmp((const char *)received_data, expected_stringyy, received_data_len) == 0)
    {
        printf("Received data matches the string 'Oscilloscope'\n");
        // Add your code to handle the matching case
    }
    else if (received_data_len == strlen(expected_stringyy_) && strncmp((const char *)received_data, expected_stringyy_, received_data_len) == 0)
    {
        printf("Received data matches the string 'OscData'\n");
        // Add your code to handle the matching case
    }
    else if (received_data_len == strlen(expected_stringyyy) && strncmp((const char *)received_data, expected_stringyyy, received_data_len) == 0)
    {
        printf("Received data matches the string '5V'\n");

        condition += 1;

        // Add your code to handle the matching case
        // Configure GPIO4 as output
        esp_rom_gpio_pad_select_gpio(GPIO_OUTPUT_PIN);
        gpio_set_direction(GPIO_OUTPUT_PIN, GPIO_MODE_OUTPUT);

        // Use if-else statement to control the GPIO pin
        if (condition % 2 != 0)
        {
            // Set GPIO4 to HIGH
            gpio_set_level(GPIO_OUTPUT_PIN, 1);
            ESP_LOGI("GPIO", "GPIO4 set to HIGH");
        }
        else if (condition % 2 == 0)
        {
            // Set GPIO4 to LOW
            gpio_set_level(GPIO_OUTPUT_PIN, 0);
            ESP_LOGI("GPIO", "GPIO4 set to LOW");
        }
    }

    else if (received_data_len == strlen(expected_stringyyy_) && strncmp((const char *)received_data, expected_stringyyy_, received_data_len) == 0)
    {
        printf("Received data matches the string 'Neg5V'\n");
        condition_ += 1;

        // Add your code to handle the matching case
        // Configure GPIO19 as output
        esp_rom_gpio_pad_select_gpio(GPIO_OUTPUT_PIN_);
        gpio_set_direction(GPIO_OUTPUT_PIN_, GPIO_MODE_OUTPUT);

        // Use if-else statement to control the GPIO pin
        if (condition_ % 2 != 0)
        {
            // Set GPIO19 to HIGH
            gpio_set_level(GPIO_OUTPUT_PIN_, 1);
            ESP_LOGI("GPIO", "GPIO19 set to HIGH");
        }
        else if (condition_ % 2 == 0)
        {
            // Set GPIO19 to LOW
            gpio_set_level(GPIO_OUTPUT_PIN_, 0);
            ESP_LOGI("GPIO", "GPIO19 set to LOW");
        }
    }
    else
    {
        printf("Received data does not match any commands.\n");
        // Add your code to handle the non-matching case
    }
}

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event)
    {
    case ESP_GATTS_REG_EVT:
        ESP_LOGI(GATTS_TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
        if (set_dev_name_ret)
        {
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
#ifdef CONFIG_SET_RAW_ADV_DATA
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret)
        {
            ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        adv_config_done |= adv_config_flag;
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret)
        {
            ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
        }
        adv_config_done |= scan_rsp_config_flag;
#else
        // config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        // config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret)
        {
            ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;

#endif
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);
        break;
        /*#####################################################Test OSC Messages ^###########################################*/
        // case ESP_GATTS_READ_EVT:
        // {
        //     ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d",
        //              param->read.conn_id, param->read.trans_id, param->read.handle);
        //     esp_gatt_rsp_t rsp;
        //     memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        //     rsp.attr_value.handle = param->read.handle;

        //     // Static variable to keep track of the count across function calls
        //     static int counter = 0;

        //     // Format counter as "x,y" and set it as the response value
        //     snprintf((char *)rsp.attr_value.value, sizeof(rsp.attr_value.value), "%d,%d", counter, counter);
        //     rsp.attr_value.len = strlen((char *)rsp.attr_value.value);

        //     // Increment counter and reset if it exceeds 5
        //     counter = (counter + 1) % 6;

        //     esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
        //                                 ESP_GATT_OK, &rsp);
        //     break;
        // }

#include <math.h> // Include math library for sine calculations

    case ESP_GATTS_READ_EVT:
    {
        ESP_LOGI(GATTS_TAG, "GATT_READ_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(rsp));
        rsp.attr_value.handle = param->read.handle;

        if (oscilloscope_mode)
        {
            // Send oscilloscope data to app

            // Read oscilloscope values
            if (osc_cnt == 0)
            {
                for (int i = 0; i < MAX_MEASUREMENTS; i++)
                {
                    oscilloscope();
                    vTaskDelay(1); // wait 1 milisecond
                }
            }
            oscilloscope();
            // If the oscilloscope reaches end of max measurements, it will reset
            if (osc_cnt >= MAX_MEASUREMENTS)
            {
                osc_cnt = 0;
            }

            // Send data
            char data[512];
            int offset = 0;
            offset += snprintf(data + offset, sizeof(data) - offset, "%.2f,%.2f", measurements[osc_cnt].timestamp, measurements[osc_cnt].voltage);
            osc_cnt++; // Increase count
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = strlen(data);
            memcpy(rsp.attr_value.value, data, rsp.attr_value.len);
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
            // vTaskDelay(1);
            // for (int i = 0; i < 1; i++){
            //     char data[512];
            //     // Adjust size as needed
            //     int offset = 0;
            //     if (osc_cnt >= MAX_MEASUREMENTS)
            //     {
            //         osc_cnt = 0;
            //     }
            //     offset += snprintf(data + offset, sizeof(data) - offset, "%.2f,%.2f", measurements[osc_cnt].timestamp, measurements[osc_cnt].voltage);
            //     // Increase count
            //     osc_cnt++;
            //     // Send data
            //     esp_gatt_rsp_t rsp;
            //     memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            //     rsp.attr_value.handle = param->read.handle;
            //     rsp.attr_value.len = strlen(data);
            //     memcpy(rsp.attr_value.value, data, rsp.attr_value.len);
            //     esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);

            //     // vTaskDelay(125); // 0.01 seconds
            // }
            // char data[512];
            // // Adjust size as needed
            // int offset = 0;
            // offset += snprintf(data + offset, sizeof(data) - offset, "%.2f,%.2f", measurements[0].timestamp, measurements[0].voltage);
            // // Send data
            // esp_gatt_rsp_t rsp;
            // memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            // rsp.attr_value.handle = param->read.handle;
            // rsp.attr_value.len = strlen(data);
            // memcpy(rsp.attr_value.value, data, rsp.attr_value.len);
            // esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        }
        else if (voltmeter_mode)
        {
            // Send voltmeter data to app
            // Send "12" as the response when voltmeter mode is enabled
            float temp = voltmeter(); // Get voltage

            // Send data
            char data[512];
            int offset = 0;
            offset += snprintf(data + offset, sizeof(data) - offset, "%.3f V", temp);
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = strlen(data);
            memcpy(rsp.attr_value.value, data, rsp.attr_value.len);
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        }
        else if (ammeter_mode)
        {
            // Send ammeter data to app
            // Send "3" as the response when ammeter mode is enabled
            float temp = ammeter(); //

            // Send data
            char data[512];
            int offset = 0;
            offset += snprintf(data + offset, sizeof(data) - offset, "%.6f A", temp);
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = strlen(data);
            memcpy(rsp.attr_value.value, data, rsp.attr_value.len);
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        }
        else if (ohmmeter_mode)
        {
            // Send ohmmeter data to app
            // Send "7" as the response when ohmmeter mode is enabled
            char data[512];
            float temp = ohmmeter();
            if (ohmmeter >= 1000000)
            {
                ESP_LOGI(TAG, "Very large resistance");
            }
            int offset = 0;
            offset += snprintf(data + offset, sizeof(data) - offset, "%.3f Ohms", temp);
            esp_gatt_rsp_t rsp;
            memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
            rsp.attr_value.handle = param->read.handle;
            rsp.attr_value.len = strlen(data);
            memcpy(rsp.attr_value.value, data, rsp.attr_value.len);
            esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &rsp);
        }
        else
        {
            // Send "-" as the default message when no mode is enabled
            snprintf((char *)rsp.attr_value.value, sizeof(rsp.attr_value.value), "-");
            rsp.attr_value.len = strlen((char *)rsp.attr_value.value);
        }

        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }

        /*#####################################################Test OSC Messages ^###########################################*/

    case ESP_GATTS_WRITE_EVT:
    {
        ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, conn_id %d, trans_id %" PRIu32 ", handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep)
        {
            ESP_LOGI(GATTS_TAG, "GATT_WRITE_EVT, value len %d, value :", param->write.len);
            esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);

            // Store the received data in a buffer
            if (param->write.len <= MAX_DATA_LEN)
            {
                // Clear the previous data
                memset(received_data, 0, MAX_DATA_LEN);

                // Copy the new data into the buffer
                memcpy(received_data, param->write.value, param->write.len);

                // Store the length of the received data
                received_data_len = param->write.len;

                ESP_LOGI(GATTS_TAG, "Data successfully stored in buffer, length: %d", received_data_len);
                ESP_LOGI(GATTS_TAG, "Received data: %.*s", received_data_len, received_data);

                // Check for specific messages to enable modes
                if (received_data_len == strlen("Oscilloscope") && strncmp((char *)received_data, "Oscilloscope", received_data_len) == 0)
                {
                    // start_timer();
                    if (timer_0_cnt == 0)
                    {
                        start_timer();
                        timer_0_cnt++;
                    }
                    oscilloscope_mode = true;
                    voltmeter_mode = false;
                    ammeter_mode = false;
                    ohmmeter_mode = false;
                    ESP_LOGI(GATTS_TAG, "Oscilloscope mode enabled");
                }
                else if (received_data_len == strlen("Voltmeter") && strncmp((char *)received_data, "Voltmeter", received_data_len) == 0)
                {
                    voltmeter_mode = true;
                    oscilloscope_mode = false;
                    ammeter_mode = false;
                    ohmmeter_mode = false;
                    ESP_LOGI(GATTS_TAG, "Voltmeter mode enabled");
                }
                else if (received_data_len == strlen("Ammeter") && strncmp((char *)received_data, "Ammeter", received_data_len) == 0)
                {
                    ammeter_mode = true;
                    oscilloscope_mode = false;
                    voltmeter_mode = false;
                    ohmmeter_mode = false;
                    ESP_LOGI(GATTS_TAG, "Ammeter mode enabled");
                }
                else if (received_data_len == strlen("Ohmmeter") && strncmp((char *)received_data, "Ohmmeter", received_data_len) == 0)
                {
                    ohmmeter_mode = true;
                    oscilloscope_mode = false;
                    voltmeter_mode = false;
                    ammeter_mode = false;
                    ESP_LOGI(GATTS_TAG, "Ohmmeter mode enabled");
                }
                else if (received_data_len == strlen("TimerOff") && strncmp((char *)received_data, "TimerOff", received_data_len) == 0)
                {
                    timer_0_cnt = 0;
                    timer_1_cnt = 0;
                    // Disable all modes if other messages are received
                    oscilloscope_mode = false;
                    voltmeter_mode = false;
                    ammeter_mode = false;
                    ohmmeter_mode = false;
                    ESP_LOGI(GATTS_TAG, "All modes disabled");
                }
                else
                {
                    // Disable all modes if other messages are received
                    oscilloscope_mode = false;
                    voltmeter_mode = false;
                    ammeter_mode = false;
                    ohmmeter_mode = false;
                    ESP_LOGI(GATTS_TAG, "All modes disabled");
                    osc_cnt = 0;
                }

                // Call the function to process received data
                process_received_data(received_data, received_data_len);
            }
            else
            {
                ESP_LOGW(GATTS_TAG, "Received data too long to store, max length is %d", MAX_DATA_LEN);
            }

            // Handle descriptor writes for notifications/indications
            if (gl_profile_tab[PROFILE_A_APP_ID].descr_handle == param->write.handle && param->write.len == 2)
            {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];
                if (descr_value == 0x0001)
                {
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_NOTIFY)
                    {
                        ESP_LOGI(GATTS_TAG, "notify enable");
                        uint8_t notify_data[15];
                        for (int i = 0; i < sizeof(notify_data); ++i)
                        {
                            notify_data[i] = i % 0xff;
                        }
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                    sizeof(notify_data), notify_data, false);
                    }
                }
                else if (descr_value == 0x0002)
                {
                    if (a_property & ESP_GATT_CHAR_PROP_BIT_INDICATE)
                    {
                        ESP_LOGI(GATTS_TAG, "indicate enable");
                        uint8_t indicate_data[15];
                        for (int i = 0; i < sizeof(indicate_data); ++i)
                        {
                            indicate_data[i] = i % 0xff;
                        }
                        esp_ble_gatts_send_indicate(gatts_if, param->write.conn_id, gl_profile_tab[PROFILE_A_APP_ID].char_handle,
                                                    sizeof(indicate_data), indicate_data, true);
                    }
                }
                else if (descr_value == 0x0000)
                {
                    ESP_LOGI(GATTS_TAG, "notify/indicate disable ");
                }
                else
                {
                    ESP_LOGE(GATTS_TAG, "unknown descr value");
                    esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);
                }
            }
        }
        example_write_event_env(gatts_if, &a_prepare_write_env, param);
        break;
    }

    case ESP_GATTS_EXEC_WRITE_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_EXEC_WRITE_EVT");
        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        example_exec_write_event_env(&a_prepare_write_env, param);
        break;
    case ESP_GATTS_MTU_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        ESP_LOGI(GATTS_TAG, "CREATE_SERVICE_EVT, status %d,  service_handle %d", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
        a_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        a_property,
                                                        &gatts_demo_char1_val, NULL);
        if (add_char_ret)
        {
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x", add_char_ret);
        }
        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT:
    {
        uint16_t length = 0;
        const uint8_t *prf_char;

        ESP_LOGI(GATTS_TAG, "ADD_CHAR_EVT, status %d,  attr_handle %d, service_handle %d",
                 param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].char_handle = param->add_char.attr_handle;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].descr_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;
        esp_err_t get_attr_ret = esp_ble_gatts_get_attr_value(param->add_char.attr_handle, &length, &prf_char);
        if (get_attr_ret == ESP_FAIL)
        {
            ESP_LOGE(GATTS_TAG, "ILLEGAL HANDLE");
        }

        ESP_LOGI(GATTS_TAG, "the gatts demo char length = %x", length);
        for (int i = 0; i < length; i++)
        {
            ESP_LOGI(GATTS_TAG, "prf_char[%x] =%x", i, prf_char[i]);
        }
        esp_err_t add_descr_ret = esp_ble_gatts_add_char_descr(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].descr_uuid,
                                                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE, NULL, NULL);
        if (add_descr_ret)
        {
            ESP_LOGE(GATTS_TAG, "add char descr failed, error code =%x", add_descr_ret);
        }
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        ESP_LOGI(GATTS_TAG, "ADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        ESP_LOGI(GATTS_TAG, "SERVICE_START_EVT, status %d, service_handle %d",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT:
    {
        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20; // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10; // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;  // timeout = 400*10ms = 4000ms
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        // start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CONF_EVT:
        ESP_LOGI(GATTS_TAG, "ESP_GATTS_CONF_EVT, status %d attr_handle %d", param->conf.status, param->conf.handle);
        if (param->conf.status != ESP_GATT_OK)
        {
            esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT)
    {
        if (param->reg.status == ESP_GATT_OK)
        {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        }
        else
        {
            ESP_LOGI(GATTS_TAG, "Reg app failed, app_id %04x, status %d",
                     param->reg.app_id,
                     param->reg.status);
            return;
        }
    }

    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    do
    {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++)
        {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                gatts_if == gl_profile_tab[idx].gatts_if)
            {
                if (gl_profile_tab[idx].gatts_cb)
                {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);
}

void app_main(void)
{
    esp_err_t ret;

    // Initialize NVS.
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_bluedroid_init();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }
    ret = esp_bluedroid_enable();
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s", __func__, esp_err_to_name(ret));
        return;
    }

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    ret = esp_ble_gatts_app_register(PROFILE_B_APP_ID);
    if (ret)
    {
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return;
    }
    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(500);
    if (local_mtu_ret)
    {
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    init_spi_amm_ohm(); // Initialize ammeter and ohmmeter
    init_spi_osc_volt();

    return;
}
