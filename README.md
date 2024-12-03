This is the GitHub repository for Group 30: ECEN 215 Lab Kit from Fall '24, ECEN 404 section 902. This branch contains the firmware code for the final ECEN 215 Lab Kit device. All of the group's reports, presentations, research, meeting notes, etc., can be found in the Documents folder. The PCB and Case folder contains all PCB schematics and case design drawings/markups. All mobile app code can be found in the FlutterFlow branch of this repository.

To operate this device, you must have a physical ECEN 215 Lab Kit and a mobile phone. If using a pre-configured lab kit, the user only needs to press the Enable button through the case to activate the device and connect to it via the mobile application. Please see the FlutterFlow branch for instructions on how to download the application to a mobile device.

If the lab kit is a brand-new PCB, the user will need to connect the Tx, Rx, and GND pins on the PCB to a computer. Once connected, download this code, open it in an Espressif IDF environment, and build and flash the PCB. If done successfully, the Lab Kit will be ready for Bluetooth connection via the mobile application. To operate the lab kit, connect the wires of the corresponding instrument and the device will display the data measured. The wires are given in the order from top row and from left to right:  
<div align="center">
  |  +5v power supply  |  +voltmeter/+oscilloscope  |  ground                    |  +ammeter   |  +ohmmeter |<br>
                                                   |   Wavegen           |  -5v power supply          |  -voltmeter/-oscilloscope  |  -ammeter   |  -ohmmeter  |
</div>
<br>Wires are also color coded in which the wires for +/- power supply are red, +/- voltmeter/oscilloscope are blue, +/- ammeter are orange, +/- ohmmeter are brown, ground is black, and Wavegen is yellow.<br>


The information below provides further insight into how this code enables the onboard ESP32 to connect via Bluetooth.



| Supported Targets | ESP32 | ESP32-C2 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- |

# ESP-IDF Gatt Server Example

This example shows how create a GATT service by adding attributes one by one. However, this method is defined by Bluedroid and is difficult for users to use.

Hence, we also allow users to create a GATT service with an attribute table, which releases the user from adding attributes one by one. And it is recommended for users to use. For more information about this method, please refer to [gatt_server_service_table_demo](../gatt_server_service_table).

This demo creates GATT a service and then starts advertising, waiting to be connected to a GATT client.

To test this demo, we can run the [gatt_client_demo](../gatt_client), which can scan for and connect to this demo automatically. They will start exchanging data once the GATT client has enabled the notification function of the GATT server.

Please, check this [tutorial](tutorial/Gatt_Server_Example_Walkthrough.md) for more information about this example.

## How to Use Example

Before project configuration and build, be sure to set the correct chip target using:

```bash
idf.py set-target <chip_name>
```

### Hardware Required

* A development board with ESP32/ESP32-C3/ESP32-C2/ESP32-H2/ESP-S3 SoC (e.g., ESP32-DevKitC, ESP-WROVER-KIT, etc.)
* A USB cable for Power supply and programming

See [Development Boards](https://www.espressif.com/en/products/devkits) for more information about it.

### Build and Flash

Run `idf.py -p PORT flash monitor` to build, flash and monitor the project.

(To exit the serial monitor, type ``Ctrl-]``.)

See the [Getting Started Guide](https://idf.espressif.com/) for full steps to configure and use ESP-IDF to build projects.

### Settings for UUID128

This example works with UUID16 as default. To change to UUID128, follow this steps:

1. Change the UUID16 to UUID128. You can change the UUID according to your needs.

```c
// Create a new UUID128 (using random values for this example)
uint8_t gatts_service_uuid128_test_X[ESP_UUID_LEN_128] = {0x06, 0x18, 0x7a, 0xec, 0xbe, 0x11, 0x11, 0xea, 0x00, 0x16, 0x02, 0x42, 0x01, 0x13, 0x00, 0x04};
```

By adding this new UUID128, you can remove the `#define` macros with the old UUID16.

2. Add the new UUID128 to the profile.

```c
// Change the size of the UUID from 16 to 128
gl_profile_tab[PROFILE_X_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_128;
// Copy the new UUID128 to the profile
memcpy(gl_profile_tab[PROFILE_X_APP_ID].service_id.id.uuid.uuid.uuid128, gatts_service_uuid128_test_X, ESP_UUID_LEN_128);
```

3. Remove the following line(s)
```c
gl_profile_tab[PROFILE_X_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_X;
```

## Example Output

```
I (0) cpu_start: Starting scheduler on APP CPU.
I (512) BTDM_INIT: BT controller compile version [1342a48]
I (522) system_api: Base MAC address is not set
I (522) system_api: read default base MAC address from EFUSE
I (522) phy_init: phy_version 4670,719f9f6,Feb 18 2021,17:07:07
I (922) GATTS_DEMO: REGISTER_APP_EVT, status 0, app_id 0

I (932) GATTS_DEMO: CREATE_SERVICE_EVT, status 0,  service_handle 40

I (942) GATTS_DEMO: REGISTER_APP_EVT, status 0, app_id 1

I (952) GATTS_DEMO: SERVICE_START_EVT, status 0, service_handle 40

I (952) GATTS_DEMO: ADD_CHAR_EVT, status 0,  attr_handle 42, service_handle 40

I (962) GATTS_DEMO: the gatts demo char length = 3

I (962) GATTS_DEMO: prf_char[0] =11

I (972) GATTS_DEMO: prf_char[1] =22

I (972) GATTS_DEMO: prf_char[2] =33

I (982) GATTS_DEMO: CREATE_SERVICE_EVT, status 0,  service_handle 44

I (982) GATTS_DEMO: ADD_DESCR_EVT, status 0, attr_handle 43, service_handle 40

I (992) GATTS_DEMO: SERVICE_START_EVT, status 0, service_handle 44

I (1002) GATTS_DEMO: ADD_CHAR_EVT, status 0,  attr_handle 46, service_handle 44

I (1012) GATTS_DEMO: ADD_DESCR_EVT, status 0, attr_handle 47, service_handle 44

```

## Troubleshooting

For any technical queries, please open an [issue](https://github.com/espressif/esp-idf/issues) on GitHub. We will get back to you soon.
