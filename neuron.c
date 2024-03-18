/*
 * Copyright (C) 2014 BlueKitchen GmbH
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holders nor the names of
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 * 4. Any redistribution, use, or modification is done solely for
 *    personal benefit and not for any commercial purpose or for
 *    monetary gain.
 *
 * THIS SOFTWARE IS PROVIDED BY BLUEKITCHEN GMBH AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL BLUEKITCHEN
 * GMBH OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF
 * THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * Please inquire about commercial licensing options at 
 * contact@bluekitchen-gmbh.com
 *
 */

#include <unistd.h>

#include <stdint.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/multicore.h"
#include "pico/stdlib.h"

// Flags general discoverable, BR/EDR supported (== not supported flag not set) when ENABLE_GATT_OVER_CLASSIC is enabled
#ifdef ENABLE_GATT_OVER_CLASSIC
#define APP_AD_FLAGS 0x02
static uint8_t gatt_service_buffer[70];
#else
#define APP_AD_FLAGS 0x06
#endif

//#define NEURON_LATENCY_MS 3
#define NEURON_FRAMERATE_MS 100
#define NEURON_SENSITIVITY_RATIO 32

static bool NEURON_IS_FIRING = false;

static btstack_packet_callback_registration_t hci_event_callback_registration;

const uint8_t adv_data[] = {
    // Flags general discoverable
    0x02, BLUETOOTH_DATA_TYPE_FLAGS, APP_AD_FLAGS,
    // Name
    0x0c, BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME, 'L', 'E', ' ', 'S', 't', 'r', 'e', 'a', 'm', 'e', 'r', 
    // Incomplete List of 16-bit Service Class UUIDs -- FF10 - only valid for testing!
    0x03, BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS, 0x10, 0xff,
};
const uint8_t adv_data_len = sizeof(adv_data);

/* @section GAP LE setup for receiving advertisements
 *
 * @text GAP LE advertisements are received as custom HCI events of the 
 * GAP_EVENT_ADVERTISING_REPORT type. To receive them, you'll need to register
 * the HCI packet handler, as shown in Listing GAPLEAdvSetup.
 */

/* LISTING_START(GAPLEAdvSetup): Setting up GAP LE client for receiving advertisements */
static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size);

static void gap_le_advertisements_setup(void){

    l2cap_init();

    // setup SM: Display only
    sm_init();

#ifdef ENABLE_GATT_OVER_CLASSIC
    // init SDP, create record for GATT and register with SDP
    sdp_init();
    memset(gatt_service_buffer, 0, sizeof(gatt_service_buffer));
    gatt_create_sdp_record(gatt_service_buffer, 0x10001, ATT_SERVICE_GATT_SERVICE_START_HANDLE, ATT_SERVICE_GATT_SERVICE_END_HANDLE);
    sdp_register_service(gatt_service_buffer);
    printf("SDP service record size: %u\n", de_get_len(gatt_service_buffer));

    // configure Classic GAP
    gap_set_local_name("GATT Streamer BR/EDR 00:00:00:00:00:00");
    gap_ssp_set_io_capability(SSP_IO_CAPABILITY_DISPLAY_YES_NO);
    gap_discoverable_control(1);
#endif

    // Active scanning, 100% (scan interval = scan window)
    gap_set_scan_parameters(1,48,48);
    gap_start_scan(); 

    // setup advertisements
    uint16_t adv_int_min = 0x0030;
    uint16_t adv_int_max = 0x0030;
    uint8_t adv_type = 0;
    bd_addr_t null_addr;
    memset(null_addr, 0, 6);
    gap_advertisements_set_params(adv_int_min, adv_int_max, adv_type, 0, null_addr, 0x07, 0x00);
    gap_advertisements_set_data(adv_data_len, (uint8_t*) adv_data);
    gap_advertisements_enable(1);

    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);
}

/* LISTING_END */

/* @section GAP LE Advertising Data Dumper
 *
 * @text Here, we use the definition of advertising data types and flags as specified in 
 * [Assigned Numbers GAP](https://www.bluetooth.org/en-us/specification/assigned-numbers/generic-access-profile)
 * and [Supplement to the Bluetooth Core Specification, v4](https://www.bluetooth.org/DocMan/handlers/DownloadDoc.ashx?doc_id=282152).
 */

/* LISTING_START(GAPLEAdvDataTypesAndFlags): Advertising data types and flags */
static const char * ad_types[] = {
    "", 
    "Flags",
    "Incomplete List of 16-bit Service Class UUIDs",
    "Complete List of 16-bit Service Class UUIDs",
    "Incomplete List of 32-bit Service Class UUIDs",
    "Complete List of 32-bit Service Class UUIDs",
    "Incomplete List of 128-bit Service Class UUIDs",
    "Complete List of 128-bit Service Class UUIDs",
    "Shortened Local Name",
    "Complete Local Name",
    "Tx Power Level",
    "", 
    "", 
    "Class of Device",
    "Simple Pairing Hash C",
    "Simple Pairing Randomizer R",
    "Device ID",
    "Security Manager TK Value",
    "Slave Connection Interval Range",
    "",
    "List of 16-bit Service Solicitation UUIDs",
    "List of 128-bit Service Solicitation UUIDs",
    "Service Data",
    "Public Target Address",
    "Random Target Address",
    "Appearance",
    "Advertising Interval"
};

static const char * flags[] = {
    "LE Limited Discoverable Mode",
    "LE General Discoverable Mode",
    "BR/EDR Not Supported",
    "Simultaneous LE and BR/EDR to Same Device Capable (Controller)",
    "Simultaneous LE and BR/EDR to Same Device Capable (Host)",
    "Reserved",
    "Reserved",
    "Reserved"
};
/* LISTING_END */

/* @text BTstack offers an iterator for parsing sequence of advertising data (AD) structures, 
 * see [BLE advertisements parser API](../appendix/apis/#ble-advertisements-parser-api).
 * After initializing the iterator, each AD structure is dumped according to its type.
 */

/* LISTING_START(GAPLEAdvDataParsing): Parsing advertising data */
static void dump_advertisement_data(const uint8_t * adv_data, uint8_t adv_size){
    ad_context_t context;
    bd_addr_t address;
    uint8_t uuid_128[16];
    for (ad_iterator_init(&context, adv_size, (uint8_t *)adv_data) ; ad_iterator_has_more(&context) ; ad_iterator_next(&context)){
        uint8_t data_type    = ad_iterator_get_data_type(&context);
        uint8_t size         = ad_iterator_get_data_len(&context);
        const uint8_t * data = ad_iterator_get_data(&context);
        
        if (data_type > 0 && data_type < 0x1B){
            printf("    %s: ", ad_types[data_type]);
        } 
        int i;
        // Assigned Numbers GAP
    
        switch (data_type){
            case BLUETOOTH_DATA_TYPE_FLAGS:
                // show only first octet, ignore rest
                for (i=0; i<8;i++){
                    if (data[0] & (1<<i)){
                        printf("%s; ", flags[i]);
                    }

                }
                break;
            case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_16_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_LIST_OF_16_BIT_SERVICE_SOLICITATION_UUIDS:
                for (i=0; i<size;i+=2){
                    printf("%02X ", little_endian_read_16(data, i));
                }
                break;
            case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_32_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_32_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_LIST_OF_32_BIT_SERVICE_SOLICITATION_UUIDS:
                for (i=0; i<size;i+=4){
                    printf("%04"PRIX32, little_endian_read_32(data, i));
                }
                break;
            case BLUETOOTH_DATA_TYPE_INCOMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LIST_OF_128_BIT_SERVICE_CLASS_UUIDS:
            case BLUETOOTH_DATA_TYPE_LIST_OF_128_BIT_SERVICE_SOLICITATION_UUIDS:
                reverse_128(data, uuid_128);
                printf("%s", uuid128_to_str(uuid_128));
                break;
            case BLUETOOTH_DATA_TYPE_SHORTENED_LOCAL_NAME:
            case BLUETOOTH_DATA_TYPE_COMPLETE_LOCAL_NAME:
                for (i=0; i<size;i++){
                    printf("%c", (char)(data[i]));
                }
                break;
            case BLUETOOTH_DATA_TYPE_TX_POWER_LEVEL:
                printf("%d dBm", *(int8_t*)data);
                break;
            case BLUETOOTH_DATA_TYPE_SLAVE_CONNECTION_INTERVAL_RANGE:
                printf("Connection Interval Min = %u ms, Max = %u ms", little_endian_read_16(data, 0) * 5/4, little_endian_read_16(data, 2) * 5/4);
                break;
            case BLUETOOTH_DATA_TYPE_SERVICE_DATA:
                printf_hexdump(data, size);
                break;
            case BLUETOOTH_DATA_TYPE_PUBLIC_TARGET_ADDRESS:
            case BLUETOOTH_DATA_TYPE_RANDOM_TARGET_ADDRESS:
                reverse_bd_addr(data, address);
                printf("%s", bd_addr_to_str(address));
                break;
            case BLUETOOTH_DATA_TYPE_APPEARANCE: 
                // https://developer.bluetooth.org/gatt/characteristics/Pages/CharacteristicViewer.aspx?u=org.bluetooth.characteristic.gap.appearance.xml
                printf("%02X", little_endian_read_16(data, 0) );
                break;
            case BLUETOOTH_DATA_TYPE_ADVERTISING_INTERVAL:
                printf("%u ms", little_endian_read_16(data, 0) * 5/8 );
                break;
            case BLUETOOTH_DATA_TYPE_3D_INFORMATION_DATA:
                printf_hexdump(data, size);
                break;
            case BLUETOOTH_DATA_TYPE_MANUFACTURER_SPECIFIC_DATA: // Manufacturer Specific Data 
                printf(NEURON_IS_FIRING ? "Y" : "N");
                break;
            case BLUETOOTH_DATA_TYPE_CLASS_OF_DEVICE:
            case BLUETOOTH_DATA_TYPE_SIMPLE_PAIRING_HASH_C:
            case BLUETOOTH_DATA_TYPE_SIMPLE_PAIRING_RANDOMIZER_R:
            case BLUETOOTH_DATA_TYPE_DEVICE_ID: 
            case BLUETOOTH_DATA_TYPE_SECURITY_MANAGER_OUT_OF_BAND_FLAGS:
            default:
                printf("Advertising Data Type 0x%2x not handled yet", data_type); 
                break;
        }        
        printf("\n");
    }
    printf("\n");
}
/* LISTING_END */

/* @section HCI packet handler
 * 
 * @text The HCI packet handler has to start the scanning, 
 * and to handle received advertisements. Advertisements are received 
 * as HCI event packets of the GAP_EVENT_ADVERTISING_REPORT type,
 * see Listing GAPLEAdvPacketHandler.  
 */

/* LISTING_START(GAPLEAdvPacketHandler): Scanning and receiving advertisements */

static void packet_handler(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;

    bd_addr_t address;
    uint8_t address_type;
    uint8_t event_type;
    int8_t rssi;
    uint8_t length;
    const uint8_t * data;

    switch (hci_event_packet_get_type(packet)) {
        case GAP_EVENT_ADVERTISING_REPORT:
            gap_event_advertising_report_get_address(packet, address);
            event_type = gap_event_advertising_report_get_advertising_event_type(packet);
            address_type = gap_event_advertising_report_get_address_type(packet);
            rssi = gap_event_advertising_report_get_rssi(packet);
            length = gap_event_advertising_report_get_data_length(packet);
            data = gap_event_advertising_report_get_data(packet);
            printf("Advertisement (legacy) event: evt-type %u, addr-type %u, addr %s, rssi %d, data[%u] ", event_type,
               address_type, bd_addr_to_str(address), rssi, length);
            printf_hexdump(data, length);
            dump_advertisement_data(data, length);
            break;
#ifdef ENABLE_LE_EXTENDED_ADVERTISING
        case GAP_EVENT_EXTENDED_ADVERTISING_REPORT:
            gap_event_extended_advertising_report_get_address(packet, address);
            event_type = gap_event_extended_advertising_report_get_advertising_event_type(packet);
            address_type = gap_event_extended_advertising_report_get_address_type(packet);
            rssi = gap_event_extended_advertising_report_get_rssi(packet);
            length = gap_event_extended_advertising_report_get_data_length(packet);
            data = gap_event_extended_advertising_report_get_data(packet);
            printf("Advertisement (extended) event: evt-type %u, addr-type %u, addr %s, rssi %d, data[%u] ", event_type,
               address_type, bd_addr_to_str(address), rssi, length);
            printf_hexdump(data, length);
            dump_advertisement_data(data, length);
            break;
#endif
        default:
            break;
    }
}
/* LISTING_END */


/*
 * Blink LED at a frequency relative to best Wi-Fi RSSI
 */
void led_neuron() {
  while (true) {
    NEURON_IS_FIRING = time_us_64() % NEURON_SENSITIVITY_RATIO == 0;
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, NEURON_IS_FIRING);
    write(1, NEURON_IS_FIRING ? "+" : "-", 1);

    sleep_ms(NEURON_FRAMERATE_MS);
  }
}


int main(int argv, const char *argc[]) {
  stdio_init_all();

  if (cyw43_arch_init()) {
    write(2, "[ERROR: Wi-Fi init failed]", 26);
    return 1;
  }

  multicore_launch_core1(led_neuron);

  gap_le_advertisements_setup();
  hci_power_control(HCI_POWER_ON);

  while (true) {
    sleep_ms(NEURON_FRAMERATE_MS);
  }

  cyw43_arch_deinit();
  return 0;
}
