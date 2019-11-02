
/** @file
 * ble_sdk_app_beacon
 * Beacon Transmitter with DHT11.
 */

#include <stdbool.h>
#include <stdint.h>
#include "nordic_common.h"
#include "bsp.h"
#include "nrf_soc.h"
#include "nrf_sdh.h"
#include "nrf_sdh_ble.h"
#include "ble_advdata.h"
#include "app_timer.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define ADVERTISING_LED                 BSP_BOARD_LED_0                    /**< Is on when device is advertising. */
#define APP_BLE_CONN_CFG_TAG            1                                  /**< A tag identifying the SoftDevice BLE configuration. */

#define NON_CONNECTABLE_ADV_INTERVAL    MSEC_TO_UNITS(100, UNIT_0_625_MS)  /**< The advertising interval for non-connectable advertisement (100 ms). This value can vary between 100ms to 10.24s). */

#define APP_BEACON_INFO_LENGTH          0x17                               /**< Total length of information advertised by the Beacon. */
#define APP_ADV_DATA_LENGTH             0x15                               /**< Length of manufacturer specific data in the advertisement. */
#define APP_DEVICE_TYPE                 0x02                               /**< 0x02 refers to Beacon. */
#define APP_MEASURED_RSSI               0xC3                               /**< The Beacon's measured RSSI at 1 meter distance in dBm. */
#define APP_COMPANY_IDENTIFIER          0x0059                             /**< Company identifier for Nordic Semiconductor ASA. as per www.bluetooth.org. */
#define APP_MAJOR_VALUE                 0x01, 0x02                         /**< Major value used to identify Beacons. */
#define APP_MINOR_VALUE                 0x03, 0x04                         /**< Minor value used to identify Beacons. */
#define APP_BEACON_UUID                 0x01, 0x12, 0x23, 0x34, \
                                        0x45, 0x56, 0x67, 0x78, \
                                        0x89, 0x9a, 0xab, 0xbc, \
                                        0xcd, 0xde, 0xef, 0xf0            /**< Proprietary UUID for Beacon. */

#define DEAD_BEEF                       0xDEADBEEF                         /**< Value used as error code on stack dump, can be used to identify stack location on stack unwind. */

#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
#define MAJ_VAL_OFFSET_IN_BEACON_INFO   18                                 /**< Position of the MSB of the Major Value in m_beacon_info array. */
#define UICR_ADDRESS                    0x10001080                         /**< Address of the UICR register used by this example. The major and minor versions to be encoded into the advertising data will be picked up from this location. */
#endif

#define DEVICE_NAME                       "DHT"

static ble_gap_adv_params_t m_adv_params;                                  /**< Parameters to be passed to the stack when starting advertising. */
static uint8_t              m_adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET; /**< Advertising handle used to identify an advertising set. */
static uint8_t              m_enc_advdata[BLE_GAP_ADV_SET_DATA_SIZE_MAX];  /**< Buffer for storing an encoded advertising set. */

// temperature & humidty sensor initialization
uint8_t temperature = 0;
uint8_t humidity = 0;
void DHT11_DIR_INPUT(void);
void DHT11_DIR_OUTPUT(void);
void DHT11_read(void);
uint8_t data[5]={0,0,0,0,0};
uint32_t DHT11_ONE_WIRE_INPUT_PIN = 4;
// declare advertisement data
void set_adv_data(bool init);


/**@brief Struct that contains pointers to the encoded advertising data. */
static ble_gap_adv_data_t m_adv_data =
{
    .adv_data =
    {
        .p_data = m_enc_advdata,
        .len    = BLE_GAP_ADV_SET_DATA_SIZE_MAX
    },
    .scan_rsp_data =
    {
        .p_data = NULL,
        .len    = 0

    }
};


static uint8_t m_beacon_info[APP_BEACON_INFO_LENGTH] =                    /**< Information advertised by the Beacon. */
{
    APP_DEVICE_TYPE,     // Manufacturer specific information. Specifies the device type in this implementation.
    APP_ADV_DATA_LENGTH, // Manufacturer specific information. Specifies the length of the manufacturer specific data in this implementation.
    APP_BEACON_UUID,     // 128 bit UUID value.
    APP_MAJOR_VALUE,     // Major arbitrary value that can be used to distinguish between Beacons.
    APP_MINOR_VALUE,     // Minor arbitrary value that can be used to distinguish between Beacons.
    APP_MEASURED_RSSI    // Manufacturer specific information. The Beacon's measured TX power in this implementation.

};


/**@brief Callback function for asserts in the SoftDevice.
 *
 * @details This function will be called in case of an assert in the SoftDevice.
 *
 * @warning This handler is an example only and does not fit a final product. You need to analyze
 *          how your product is supposed to react in case of Assert.
 * @warning On assert from the SoftDevice, the system can only recover on reset.
 *
 * @param[in]   line_num   Line number of the failing ASSERT call.
 * @param[in]   file_name  File name of the failing ASSERT call.
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * p_file_name)
{
    app_error_handler(DEAD_BEEF, line_num, p_file_name);
}

//flag for setting advdata
static volatile bool g_setAdvData = false;

//===============================================Get data from dht11==== Extract here==================================
//config pin as input
void DHT11_DIR_INPUT(void)
{
				nrf_gpio_cfg_input(DHT11_ONE_WIRE_INPUT_PIN, NRF_GPIO_PIN_PULLUP);
}
//config pin as output
void DHT11_DIR_OUTPUT(void)
{
				nrf_gpio_cfg_output(DHT11_ONE_WIRE_INPUT_PIN);
				//DHT11_ONE_WIRE_DIR = 1;
}

uint32_t expectPulse(bool level) 
{
  uint32_t count = 0;
  // Otherwise fall back to using digitalRead (this seems to be necessary on ESP8266
  // right now, perhaps bugs in direct port access functions?).
  
    while (nrf_gpio_pin_read(DHT11_ONE_WIRE_INPUT_PIN) == level) {
      if (count++ >= 2555) {
        return 0; // Exceeded timeout, fail.
      }
    }

  return count;
}

//initiate dht11 to give out data. Reading from the gpio pin. 
void DHT11_read(void) 
{
  // Check if sensor was read less than two seconds ago and return early
  // to use last reading.
  // Reset 40 bits of received data to zero.
  data[0] = data[1] = data[2] = data[3] = data[4] = 0;

  // Go into high impedence state to let pull-up raise data line level and
  // start the reading process.
  DHT11_DIR_OUTPUT();
  nrf_gpio_pin_write(DHT11_ONE_WIRE_INPUT_PIN, 1);
  nrf_delay_ms(250);

  // Request - First set data line low for 20 milliseconds.
  //Response - Low for 54us and then goes high for 80us.
  //To provide start pulse, pull down the data pin minimum 18ms and then pull up.
  //After the response from DHT11, then data is transmitted.
  nrf_gpio_pin_write(DHT11_ONE_WIRE_INPUT_PIN, 0);
  nrf_delay_ms(20);

  uint32_t cycles[80];
  {

    // End the start signal by setting data line high for 40 microseconds.
    nrf_gpio_pin_write(DHT11_ONE_WIRE_INPUT_PIN,1);
    nrf_delay_us(40);

    // Now start reading the data line to get the value from the DHT sensor.
    DHT11_DIR_INPUT();
    nrf_delay_us(10);  // Delay a bit to let sensor pull data line low.

    // First expect a low signal for ~80 microseconds followed by a high signal
    // for ~80 microseconds again.
    if (expectPulse(0) == 0) {
        printf("80_low wrong!\n");
      //_lastresult = false;
      //return _lastresult;
    }
    
    if (expectPulse(1) == 0) {
         printf("80_high wrong!\n");
       //_lastresult = false;
       //return _lastresult;
    }

    // Now read the 40 bits sent by the sensor.  Each bit is sent as a 50
    // microsecond low pulse followed by a variable length high pulse.  If the
    // high pulse is ~28 microseconds then it's a 0 and if it's ~70 microseconds
    // then it's a 1.  We measure the cycle count of the initial 50us low pulse
    // and use that to compare to the cycle count of the high pulse to determine
    // if the bit is a 0 (high state cycle count < low state cycle count), or a
    // 1 (high state cycle count > low state cycle count). Note that for speed all
    // the pulses are read into a array and then examined in a later step.
    for (int i=0; i<80; i+=2) 
    {
      cycles[i]   = expectPulse(0);
      cycles[i+1] = expectPulse(1);
    }
  } // Timing critical code is now complete.

  // Inspect pulses and determine which ones are 0 (high state cycle count < low
  // state cycle count), or 1 (high state cycle count > low state cycle count).
    for (int i=0; i<40; ++i)  
    {
        uint32_t lowCycles  = cycles[2*i];
        uint32_t highCycles = cycles[2*i+1];
        if ((lowCycles == 0) || (highCycles == 0))  
        {
            printf("Timeout waiting for pulse.\n\n");
             printf("data_levs wrong!\n");
           // _lastresult = false;
            //return _lastresult;
         }
    data[i/8] <<= 1;
    // Now compare the low and high cycle times to see if the bit is a 0 or 1.
    if (highCycles > lowCycles) {
      // High cycles are greater than 50us low cycle count, must be a 1.
      data[i/8] |= 1;
    }
    // Else high cycles are less than (or equal to, a weird case) the 50us low
    // cycle count so this must be a zero.  Nothing needs to be changed in the
    // stored data.
  }

  printf("Received:\r\n");
  printf("%d",data[0]); printf(", ");
  printf("%d",data[1]); printf(", ");
  printf("%d",data[2]); printf(", ");
  printf("%d",data[3]); printf(", ");
  printf("%d",data[4]); printf(" =? ");
  printf("%d\r\n",(data[0] + data[1] + data[2] + data[3]) & 0xFF);

  // Check we read 40 bits and that the checksum matches.
  if (data[4] == ((data[0] + data[1] + data[2] + data[3]) & 0xFF)) {
    humidity = data[0];
    temperature = data[2];
  }
  else {
         printf("Checksum failure!\n");
             // _lastresult = false;
              // return _lastresult;
       }
}
/*The "init" flag is used so that we don’t try to read sensors before they are initialized. 
* The main thing to understand above is that we are packing off custom sensor data in the manuf_data.data.p_data field. 
* At the receiving end, we have to unpack and put the data together in the same order, as we will see.
* So at this point our firmware is ready, the beacon is sending advertising packets containing sensor data periodically.*/

// set adv data
void set_adv_data(bool init) //call in main, execute when false
{
  uint32_t                  err_code;
  ble_advdata_t             advdata;  //declare data
  ble_advdata_manuf_data_t  manuf_specific_data; // Variable to hold manufacturer specific data
  uint8_t                   flags = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;

  // Initialize with easily identifiable data
  uint8_t data[] = {0xa1, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8};

  // get sensor data
  if (!init) //if not false
  {
    DHT11_read(); //getting data here
    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
    data[3] = 0;
    data[4] = 0;
    data[5] = 0;
    data[6] = temperature;
    data[7] = humidity;
    
  }
 manuf_specific_data.company_identifier = APP_COMPANY_IDENTIFIER;
 //manuf_specific_data.data.p_data = (uint8_t *) m_beacon_info;
manuf_specific_data.data.p_data = data;
//manuf_specific_data.data.size   = APP_BEACON_INFO_LENGTH;
manuf_specific_data.data.size   = sizeof(data);
   
    // Build and set advertising data.
    memset(&advdata, 0, sizeof(advdata));

    //advdata.name_type             = BLE_ADVDATA_NO_NAME;
    advdata.name_type             = BLE_ADVDATA_SHORT_NAME;
    advdata.short_name_len          = 5;
    advdata.flags                 = flags;
    advdata.p_manuf_specific_data = &manuf_specific_data;
    advdata.include_appearance    = true;

    

// Build advertising data struct to pass into @ref ble_advertising_init.
// void *memset(void *ptr, int x, size_t n);
// ptr ==> Starting address of memory to be filled
// x   ==> Value to be filled
// n   ==> Number of bytes to be filled starting from ptr to be filled

    err_code = ble_advdata_encode(&advdata, m_adv_data.adv_data.p_data, &m_adv_data.adv_data.len);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&m_adv_handle, &m_adv_data, &m_adv_params);
    APP_ERROR_CHECK(err_code);

}
/**@brief Function for initializing the Advertising functionality.
 *
 * @details Encodes the required advertising data and passes it to the stack.
 *          Also builds a structure to be passed to the stack when starting advertising.
 */
static void advertising_init(void)
{
    uint32_t      err_code;
    ble_gap_conn_sec_mode_t sec_mode;
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);
    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);
    
    set_adv_data(true); //call set_adv_data to assemble the data
    // Initialize advertising parameters (used when starting advertising).
    memset(&m_adv_params, 0, sizeof(m_adv_params));

    m_adv_params.properties.type = BLE_GAP_ADV_TYPE_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    m_adv_params.p_peer_addr     = NULL;    // Undirected advertisement.
    m_adv_params.filter_policy   = BLE_GAP_ADV_FP_ANY;
    m_adv_params.interval        = NON_CONNECTABLE_ADV_INTERVAL;
    m_adv_params.duration        = 0;       // Never time out.


#if defined(USE_UICR_FOR_MAJ_MIN_VALUES)
    // If USE_UICR_FOR_MAJ_MIN_VALUES is defined, the major and minor values will be read from the
    // UICR instead of using the default values. The major and minor values obtained from the UICR
    // are encoded into advertising data in big endian order (MSB First).
    // To set the UICR used by this example to a desired value, write to the address 0x10001080
    // using the nrfjprog tool. The command to be used is as follows.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val <your major/minor value>
    // For example, for a major value and minor value of 0xabcd and 0x0102 respectively, the
    // the following command should be used.
    // nrfjprog --snr <Segger-chip-Serial-Number> --memwr 0x10001080 --val 0xabcd0102
    
    uint16_t major_value = ((*(uint32_t *)UICR_ADDRESS) & 0xFFFF0000) >> 16;
    uint16_t minor_value = ((*(uint32_t *)UICR_ADDRESS) & 0x0000FFFF);

    uint8_t index = MAJ_VAL_OFFSET_IN_BEACON_INFO;

    m_beacon_info[index++] = MSB_16(major_value);
    m_beacon_info[index++] = LSB_16(major_value);

    m_beacon_info[index++] = MSB_16(minor_value);
    m_beacon_info[index++] = LSB_16(minor_value);
#endif   
    
}


/**@brief Function for starting advertising.
 */
static void advertising_start(void)
{
    ret_code_t err_code;

    err_code = sd_ble_gap_adv_start(m_adv_handle, APP_BLE_CONN_CFG_TAG);
    APP_ERROR_CHECK(err_code);

    err_code = bsp_indication_set(BSP_INDICATE_ADVERTISING);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing the BLE stack.
 *
 * @details Initializes the SoftDevice and the BLE event interrupt.
 */
static void ble_stack_init(void)
{
    ret_code_t err_code;

    err_code = nrf_sdh_enable_request();
    APP_ERROR_CHECK(err_code);

    // Configure the BLE stack using the default settings.
    // Fetch the start address of the application RAM.
    uint32_t ram_start = 0;
    err_code = nrf_sdh_ble_default_cfg_set(APP_BLE_CONN_CFG_TAG, &ram_start);
    APP_ERROR_CHECK(err_code);

    // Enable BLE stack.
    err_code = nrf_sdh_ble_enable(&ram_start);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing logging. */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

/**@brief Function for initializing LEDs. */
static void leds_init(void)
{
    ret_code_t err_code = bsp_init(BSP_INIT_LEDS, NULL);
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing timers. */
static void timers_init(void)
{
    ret_code_t err_code = app_timer_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for initializing power management.
 */
static void power_management_init(void)
{
    ret_code_t err_code;
    err_code = nrf_pwr_mgmt_init();
    APP_ERROR_CHECK(err_code);
}


/**@brief Function for handling the idle state (main loop).
 *
 * @details If there is no pending log operation, then sleep until next the next event occurs.
 */
static void idle_state_handle(void)
{
    if (NRF_LOG_PROCESS() == false)
    {
        nrf_pwr_mgmt_run();
    }
}


/**
 * @brief Function for application main entry.
 */
int main(void)
{
    // Initialize.
    bsp_board_led_on(ADVERTISING_LED);
    log_init();
    timers_init();
    leds_init();
    power_management_init();
    ble_stack_init();
    advertising_init();

    // Start execution.
    NRF_LOG_INFO("Beacon example started.");
    advertising_start();

    // Enter main loop.
    for (;; )
    {
        idle_state_handle();
        // one-shot flag to set adv data
     if(g_setAdvData) {
        set_adv_data(false); //call DHT11_read()
        g_setAdvData = false;
      }
    
    }
}


