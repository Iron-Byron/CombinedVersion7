#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/sys/printk.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/types.h>
#include <stddef.h>
#include <string.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/settings/settings.h>
#include <hal/nrf_gpio.h>
#include <nrfx_timer.h>
#include <nrfx_clock.h>
#include <nrfx_gpiote.h>
#include <helpers/nrfx_gppi.h>
#include <nrfx_ppi.h>
#include <zephyr/sys/reboot.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/fs/nvs.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/adc.h>
#include <hal/nrf_power.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/settings/settings.h>
#include <mpsl_clock.h>
#include <math.h>

/* DEFINE THE TYPE OF PROBE BELOW 'BigProbe' = The Large Probe, any other spelling is the small probe */
#define BigProb
/* DEFINE THE TYPE OF PROBE BELOW 'BigProbe' = The Large Probe, any other spelling is the small probe */
#define NVS_PARTITION		storage_partition
#define NVS_PARTITION_DEVICE	FIXED_PARTITION_DEVICE(NVS_PARTITION)
#define NVS_PARTITION_OFFSET	FIXED_PARTITION_OFFSET(NVS_PARTITION)
#define WET_CAL_ID     10
#define DRY_CAL_ID     11
#define TEMP25_CAL_ID  12
#define PROBE_ID       13
#define COUNTS_CAL_ID  14   // Unique ID for storing Counts[0] to Counts[5]
#define WET_COUNTS_ID  15  // Unique ID for Wet Counts array
#define DRY_COUNTS_ID  16  // Unique ID for Dry Counts array

int wet_cal=0;
int dry_cal=0;
int temp25_cal=0;
char probe_id[10] = "GE000001"; // Default ID
volatile bool turnoffboot=false;
volatile bool ble_connected=false;  // Flag to track BLE connection state
volatile bool advertiseover=false;
volatile bool BLEon=false;
volatile bool startup=true;
volatile bool waitforRS =false;
volatile bool enableBoot=false;
volatile bool readbatt=false;
volatile uint32_t Counts[6];
volatile float MoisturePcnt[6];
volatile uint32_t AvgCount=0;
volatile uint8_t loopCnt=0;
volatile uint8_t arrayIndex=0;
volatile int temperature[7];
volatile float TempaeratureDegrees[6];
volatile int AvgTemperature=0;
volatile int internalTEMP=0;
volatile int voltage22=0;
volatile bool wakeup_flag=false; // Flag to indicate wake-up
volatile bool timesup=false;
volatile int TIMERhandlercounter=0;
volatile bool calibrateGetReadings=false;
volatile bool calibrateSaveReadingNVS=false;
volatile bool WetCalibration=false;
volatile bool DryCalibration=false;
volatile bool TemperatureCalibration=false;
volatile bool one_sec_window_active = false;
volatile bool temperature_measure_required = false;
volatile bool start_one_sec_counter_timer = false;

static const struct adc_dt_spec adc_channels[] = {
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 0),
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 1),
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 2),
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 3),
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 4),
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 5),
    ADC_DT_SPEC_GET_BY_IDX(DT_PATH(zephyr_user), 6),
};

#define DirPIN7 NRF_GPIO_PIN_MAP(0,7)
#define spPwrEnable1PIN4 NRF_GPIO_PIN_MAP(0,4)
#define PwrEnable1PIN17 NRF_GPIO_PIN_MAP(0,17)
#define PwrEnable2PIN26 NRF_GPIO_PIN_MAP(0,26)
#define PwrEnable3PIN18 NRF_GPIO_PIN_MAP(0,18)
#define PwrEnable4PIN15 NRF_GPIO_PIN_MAP(0,15)
#define PwrEnable5PIN20 NRF_GPIO_PIN_MAP(0,20)
#define PwrEnable6PIN19 NRF_GPIO_PIN_MAP(0,19)

const uint32_t pwr_pins[] = {
    PwrEnable1PIN17,
    PwrEnable2PIN26,
    PwrEnable3PIN18,
    PwrEnable4PIN15,
    PwrEnable5PIN20,
    PwrEnable6PIN19
};

#define Count1_PIN5  NRF_GPIO_PIN_MAP(0,5)
#define Count2_PIN11 NRF_GPIO_PIN_MAP(0,11)
#define Count3_PIN12 NRF_GPIO_PIN_MAP(0,12)
#define Count4_PIN16 NRF_GPIO_PIN_MAP(0,16)
#define Count5_PIN14 NRF_GPIO_PIN_MAP(0,14)
#define Count6_PIN13 NRF_GPIO_PIN_MAP(0,13)

const uint32_t count_pins[] = {
    Count1_PIN5,
    Count2_PIN11,
    Count3_PIN12,
    Count4_PIN16,
    Count5_PIN14,
    Count6_PIN13
};

#define SLEEP_TIME K_MINUTES(2) // 2-minute sleep time
#define INPUT_PIN    DT_GPIO_PIN(DT_ALIAS(sw0), gpios)
#define GPIOTE_INST    NRF_DT_GPIOTE_INST(DT_ALIAS(sw0), gpios)
#define GPIOTE_NODE    DT_NODELABEL(_CONCAT(gpiote, GPIOTE_INST))

void hfclk_start(void);
void boot_timer_handler(struct k_timer *timer_id);
void advertise_timer_handler(struct k_timer *timer_id);
void wakeup_timer_handler(struct k_timer *timer_id);
void one_sec_window_timer_handler(struct k_timer *timer_id);
void calibrate_timer_handler(struct k_timer *timer_id);
void print_stored_calibration_data(void);
void clear_calibration_data(void);
void enable_calibratetimer(void);
void disable_one_sec_window_timer(void);

K_TIMER_DEFINE(boot_timer, boot_timer_handler, NULL);
K_TIMER_DEFINE(advertise_timer, advertise_timer_handler, NULL);
K_TIMER_DEFINE(wakeup_timer, wakeup_timer_handler, NULL);
K_TIMER_DEFINE(one_sec_window_timer, one_sec_window_timer_handler, NULL);
K_TIMER_DEFINE(calibrate_timer, calibrate_timer_handler, NULL);

BUILD_ASSERT(IS_ENABLED(_CONCAT(CONFIG_, _CONCAT(NRFX_GPIOTE, GPIOTE_INST))),
    "NRFX_GPIOTE" STRINGIFY(GPIOTE_INST) " must be enabled in Kconfig");

// Get a reference to the TIMER1 instance
static const nrfx_timer_t my_counter = NRFX_TIMER_INSTANCE(1);
static nrfx_gpiote_t gpiote = NRFX_GPIOTE_INSTANCE(GPIOTE_INST);
static uint8_t in_channel;
static uint8_t ppi_channel;
static struct nvs_fs fs;
static uint8_t command_buf[20]; // Buffer for receiving GATT commands

// Thermistor parameters
#define R_FIXED 1000.0               // 1kΩ fixed resistor
#define BETA 4450//3950.0                  // Beta coefficient of the NTC thermistor
#define R_0 1000.0                    // Thermistor resistance at 25°C (1kΩ)
#define T_0 298.15                   // Temperature in Kelvin at 25°C

#define TIMER_1000MS_INSTANCE 3  // Using TIMER3 for 1000ms timer
#define TIMER_INTERVAL_US 1000000 // 1s in microseconds

static const nrfx_timer_t timer_1000ms = NRFX_TIMER_INSTANCE(4);


#ifdef BigProbe

#define RECEIVE_BUFF_SIZE 8
#define RECEIVE_TIMEOUT 100
LOG_MODULE_REGISTER(nrfx_sample, LOG_LEVEL_INF);
const struct device *uart= DEVICE_DT_GET(DT_NODELABEL(uart0));
static char my_msg[128]; // Non-const buffer
static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = {0};
static uint8_t buffer[RECEIVE_BUFF_SIZE];
static size_t pointerhead=0;
const uint8_t targetSequence1[] = {0x09, 0x10, 0x00, 0x00, 0x00, 0x00, 0xC1, 0x8D};
#define TARGET_SEQUENCE1_SIZE sizeof(targetSequence1)
const uint8_t targetSequence2[] = {0x07, 0x10, 0x00, 0x00, 0x00, 0x00, 0xC1, 0x8D};
#define TARGET_SEQUENCE2_SIZE sizeof(targetSequence2)
static bool new_data_available = false; // Flag to indicate new data has arrived
// static size_t head = 0;

void checkForTargetSequence(void);

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data) {
    static size_t head = 0; // Head of the circular buffer
    switch (evt->type) {
    case UART_RX_RDY:
        for (size_t i = 0; i < evt->data.rx.len; i++) {
            char received_char = evt->data.rx.buf[evt->data.rx.offset + i];

            if (received_char == '\r' || received_char == '\n') {
                continue;
            }
            buffer[head] = received_char;
            head = (head + 1) % sizeof(buffer);
            pointerhead=head;
        }
        // printk("Pointer tail (circular): %d\n",tail);
        checkForTargetSequence();
        // new_data_available = true; // Set flag to process data in main loop
        break;

    case UART_RX_DISABLED:
        // Re-enable UART RX after it is disabled
        uart_rx_enable(dev, rx_buf, sizeof(rx_buf), RECEIVE_TIMEOUT);
        break;

    default:
        break;
    }
}

void checkForTargetSequence(void)
{
    uint8_t match1 = 1;
    uint8_t match2 = 1;
    // Check for match with targetSequence1
    for (uint8_t i = 0; i < TARGET_SEQUENCE1_SIZE; i++)
    {
        uint8_t index = (pointerhead + RECEIVE_BUFF_SIZE - TARGET_SEQUENCE1_SIZE + i) % RECEIVE_BUFF_SIZE;

        if (buffer[index] != targetSequence1[i])
        {
            match1 = 0;
            break;
        }
    }
    // Check for match with targetSequence2
    for (uint8_t i = 0; i < TARGET_SEQUENCE2_SIZE; i++)
    {
        uint8_t index = (pointerhead + RECEIVE_BUFF_SIZE - TARGET_SEQUENCE2_SIZE + i) % RECEIVE_BUFF_SIZE;

        if (buffer[index] != targetSequence2[i])
        {
            match2 = 0;
            break;
        }
    }
    // Perform actions based on which sequence was detected
    if (match1)
    {
        nrf_gpio_pin_set(DirPIN7);
        enableBoot=true;
        printk("Boot sequence detected!\n");
        nrf_gpio_pin_clear(DirPIN7);
    }
    else if (match2)
    {
        nrf_gpio_pin_set(DirPIN7);
        printk("GWS,%d,%.2f,%d,%.2f,%d,%.2f,%d,%.2f,%d,%.2f,%d,%.2f,%d,15,16,1234567891234567891234567891234\r\n",
            Counts[0],TempaeratureDegrees[0],
            Counts[1],TempaeratureDegrees[1],
            Counts[2],TempaeratureDegrees[2],
            Counts[3],TempaeratureDegrees[3],
            Counts[4],TempaeratureDegrees[4],
            Counts[5],TempaeratureDegrees[5],
                    temperature[6]);
        // printk("GWS,%.1f,%.2f,%.1f,%.2f,%.1f,%.2f,%.1f,%.2f,%.1f,%.2f,%.1f,%.2f,%d,15,16,1234567891234567891234567891234\r\n",
        //     MoisturePcnt[0],TempaeratureDegrees[0],
        //     MoisturePcnt[1],TempaeratureDegrees[1],
        //     MoisturePcnt[2],TempaeratureDegrees[2],
        //     MoisturePcnt[3],TempaeratureDegrees[3],
        //     MoisturePcnt[4],TempaeratureDegrees[4],
        //     MoisturePcnt[5],TempaeratureDegrees[5],
        //               temperature[6]);
        nrf_gpio_pin_clear(DirPIN7);
    }
}
#endif

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// **Custom GATT Service UUIDs**
#define BT_UUID_CALIBRATION_SERVICE BT_UUID_DECLARE_16(0x180C)
#define BT_UUID_CALIBRATION_CHAR BT_UUID_DECLARE_16(0x2A56)

// **GATT Write Callback**
static ssize_t calibration_write_cb(struct bt_conn *conn,
    const struct bt_gatt_attr *attr,
    const void *buf, uint16_t len,
    uint16_t offset, uint8_t flags)
{
if (len >= sizeof(command_buf))
return BT_GATT_ERR(BT_ATT_ERR_INVALID_ATTRIBUTE_LEN);

memcpy(command_buf, buf, len);
command_buf[len] = '\0';  // Null-terminate string

nrf_gpio_pin_set(DirPIN7);
printk("Received BLE Command: %s\n", command_buf);

// **Handle Different Calibration Commands**
if (strcmp(command_buf, "CAL_WET") == 0) {
enable_calibratetimer();
WetCalibration = true;
printk("Wet Calibration Command Received\n");
} 
else if (strcmp(command_buf, "CAL_DRY") == 0) {
enable_calibratetimer();
DryCalibration = true;
printk("Dry Calibration Command Received\n");
} 
else if (strncmp(command_buf, "CAL_TEMP", 8) == 0) {
char *temp_str = command_buf + 9;  // Extract temperature value
while (*temp_str == ' ') temp_str++;  // Skip spaces
float temp = atof(temp_str);
TemperatureCalibration = true;
printk("Temperature Calibration Command Received: %.2f Degrees\n", temp);
}
else if (strcmp(command_buf, "GET_CAL") == 0) {
// Send stored calibration data via BLE
printk("Sending Stored Calibration Data...\n");
print_stored_calibration_data();
}
else if (strncmp(command_buf, "CAL_CLEAR", len) == 0) {
    clear_calibration_data();
    printk("Calibration data reset requested via BLE!\n");
}
else {
printk("Unknown BLE Command Received\n");
}

nrf_gpio_pin_clear(DirPIN7);
return len;
}

BT_GATT_SERVICE_DEFINE(calibration_svc,
    BT_GATT_PRIMARY_SERVICE(BT_UUID_CALIBRATION_SERVICE),
    BT_GATT_CHARACTERISTIC(BT_UUID_CALIBRATION_CHAR,
        BT_GATT_CHRC_WRITE,
        BT_GATT_PERM_WRITE,
        NULL, calibration_write_cb, command_buf),
);

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

// static const struct bt_data ad[] = {
//     BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
//     // BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(0x180C)), 
//     BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
// };

static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)), 
    BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID16_ALL, BT_UUID_16_ENCODE(0x180C)),  
};

////////////////////////////////////////////////////////////////////////////////////////////////////////////

static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) 
    {
        printk("Connection failed (err %u)\n", err);
        return;
    }
    printk("Connected\n");
    ble_connected = true;
    k_timer_stop(&boot_timer);
    enableBoot=false;
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    printk("Disconnected (reason %u)\n", reason);
    ble_connected = false;
    k_timer_start(&boot_timer, K_SECONDS(15), K_NO_WAIT);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected = connected,
    .disconnected = disconnected,
};

static void timer_init(void)
{
    nrfx_timer_config_t timer_config = NRFX_TIMER_DEFAULT_CONFIG(1000000); // 1 MHz frequency
    timer_config.bit_width = NRF_TIMER_BIT_WIDTH_32;    // Use a 32-bit counter
    timer_config.mode = NRF_TIMER_MODE_COUNTER;         // Set to counter mode

    int err = nrfx_timer_init(&my_counter, &timer_config, NULL); // No event handler needed for counter mode
    if (err != NRFX_SUCCESS) {
        // printk("Error initializing timer: %x\n", err);
        return;
    }

    // printk("Timer initialized in counter mode");
}

/* Function to initialize GPIOTE and PPI */
static void PPI_GPIOE_init(void)
{
    nrfx_err_t err;
    err = nrfx_gpiote_channel_alloc(&gpiote, &in_channel);
    if (err != NRFX_SUCCESS) {
        // printk("Failed to allocate in_channel, error: 0x%08X", err);
        return;
    }

    /* Allocate a PPI channel */
    err = nrfx_gppi_channel_alloc(&ppi_channel);
    if (err != NRFX_SUCCESS) {
        // printk("nrfx_gppi_channel_alloc error: 0x%08X", err);
        return;
    }

    // printk("GPIOTE and PPI initialized");
}

/* Function to assign a GPIO pin for event counting */
static void PPI_GPIOE_assign_pin(uint32_t pin)
{
    nrfx_err_t err;

    /* Configure the input pin for event detection */
    static const nrf_gpio_pin_pull_t pull_config = NRF_GPIO_PIN_PULLDOWN;
    nrfx_gpiote_trigger_config_t trigger_config = {
        .trigger = NRFX_GPIOTE_TRIGGER_HITOLO,
        .p_in_channel = &in_channel,
    };
    static const nrfx_gpiote_handler_config_t handler_config = {
        .handler = NULL, // No software handler needed; use PPI
    };
    nrfx_gpiote_input_pin_config_t input_config = {
        .p_pull_config = &pull_config,
        .p_trigger_config = &trigger_config,
        .p_handler_config = &handler_config
    };

    err = nrfx_gpiote_input_configure(&gpiote, pin, &input_config);
    if (err != NRFX_SUCCESS) {
        // printk("nrfx_gpiote_input_configure error: 0x%08X", err);
        return;
    }

    nrfx_gpiote_trigger_enable(&gpiote, pin, true);
    // printk("GPIOTE configured for pin %d", pin);

    /* Configure PPI to connect GPIOTE event to TIMER1 COUNT task */
    nrfx_gppi_channel_endpoints_setup(ppi_channel,
        nrfx_gpiote_in_event_address_get(&gpiote, pin),
        nrfx_timer_task_address_get(&my_counter, NRF_TIMER_TASK_COUNT));

    /* Enable the PPI channel */
    nrfx_gppi_channels_enable(BIT(ppi_channel));

    // printk("PPI configured, counter will increment on pin %d event", pin);
}

// Timer handler function
void one_sec_window_timer_handler(struct k_timer *timer_id)
{
    disable_one_sec_window_timer();
    one_sec_window_active=false;
    start_one_sec_counter_timer=true;
}
// Function to enable the timer
void enable_one_sec_window_timer(void)
{
    k_timer_start(&one_sec_window_timer, K_SECONDS(1), K_SECONDS(1));
}
// Function to disable the timer
void disable_one_sec_window_timer(void)
{
    k_timer_stop(&one_sec_window_timer);
}

void boot_timer_handler(struct k_timer *timer_id)
{
    turnoffboot=true;
    enableBoot=false;
}
void enable_boottimer(void)
{
    k_timer_start(&boot_timer, K_SECONDS(40), K_NO_WAIT);
}

void advertise_timer_handler(struct k_timer *timer_id)
{
    advertiseover=true;
}

void enable_advertisetimer(void)
{
    k_timer_start(&advertise_timer, K_SECONDS(30), K_NO_WAIT);
}

// Timer handler function (executed when timer expires)
void wakeup_timer_handler(struct k_timer *timer_id)
{
    wakeup_flag = true;
}

void calibrate_timer_handler(struct k_timer *timer_id)
{
    // WetCalibration=true;
    calibrateGetReadings=true;
    // nrf_gpio_pin_set(PwrEnable1PIN17);
}
void enable_calibratetimer(void)
{
    k_timer_start(&calibrate_timer, K_SECONDS(10), K_NO_WAIT);
}

void enter_low_power_mode(void)
{
    k_timer_start(&wakeup_timer, SLEEP_TIME, K_NO_WAIT);

    while (!wakeup_flag) {
        k_sleep(K_MSEC(100)); // Sleep to save power
    }
}


void timer_1000ms_handler(nrf_timer_event_t event_type, void *p_context)
{
    if (event_type == NRF_TIMER_EVENT_COMPARE2)
    {
        // nrf_gpio_pin_set(DirPIN7);
        // nrf_gpio_pin_toggle(PwrEnable1PIN17);
        // printk("1000ms Timer Triggered!\n");
        // nrf_gpio_pin_clear(DirPIN7);
        TIMERhandlercounter++;
        timesup = true;
    }
}
void timer_1000ms_init(void)
{
    nrfx_timer_config_t timer_cfgg = NRFX_TIMER_DEFAULT_CONFIG(1000000); // 1 MHz frequency
    timer_cfgg.bit_width = NRF_TIMER_BIT_WIDTH_32;    // Use a 32-bit counter
    timer_cfgg.mode = NRF_TIMER_MODE_TIMER;        // Set to counter mode
    timer_cfgg.interrupt_priority = 1;

    nrfx_err_t err = nrfx_timer_init(&timer_1000ms, &timer_cfgg, timer_1000ms_handler);
    if (err != NRFX_SUCCESS) {
        printk("Timer4 initialization failed! Error: %d\n", err);
        return;
    }
    IRQ_CONNECT(NRFX_IRQ_NUMBER_GET(NRF_TIMER4),
            1,                         // Match your interrupt_priority
            nrfx_timer_4_irq_handler,  // Provided by nrfx
            NULL,
            0);

irq_enable(NRFX_IRQ_NUMBER_GET(NRF_TIMER4));

    uint32_t ticks = nrfx_timer_us_to_ticks(&timer_1000ms, TIMER_INTERVAL_US);
    nrfx_timer_extended_compare(&timer_1000ms, NRF_TIMER_CC_CHANNEL2, ticks, NRF_TIMER_SHORT_COMPARE2_CLEAR_MASK, true);
    // nrfx_timer_enable(&timer_1000ms);

    // printk("1000ms Timer (TIMER4) Started!\n");
}

void ble_init(void)
{
    int err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return;
    }
    printk("Bluetooth initialized\n");
}

void ADC_init_funct(int ADCIndex, struct adc_sequence *sequence)
{
	int err;
	if (!adc_is_ready_dt(&adc_channels[ADCIndex])) {
	    // printk("ADC controller devivce %s not ready", adc_channels[ADCIndex].dev->name);
    }
	err = adc_channel_setup_dt(&adc_channels[ADCIndex]);
    if (err < 0) {
	    // printk("Could not setup channel #%d (%d)", ADCIndex, err);
    }
	err = adc_sequence_init_dt(&adc_channels[ADCIndex], sequence);
	if (err < 0) {
		// printk("Could not initalize sequnce");
	}
	err = adc_read(adc_channels[ADCIndex].dev, sequence);
	if (err < 0) {
		// printk("Could not read (%d)", err);
	}
}

// void ADC_Get_reading(int ADCIndex, struct adc_sequence *sequence)
// {
// 	int err;
// 	err = adc_read(adc_channels[ADCIndex].dev, sequence);
// 	if (err < 0) {
// 		// printk("Could not read (%d)", err);
// 	}


// 	int16_t *buf = (int16_t *)sequence->buffer; // Cast buffer pointer to int16_t

// 	// Print raw ADC value
//     // printk("ADC reading: %s, channel %d: Raw value = %d", 
//     //         adc_channels[ADCIndex].dev->name, 
//     //         adc_channels[ADCIndex].channel_id, 
//     //         *buf);

//     // Convert the raw value to millivolts
//     int val_mv = *buf;
//     err = adc_raw_to_millivolts_dt(&adc_channels[ADCIndex], &val_mv);
//     if (err < 0) {
//         // printk("Conversion to mV not available for channel #%d", ADCIndex);
//     } else {
//         // printk("Converted value = %d mV\n", val_mv);
//     }
//     AvgTemperature+=val_mv;
// }

#define NUM_ADC_SAMPLES 6

int adc_readings_mv[NUM_ADC_SAMPLES];
int adc_sample_count = 0;

void ADC_Get_reading(int ADCIndex, struct adc_sequence *sequence)//experimental
{
    // nrf_gpio_pin_set(DirPIN7);
    int err = adc_read(adc_channels[ADCIndex].dev, sequence);
    if (err < 0) {
        printk("ADC read failed on channel %d (err: %d)\n", ADCIndex, err);
        return;
    }

    int16_t *buf = (int16_t *)sequence->buffer;
    int val_mv = *buf;

    err = adc_raw_to_millivolts_dt(&adc_channels[ADCIndex], &val_mv);
    if (err < 0) {
        printk("Conversion to mV failed on channel %d (err: %d)\n", ADCIndex, err);
        return;
    }

    if (adc_sample_count < NUM_ADC_SAMPLES) {
        adc_readings_mv[adc_sample_count++] = val_mv;
        // printk("ADC Sample %d = %d mV\n", adc_sample_count, val_mv);
    }
    // nrf_gpio_pin_clear(DirPIN7);
}

int calculate_filtered_average(void) /////experimental
{
    if (adc_sample_count < NUM_ADC_SAMPLES) {
        // printk("Not enough samples collected yet: %d/%d\n", adc_sample_count, NUM_ADC_SAMPLES);
        return -1;
    }

    int min = adc_readings_mv[0];
    int max = adc_readings_mv[0];
    int sum = adc_readings_mv[0];

    // printk("ADC readings: ");
    // printk("[");
    for (int i = 1; i < NUM_ADC_SAMPLES; i++) {
        int val = adc_readings_mv[i];
        // printk("%d", val);
        // if (i < NUM_ADC_SAMPLES - 1) printk(", ");

        if (val < min) min = val;
        if (val > max) max = val;
        sum += val;
    }
    // printk("]\n");

    int adjusted_sum = sum - min - max;
    int avg = adjusted_sum / (NUM_ADC_SAMPLES - 2);

    // printk("Min: %d, Max: %d, Adjusted Sum: %d, Average: %d mV\n", min, max, adjusted_sum, avg);

    adc_sample_count = 0; // reset after calculation
    return avg;
}

// Configure GPIO pins as outputs with pull-down resistors
void configure_gpio_pins_with_pull_down(void)
{

    // Repeat for all power enable pins
    for (int i = 0; i < sizeof(pwr_pins) / sizeof(pwr_pins[0]); i++) {
        nrf_gpio_cfg(
            pwr_pins[i],                // Pin number
            NRF_GPIO_PIN_DIR_OUTPUT,    // Set as output
            NRF_GPIO_PIN_INPUT_DISCONNECT, // Disconnect input
            NRF_GPIO_PIN_PULLDOWN,      // Enable pull-down resistor
            NRF_GPIO_PIN_S0S1,          // Standard drive strength
            NRF_GPIO_PIN_NOSENSE        // No sense detection
        );
    }
}

#ifdef BigProbe
#else

#define DEVICE_NAME CONFIG_BT_DEVICE_NAME
#define DEVICE_NAME_LEN (sizeof(DEVICE_NAME) - 1)

static uint8_t mfg_data[] = { 0x59, 0x00, 0x00, 0xff, 0x00, 0x33, 0x12, 0x34}; // Nordic's company ID is 0x0059
// Local name
static const char device_name[] = "Gentick-SMP 4";

// Advertising data
static const struct bt_data adman[] = {
    // Include the device name in the advertising packet
    BT_DATA(BT_DATA_NAME_COMPLETE, device_name, sizeof(device_name) - 1),

    // Include manufacturer-specific data
    BT_DATA(BT_DATA_MANUFACTURER_DATA, mfg_data, sizeof(mfg_data)),

    // Include additional custom data
    // BT_DATA(BT_DATA_SVC_DATA16, custom_data, sizeof(custom_data)),
};

/* Define the alias for the die temperature sensor */
#define DIE_TEMP_NODE DT_ALIAS(die_temp)
/* Get the single sensor device */
static const struct device *sensor_dev = DEVICE_DT_GET(DIE_TEMP_NODE);
volatile int die_temperature_scaled = 0;
static uint16_t temp_high;  // Upper 2 bytes
static uint16_t temp_low;   // Lower 2 bytes

static int print_die_temperature(void)
{
    struct sensor_value val;
    int rc;
    /* Fetch sensor samples */
    rc = sensor_sample_fetch(sensor_dev);
    if (rc) {
        // printk("Failed to fetch sample (%d)\n", rc);
        return rc;
    }
    /* Get the temperature value */
    rc = sensor_channel_get(sensor_dev, SENSOR_CHAN_DIE_TEMP, &val);
    if (rc) 
    {
        // printk("Failed to get data (%d)\n", rc);
        return rc;
    }
    /* Convert temperature to scaled integer (x10 for 0.1°C precision) */
    die_temperature_scaled += (val.val1 * 10) + (val.val2 / 100000);
    return 0;
}

#endif

void take_readings(struct adc_sequence *sequence)
{
    #ifdef BigProbe
    TIMERhandlercounter=0;
    loopCnt=0;
    arrayIndex=0;
    AvgCount = 0; 
    AvgTemperature = 0;
    nrf_gpio_pin_set(pwr_pins[arrayIndex]);
    temperature_measure_required=true;
    one_sec_window_active=true;
    enable_one_sec_window_timer();
    PPI_GPIOE_assign_pin(count_pins[arrayIndex]);
    ADC_init_funct(arrayIndex,sequence);
    start_one_sec_counter_timer = false;//Not sure if necessary 
    #else
    hfclk_start();//Must start the HFXCLK to get accurate internal temperature readings
    TIMERhandlercounter=0;
    loopCnt=0;
    arrayIndex=0;
    AvgCount = 0;  // Ensure counter values don't accumulate
    AvgTemperature = 0;
    nrf_gpio_pin_set(spPwrEnable1PIN4);
    nrf_gpio_pin_set(pwr_pins[arrayIndex]);
    nrf_gpio_pin_set(pwr_pins[5]);
    temperature_measure_required=true;
    one_sec_window_active=true;
    enable_one_sec_window_timer();
    start_one_sec_counter_timer = false;//Not sure if necessary 
    #endif
}

void bootloaderON(void)
{
    int err;
    err = bt_enable(NULL);
    BLEon=true;
    if (err) 
    {
        printk("Bluetooth init failed (err %d)\n", err);
    }
    if (IS_ENABLED(CONFIG_SETTINGS)) {
        settings_load();
    }
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), sd, ARRAY_SIZE(sd));
    // err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad), NULL, 0);
    if (err) 
    {
        printk("Advertising failed to start (err %d)\n", err);
    }
}

void nvs_init(void) {
    int rc;
    struct flash_pages_info info;
    fs.flash_device = NVS_PARTITION_DEVICE;
    if (!device_is_ready(fs.flash_device)) {
        printk("Flash device %s is not ready\n", fs.flash_device->name);
        return;
    }
    fs.offset = NVS_PARTITION_OFFSET;
    rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
    if (rc) {
        printk("Unable to get page info\n");
        return;
    }
    fs.sector_size = info.size;
    fs.sector_count = 6;  // Ensure full 24 KB is used
    rc = nvs_mount(&fs);
    if (rc) {
        printk("NVS Mount failed\n");
        return;
    }
    // printk("NVS Mounted Successfully\n");
}

void print_nvs_info(void)
{
    struct flash_pages_info info;
    int rc;
    fs.offset = NVS_PARTITION_OFFSET;
    rc = flash_get_page_info_by_offs(fs.flash_device, fs.offset, &info);
    if (rc) {
        printk("Unable to get page info\n");
        return;
    }
    fs.sector_size = info.size;
    fs.sector_count = 3U;
    printk("\n===== NVS Flash Usage =====\n");
    printk("Flash Page Size: %d bytes\n", fs.sector_size);
    printk("Number of Sectors: %d\n", fs.sector_count);
    printk("Total Flash Used: %d bytes\n", fs.sector_size * fs.sector_count);
    printk("=================================\n");
}

void load_calibration_data(void) {
    int rc;
    rc = nvs_read(&fs, WET_CAL_ID, &wet_cal, sizeof(wet_cal));
    if (rc <= 0) {
        printk("No wet calibration found, setting default\n");
        wet_cal = 100;  // Default value
        nvs_write(&fs, WET_CAL_ID, &wet_cal, sizeof(wet_cal));
    }
    rc = nvs_read(&fs, DRY_CAL_ID, &dry_cal, sizeof(dry_cal));
    if (rc <= 0) {
        printk("No dry calibration found, setting default\n");
        dry_cal = 500;  // Default value
        nvs_write(&fs, DRY_CAL_ID, &dry_cal, sizeof(dry_cal));
    }
    rc = nvs_read(&fs, TEMP25_CAL_ID, &temp25_cal, sizeof(temp25_cal));
    if (rc <= 0) {
        printk("No temp25 calibration found, setting default\n");
        temp25_cal = 25;  // Default value
        nvs_write(&fs, TEMP25_CAL_ID, &temp25_cal, sizeof(temp25_cal));
    }
    rc = nvs_read(&fs, PROBE_ID, probe_id, sizeof(probe_id));
    if (rc <= 0) {
        printk("No probe ID found, setting default\n");
        strcpy(probe_id, "GE000001");  // Default ID
        nvs_write(&fs, PROBE_ID, probe_id, sizeof(probe_id));
    }
    // printk("Loaded Calibration Data:\nWet: %d, Dry: %d, Temp25: %d, Probe ID: %s\n",
        //    wet_cal, dry_cal, temp25_cal, probe_id);
}

void save_calibration_data(int new_wet, int new_dry, int new_temp25, const char *new_id) {
    nvs_write(&fs, WET_CAL_ID, &new_wet, sizeof(new_wet));
    nvs_write(&fs, DRY_CAL_ID, &new_dry, sizeof(new_dry));
    nvs_write(&fs, TEMP25_CAL_ID, &new_temp25, sizeof(new_temp25));
    nvs_write(&fs, PROBE_ID, new_id, strlen(new_id) + 1); // Include null terminator
    printk("Calibration data saved!\n");
}

void print_stored_calibration_data(void) {
    int rc;
    int stored_temp25;
    char stored_probe_id[10];
    printk("\n===== Stored Calibration Data in NVS =====\n");
    int stored_wet_counts[6];
    int stored_dry_counts[6];

    rc = nvs_read(&fs, WET_COUNTS_ID, stored_wet_counts, sizeof(stored_wet_counts));
    if (rc > 0) {
        printk("Stored Wet Calibration Counts:\n");
        for (int i = 0; i < 6; i++) {
            printk("WetCounts[%d]: %d\n", i, stored_wet_counts[i]);
        }
    } else {
        printk("No Wet Calibration Counts Found!\n");
    }

    rc = nvs_read(&fs, DRY_COUNTS_ID, stored_dry_counts, sizeof(stored_dry_counts));
    if (rc > 0) {
        printk("Stored Dry Calibration Counts:\n");
        for (int i = 0; i < 6; i++) {
            printk("DryCounts[%d]: %d\n", i, stored_dry_counts[i]);
        }
    } else {
        printk("No Dry Calibration Counts Found!\n");
    }
    // Read Temp25 Calibration Value
    rc = nvs_read(&fs, TEMP25_CAL_ID, &stored_temp25, sizeof(stored_temp25));
    if (rc > 0) {
        printk("Stored Temp25 Calibration: %d\n", stored_temp25);
    } else {
        printk("No Temp25 Calibration Data Found!\n");
    }
    // Read Probe ID
    rc = nvs_read(&fs, PROBE_ID, stored_probe_id, sizeof(stored_probe_id));
    if (rc > 0) {
        printk("Stored Probe ID: %s\n", stored_probe_id);
    } else {
        printk("No Probe ID Found!\n");
    }
    printk("=========================================\n");
}

void save_dry_calibration_data(void) {
    int stored_dry_counts[6];  // Array to store previously saved dry calibration values
    int rc;

    // Read the currently stored dry calibration counts from NVS
    rc = nvs_read(&fs, DRY_COUNTS_ID, stored_dry_counts, sizeof(stored_dry_counts));

    if (rc > 0) {
        bool updated = false;  // Flag to track if we need to update NVS

        for (int i = 0; i < 6; i++) {
            // Ensure we only save a higher value
            if (Counts[i] > stored_dry_counts[i]) {
                stored_dry_counts[i] = Counts[i];  // Update with the new higher value
                updated = true;  // Mark that we need to save
            }
        }

        if (updated) {
            rc = nvs_write(&fs, DRY_COUNTS_ID, stored_dry_counts, sizeof(stored_dry_counts));
            if (rc > 0) {
                printk("Dry Calibration Counts updated successfully!\n");
            } else {
                printk("Failed to save dry calibration counts (error %d)\n", rc);
            }
        } else {
            printk("Dry Calibration Skipped: No higher values detected.\n");
        }

    } else {
        // No previous dry calibration data found, write the current values
        rc = nvs_write(&fs, DRY_COUNTS_ID, (const uint32_t *)Counts, sizeof(Counts));//Remove the (const uint32_t *) if it does not work
        if (rc > 0) {
            printk("First Dry Calibration Counts saved successfully!\n");
        } else {
            printk("Failed to save dry calibration counts (error %d)\n", rc);
        }
    }
}

void save_wet_calibration_data(void) {
    int stored_wet_counts[6];  // Array to store previously saved wet calibration values
    int rc;

    // Read the currently stored wet calibration counts from NVS
    rc = nvs_read(&fs, WET_COUNTS_ID, stored_wet_counts, sizeof(stored_wet_counts));

    if (rc > 0) {
        bool updated = false;  // Flag to track if we need to update NVS

        for (int i = 0; i < 6; i++) {
            // Ensure we only save a lower value (wet calibration should be lower)
            if (Counts[i] < stored_wet_counts[i] || stored_wet_counts[i] == 0) {
                stored_wet_counts[i] = Counts[i];  // Update with the new lower value
                updated = true;  // Mark that we need to save
            }
        }

        if (updated) {
            rc = nvs_write(&fs, WET_COUNTS_ID, stored_wet_counts, sizeof(stored_wet_counts));
            if (rc > 0) {
                printk("Wet Calibration Counts updated successfully!\n");
            } else {
                printk("Failed to save wet calibration counts (error %d)\n", rc);
            }
        } else {
            printk("ℹWet Calibration Skipped: No lower values detected.\n");
        }

    } else {
        // No previous wet calibration data found, write the current values
        rc = nvs_write(&fs, WET_COUNTS_ID, (const uint32_t *)Counts, sizeof(Counts)); //Remove the (const uint32_t *) if it does not work
        if (rc > 0) {
            printk("First Wet Calibration Counts saved successfully!\n");
        } else {
            printk("Failed to save wet calibration counts (error %d)\n", rc);
        }
    }
}

void clear_calibration_data(void) {
    int rc;

    // Create an empty array filled with 0xFF (default erased flash value)
    int dry_counts[6];
    memset(dry_counts, 0x00, sizeof(dry_counts));
    int wet_counts[6]= {30000, 30000, 30000, 30000, 30000, 30000};

    // Clear Wet Calibration Data
    rc = nvs_write(&fs, WET_COUNTS_ID, wet_counts, sizeof(wet_counts));
    if (rc > 0) {
        printk("Wet Calibration Data Cleared!\n");
    } else {
        printk("Failed to clear Wet Calibration Data (error %d)\n", rc);
    }

    // Clear Dry Calibration Data
    rc = nvs_write(&fs, DRY_COUNTS_ID, dry_counts, sizeof(dry_counts));
    if (rc > 0) {
        printk("Dry Calibration Data Cleared!\n");
    } else {
        printk("Failed to clear Dry Calibration Data (error %d)\n", rc);
    }
}

void hfclk_callback(void)
{
    // printk("HFCLK started!\n");
}

void hfclk_start(void)
{
    uint32_t err_code;
    uint32_t is_running = 0;  // Variable to store clock status

    // Request HFCLK and provide a NULL callback (if you don't need one)
    err_code = mpsl_clock_hfclk_request(hfclk_callback);
    if (err_code != 0) {
        printk("Error: %d\n", err_code);
    }

    // Wait until HFCLK is running
    while (!is_running)
    {
        err_code = mpsl_clock_hfclk_is_running(&is_running);  // Pass pointer
        if (err_code != 0) {
            printk("Error: %d\n", err_code);
        }
        k_sleep(K_MSEC(1));  // Avoid busy-waiting
    }
}

void calculate_moisture(void)
{

    int rc;
    // printk("\n===== Stored Calibration Data in NVS from Calc Moisture =====\n");
    int stored_wet_counts[6];
    int stored_dry_counts[6];

    rc = nvs_read(&fs, WET_COUNTS_ID, stored_wet_counts, sizeof(stored_wet_counts));
    // if (rc > 0) {
    //     printk("Stored Wet Calibration Counts:\n");
    //     for (int i = 0; i < 6; i++) {
    //         printk("WetCounts[%d]: %d\n", i, stored_wet_counts[i]);
    //     }
    // } else {
    //     printk("No Wet Calibration Counts Found!\n");
    // }

    rc = nvs_read(&fs, DRY_COUNTS_ID, stored_dry_counts, sizeof(stored_dry_counts));
    // if (rc > 0) {
    //     printk("Stored Dry Calibration Counts:\n");
    //     for (int i = 0; i < 6; i++) {
    //         printk("DryCounts[%d]: %d\n", i, stored_dry_counts[i]);
    //     }
    // } else {
    //     printk("No Dry Calibration Counts Found!\n");
    // }
    #ifdef BigProbe
    for(int i=0; i<6; i++)
    #else
    for(int i=0; i<1; i++)
    #endif
    {
        if(stored_dry_counts[i]>=Counts[i] && stored_dry_counts[i]>stored_wet_counts[i])
        {
            // printk("Dry value: %d\n",stored_dry_counts[i]);
            // printk("Wet value: %d\n",stored_wet_counts[i]);
            // printk("Read value: %d\n",Counts[i]);
            MoisturePcnt[i]= (100.0*(stored_dry_counts[i]-Counts[i]))/(stored_dry_counts[i]-stored_wet_counts[i]);
            if(MoisturePcnt[i]>100) MoisturePcnt[i]=100;
            if(MoisturePcnt[i]<0) MoisturePcnt[i]=0;
            // printk("Moisture Perecentage %d: %.1f\n", i,MoisturePcnt[i]);
        }
        else
        {   MoisturePcnt[i]=0;
            // printk("Moisture Perecentage %d: %.1f\n", i,MoisturePcnt[i]);
            // printk("Dry Calibration is required!\n");
        }
    }
}

static float calc_temp(int v_out_mv) {
    float v_out = v_out_mv;  // Convert mV to V
    float r_ntc = ((3300 / v_out) - 1) * R_FIXED;  // Calculate NTC resistance
    float temp_k = BETA / (log(r_ntc / R_0) + (BETA / T_0));  // Compute temp in Kelvin
    return temp_k - 273.15;  // Convert to Celsius
}

void calculate_temperature(void)
{
    for(int i=0; i<6; i++)
    {
        if(temperature[i]>0)
        {
            TempaeratureDegrees[i]=calc_temp(temperature[i]);
        }
        else
        {
            TempaeratureDegrees[i]=0.0;
        }
        // printk("Temp %d: %d.%01d °C\n", i, 
        //     (int)TempaeratureDegrees[i], 
        //     (int)(TempaeratureDegrees[i] * 10) % 10);
    }
}

// Define unused GPIO pins (Modify based on your hardware)
static const uint32_t unused_pins[] = {9, 10, 22, 23, 24, 25};

void configure_unused_pins(void) {
    for (int i = 0; i < ARRAY_SIZE(unused_pins); i++) {
        nrf_gpio_cfg(
            unused_pins[i],
            NRF_GPIO_PIN_DIR_INPUT,          // Set as input
            NRF_GPIO_PIN_INPUT_DISCONNECT,   // Disable input buffer (reduces leakage)
            NRF_GPIO_PIN_PULLDOWN,           // Pulled low to prevent floating state
            NRF_GPIO_PIN_S0S1,               // Standard drive strength
            NRF_GPIO_PIN_NOSENSE             // No sense detection (prevents wakeups)
        );
    }
}

void disable_unused_peripherals(void) {
    // Ensure HFCLK is stopped unless needed
    // nrfx_clock_stop(NRF_CLOCK_DOMAIN_HFCLK);

    // Stop any unused peripherals
    // NRF_UART0->ENABLE = 0;
    NRF_TWIM0->ENABLE = 0;
    NRF_TWIM1->ENABLE = 0;
    NRF_SPI0->ENABLE = 0;
    NRF_SPI1->ENABLE = 0;
    NRF_SPI2->ENABLE = 0;
    NRF_PWM0->ENABLE = 0;
    NRF_PWM1->ENABLE = 0;
    NRF_PWM2->ENABLE = 0;
    // NRF_TIMER0->TASKS_STOP = 1;
    // NRF_TIMER1->TASKS_STOP = 1;
    // NRF_TIMER2->TASKS_STOP = 1;
    // NRF_SAADC->ENABLE = 0;
    
    printk("Unused pins & peripherals configured for low power.\n");
}

int main(void)
{
    // disable_unused_peripherals();
    configure_unused_pins();
    #ifdef BigProbe
    int ret;
    nrf_gpio_cfg_output(DirPIN7);
    nrf_gpio_pin_clear(DirPIN7);
    nrf_gpio_cfg_output(pwr_pins[0]);
    nrf_gpio_cfg_output(pwr_pins[1]);
    nrf_gpio_cfg_output(pwr_pins[2]);
    nrf_gpio_cfg_output(pwr_pins[3]);
    nrf_gpio_cfg_output(pwr_pins[4]);
    nrf_gpio_cfg_output(pwr_pins[5]);
    // nrf_gpio_cfg_output(PwrEnable1PIN17);//Just for testing the calibrate timer
    // nrf_gpio_pin_clear(PwrEnable1PIN17);//Just for testing the calibrate timer
    #else
    int err;
    nrf_gpio_cfg_output(spPwrEnable1PIN4);
    nrf_gpio_cfg_output(pwr_pins[0]);
    nrf_gpio_cfg_output(pwr_pins[5]);
    // nrf_gpio_cfg_output(PwrEnable1PIN17);//Just for testing the calibrate timer
    // nrf_gpio_pin_set(PwrEnable1PIN17);//Just for testing the calibrate timer
    #endif

	int16_t buf;
	struct adc_sequence sequence = {
		.buffer = &buf,
		.buffer_size = sizeof(buf),
	};

    #ifdef BigProbe
	if (!device_is_ready(uart))
    {
		// printk("UART device not ready\r\n");
		return 1 ;
	}

	ret = uart_callback_set(uart, uart_cb, NULL);
	if (ret) 
    {
		return 1;
	}

    ret = uart_rx_enable(uart ,rx_buf,sizeof rx_buf,RECEIVE_TIMEOUT);      
	if (ret) 
    {
		return 1;
	}

    #else
    if (!device_is_ready(sensor_dev)) {
        printk("Sensor device %s is not ready.\n", sensor_dev->name);
        return 0;
    }
    #endif
    // nrf_gpio_pin_set(DirPIN7); //JUST FOR TESTING
    hfclk_start();
    // k_msleep(200);

    // ble_init();  // Initialize BLE
    timer_1000ms_init(); // Initialize and start the 1000ms timer
    // start_timer_500ms_flag();
    // nrf_gpio_pin_set(DirPIN7); //JUST FOR TESTING 
    nvs_init();
	// printk("Starting NVS Flash Usage Check...\n");
    // print_nvs_info();
	// printk("NVS Flash Storage at: 0x%x, Size: %d KB\n", fs.offset, fs.sector_size * fs.sector_count / 1024);
    load_calibration_data();
	// Print saved values from flash
    // print_stored_calibration_data(); //JUST FOR TESTING
    // Example: Saving new calibration values
    // save_calibration_data(81, 195, 133, "GE000004"); //JUST FOR TESTING
    // nrf_gpio_pin_clear(DirPIN7);

    timer_init();
    nrfx_timer_enable(&my_counter);
	PPI_GPIOE_init();
    #ifdef BigProbe
    nrf_gpio_pin_set(pwr_pins[arrayIndex]);
    temperature_measure_required=true;
    one_sec_window_active=true;
    enable_one_sec_window_timer();
    PPI_GPIOE_assign_pin(count_pins[arrayIndex]);
    ADC_init_funct(arrayIndex,&sequence);
    #else
    nrf_gpio_pin_set(spPwrEnable1PIN4);
    nrf_gpio_pin_set(pwr_pins[arrayIndex]);
    nrf_gpio_pin_set(pwr_pins[5]);
    PPI_GPIOE_assign_pin(count_pins[0]);
    ADC_init_funct(5,&sequence);
    temperature_measure_required=true;
    one_sec_window_active=true;
    enable_one_sec_window_timer();
    #endif

	while (1) {
        if(start_one_sec_counter_timer&&!temperature_measure_required)
        {
            start_one_sec_counter_timer=false;
            nrfx_timer_clear(&my_counter);
            #ifdef BigProbe 
            if(loopCnt<=5)
            #else 
            if(loopCnt<=0) 
            #endif
            {               
                nrfx_timer_enable(&timer_1000ms);
            }
        }
        if(temperature_measure_required&&one_sec_window_active)
        {
            #ifdef BigProbe
            k_msleep(5);
            for(int i=0;i<6;i++)
            {
                ADC_Get_reading(arrayIndex,&sequence);
                k_msleep(20);
            }
            temperature[arrayIndex]=calculate_filtered_average();
            #else
            // hfclk_start(); //Might not need this here
            for(int i=0;i<6;i++)
            {
                ADC_Get_reading(5,&sequence);
                print_die_temperature();
                k_msleep(20);
            }
            temperature[arrayIndex]=calculate_filtered_average();
            internalTEMP=die_temperature_scaled/6;
            die_temperature_scaled=0;
            printk("Internal Temperature average: %d°C\n",internalTEMP);
            temp_high = (internalTEMP >> 8) & 0xFF;  // Upper 2 bytes
            temp_low  = internalTEMP & 0xFF;         // Lower 2 bytes
            printk("Stored as HEX: temp_high=0x%02X, temp_low=0x%02X\n", temp_high, temp_low);
            voltage22 = (uint16_t)(temperature[arrayIndex]/20);   
            printk("Voltage /20: %d\n",voltage22);
            #endif
            temperature_measure_required=false;
        }
        if(timesup&&!readbatt)
        {   
            nrfx_timer_disable(&timer_1000ms);
            timesup=false;
            uint32_t counter_val = nrfx_timer_capture(&my_counter, NRF_TIMER_CC_CHANNEL0);
            AvgCount+=counter_val;
            loopCnt++;
            Counts[arrayIndex]=AvgCount;
            AvgCount=0;
            AvgTemperature=0;
            TIMERhandlercounter=0;
            #ifdef BigProbe
            #else
            // internalTEMP=die_temperature_scaled;
            // die_temperature_scaled=0;
            // printk("Internal Temperature average: %d°C\n",internalTEMP);
            // temp_high = (internalTEMP >> 8) & 0xFF;  // Upper 2 bytes
            // temp_low  = internalTEMP & 0xFF;         // Lower 2 bytes
            // printk("Stored as HEX: temp_high=0x%02X, temp_low=0x%02X\n", temp_high, temp_low);
            // voltage22 = (uint16_t)(temperature[arrayIndex]/20);   
            // printk("Voltage /20: %d\n",voltage22);
            #endif
            // printk("Counter %d average: %d\n",arrayIndex+1, Counts[arrayIndex]);
            // printk("Voltage %d average: %d\n",arrayIndex+1, temperature[arrayIndex]);
            #ifdef BigProbe
            if(arrayIndex<5)
            {
                arrayIndex++;
                PPI_GPIOE_assign_pin(count_pins[arrayIndex]);
                ADC_init_funct(arrayIndex,&sequence);
            }
            if(loopCnt<=5)
            {
                if (loopCnt <= 5) 
                {
                    // printk("Nested Loop counter: %d\n", loopCnt);
                    if (arrayIndex >= 1 && arrayIndex <= 5) 
                    {
                        // printk("Switching PwrPin: %d off\n", arrayIndex - 1);
                        nrf_gpio_pin_clear(pwr_pins[arrayIndex - 1]);         ////Uncomment
                        // printk("Switching PwrPin: %d on\n", arrayIndex);
                        nrf_gpio_pin_set(pwr_pins[arrayIndex]);               ////Uncomment
                        enable_one_sec_window_timer();
                        // nrf_gpio_pin_set(PwrEnable1PIN17);//REMOVE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                        temperature_measure_required=true;
                        one_sec_window_active=true;
                    }
                }
            }
            #endif
            if(loopCnt>5)
            {
                #ifdef BigProbe
                // enableBoot=true;
                // printk("Switching PwrPin: %d off\n",arrayIndex);
                // printk("Done with readings");
                nrf_gpio_pin_clear(PwrEnable6PIN19);
                for(int i=0; i<6;i++)
                {
                // nrf_gpio_pin_set(DirPIN7);
                // printk("Array %d Counter average: %d\n",i, Counts[i]);
                // printk("Array %d Voltage average: %d\n",i, temperature[i]);
                // nrf_gpio_pin_clear(DirPIN7);
                }
                //Take battery voltage reading
                readbatt=true;
                // enable_readtimer();
                // nrfx_timer_enable(&timer_1000ms);//experimental
                ADC_init_funct(6,&sequence);
                //Take battery voltage reading
                #else
                // printk("Switching PwrPin: %d off\n",arrayIndex);
                #endif
            }
            #ifdef BigProbe
            #else
            if(loopCnt==1)
            {
                if(startup)
                {
                    startup=false;
                    enableBoot=true; //Switching bootloader on for 40s
                    enable_boottimer();
                }
                else if(WetCalibration||DryCalibration)
                {
                    calibrateSaveReadingNVS=true;
                }
                else
                {
                    turnoffboot=true;
                }

            }
            #endif
            // nrf_gpio_pin_clear(DirPIN7);
        }
        if(enableBoot)
        {
            bootloaderON();
            enableBoot=false;
        }
        #ifdef BigProbe
        if (new_data_available&&waitforRS) 
        { 
            // new_data_available = false;
            checkForTargetSequence();
        }
        if(readbatt)
        {
            // nrf_gpio_pin_set(DirPIN7);
            // printk("readingvoltage\n ");
            ADC_Get_reading(6,&sequence);
            for(int i=0;i<10;i++){
                // printk("AM inside the for loop\n");
                ADC_Get_reading(arrayIndex,&sequence);
                k_msleep(20);
            }
            readbatt=false;
            // loopCnt++;
            temperature[6]=AvgTemperature/10;
            AvgTemperature=0;
            // printk("Battery Voltage %d average: %d\n",7, temperature[6]);
            waitforRS =true;
            if(WetCalibration||DryCalibration)
            {
                calibrateSaveReadingNVS=true;
            }
            // nrf_gpio_pin_set(DirPIN7);
            // calculate_moisture();
            calculate_temperature();
            // nrf_gpio_pin_clear(DirPIN7);
            // take_readings(&sequence); //////////////////////////////////////////Remove, just for testing//////////////////////////////////////
        }
        if(turnoffboot)
        {
            // nrf_gpio_pin_set(DirPIN7);//just for testing
            // printk("Went into this loop functions");
            turnoffboot=false;
            if(BLEon)
            {
                bt_disable();
                BLEon=false;
            }
            // nrf_gpio_pin_clear(DirPIN7);
            enter_low_power_mode();// Enter low-power mode
        }
        #else
        if(turnoffboot)
        {
            turnoffboot=false;
            if(BLEon)
            {
                bt_disable();
                BLEon=false;
            }
            k_msleep(100);
            printk("Starting Advertising\n");
            calculate_moisture();
            mfg_data[2] = (uint16_t)(Counts[0]/200);
            // mfg_data[2] =(uint16_t)(MoisturePcnt[0]);
            mfg_data[3] = (uint16_t)(voltage22);
            mfg_data[4] = temp_high;
            mfg_data[5] = temp_low;
            // mfg_data[6] = wet_cal;
            err = bt_enable(NULL);
            BLEon=true;
            if (err) 
            {
                printk("Bluetooth init failed (err %d)\n", err);
                return 0;
            }
            printk("Bluetooth initialized\n");
            printk("Sending advertising data: 0x%02X\n", mfg_data[2]);
		    err = bt_le_adv_start(BT_LE_ADV_NCONN, adman, ARRAY_SIZE(ad),
				      NULL, 0);
		    if (err) 
            {
			    printk("Advertising failed to start (err %d)\n", err);
			    return 0;
		    }
            enable_advertisetimer();
        }
        if(advertiseover)
        {
            advertiseover=false;
            if(BLEon)
            {
                bt_disable();
                BLEon=false;
            }
            disable_unused_peripherals();
            enter_low_power_mode();// Enter low-power mode
            wakeup_flag=false;
            take_readings(&sequence);
        }
        
        #endif
        if(calibrateGetReadings)
        {
            calibrateGetReadings=false;
            take_readings(&sequence);
        }
        if(calibrateSaveReadingNVS)
        {
            calibrateSaveReadingNVS=false;
            if (WetCalibration)
            {
                nrf_gpio_pin_set(DirPIN7); // JUST FOR TESTING
                WetCalibration = false;
                save_wet_calibration_data();
                print_stored_calibration_data(); // JUST FOR TESTING
                nrf_gpio_pin_clear(DirPIN7); // JUST FOR TESTING
            }
            if (DryCalibration)
            {
                nrf_gpio_pin_set(DirPIN7); // JUST FOR TESTING
                DryCalibration = false;
                save_dry_calibration_data();
                print_stored_calibration_data(); // JUST FOR TESTING
                nrf_gpio_pin_clear(DirPIN7); // JUST FOR TESTING
            }
        }
        if(BLEon)
        {
            k_msleep(100);
        }
	}
}