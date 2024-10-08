/* Scanner, beacon selector and re-player 09-2024 vj120086d@student.etf.bg.ac.rs */

// Zephyr/core libraries
#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
// Bluetooth libraries
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/sys/byteorder.h>
// Buttons, LED, and UART/console communitaction
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/uart.h>


// System states (user actions in progress)
#define STATE_INIT       0
#define STATE_SCAN       1
#define STATE_THINKING   3
#define STATE_BROADCAST  4

// Scan limmit (prototype)
#define MAX_SCAN 10

// Input data for console talk
#define RECEIVE_BUFF_SIZE 10
#define RECEIVE_TIMEOUT 100

// Scanned device structure
struct scanned_device{
	bt_addr_le_t *addr;
	int8_t rssi;
	uint8_t type;
	struct net_buf_simple *ad;
} scanned_dev[MAX_SCAN];

// Thread statuses and variables
#define STACKSIZE 1024
#define THREAD0_PRIORITY 7
#define THREAD1_PRIORITY 7


// System state variales
static unsigned int count_scans = 0;
static unsigned int system_state = STATE_INIT;
static struct scanned_device selected_beacon;

/* Peripherial initializations */

/* Button initializations */
#define SW0_NODE	DT_ALIAS(sw0)
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
#define SW1_NODE	DT_ALIAS(sw1)
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
#define SW2_NODE	DT_ALIAS(sw2)
static const struct gpio_dt_spec button2 = GPIO_DT_SPEC_GET(SW2_NODE, gpios);

/* LED initialization */
#define LED0_NODE DT_ALIAS(led0)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

/* UART initialization */
const struct device *uart= DEVICE_DT_GET(DT_NODELABEL(uart0));

/* Peripherial initializations end */

/* BLE scanner */
static void start_scan(void);

static void device_found(const bt_addr_le_t *addr, int8_t rssi, uint8_t type,
			 struct net_buf_simple *ad)
{
	char addr_str[BT_ADDR_LE_STR_LEN];
	int err;

	/* We're only interested in connectable events -
		any real mobile/smart phone is connectible.
	*/
	if (type != BT_GAP_ADV_TYPE_ADV_IND &&
	    type != BT_GAP_ADV_TYPE_ADV_DIRECT_IND) {
		return;
	}
    
	bt_addr_le_to_str(addr, addr_str, sizeof(addr_str));
	printk("COUNT: %d, Device found: %s (RSSI %d)\n", count_scans, addr_str, rssi);

    // Save this data to the list
	memcpy(scanned_dev[count_scans].addr, addr, sizeof(bt_addr_le_t));
	scanned_dev[count_scans].rssi = rssi;
	scanned_dev[count_scans].type = type;
	memcpy(scanned_dev[count_scans].ad, ad, sizeof(struct net_buf_simple));

	count_scans++;

    if (count_scans >= MAX_SCAN)
	{
		bt_le_scan_stop();
		printk("Stop scan triggered.\n");
		return;
	}
}

static void start_scan(void)
{
	int err;
	count_scans = 0;

	err = bt_le_scan_start(BT_LE_SCAN_PASSIVE, device_found);
	if (err) {
		printk("Scanning failed to start (err %d)\n", err);
		return;
	}

	printk("Scanning successfully started\n");
}
/* BLE scanner end*/


static void start_broadcast(void)
{
	int err;

	err = bt_le_adv_start(BT_LE_ADV_NCONN, selected_beacon.ad, sizeof(struct net_buf_simple), NULL, 0);
	if (err == ECONNREFUSED || err == ENOMEM ) {
		printk("Advertasing failed to start (err %d)\n", err);
		return;
	}

	printk("Broadcasting successfully started\n");
}

/* Thread - scanner */
void thread0(void)
{
	while (1) {
		k_busy_wait(1000000);
		if (system_state == STATE_SCAN){
          printk("Hello, I am scanner thread.\n");
		  start_scan();
		  system_state = STATE_THINKING;
		}
		k_yield();
	}
}

/* Thread - broadcaster */
void thread1(void)
{
	while (1) {
		k_busy_wait(5000);
		if (system_state == STATE_BROADCAST){
			start_broadcast();
		}
	}
}

/* Threads end*/

/* Button acction(s) */
void button_pressed(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Hello, I am button interrupt.\n");

	// Test, give me a sign on board
    gpio_pin_toggle_dt(&led);

	if (system_state == STATE_THINKING || system_state == STATE_INIT){
		system_state = STATE_SCAN;
		printk("			Start scan.\n");
	}
	else {
	    printk("Not thinking time, no scan.\n");
	}
}

void button_pressed_choose(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	printk("Hello, List of available beacons to re-play:.\n");

	// Test, give me a sign on board
    gpio_pin_toggle_dt(&led);

	// re-parse address to readable form
	char addr_str[BT_ADDR_LE_STR_LEN];

	for (int i = 0; i < MAX_SCAN; i++){
	    bt_addr_le_to_str(scanned_dev[i].addr, addr_str, sizeof(addr_str));
		printk("!!COUNT: %d, Device from list: %s (RSSI %d)\n",i, addr_str, scanned_dev[i].rssi);
	}
	
	bt_addr_le_to_str(selected_beacon.addr, addr_str, sizeof(addr_str));
	printk("\nCurrent re-playable: %s (RSSI %d)\n\n", addr_str, selected_beacon.rssi);


	printk("\nEnter a beacon number to re-play:\n\n");
}
void button_broadcast(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
	// Test, give me a sign on board
    gpio_pin_toggle_dt(&led);

	system_state = STATE_BROADCAST;

}

// UART callback

static uint8_t rx_buf[RECEIVE_BUFF_SIZE] = {0};
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
	switch (evt->type) {

	case UART_RX_RDY:

	if((evt->data.rx.len) == 1){
		
		if(evt->data.rx.buf[evt->data.rx.offset] == '0')
			memcpy(&selected_beacon, &scanned_dev[0], sizeof(struct scanned_device));
		if(evt->data.rx.buf[evt->data.rx.offset] == '1')
			memcpy(&selected_beacon, &scanned_dev[1], sizeof(struct scanned_device));
		else if (evt->data.rx.buf[evt->data.rx.offset] == '2')
			memcpy(&selected_beacon, &scanned_dev[2], sizeof(struct scanned_device));
		else if (evt->data.rx.buf[evt->data.rx.offset] == '3')	
			memcpy(&selected_beacon, &scanned_dev[3], sizeof(struct scanned_device));
		else if (evt->data.rx.buf[evt->data.rx.offset] == '4')	
			memcpy(&selected_beacon, &scanned_dev[4], sizeof(struct scanned_device));
		else if (evt->data.rx.buf[evt->data.rx.offset] == '5')	
			memcpy(&selected_beacon, &scanned_dev[5], sizeof(struct scanned_device));
		else if (evt->data.rx.buf[evt->data.rx.offset] == '6')	
			memcpy(&selected_beacon, &scanned_dev[6], sizeof(struct scanned_device));
		else if (evt->data.rx.buf[evt->data.rx.offset] == '7')	
			memcpy(&selected_beacon, &scanned_dev[7], sizeof(struct scanned_device));
		else if (evt->data.rx.buf[evt->data.rx.offset] == '8')	
			memcpy(&selected_beacon, &scanned_dev[8], sizeof(struct scanned_device));
		else if (evt->data.rx.buf[evt->data.rx.offset] == '9')	
			memcpy(&selected_beacon, &scanned_dev[9], sizeof(struct scanned_device));					
		
		}

	break;
	case UART_RX_DISABLED:
		uart_rx_enable(dev ,rx_buf,sizeof rx_buf,RECEIVE_TIMEOUT);
		break;
		
	default:
		break;
	}

	char addr_str[BT_ADDR_LE_STR_LEN];
	bt_addr_le_to_str(selected_beacon.addr, addr_str, sizeof(addr_str));
	printk("Current re-playable: %s (RSSI %d)\n", addr_str, selected_beacon.rssi);
}


static struct gpio_callback button_cb_data;
static struct gpio_callback button_cb_data1;
static struct gpio_callback button_cb_data2;

/* Button end */


int main(void)
{
	system_state = STATE_INIT;
	int err;

	for (int i = 0; i < MAX_SCAN; i++){
		scanned_dev[i].addr = malloc(sizeof(bt_addr_le_t));
//        scanned_dev[i].addr = 0;
		scanned_dev[i].rssi = 0;
		scanned_dev[i].type = 0;
		scanned_dev[i].ad = malloc(sizeof(struct net_buf_simple));
	}

	/* Peripherial initializations */
    /* Configure Button 0 - scanner control */
	if (!device_is_ready(button0.port)) { return -1; }
	if (gpio_pin_configure_dt(&button0, GPIO_INPUT) < 0) { return -1; }

	/* Configure Button 1 - user beacon re-play selection control */
	if (!device_is_ready(button1.port)) { return -1; }
	if (gpio_pin_configure_dt(&button1, GPIO_INPUT) < 0) { return -1; }
	
	/* Configure Button 2 - re-play beacon */
	if (!device_is_ready(button2.port)) { return -1; }
	if (gpio_pin_configure_dt(&button2, GPIO_INPUT) < 0) { return -1; }

    /* Configure LED - test control */
	if (!device_is_ready(led.port)) { return -1; }
	if (gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE) < 0) { return -1; }

	/* UART setup */
	if (!device_is_ready(uart)) { return -1 ;}
	if (uart_callback_set(uart, uart_cb, NULL)) { return -1; }
	if (uart_rx_enable(uart ,rx_buf,sizeof rx_buf,RECEIVE_TIMEOUT)) { return 1; }

    /* Bluetooth init */
	err = bt_enable(NULL);
	if (err) {
		printk("Bluetooth init failed (err %d)\n", err);
		return 0;
	}
	printk("Bluetooth initialized\n");	


	/* Button0 interrupt configurations */
	err = gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_TO_ACTIVE );
    gpio_init_callback(&button_cb_data, button_pressed, BIT(button0.pin)); 	
	gpio_add_callback(button0.port, &button_cb_data);
	
	/* Button1 interrupt configurations */
	err = gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE );
    gpio_init_callback(&button_cb_data1, button_pressed_choose, BIT(button1.pin)); 	
	gpio_add_callback(button1.port, &button_cb_data1);
		
	/* Button2 interrupt configurations */
	err = gpio_pin_interrupt_configure_dt(&button2, GPIO_INT_EDGE_TO_ACTIVE );
    gpio_init_callback(&button_cb_data2, button_broadcast, BIT(button2.pin)); 	
	gpio_add_callback(button2.port, &button_cb_data2);
	
	/* All inits are done, start system */
	system_state == STATE_THINKING;
	

	while (1) {
        k_msleep(1000);
	}

	return 0;
}

K_THREAD_DEFINE(thread0_scanner, STACKSIZE, thread0, NULL, NULL, NULL,
		THREAD0_PRIORITY, 0, 0);
K_THREAD_DEFINE(thread1_button1, STACKSIZE, thread1, NULL, NULL, NULL,
		THREAD0_PRIORITY, 0, 0);
		 