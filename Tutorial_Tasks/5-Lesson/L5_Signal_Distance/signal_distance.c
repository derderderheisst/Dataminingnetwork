/*
   Wireless Sensor Networks Laboratory

   Technische Universität München
   Lehrstuhl für Kommunikationsnetze
   http://www.lkn.ei.tum.de

   copyright (c) 2017 Chair of Communication Networks, TUM

   contributors:
   * Thomas Szyrkowiec
   * Mikhail Vilgelm
   * Octavio Rodríguez Cervantes
   * Angel Corona

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, version 2.0 of the License.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   LESSON 4: Signal Distance
*/

// Contiki-specific includes:
#include "contiki.h"
#include "net/rime/rime.h"     // Establish connections.
#include "net/netstack.h"      // Wireless-stack definitions
#include "dev/leds.h"          // Use LEDs.
#include "dev/button-sensor.h" // User Button
#include "cpu/cc2538/dev/uart.h"
// Standard C includes:
#include <stdio.h>      // For printf.

//#include "cpu/cc2538/usb/usb-serial.h"	// For UART-like I/O over USB.
#include "dev/serial-line.h"			// For UART-like I/O over USB.


// Definition of UART special characters. 
// 	Note:	Contiki seems to have its own definitions.
// 			Incompatibility may cause unexpected behavior.
// 	For more details, check usb-serial.h and serial-line.h.
#define START_CHAR                  1
#define DEACTIVATION_CHAR           0x0d //25xxx4
#define END_CHAR                    0x0a //255

// Packet type flags used in wireless transmission.
#define RADIO_PACKET_TYPE_CONFIG_POWER	0
#define RADIO_PACKET_TYPE_INFO_POWER	1

// Packet type flags used in UART/serial transmission.
#define SERIAL_PACKET_TYPE_CONFIG_POWER	0
#define SERIAL_PACKET_TYPE_INFO_POWER	1

// Tx buffer size in bytes.
#define MAX_RF_PAYLOAD_SIZE 	3
#define MAX_USB_PAYLOAD_SIZE	4

// Each packet sent is numbered.
// Numbers will cycle from 1 to 255.
#define MAX_TX_PACKET_NO 255

//--------------------- VARIABLES ---------------------
static unsigned int txPower = 0xD5;

//---------------- FUNCTION PROTOTYPES ----------------

// Broadcast connection setup:
static void
broadcast_rx_radio(struct broadcast_conn *c, const linkaddr_t *from);
static struct broadcast_conn broadcastConn;
static const struct broadcast_callbacks broadcast_callbacks = {broadcast_rx_radio};

// Prints the current settings.
static void print_settings(void);

//--------------------- PROCESS CONTROL BLOCK ---------------------
PROCESS(txRadio_process, "Sends Tx power value over wireless antenna.");
PROCESS(rxUSB_process, "Receives data from UART/serial (USB).");
AUTOSTART_PROCESSES(&txRadio_process,&rxUSB_process);

//------------------------ PROCESS' THREAD ------------------------

// Configures the wireless antenna and unicast connection.
PROCESS_THREAD(txRadio_process, ev, data) {
	PROCESS_EXITHANDLER(broadcast_close(&broadcastConn);)

    PROCESS_BEGIN();

	static int
		packetNumber = 0,	// Tx packet counter.
		timerInterval = 5;	// In seconds.
	static struct etimer et;

	// Configure your team's channel.
	NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_CHANNEL,15);

	print_settings();

	// Open a broadcast connection.
	broadcast_open(&broadcastConn, 129, &broadcast_callbacks);

	// We start listening for data over USB.
	//process_start(&rxUSB_process, NULL);

	while(1) {
		etimer_set(&et, CLOCK_SECOND * timerInterval);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		leds_on(LEDS_RED);

		// Increase the RR packet number.
		packetNumber = (packetNumber % MAX_TX_PACKET_NO)+1;

		// Array of bytes we will send over to the destination.
		uint8_t txBuffer[MAX_RF_PAYLOAD_SIZE];

		// Fill the Tx buffer with info and send it.
		txBuffer[0] = RADIO_PACKET_TYPE_INFO_POWER;
		txBuffer[1] = packetNumber;
		txBuffer[2] = txPower;

		// Broadcast power information.
		packetbuf_copyfrom(&txBuffer, MAX_RF_PAYLOAD_SIZE);
		broadcast_send(&broadcastConn);

		leds_off(LEDS_RED);
	}
	PROCESS_END();
}

// Listens for data coming from the USB connection (UART0)
// and prints it.
PROCESS_THREAD(rxUSB_process, ev, data) {
	PROCESS_BEGIN();

	uint8_t *uartRxBuffer;


	/* In this example, whenever data is received from UART0, the
	 * default handler (i.e. serial_line_input_byte) is called.
	 * But how does it work?
	 *
	 * The handler runs in a process that puts all incoming data
	 * in a buffer until it gets a newline or END character
	 * ('\n' and 0x0A are equivalent). Also good to know is that
	 * there is an IGNORE character: 0x0D.
	 *
	 * Once a newline or END is detected, a termination char ('\0')
	 * is appended to the buffer and the process broadcasts a
	 * "serial_line_event_message" along with the buffer contents.
	 *
	 * For more details, check contiki/core/dev/serial-line.c
	 * For an "example", check contiki/apps/serial-shell/serial-shell.c
	 */
	while(1){
		PROCESS_YIELD();

		if (ev == serial_line_event_message)
		{

			leds_toggle(LEDS_BLUE);

			uartRxBuffer = (uint8_t*)data;

			// If the received packet is a packet to set the radio power
			if(uartRxBuffer[0] == SERIAL_PACKET_TYPE_CONFIG_POWER) {

				txPower = uartRxBuffer[1];
				// Array of bytes we will send over to the destination.
				uint8_t txBuffer[MAX_RF_PAYLOAD_SIZE];

				printf("[SRC SERIAL]: Set TxPwr (0x%x)\r\n", txPower);

				// Fill the Tx buffer with info and send it.
				txBuffer[0] = RADIO_PACKET_TYPE_CONFIG_POWER;
				txBuffer[1] = uartRxBuffer[1];

				printf("[FWD to RADIO]\r\n");

				// Change the transmission power. Must be a valid value.
				NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_TXPOWER,txPower);

				packetbuf_copyfrom(&txBuffer, MAX_RF_PAYLOAD_SIZE);
				broadcast_send(&broadcastConn);
			}
		//leds_off(LEDS_BLUE);
		}
	}
    PROCESS_END();
}

//---------------- FUNCTION DEFINITIONS ----------------

// Defines the behavior of a unicast connection upon receiving data.
static void
broadcast_rx_radio(struct broadcast_conn *c, const linkaddr_t *from) {
	leds_on(LEDS_GREEN);

	// Buffers
    static uint8_t
    	radioRxBuffer[MAX_RF_PAYLOAD_SIZE],
    	uartTxBuffer[MAX_USB_PAYLOAD_SIZE];
	static int
		radioRxBytes = 0,	// Amount of received bytes.
		rssi = 0,			// RSSI of the received packet.
		i;					// FOR loop index.

    rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);

    printf("[SRC 0x%x%x, BCAST, RSSI %d]: ",
			from->u8[0], from->u8[1], rssi);

	// Get contents and number of bytes of the packet buffer.
    radioRxBytes = packetbuf_copyto(&radioRxBuffer);

	/* IMPORTANT: The use switch statements in contiki apps is
	 * discouraged because they *may* disrupt with the structure
	 * of the process and cause unexpected behavior.
	 *
	 * For more details read about "protothread states",
	 * "Local Continuations" and "Duff's Device":
	 *
	 * https://github.com/contiki-os/contiki/wiki/Processes
	 * http://senstools.gforge.inria.fr/doku.php?id=os:contiki#processes
	 * http://www.chiark.greenend.org.uk/~sgtatham/coroutines.html
	 */

	// Detects the type of the received packet
	if(radioRxBuffer[0] == RADIO_PACKET_TYPE_CONFIG_POWER){
        // ... Set the radio power.
        txPower = radioRxBuffer[1];
        printf("Set TxPwr (0x%x)\r\n", txPower);

        // Change the transmission power. Must be a valid value.
    	NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_TXPOWER,txPower);
	}
	else if(radioRxBuffer[0] == RADIO_PACKET_TYPE_INFO_POWER){
        printf("Info: [%d] TxPwr %d.",
        		radioRxBuffer[1],radioRxBuffer[2]);

        uartTxBuffer[0] = RADIO_PACKET_TYPE_INFO_POWER;
        uartTxBuffer[1] = radioRxBuffer[1];
        uartTxBuffer[2] = radioRxBuffer[2];
        uartTxBuffer[3] = rssi;

        printf("[FWD to USB ");
        uart_write_byte(0,START_CHAR);
        for (i = 0; i < 5; i++){
        	uart_write_byte(0,uartTxBuffer[i]);
        }
        uart_write_byte(0,END_CHAR);
        printf("]\r\n");

    } else {
    	printf("RADIO Rx ERR: Unknown packet type\r\n");
    	return;
    }
	leds_off(LEDS_GREEN);
}


// Prints the current settings.
static void print_settings(void){
	radio_value_t channel;

	NETSTACK_CONF_RADIO.get_value(RADIO_PARAM_CHANNEL,&channel);

	printf("\n-------------------------------------\n");
	printf("RIME addr = \t0x%x%x\n",
			linkaddr_node_addr.u8[0],
			linkaddr_node_addr.u8[1]);
	printf("Using radio channel %d\n", channel);
	printf("---------------------------------------\n");
}
