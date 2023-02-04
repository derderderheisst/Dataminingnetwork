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

   LESSON 3: Gateway
*/

// Contiki-specific includes:
#include "contiki.h"
#include "dev/leds.h"			// Use LEDs.
#include "sys/clock.h"			// Use CLOCK_SECOND.
#include "net/rime/rime.h"		// Establish connections.
#include "dev/cc2538-rf.h"

// Standard C includes:
#include <stdio.h>			// For printf.

//---------------- FUNCTION PROTOTYPES ----------------

// Broadcast connection setup:
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from);
static struct broadcast_conn broadcastConn;
static const struct broadcast_callbacks broadcast_callbacks = {broadcast_recv};

// Prints the contents of the Rx buffer.
static void print_buffer_contents(void);


// Prints the current settings.
static void print_settings(void);

//--------------------- PROCESS CONTROL BLOCK ---------------------
PROCESS(gateway_main_process, "Lesson 3: Gateway");
AUTOSTART_PROCESSES(&gateway_main_process);

//------------------------ PROCESS' THREADS ------------------------
PROCESS_THREAD(gateway_main_process, ev, data) {

	PROCESS_EXITHANDLER( broadcast_close(&broadcastConn); )
	PROCESS_BEGIN();

	/*
	* set your group's channel
	*/
	NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_CHANNEL, 15);

	print_settings();

	broadcast_open(&broadcastConn, 129, &broadcast_callbacks);

	// If all is OK, we can start the other two processes:

	while(1) {
		// Contiki processes cannot start loops that never end.
		PROCESS_WAIT_EVENT();
	}
	PROCESS_END();
}

//---------------- FUNCTION DEFINITIONS ----------------

// Defines the behavior of a broadcast connection upon receiving data.
static void
broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from) {
	leds_on(LEDS_GREEN);
	printf("[Source 0x%x%x, broadcast, RSSI %d]:\t",
			from->u8[0], from->u8[1],
			(int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI));
	print_buffer_contents();
	leds_off(LEDS_GREEN);
}

/**
 * Print the received message
 */
static void print_buffer_contents(void){
	char rxBuffer[PACKETBUF_SIZE];
	int rxBytes = 0;

	/*
	 * get the buffered message and message size
	 */
	rxBytes = packetbuf_copyto(&rxBuffer);

	if (rxBytes>0) {
		printf("%s\r\n\n",rxBuffer);
	}

}

// Prints the current settings.
static void print_settings(void) {

	radio_value_t channel = 0;

	NETSTACK_CONF_RADIO.get_value(RADIO_PARAM_CHANNEL,&channel);

	printf("\n------- GATEWAY NODE -------\n");
	printf("RIME address  = \t0x%x%x\n",
			linkaddr_node_addr.u8[0],
			linkaddr_node_addr.u8[1]);
	printf("Radio channel = %d\n",channel );
	printf("-----------------------------\n");
}
