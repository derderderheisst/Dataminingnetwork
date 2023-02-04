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

   LESSON 5: Flooding (broadcast)
*/

// Contiki-specific includes:
#include "contiki.h"
#include "net/rime/rime.h"     // Establish connections.
#include "net/netstack.h"      // Wireless-stack definitions
#include "dev/leds.h"          // Use LEDs.
#include "core/net/linkaddr.h"
// Standard C includes:
#include <stdio.h>      // For printf.



//------------------------ FUNCTIONS ------------------------
// Defines the behavior of a connection upon receiving data.
// Creates an instance of a broadcast connection.
// Creates an instance of a broadcast connection.

static struct broadcast_conn broadcast;
// Defines the functions used as callbacks for a broadcast connection.
char *buff_data;


static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from) {
	leds_on(LEDS_GREEN);
	 printf("Broadcast message received from 0x%x%x: '%s' [RSSI %d]\n",
			 from->u8[0], from->u8[1],
			(char *)packetbuf_dataptr(),
			(int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI));

	 packetbuf_copyto(&buff_data);
	 forward_msg(&buff_data);
	 leds_off(LEDS_GREEN);
}

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};

void forward_msg(const char * message) {
//open the connection, if necessary
	//broadcast_open(&broadcast, 129, &broadcast_call);
	packetbuf_copyfrom(message,strlen(message));
	broadcast_send(&broadcast);
	printf(" - Strlen of retransmit: %d\n",strlen(message));
}

/**
* @param message - message to be broadcasted
*/


static void check_for_invalid_addr(void) {
	// Boolean flag to check invalid address.
	static int iAmError = 0;

	// All-zeroes address.
	static linkaddr_t errAddr;
	errAddr.u8[0] = 0;
	errAddr.u8[1] = 0;

	// Check if this mote got an invalid address.
	iAmError = linkaddr_cmp(&errAddr, &linkaddr_node_addr);

	// Turn ON all LEDs if we loaded an invalid address.
	if(iAmError){
		printf("\nLoaded an invalid RIME address (0x%x%x)! "
				"Reset the device.\n\n",
				linkaddr_node_addr.u8[0],
				linkaddr_node_addr.u8[1]);

		// Freezes the app here. Reset needed.
		while (1){
			leds_on(LEDS_RED);
		}
	}
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

//--------------------- PROCESS CONTROL BLOCK ---------------------
PROCESS(flooding_process, "Lesson 5: Flooding");
AUTOSTART_PROCESSES(&flooding_process);

//------------------------ PROCESS' THREAD ------------------------
PROCESS_THREAD(flooding_process, ev, data) {

	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
	PROCESS_BEGIN();

	static uint8_t timer_interval = 3;	// In seconds

	static struct etimer et;

	// Configure your team's channel (11 - 26).
	NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_CHANNEL,15);

	print_settings();
	check_for_invalid_addr();

	// Open broadcast connection.
	broadcast_open(&broadcast, 129, &broadcast_call);

	while(1) {
		etimer_set(&et, CLOCK_SECOND * timer_interval);

		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		leds_on(LEDS_RED);
		packetbuf_copyfrom("abcde", strlen("abcde"));
		broadcast_send(&broadcast);
		printf("Broadcast message sent. Strlen = %d\n", strlen("abcde") );
		leds_off(LEDS_RED);
	}
	PROCESS_END();
}
