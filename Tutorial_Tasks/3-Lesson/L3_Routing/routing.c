/*
   Wireless Sensor Networks Laboratory

   Technische Universität München
   Lehrstuhl für Kommunikationsnetze
   http://www.lkn.ei.tum.de

   copyright (c) 2018 Chair of Communication Networks, TUM

   contributors:
   * Thomas Szyrkowiec
   * Mikhail Vilgelm
   * Octavio Rodríguez Cervantes
   * Angel Corona
   * Donjeta Elshani
   * Onur Ayan
   * Benedikt Hess

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, version 2.0 of the License.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   LESSON 5: Routing
*/

/*----------------------------INCLUDES----------------------------------------*/
// standard C includes:
#include <stdio.h>
#include <stdint.h>

// Contiki-specific includes:
#include "contiki.h"
#include "net/rime/rime.h"     // Establish connections.
#include "net/netstack.h"      // Wireless-stack definitions
#include "dev/button-sensor.h" // User Button
#include "dev/leds.h"          // Use LEDs.

// Header
#include "routing.h"
#include "buffer.h"
#include "helpers.h"

/*---------------------------Variables----------------------------------------*/
static uint8_t node_id;		        // Stores node id

static uint8_t led_color;	        // Each id has unique led color. When this
                                    // node generates a packet, it has this led
                                    //color.

static Buffer buffer;               // the buffer to store packets until they
                                    // are transmitted

static struct unicast_conn unicast; // Creates an instance of a unicast
                                    //connection.


static l_table lut[TOTAL_NODES] = {

	/*
	 * First node lookup table
	 * */
	{
		// First entry
		.dest[0].u8[1] = 0x01, .next_hop[0].u8[1] = 0x01, .cost[0] = 0,
		// Second entry
		.dest[1].u8[1] = 0x02, .next_hop[1].u8[1] = 0x02, .cost[1] = 1,
		// Third entry
		.dest[2].u8[1] = 0x03, .next_hop[2].u8[1] = 0x02, .cost[2] = 2,
		/*
		 * Add here more entries if using more than three nodes.
		 */
	},

	/*
	 * Second node lookup table
	 * */
	{
		.dest[0].u8[1] = 0x01, .next_hop[0].u8[1] = 0x03, .cost[0] = 2,
		.dest[1].u8[1] = 0x02, .next_hop[1].u8[1] = 0x02, .cost[1] = 0,
		.dest[2].u8[1] = 0x03, .next_hop[2].u8[1] = 0x03, .cost[2] = 1,
	},

	/*
	 * Third node lookup table
	 * */
	{
		.dest[0].u8[1] = 0x01, .next_hop[0].u8[1] = 0x01, .cost[0] = 1,
		.dest[1].u8[1] = 0x02, .next_hop[1].u8[1] = 0x01, .cost[1] = 2,
		.dest[2].u8[1] = 0x03, .next_hop[2].u8[1] = 0x03, .cost[2] = 0,
	}
	,
	/*
	 * Add here more lookup table entries if using more than three nodes.
	 */
};

//--------------------- PROCESS CONTROL BLOCK ---------------------
PROCESS(routing_process, "Lesson 3: Routing");
PROCESS(send_process, "Process to send packets");
PROCESS(destination_reaced_process, "Process indicate a packets has reached its"
		" destination");
AUTOSTART_PROCESSES(&routing_process, &send_process,
		&destination_reaced_process);

//------------------------ FUNCTIONS ------------------------

static void send_packet(packet_t tx_packet){
	uint8_t i;
	// Define next hop and forward packet
	for(i = 0; i < TOTAL_NODES; i++)
	{
		if(linkaddr_cmp(&tx_packet.dest, &lut[node_id - 1].dest[i]))
		{
			packetbuf_copyfrom(&tx_packet, sizeof(packet_t));
			unicast_send(&unicast, &lut[node_id - 1].next_hop[i]);
			break;
		}
	}
	turn_off(tx_packet.message);
}


// sets a timer and puts this timer and the packet in the buffer
static void enqueue_packet(packet_t tx_packet){
	uint8_t return_code;
	struct timer packet_timer;

	timer_set(&packet_timer, CLOCK_SECOND * TIMER_INTERVAL);

	// put packet and timer in the queue
	return_code = BufferIn(&buffer, tx_packet, packet_timer);

	// check if storing was successful
	if (return_code == BUFFER_FAIL) {
		printf("The buffer is full. Consider increasing the 'BUFFER_SIZE' value"
				" in 'buffer.h'\n");
	} else {
		// inform the send process a new packet was enqueued
		process_post(&send_process, PROCESS_EVENT_MSG, 0);
	}
}


// Defines the behavior of a connection upon receiving data.
static void
unicast_recv(struct unicast_conn *c, const linkaddr_t *from) {
	packet_t rx_packet;
	packet_t tx_packet;

	packetbuf_copyto(&rx_packet);

	// For Debug purposes
	// printf("Unicast message received from 0x%x%x: '%s' [RSSI %d]\n",
	//		 from->u8[0], from->u8[1],
	//		(char *)packetbuf_dataptr(),
	//		(int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI));

	// Check if packet reached destination
	if(linkaddr_cmp(&rx_packet.dest, &linkaddr_node_addr))
	{
		printf("Packet reached destination \n");
		process_post(&destination_reaced_process, PROCESS_EVENT_MSG,
				&rx_packet.message);

		//Rotation by defining a new destination
		int dest_id = calculate_destination(node_id, TOTAL_NODES);

		// Rotate in opposite direction for red messages
		if(rx_packet.message == LEDS_RED){
			dest_id = 6 - dest_id - node_id;
		}
		// Retransmit
		tx_packet.dest.u8[0] = (dest_id >> 8) & 0xFF;
		tx_packet.dest.u8[1] = dest_id & 0xFF;
		tx_packet.message = rx_packet.message;

		//return;
	}
	else
	{
		tx_packet.dest.u8[0] = rx_packet.dest.u8[0];
		tx_packet.dest.u8[1] = rx_packet.dest.u8[1];
		tx_packet.message = rx_packet.message;
	}

	turn_on(rx_packet.message);
	enqueue_packet(tx_packet);
}


// Defines the functions used as callbacks for a unicast connection.
static const struct unicast_callbacks unicast_call = {unicast_recv};


//------------------------ PROCESS' THREAD ------------------------
PROCESS_THREAD(routing_process, ev, data) {

	PROCESS_EXITHANDLER(unicast_close(&unicast);)
	PROCESS_BEGIN();

	packet_t tx_packet;

	// Configure your team's channel (11 - 26).
	NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_CHANNEL,15);

	/* Configure the user button */
	button_sensor.configure(BUTTON_SENSOR_CONFIG_TYPE_INTERVAL, CLOCK_SECOND);

	print_settings();
	check_for_invalid_addr();
	node_id = (linkaddr_node_addr.u8[1] & 0xFF);
	led_color = get_led_color(node_id);
	print_lookup_table(lut[node_id - 1], TOTAL_NODES);

	// Open unicast connection.
	unicast_open(&unicast, 129, &unicast_call);

	while(1) {
		PROCESS_WAIT_EVENT();

		// check if button was pressed
		if(ev == sensors_event)
		{
			if(data == &button_sensor)
			{
				if( button_sensor.value(BUTTON_SENSOR_VALUE_TYPE_LEVEL) ==
						BUTTON_SENSOR_PRESSED_LEVEL ) {

					// generate packet
					int dest_id = calculate_destination(node_id, TOTAL_NODES);
					tx_packet.dest.u8[0] = (dest_id >> 8) & 0xFF;
					tx_packet.dest.u8[1] = dest_id & 0xFF;
					tx_packet.message = led_color;

					turn_on(led_color);

					enqueue_packet(tx_packet);
				}
			}
		}
	}

	PROCESS_END();
}


PROCESS_THREAD(send_process, ev, data) {
	PROCESS_BEGIN();
	printf("send_process: started\n");

	static struct etimer t;
	static struct timer packet_timer;
	static packet_t tx_packet;
	static uint8_t return_code;
	static uint8_t is_processing_packet = 0;
	static clock_time_t remaining_time;

	while(1) {
		PROCESS_WAIT_EVENT();

		// a new packet has been added to the buffer
		if (ev == PROCESS_EVENT_MSG) {
			// is a packet already processed?
			if ( ! is_processing_packet ) {
				is_processing_packet = 1;
				// get a packet from the buffer
				return_code = BufferOut(&buffer, &tx_packet, &packet_timer);
				// there was nothing in the buffer
				if (return_code == BUFFER_FAIL){
					is_processing_packet = 0;
				}
				// there was something in the buffer
				else {
					remaining_time = timer_remaining(&packet_timer);
					// check if the timer has already expired
					if ( timer_expired(&packet_timer) ) {
						send_packet(tx_packet);
						is_processing_packet = 0;
						process_post(&send_process, PROCESS_EVENT_MSG, 0);
					}
					else {
						// wait for the remaining time
						etimer_set(&t, remaining_time);
					}
				}
			}
		} else if (ev == PROCESS_EVENT_TIMER) {
			// timer indicating time to send expired
			if (etimer_expired(&t)) {
				send_packet(tx_packet);
				is_processing_packet = 0;
				// tell the process to check if there is another packet in the
				// buffer
				process_post(&send_process, PROCESS_EVENT_MSG, 0);
			}
		}
	}
	PROCESS_END();
}


PROCESS_THREAD(destination_reaced_process, ev, data) {
	PROCESS_BEGIN();

	static struct etimer blink_timer;
	static uint8_t count = 0;         // led toggle counter
	static uint8_t active = 0;        // to simplify only one packet can blink
	static uint8_t color;

	while(1) {
		PROCESS_WAIT_EVENT();
		// a packet has reached its destination
		if ( ( ev == PROCESS_EVENT_MSG ) && ( ! active ) ) {
			active = 1;
			color = *(uint8_t *)data;  // cast void pointer and store in color
			etimer_set(&blink_timer, CLOCK_SECOND / 16);
		}
		else if (ev == PROCESS_EVENT_TIMER) {
			if (etimer_expired(&blink_timer)) {
				leds_toggle(color);
				count ++;
				// blink 8 times
				if (count < 16) {
					etimer_set(&blink_timer, CLOCK_SECOND / 16);
				}
				else {
					// reset active and count
					active = 0;
					count = 0;
				}
			}
		}
	}

	PROCESS_END();
}
