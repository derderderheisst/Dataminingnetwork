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
   * Donjeta Elshani

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, version 2.0 of the License.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   LESSON 2: Radio Settings
*/

// Contiki-specific includes:
#include "contiki.h"
#include "net/rime/rime.h"		// Establish connections.
#include "dev/button-sensor.h" 		// User Button
#include "lib/random.h"
#include "dev/leds.h"
#include "dev/cc2538-rf.h"

#include "rtimer.h"
#include "project-conf.h"

// Standard C includes:
#include <stdio.h>
#include <stdint.h>

// Support function includes:
#include "helpers.h"


typedef struct{
	uint16_t unique_id;					// Message ID
	char message[MAX_MESSAGE_LENGTH];			// Packet Message
}packet_t;

/* Global variables */
rtimer_clock_t tx_time[TOTAL_TX_PACKETS], rx_time;		// Stores packet transmission time.
float rtt[TOTAL_TX_PACKETS] = {0};				// Stores packet round-trip time
uint8_t packet_counter = 0;					// Counts transmitted packet.

/* Packet to be transmitted.*/
packet_t tx_packet[TOTAL_TX_PACKETS];


/*** CONNECTION DEFINITION***/

/**
 * Connection information
 */
static struct broadcast_conn broadcastConn;

/**
* @param message - message to be broadcasted
*/
void echo_packet(packet_t * packet) {
	packetbuf_copyfrom(packet, sizeof(packet_t));
	broadcast_send(&broadcastConn);
}

/**
 * Callback function for received packet processing.
 *
 */
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from) {

	packet_t rx_packet;
	packetbuf_copyto(&rx_packet);
	rx_time = RTIMER_NOW();
	uint8_t len = strlen( (char *)packetbuf_dataptr() );
	int16_t rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);
	uint8_t id = rx_packet.unique_id;

	leds_on(LEDS_GREEN);
	printf("Got RX packet (broadcast) from: 0x%x%x, len: %d, RSSI: %d\r\n",from->u8[0], from->u8[1],len,rssi);
			
	/* Check type of packet. */
	if (strcmp(rx_packet.message, "Request") == 0)	{
		strcpy(rx_packet.message, "Reply");
		echo_packet(&rx_packet);	// Echo received packet.
		printf("Request received - Packet id. %d\n", id);
	}
	else	{
		rtt[id] = (rx_time - tx_time[id]);	// Calculate round trip time, in ticks.
		rtt[id] = (float) rtt[id]/RTIMER_SECOND;						// Convert ticks to seconds.
		char rtt_buffer[5];									// Buffer to store Round-trip time.
		ftoa(rtt[id], rtt_buffer, 3);								// Covert Round-trip time from float to char.
		printf("Reply received - Packet id. %d RRT= %s seconds. \n", id, rtt_buffer);
	}

	leds_off(LEDS_GREEN);

}

/**
 * Assign callback functions to the connection
 */
static const struct broadcast_callbacks broadcast_callbacks = {broadcast_recv};

/*** CONNECTION DEFINITION END ***/


/*** MAIN PROCESS DEFINITION ***/
PROCESS(mac_process, "Lesson 2: MAC settings");
AUTOSTART_PROCESSES(&mac_process);

/*** MAIN THREAD ***/
PROCESS_THREAD(mac_process, ev, data) {

	static struct etimer et;
	static int i;
	static int packets_lost;
	float mean_time, packet_delivery_ratio;
	char mean_time_buffer[5], packet_delivery_ratio_buffer[6];	// Buffer to store chars from float

	PROCESS_EXITHANDLER(broadcast_close(&broadcastConn));
	PROCESS_BEGIN();

	/* Configure the user button */
	button_sensor.configure(BUTTON_SENSOR_CONFIG_TYPE_INTERVAL, CLOCK_SECOND/5);

	/*
	 * set your group's channel
	 */
	NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_CHANNEL, CHANNEL);

	/*
	 * Change the transmission power
	 */
	NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_TXPOWER,TX_POWER);

	/*
	 * open the connection
	 */
	broadcast_open(&broadcastConn,129,&broadcast_callbacks);

	while(1){

		/* Wait until transmission activated by pushing user button. */
		PROCESS_WAIT_EVENT_UNTIL(ev == sensors_event && data == &button_sensor);

		/* Reset variables. */
		packet_counter = 0;

		/* Generate and transmit packets.*/
		while(packet_counter < TOTAL_TX_PACKETS)
		{
			/*
			 * Wait 1 second  Inter-Packet Arrival Time.
			 * */
			etimer_set(&et, INTER_PACKET_TIME);

			PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

			leds_on(LEDS_RED);

			/* Generate packet. */
			tx_packet[packet_counter].unique_id = packet_counter;
			strcpy(tx_packet[packet_counter].message, "Request");

			/*
			 * fill the packet buffer & send the packet
			 */
			packetbuf_copyfrom(&tx_packet[packet_counter],sizeof(packet_t));
			broadcast_send(&broadcastConn);

			/* Store packet sending time. */
			tx_time[packet_counter] = RTIMER_NOW();

			/*
			 * reset the timer
			 */
			etimer_reset(&et);

			leds_off(LEDS_RED);
			packet_counter++;
		}

		etimer_set(&et, 5*CLOCK_SECOND); // Randomize the sending time a little
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		/* Calculate number of lost packets, by checking Round trip time.
		 * If a packet was not received its rtt = 0;
		 */
		packets_lost = 0;
		for(i = 0; i<TOTAL_TX_PACKETS; i++)
		if(rtt[i] == 0)
			packets_lost++;
		packet_delivery_ratio = (float)(TOTAL_TX_PACKETS-packets_lost)/(TOTAL_TX_PACKETS)*100;
		ftoa(packet_delivery_ratio, packet_delivery_ratio_buffer,2);		
		printf("\nTotal number of packets lost = %d.\n", packets_lost);
		printf("Packet delivery rate = %s %.\n", packet_delivery_ratio_buffer);

		/*
		 * Calculate packet transmission mean time.
		 * Exclude lost packets.
		 * */
		mean_time = 0;
		for(i = 0; i<TOTAL_TX_PACKETS; i++)
			if(rtt[i] != 0)
				mean_time = mean_time + rtt[i];
		mean_time = (float)(mean_time/(TOTAL_TX_PACKETS-packets_lost)); // Divide by total round-trip time by total packets send.
		ftoa(mean_time, mean_time_buffer,3);
		printf("Mean round-trip time = %s seconds. \n", mean_time_buffer);

	}

	PROCESS_END();
}
