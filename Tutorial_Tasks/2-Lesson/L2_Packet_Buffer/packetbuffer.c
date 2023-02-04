/*
   Wireless Sensor Networks Laboratory

   Technische Universität München
   Lehrstuhl für Kommunikationsnetze
   http://www.lkn.ei.tum.de

   copyright (c) 2014 Chair of Communication Networks, TUM

   contributors:
 * Thomas Szyrkowiec
 * Mikhail Vilgelm
 * Octavio Rodríguez Cervantes

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, version 2.0 of the License.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.

   LESSON 1: Blinking LEDs
 */

// Contiki-specific includes:
#include "contiki.h"
#include "net/rime/rime.h"	// Establish connections.
#include "lib/random.h"
#include "dev/leds.h"
#include "dev/cc2538-rf.h"

// Standard C includes:
#include <stdio.h>
#include <stdint.h>

// Definition of Processes
PROCESS(packet_buffer_process, "Lesson 2: Packet Buffer");

//Function prototypes
static void print_packetbuffer();
static void copy_and_print_packetbuffer();


/*
 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
   	   	   	   	   	   	   	  BROADCAST CONNECTION
 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

// Creates an instance of a broadcast connection.
static struct broadcast_conn broadcast;

// Defines the behavior of a connection upon receiving data.
static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from)
{
	leds_on(LEDS_GREEN);

	uint8_t len = strlen( (char *)packetbuf_dataptr() );
	int16_t rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);



	printf("Got RX packet (broadcast) from: 0x%x%x, len: %d, RSSI: %d\r\n",from->u8[0], from->u8[1],len,rssi);

	printf("RECEIVED - Copy \n\r");
	copy_and_print_packetbuffer();

	printf("RECEIVED - Print \n\r");
	print_packetbuffer();

	leds_off(LEDS_GREEN);
}

/**
 * Connection information
 */
static struct broadcast_conn broadcastConn;

/**
 * Assign callback functions to the connection
 */
static const struct broadcast_callbacks broadcast_callbacks = {broadcast_recv};

/*
 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                                    MAIN PROCESS
 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

AUTOSTART_PROCESSES(&packet_buffer_process);

PROCESS_THREAD(packet_buffer_process, ev, data)
{
	static struct etimer et;

	PROCESS_EXITHANDLER(broadcast_close(&broadcast);)

	PROCESS_BEGIN();

	/*
	 * set your group's channel
	 */
	NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_CHANNEL, 15);

	/*
	 * Change the transmission power
	 */
	NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_TXPOWER,3);

	/*
	 * open the connection
	 */
	broadcast_open(&broadcastConn,129,&broadcast_callbacks);

	etimer_set(&et, CLOCK_SECOND);

	while(1) //MAIN LOOP
	{

		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));

		leds_on(LEDS_RED);

		/*
		 * fill the packet buffer
		 */
		packetbuf_copyfrom("Hello",6);

		/*
		 * send the message
		 */
		broadcast_send(&broadcastConn);

		/*
		 * for debugging
		 */
		printf("Broadcast message sent in channel %d with power: %d\r\n", 26, 3);

		print_packetbuffer();

		/*
		 * reset the timer
		 */
		etimer_reset(&et);

		leds_off(LEDS_RED);
	}
	PROCESS_END();

}


/* ----------------------- Exercise prototypes ----------------------- */

static void copy_and_print_packetbuffer(){
	char rec_str_ext[strlen((char *)packetbuf_dataptr())];
	packetbuf_copyto(rec_str_ext);
	//rec_str_ext = (char *)rec_str_ext;
	printf("%s\n",rec_str_ext);
	//copy the payload of the packetbuffer to a given memory location
	/*** YOUR CODE HERE ***/
	//print the content of the memory location
	/*** YOUR CODE HERE ***/
}

static void print_packetbuffer(){
	printf("%s\n",(char *)packetbuf_dataptr());
	//retrieve the pointer to the payload and the length of it
	/*** YOUR CODE HERE ***/
	//use the retrieved information to print the content of the payload
	/*** YOUR CODE HERE ***/
}
