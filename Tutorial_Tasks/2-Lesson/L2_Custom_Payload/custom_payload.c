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

   LESSON 2: Custom Payload
*/

// Contiki-specific includes:
#include "contiki.h"
#include "net/rime/rime.h"	// Establish connections.
#include "net/netstack.h"
#include "lib/random.h"
#include "dev/leds.h"

// Standard C includes:
#include <stdio.h>
#include <stdint.h>


#define BROADCAST_RIME_CHANNEL 129
#define BROADCAST_INTERVAL 5
#define CHANNEL 14
#define POWER 3
#define MY_ADDR 2
#define MAX_PAYLOAD 80
#define MAX_N 20

typedef struct{
	char name[MAX_N];
	char surname[MAX_N];
	uint8_t tel[MAX_N];
}contact;

//Global variables
static contact tx_contacts;
static contact contact_rec;
// Definition of Processes
PROCESS(custom_payload_process, "Lesson 2: Custom Payload");

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

	printf("Broadcast Routing message received from 0x%x%x: \n\r [RSSI %d]\n\r",
			from->u8[0], from->u8[1],
			packetbuf_attr(PACKETBUF_ATTR_RSSI));

		//Copy the payload of the packetbuffer to a given memory location
		packetbuf_copyto(&contact_rec);
		//print the content of the memory location
		printf("%s\n",contact_rec.name);
		printf("%s\n",contact_rec.surname);
		printf("Telephones:\n",contact_rec.surname);
		for(int i=0;i<10;i++){
			printf("%d\n",contact_rec.tel[i]);
		}
	leds_off(LEDS_GREEN);
}

// Defines the functions used as callbacks for a broadcast connection.
static const struct broadcast_callbacks broadcast_call = {broadcast_recv};

/*
 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
                                    MAIN PROCESS
 +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
 */

AUTOSTART_PROCESSES(&custom_payload_process);

PROCESS_THREAD(custom_payload_process, ev, data)
{
	static uint8_t broadcast_time_interval = BROADCAST_INTERVAL;	// In seconds
	static struct etimer broadcast_table_timer;

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


	// Open broadcast connection.
	broadcast_open(&broadcast, BROADCAST_RIME_CHANNEL, &broadcast_call);

	static int i;

	while(1) //MAIN LOOP
	{
		// Transmitted values
		strcpy(tx_contacts.name,"Contiki123");
		strcpy(tx_contacts.surname,"WSN");
		for(i=0;i<10;i++){
			tx_contacts.tel[i]=i;
		}

		leds_on(LEDS_RED);

		//Copy the content of tx_contacts to the buffer.
		//Hint: use packetbuf_copyfrom()
		/*** YOUR CODE HERE ***/

		packetbuf_copyfrom(&tx_contacts,sizeof(tx_contacts));

		broadcast_send(&broadcast);
		leds_off(LEDS_RED);
		printf("CONTACT SENT\n\r\n\r");

		etimer_set(&broadcast_table_timer, CLOCK_SECOND * broadcast_time_interval);
		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&broadcast_table_timer));
	}
	PROCESS_END();
	return 0;
}
