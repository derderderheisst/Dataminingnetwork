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

   LESSON 2: TRansmission Power
*/

// Contiki-specific includes:
#include "contiki.h"
#include "net/rime/rime.h"	// Establish connections.
#include "lib/random.h"
#include "dev/leds.h"
#include "dev/button-sensor.h"
#include "dev/cc2538-rf.h"
#include "dev/serial-line.h"

// Standard C includes:
#include <stdio.h>
#include <stdint.h>




/*** CONNECTION DEFINITION***/

/**
 * Callback function for received packet processing.
 *
 */
static int valid_power_vals[14] = {-24,-15,-13, -11,-9,-7,-5,-3,-1,0,1,3,5,7};
int curr_pow_ind = 11;
int curr_power = 3;

static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from) {

	leds_on(LEDS_GREEN);

	uint8_t len = strlen( (char *)packetbuf_dataptr() );
	int16_t rssi = packetbuf_attr(PACKETBUF_ATTR_RSSI);

	printf("Got RX packet (broadcast) from: 0x%x%x, len: %d, RSSI: %d\r\n",from->u8[0], from->u8[1],len,rssi);

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

/*** CONNECTION DEFINITION END ***/


/*** MAIN PROCESS DEFINITION ***/
PROCESS(transmission_power_process, "Lesson 2: Transmission Power");
AUTOSTART_PROCESSES(&transmission_power_process);


/*** MAIN THREAD ***/
PROCESS_THREAD(transmission_power_process, ev, data) {

	static struct etimer et;

	PROCESS_EXITHANDLER(broadcast_close(&broadcastConn));
	PROCESS_BEGIN();

	/*
	 * set your group's channel
	 */
	NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_CHANNEL, 15);

	/*
	 * Change the transmission power here
	 */

	/*
	 * open broadcast connection
	 */
	broadcast_open(&broadcastConn,129,&broadcast_callbacks);

	etimer_set(&et, CLOCK_SECOND + 0.1*random_rand()/RANDOM_RAND_MAX); //randomize the sending time a little

	while(1){

		PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&et));
		if(ev == sensors_event) {
					if(data == &button_sensor) {
						if(button_sensor.value(BUTTON_SENSOR_VALUE_TYPE_LEVEL) == BUTTON_SENSOR_PRESSED_LEVEL) {
								printf("Button, %d",curr_pow_ind);
								if (curr_pow_ind>=13){
										curr_pow_ind = 0;
								}else{
									curr_pow_ind++;
								}
								curr_power = valid_power_vals[curr_pow_ind];
						}
					}
		}
		leds_on(LEDS_RED);

		/*
		 * fill the packet buffer
		 */
		char transmit_str[30];
		sprintf(transmit_str, "Transmit power %d dbm",curr_power);
		packetbuf_copyfrom(transmit_str,30);

		/*
		 * send the message
		 */
		broadcast_send(&broadcastConn);

//		if(ev == serial_line_event_message){
//			printf(data);
//			if(!strncmp(data,"power",5)){
//				int power_val =2;
//				//sscanf(data,"Power %d",&power_val);
//				char compare_str[] = "power ";
//
//				for(int i=0;i<14;i++){
//
//					strcat(compare_str, itoa(valid_power_vals[i]));
//					strcat(compare_str,"\n");
//					printf(compare_str);
//					if(strcmp(data, compare_str)){
//						curr_power = valid_power_vals[i];
//						printf("Broadcast power is changed to %d\n",curr_power);
//					}
//				}
//
//			}
//		}

		/*
		 * for debugging
		 */
		printf("Broadcast message sent with power: %d\r\n",curr_power); // or the configured Power

		/*
		 * reset the timer
		 */
		etimer_reset(&et);

		leds_off(LEDS_RED);
	}

	PROCESS_END();
}

