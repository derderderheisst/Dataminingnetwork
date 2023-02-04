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

   LESSON 1: Multithreading
*/

// Contiki-specific includes:
#include "contiki.h"
#include "dev/leds.h" // Enables use of LEDs.

// Standard C includes:
#include <stdio.h>    // For printf.


//--------------------- PROCESS CONTROL BLOCK ---------------------
PROCESS(red_process, "Lesson 1: Red");
PROCESS(green_process, "Lesson 1: Green");
PROCESS(blue_process, "Lesson 1: Blue");
AUTOSTART_PROCESSES(&red_process, &green_process, &blue_process);

//------------------------ PROCESS' THREAD ------------------------
PROCESS_THREAD(red_process, ev, data){

	static struct etimer timerRed;
	static struct etimer timerTerminateRed;
	PROCESS_BEGIN();

	printf("Red Timer set!\r\n ");
	/*
	 * Set timers
	 */
	etimer_set(&timerRed, CLOCK_SECOND);
	etimer_set(&timerTerminateRed, 2*CLOCK_SECOND);

	while(1) {
		PROCESS_WAIT_EVENT();
		if(etimer_expired(&timerRed)) {
			printf("Timer expired for RED...\r\n");
			leds_toggle(LEDS_RED);
			etimer_reset(&timerRed);
		}
		else if(etimer_expired(&timerTerminateRed)){
			printf("Red Process Terminated \n");
			PROCESS_EXIT();
		}

	}
	PROCESS_END();
}

PROCESS_THREAD(green_process, ev, data){

	static struct etimer timerGreen;

	PROCESS_BEGIN();

	printf("Green Timer set!\r\n ");
	/*
	 * Set timers
	 */
	etimer_set(&timerGreen, CLOCK_SECOND/2);


	while(1) {
		PROCESS_WAIT_EVENT();
		if(etimer_expired(&timerGreen)) {
			printf("Timer expired for Green...\r\n");
			leds_toggle(LEDS_GREEN);
			etimer_reset(&timerGreen);
		}

	}
	PROCESS_END();
}

PROCESS_THREAD(blue_process, ev, data){

	static struct etimer timerBlue;

	PROCESS_BEGIN();

	printf("Blue Timer set!\r\n ");
	/*
	 * Set timers
	 */
	etimer_set(&timerBlue, CLOCK_SECOND/4);


	while(1) {
		PROCESS_WAIT_EVENT();
		if(etimer_expired(&timerBlue)) {
			printf("Timer expired for Blue...\r\n");
			leds_toggle(LEDS_BLUE);
			etimer_reset(&timerBlue);
		}

	}
	PROCESS_END();
}





