// Exercise 1: etimer

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

   LESSON 1: Timers and Threads
*/


// Contiki-specific includes:
#include "contiki.h"
#include "dev/leds.h"			// Enables use of LEDs

// Standard C includes:
#include <stdio.h>		// For printf


PROCESS(timers_and_threads_process, "Lesson 1: Timers and Threads");
AUTOSTART_PROCESSES(&timers_and_threads_process);

//------------------------ PROCESS' THREAD ------------------------

// Main process:
PROCESS_THREAD(timers_and_threads_process, ev, data) {


	PROCESS_EXITHANDLER( printf("main_process terminated!\n"); )

    PROCESS_BEGIN();

	//static struct timer freq_timer;
	static struct etimer freq_timer;
	// timer_set(&freq_timer,CLOCK_SECOND);
	etimer_set(&freq_timer,CLOCK_SECOND);
    while (1){
    	PROCESS_WAIT_EVENT();
//    	 if(timer_expired(&freq_timer)){
//    		 /* If timer expired, toggle LED*/
//			leds_toggle(LEDS_RED);
//			/* Reset Timer */
//			timer_reset(&freq_timer);
//    	 }
    	if(ev == PROCESS_EVENT_TIMER){
    		leds_toggle(LEDS_RED);
    		/* Reset Timer */
    		etimer_reset(&freq_timer);
    	}
    }

    PROCESS_END();
}

