// Exercise 3: Threads:
//##############################################################################

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

static struct ctimer c;

PROCESS(timers_and_threads_process, "Lesson 1: Timers and Threads");
PROCESS(process2, "Process 2 for Exercise 3");
AUTOSTART_PROCESSES(&timers_and_threads_process, &process2);

//------------------------ PROCESS' THREAD ------------------------

static void callback_func(void *data){
	leds_toggle(LEDS_RED);
	ctimer_reset(&c);
}
// Main process:
PROCESS_THREAD(timers_and_threads_process, ev, data) {


	PROCESS_EXITHANDLER( printf("main_process terminated!\n"); )

    PROCESS_BEGIN();


	ctimer_set(&c,CLOCK_SECOND,callback_func, NULL);

    while (1){
    	PROCESS_WAIT_EVENT();

    }

    PROCESS_END();
}



// Main process 2:
PROCESS_THREAD(process2, ev, data) {

	PROCESS_EXITHANDLER( printf("second_process terminated!\n"); )

    PROCESS_BEGIN();
	static struct etimer eventtimer;
	etimer_set(&eventtimer,CLOCK_SECOND);

    while (1){
    	PROCESS_WAIT_EVENT();

    	if(ev == PROCESS_EVENT_TIMER){
    		leds_toggle(LEDS_BLUE);
    		/* Reset Timer */
    		etimer_reset(&eventtimer);
    	}
    }

    PROCESS_END();
}

