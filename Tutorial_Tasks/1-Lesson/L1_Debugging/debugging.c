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

   LESSON 1: Debugging with Serial Port Terminal
*/

// Contiki-specific includes:
#include "contiki.h"
#include "dev/uart.h"
#include "dev/leds.h"          // Use LEDs.
#include "dev/serial-line.h"


// Standard C includes:
#include <stdio.h>			// For printf.


//--------------------- PROCESS CONTROL BLOCK ---------------------
PROCESS(debug_process, "Lesson 1: Debugging");
AUTOSTART_PROCESSES(&debug_process);

//------------------------ PROCESS' THREAD ------------------------
PROCESS_THREAD(debug_process, ev, data){
	PROCESS_BEGIN();
	printf("Hello world\n");
	int i;
	for(i=5;i<=3056;i++){
		printf("%d\n",i);
	}

    // Main process loop:
    while(1) {

    	/* NOTE: Contiki processes cannot start loops that never end.
    	 * In order to stay safe, we need to wait for an event in here.
    	 * When a Contiki process waits for events, it returns control
    	 * to the Contiki kernel.
    	 * The Contiki kernel will service other processes while this
    	 * process is waiting.
    	 */
    	PROCESS_WAIT_EVENT();
    }

    PROCESS_END();
}
