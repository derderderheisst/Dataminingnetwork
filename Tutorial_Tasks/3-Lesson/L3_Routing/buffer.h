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

   Idea from https://www.mikrocontroller.net/articles/FIFO

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

#ifndef BUFFER_H
#define BUFFER_H

#include "contiki.h"
#include "routing.h"

#include <stdint.h>

// maximum size of the buffer
#ifndef BUFFER_SIZE
#define BUFFER_SIZE 10
#endif

// return code for buffer failure
#ifndef BUFFER_FAIL
#define BUFFER_FAIL 0
#endif

// return code for buffer success
#ifndef BUFFER_SUCCESS
#define BUFFER_SUCCESS  1
#endif

// buffer structure
typedef struct
{
	struct timer timers[BUFFER_SIZE];
	packet_t packets[BUFFER_SIZE];
	uint8_t read;
	uint8_t write;
}Buffer;

// puts a packet and a timer in the buffer
// returns BUFFER_FAIL if buffer is full
uint8_t BufferIn(Buffer *buffer, packet_t packet, struct timer packet_timer);

// removes a packet from a buffer
// returns BUFFER_FAIL if buffer is empty
uint8_t BufferOut(Buffer *buffer, packet_t *packet, struct timer *packet_timer);

#endif /* BUFFER_H */
