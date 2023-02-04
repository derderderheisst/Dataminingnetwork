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
// #include <stdio.h>  // uncomment if using the debug
#include "buffer.h"

uint8_t BufferIn(Buffer *buffer, packet_t packet, struct timer packet_timer)
{
	// for debug:
	// printf("BufferIn: write: %d, read: %d\r\n", buffer->write, buffer->read);

	// check if buffer is full
	if ( ( buffer->write + 1 == buffer->read ) ||
			( buffer->read == 0 && buffer->write + 1 == BUFFER_SIZE ) )
		return BUFFER_FAIL;

	// store packet and timer in the buffer
	buffer->packets[buffer->write] = packet;
	buffer->timers[buffer->write] = packet_timer;

	buffer->write++;
	// if reached end of buffer set write pointer to 0
	if (buffer->write >= BUFFER_SIZE)
		buffer->write = 0;

	return BUFFER_SUCCESS;
}

uint8_t BufferOut(Buffer *buffer, packet_t *packet, struct timer *packet_timer)
{
	// for debug:
	// printf("BufferOut: write: %d, read: %d\r\n", buffer->write, buffer->read);

	// check if buffer is empty
	if (buffer->read == buffer->write)
		return BUFFER_FAIL;

	// get packet and timer from the buffer
	*packet = buffer->packets[buffer->read];
	*packet_timer = buffer->timers[buffer->read];

	buffer->read++;
	// if reached end of buffer set read pointer to 0
	if (buffer->read >= BUFFER_SIZE)
		buffer->read = 0;

	return BUFFER_SUCCESS;
}
