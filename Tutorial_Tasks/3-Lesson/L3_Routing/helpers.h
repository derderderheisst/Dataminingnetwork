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

#ifndef HELPERS_H_
#define HELPERS_H_

#include <stdint.h>

#include "routing.h"

// turns a LED color on or off and keeps track of the number of packets
// with specific color, so LEDS will not be tured off if there are more
// packets of this color present.
void turn_on(uint8_t color);
void turn_off(uint8_t color);

void print_lookup_table(l_table table, uint8_t number_of_nodes);

// defines the led color based on the node id
uint8_t get_led_color(uint8_t id);

void print_settings(void);

// checks if address is valid and will freeze the node if not
void check_for_invalid_addr(void);

// calculates the destination to send a new generated packet
uint8_t calculate_destination(uint8_t id, uint8_t network_size);

#endif /* HELPERS_H_ */
