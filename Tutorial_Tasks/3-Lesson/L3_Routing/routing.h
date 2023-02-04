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

#ifndef ROUTING_H
#define ROUTING_H

#include "core/net/linkaddr.h"

// Standard C includes:
#include <stdint.h>

// the number of nodes present in the network
#ifndef TOTAL_NODES
#define TOTAL_NODES 3
#endif

// the number of nodes present in the network
#ifndef TIMER_INTERVAL
#define TIMER_INTERVAL 1.5
#endif

// custom structures
typedef struct
{
	linkaddr_t 	dest[TOTAL_NODES];			// Destination id. Every node should be able to reach every other node plus itself. Thus total entries are equal to total number of nodes.
	linkaddr_t 	next_hop[TOTAL_NODES];		// Next hop in route to destination.
	uint8_t 	cost[TOTAL_NODES]; 			// Number of total hops of the packet route. Maximum 10.
}l_table;

typedef struct{
	linkaddr_t dest;
	uint8_t message;			// Packet Message (LED COLOR)
}packet_t;

#endif /* ROUTING_H */
