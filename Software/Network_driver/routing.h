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



// the maximum number of neighbors for a node in the network
#ifndef MAX_NEIGHBORS
#define MAX_NEIGHBORS 10
#endif

// the maximum number of nodes in the network (used in the size of adjacency matrix)
#ifndef MAX_NODES
#define MAX_NODES 20
#endif

// the maximum number of hops for a routing or control packet in the network
#ifndef MAX_HOPS
#define MAX_HOPS 10
#endif

// the maximum number of retransmissions for runicast messages in the network
#ifndef RUNICAST_MAX_RETRANSMIT
#define RUNICAST_MAX_RETRANSMIT 3
#endif

// the minimum rssi between 2 nodes for them to be considered neighbors in the network
#ifndef MIN_NEIGHBOR_RSSI
#define MIN_NEIGHBOR_RSSI -65
#endif

// If no sensor data packet is received from a node for this time period in seconds, it is considered dead
#ifndef ALIVE_MAX_NODE_DELAY
#define ALIVE_MAX_NODE_DELAY 5
#endif

// custom structures
typedef struct
{
	linkaddr_t 	dest[3];			// Destination id. Every node should be able to reach every other node plus itself. Thus total entries are equal to total number of nodes.
	linkaddr_t 	next_hop[3];		// Next hop in route to destination.
	uint8_t 	cost[3]; 			// Number of total hops of the packet route. Maximum 10.
}l_table;
//
//static void checkAddNeighbor(discovery_packet disc_packet);
//static unsigned short rssi_to_cost(int16_t rssi);
//static void print_neighbors_table(neighbor_data nbors[], uint8_t n);

#endif /* ROUTING_H */
