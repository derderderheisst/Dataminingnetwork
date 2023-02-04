#ifndef PACKET_STRUCTURES_H
#define PACKET_STRUCTURES_H

#include "core/net/linkaddr.h"
#include "list_structures.h"
#include "routing.h"

// Standard C includes:
#include <stdint.h>


typedef struct{ // Packet Structure for discovery phase
	linkaddr_t src; // source link address
	unsigned short src_dist_to_gateway;
	uint8_t disc_type; // = 0 for initial network discovery,
					// = 1 for subsequent beacon broadcasts to add new nodes to the established network.
					// = 2 for the discovery broadcast of a newly added node (as a response to beacon)
}discovery_packet;

typedef struct{ // Packet Structure for carrying the neighbor data to the gateway in routing phase
	linkaddr_t src; // source link address
	uint8_t num_neighbors;
	neighbor_data ntable[MAX_NEIGHBORS]; // RSSI values are not used in msgtype 1.
}neighbor_data_packet;

typedef struct{ // Packet Structure for carrying the neighbor data to the gateway for a newly added node
	linkaddr_t src; // source link address
	linkaddr_t conn_addr;
	uint8_t num_neighbors;
	neighbor_data ntable[MAX_NEIGHBORS]; // RSSI values are not used in msgtype 1.
}newnode_data_packet;

typedef struct{ // Packet Structure for carrying routing information from gateway to all other nodes.
	linkaddr_t hops_list[MAX_HOPS]; // Destination is at the first index "[0]", and hops go in reverse
	uint8_t hops_left;
	unsigned short cost_to_gateway;
	uint8_t num_active_neighbors;
	uint8_t node_index;
	linkaddr_t nlist[MAX_NEIGHBORS]; // RSSI values are not used in msgtype 1.
}routing_paths_packet;

typedef struct{
	linkaddr_t src_addr;
	//uint8_t src_ind;
    int tempval;
    int mvoltval;
    int16_t vibration_val;
}sensordata_packet;

typedef struct{
	linkaddr_t src;
	linkaddr_t conn;
	unsigned short dist;
}update_dist_packet;

typedef struct{ // Packet Structure for routing phase

	uint8_t msgtype; // 0: Msg for sending neighbor data to the gateway for optimal route calculations
					 // 1: Msg from gateway to send the calculated optimal route info to the other nodes
					 // 2: Msg from nodes to the gateway that includes the sensor data
					 // 3: Msg as a response to check msg or beacon broadcast after initial routing
					 // 4: Msg to inform the neighbor node and the gateway about the newly added node
					//  5: Msg to update the distance of a node to the gateway (used in failure recovery)
	union{
		neighbor_data_packet n;
		routing_paths_packet r;
		sensordata_packet s;
		discovery_packet d;
		newnode_data_packet newnode;
		update_dist_packet ud;
	}msg;
}wrapper_packet;



#endif // PACKET_STRUCTURES_H
