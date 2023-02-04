#ifndef LIST_STRUCTURES_H
#define LIST_STRUCTURES_H

#include "core/net/linkaddr.h"

// Standard C includes:
#include <stdint.h>



typedef struct
{
	linkaddr_t 	addr;			// Address of the neighbor
	int16_t rssi; 					// RSSI measured from the connection to the neighbor
}neighbor_data;


#endif // LIST_STRUCTURES_H
