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

/*----------------------------INCLUDES----------------------------------------*/
// standard C includes:
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>

// Contiki-specific includes:

#include "contiki.h"
#include "net/rime/rime.h"     // Establish connections.
#include "net/netstack.h"      // Wireless-stack definitions
#include "sys/energest.h"
#include "dev/watchdog.h"
#include "dev/button-sensor.h" // User Button
#include "dev/serial-line.h"

#include "dev/leds.h"          // Use LEDs.
#include "lib/random.h"			// Psuedo random number generator
// includes from sensors.c
#include "dev/adc-zoul.h"      // ADC
#include "dev/zoul-sensors.h"  // Sensor functions
#include "dev/sys-ctrl.h"
#include "dev/gpio.h"


// Header
#include "routing.h"
//#include "buffer.h"
#include "helpers.h"
#include "packet_structures.h"
#include "list_structures.h"

/*---------------------------Variables----------------------------------------*/

static uint8_t current_phase;		 // 0 : Node discovery phase, 1 : Routing phase, 2: Sensor data sending phase, 3: Failure recovery phase, 4:New addition phase for a new node,
									// 6 : Failed State
static bool discovery_broadcast_done;
static bool isGateway = false;
static uint8_t num_neighbors;
static uint8_t num_total_nodes = 0;
static uint8_t node_ind;
static linkaddr_t nodelist[MAX_NODES];

static linkaddr_t active_nlist[MAX_NEIGHBORS];
static uint8_t num_active_neighbors;
static uint8_t prev_nodes[MAX_NODES];
static unsigned short *node_dists;
static unsigned short dist_to_gateway = 9999;
static uint8_t curr_route;

static unsigned short **adjmx;
static unsigned int *sensordata_times;

static struct etimer packet_delay_timer;
static neighbor_data neighbors[MAX_NEIGHBORS];
static struct etimer discovery_finished_timer;
static struct etimer sensor_send_timer;
static struct ctimer compute_routing_timer;
static struct ctimer send_routing_info_timer;
static struct ctimer check_node_alive_timer;
static struct ctimer beacon_send_timer;
static struct ctimer reconnect_broadcast_send_timer;
static uint8_t led_color;	        // Each id has unique led color. When this
                                    // node generates a packet, it has this led
                                    //color.


static struct unicast_conn unicast; // Creates an instance of a unicast
                                    //connection.
static struct broadcast_conn broadcast;

static struct runicast_conn runicast;


//--------------------- PROCESS CONTROL BLOCK ---------------------
PROCESS(routing_process, "Routing test");
PROCESS(sensor_data_sending_process, "Periodically send sensor data from the nodes");
PROCESS(sensor_process_gateway, "Receive the periodically sent sensor data from the nodes (for gateway node)");
//PROCESS(destination_reaced_process, "Process indicate a packets has reached its"
//		" destination");
AUTOSTART_PROCESSES(&routing_process);

//-----------------// Sensors:
static void configure_node_sensors(){
	/* Configure the ADC port for the vibration sensor */
	adc_zoul.configure(SENSORS_HW_INIT, ZOUL_SENSORS_ADC1 | ZOUL_SENSORS_ADC3);
}


static sensordata_packet create_sensor_packet(){
	sensordata_packet spacket;
	spacket.vibration_val = adc_zoul.value(ZOUL_SENSORS_ADC3) >> 4;
	spacket.mvoltval = vdd3_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED);
	spacket.tempval = cc2538_temp_sensor.value(CC2538_SENSORS_VALUE_TYPE_CONVERTED);
	//printf("Temperature: %d mC\nVibration: %d\nBattery Voltage: %u mV\n\n", spacket.tempval, spacket.vibration_val, spacket.mvoltval);
	linkaddr_copy(&spacket.src_addr, &linkaddr_node_addr);
	//spacket.src_ind = node_ind;
	return spacket;
}

static void buzzer_update(sensordata_packet spacket){
	if(spacket.vibration_val < 500)
	{
		GPIO_SET_PIN(GPIO_C_BASE, 0x08);
		clock_delay(2*CLOCK_SECOND);
		GPIO_CLR_PIN(GPIO_C_BASE, 0x08);

	}
	else if(spacket.tempval > 40000 )
	{
		GPIO_SET_PIN(GPIO_C_BASE, 0x08);
		clock_delay(3*CLOCK_SECOND);
		GPIO_CLR_PIN(GPIO_C_BASE, 0x08);
	}
	else if(spacket.mvoltval < 1000)
	{
		GPIO_SET_PIN(GPIO_C_BASE, 0x08);
		clock_delay(3*CLOCK_SECOND);
		GPIO_CLR_PIN(GPIO_C_BASE, 0x08);
	}
	else
		GPIO_CLR_PIN(GPIO_C_BASE, 0x08);
}


// Sensors end
static unsigned short rssi_to_cost(int16_t rssi){
	float a = -3.3333;
	float b = -66.6666;
	int cost = (int)(a*rssi)+b; // linear relation to cost
	//check min cost
	if(cost < 50) {return 50;}
	else {return cost;}
}

static void checkAddNeighbor(discovery_packet disc_packet){

	// Add to neighbors if not currently in the list

	for(int i=0;i<num_neighbors;i++){
		if(linkaddr_cmp(&disc_packet.src, &neighbors[i].addr)){
			// Update the RSSI with an average and return if this address is known:
			neighbors[i].rssi = (neighbors[i].rssi + (int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI)) / 2;
			//printf("Address is known !!!\n\n");
			return;
		}
	}
	// Reaches this part only if the address is not in the list yet:
	neighbors[num_neighbors].addr = disc_packet.src;
	neighbors[num_neighbors].rssi = (int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI);
	num_neighbors++;

//	// Print the neighbors list
//	printf("\nNeighbors Table:\n___________________\n");
//	for(int i=0;i<num_neighbors;i++){
//		printf("0x%x%x:    RSSI: %d\n", neighbors[i].addr.u8[0], neighbors[i].addr.u8[1], neighbors[i].rssi);
//	}
//	printf("\n");

}
static void checkAddNeighbor_withDist(discovery_packet disc_packet){

	// Add to neighbors if not currently in the list
	for(int i=0;i<num_neighbors;i++){
		if(linkaddr_cmp(&disc_packet.src, &neighbors[i].addr)){
			//printf("Address is known !!!\n\n");
			return;
		}
	}
	// Reaches this part only if the address is not in the list yet:
	neighbors[num_neighbors].addr = disc_packet.src;
	neighbors[num_neighbors].rssi = (int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI);
	// Save the cumulative distances for each node to connect to for a newly added node
	node_dists[num_neighbors] = disc_packet.src_dist_to_gateway + rssi_to_cost(neighbors[num_neighbors].rssi);

	num_neighbors++;

//	// Print the neighbors list
//	printf("\nNeighbors Table:\n___________________\n");
//	for(int i=0;i<num_neighbors;i++){
//		printf("0x%x%x:    RSSI: %d\n", neighbors[i].addr.u8[0], neighbors[i].addr.u8[1], neighbors[i].rssi);
//	}
//	printf("\n");

}
static void send_newnode_msg(){
	newnode_data_packet ndpacket;
	linkaddr_copy(&ndpacket.src, &linkaddr_node_addr);
	linkaddr_copy(&ndpacket.conn_addr, &active_nlist[0]);
	ndpacket.num_neighbors = num_neighbors;
	memcpy(ndpacket.ntable, neighbors, sizeof(ndpacket.ntable));

	wrapper_packet tx_packet;
	tx_packet.msgtype = 4; // "New node added info" message
	tx_packet.msg.newnode = ndpacket;
	packetbuf_copyfrom(&tx_packet, sizeof(wrapper_packet));
	runicast_send(&runicast, &active_nlist[0], RUNICAST_MAX_RETRANSMIT); // Resend max 2 times
	printf("New node neighbor data sent.\n");
}


static void print_neighbors_table(neighbor_data nbors[], uint8_t n){
	 // Print the neighbors table
	 printf("\nNeighbors Table:\n___________________\n");
	 printf("Num neigh = %d\n",n);
	 for(uint8_t i=0;i<n;i++){
		 printf("0x%x%x:    RSSI: %d\n", nbors[i].addr.u8[0], nbors[i].addr.u8[1], nbors[i].rssi);
	 }
	 printf("\n");
}

static int getIdFromNodelist(linkaddr_t addr){
	// Returns the index of a given link address (addr) in the nodelist if it exists,
	// otherwise it adds the addr to the end of the node list and returns its index.
	for(int i=0;i<num_total_nodes;i++){
		if(linkaddr_cmp(&nodelist[i],&addr)){
			return i;
		}
	}
	linkaddr_copy(&nodelist[num_total_nodes],&addr);
	num_total_nodes++;
	return num_total_nodes-1;
}


static void update_adjacency_matrix(neighbor_data nbors[], linkaddr_t src, uint8_t n){
	// Get the source node id from nodelist and add its address to the nodelist if it is not added yet.
	int src_id = getIdFromNodelist(src);
	int neigh_id;
	int curr_cost;
	// Loop over the neighbors list of the node
	for(int i=0;i<n;i++){
		neigh_id = getIdFromNodelist(nbors[i].addr);
		if(adjmx[src_id][neigh_id] != 0){ // Take average and update if a previous value for this connection exists
			curr_cost = (adjmx[src_id][neigh_id] + rssi_to_cost(nbors[i].rssi))/2;
		}else{ // Calculate cost value from rssi
			curr_cost = rssi_to_cost(nbors[i].rssi);
		}
		adjmx[src_id][neigh_id] = curr_cost;
		adjmx[neigh_id][src_id] = curr_cost;
	}
}

static void print_adjacency_matrix(){
	int y, x;
	printf("\n");
	printf("routingdata:adjmx = ");
	for (y=0; y<num_total_nodes; y++)
	{
		printf("%d", adjmx[y][0]);
	    for(x=1; x<num_total_nodes; x++)
	    {
	         printf(",%d", adjmx[y][x]);
	    }
	    printf(";");
	}
	printf("\n");
}


static void print_nodelist(){
	 // Print the nodelist
	 printf("\nroutingdata:nodelist = 0x%x%x",nodelist[0].u8[0], nodelist[0].u8[1]);
	 for(uint8_t i=1;i<num_total_nodes;i++){
		 printf(",0x%x%x",nodelist[i].u8[0], nodelist[i].u8[1]);
	 }
	 printf("\n");
}

static void print_sensor_data(sensordata_packet s){
	// printf("Sensor data from node %d, Addr: 0x%x%x:\n", s.src_ind, s.src_addr.u8[0],s.src_addr.u8[1]);
	//float temperature = s.tempval*0.001;
	//float voltage = s.mvoltval*0.001;
	// printf("Temperature: %d C\nVibration: %d\nBattery Voltage: %d mV\n\n",s.tempval, s.vibration_val, s.mvoltval);
	uint8_t src_index = getIdFromNodelist(s.src_addr);
	printf("\nsensordata:src_addr = 0x%x%x\nsensordata:src_id = %d\n",s.src_addr.u8[0],s.src_addr.u8[1], src_index);
	printf("sensordata:temp = %d\n",s.tempval);
	printf("sensordata:vibr = %d\n",s.vibration_val);
	printf("sensordata:mvolt = %d\n",s.mvoltval);
}

static void check_node_alive_callback(void *data){
	// Check if there are any dead nodes based on time passed from the last time a sensor data was received from this node
	int delay;
	for(int i = 0;i<num_total_nodes-1;i++){
		delay = clock_seconds() - sensordata_times[i];
		if(delay > ALIVE_MAX_NODE_DELAY){
			printf("\nstatus:nodedown:%d\n",i+1);
			//printf("Node %d is dead ! No signal for %d seconds\n",i+1,delay);
			// TODO: Call function for failure recovery
			// fail_recovery();
		}
	}
	// Set up the timer again (if everything is ok)
	// Reset the timer only after failure recovery if there is a problem
	ctimer_reset(&check_node_alive_timer);
}
static void sensordata_times_reset(){
	for(int i = 0;i<num_total_nodes-1;i++){
		sensordata_times[i] = clock_seconds();
	}
}

static void beacon_send_callback(void *data){
	// Send a beacon broadcast from each node periodically to discover nodes that are newly added to the network
	leds_on(LEDS_RED);
	discovery_packet dpacket;
	linkaddr_copy(&dpacket.src, &linkaddr_node_addr);
	dpacket.src_dist_to_gateway = dist_to_gateway;
	dpacket.disc_type = 1; // Beacon discovery
	packetbuf_copyfrom(&dpacket, sizeof(discovery_packet));
	broadcast_send(&broadcast);
	//printf("Beacon broadcast message sent.\n");
	ctimer_reset(&beacon_send_timer);
	leds_off(LEDS_RED);

}

static routing_paths_packet create_route_packet(){
	routing_paths_packet rpacket;
	uint8_t gwnode = prev_nodes[curr_route];
	rpacket.nlist[0] = nodelist[gwnode]; // Put the node towards the gateway first.
	int curr_ind = 1;
	for(int i=1;i<num_total_nodes;i++){ // Find active neighbors of the node and put them on the list
		if(prev_nodes[i] == curr_route){ // Find nodes that this node is the prev step on:
			rpacket.nlist[curr_ind] = nodelist[i];
			curr_ind++;
		}
	}
	// Distance to the gateway of the current node:
	rpacket.cost_to_gateway = node_dists[curr_route];
	rpacket.num_active_neighbors = curr_ind;

	// Calculate the hops list:
	curr_ind = 0;
	rpacket.hops_list[0] = nodelist[curr_route]; // Destination address
	int curr_node = prev_nodes[curr_route];
	while(curr_node != 0){ // Check prev until we reach the gateway from the current node that we are sending the route for
		curr_ind++;
		rpacket.hops_list[curr_ind]	= nodelist[curr_node];
		curr_node = prev_nodes[curr_node];

	}
	rpacket.hops_left = curr_ind;
	rpacket.node_index = curr_route;
	return rpacket;
}

static void send_routing_info_callback(void *data){
	routing_paths_packet rpacket = create_route_packet();
	wrapper_packet tx_packet;
	tx_packet.msg.r = rpacket;
	tx_packet.msgtype = 1;
	// Send the routing packet
	linkaddr_t firsthop_addr = rpacket.hops_list[rpacket.hops_left];

	// Send to the first hop via runicast
	packetbuf_copyfrom(&tx_packet, sizeof(wrapper_packet));
	runicast_send(&runicast, &firsthop_addr, RUNICAST_MAX_RETRANSMIT); // Resend max 2 times.

	//printf("Sent data for node %d, prev is %d\n",curr_route, gwnode);

	// If there are still nodes left to send the routing data, send the next one
	if(curr_route < num_total_nodes-1){
		curr_route++;
		ctimer_reset(&send_routing_info_timer);
	}else{
		// If finished sending routing information to all of the nodes
		process_start(&sensor_process_gateway, NULL);
	}
}

static void compute_dijkstra_routing(){
	// Use Dijkstra algorithm to compute the optimal path routing to and from the gateway node
	int cost[num_total_nodes][num_total_nodes], distance[num_total_nodes];
	int visited[num_total_nodes], count, mindistance, nextnode, i, j;
	//prev_nodes = malloc(num_total_nodes * sizeof(uint8_t));
	node_dists = malloc(num_total_nodes * sizeof(unsigned short));
	// Creating cost matrix by setting a very large value to the non-connected node connections
	for (i = 0; i < num_total_nodes; i++){
		for (j = 0; j < num_total_nodes; j++){
			if (adjmx[i][j] == 0)
				cost[i][j] = 9999999;
			else
				cost[i][j] = adjmx[i][j];
		}
	}
	for (i = 0; i < num_total_nodes; i++) {
		node_dists[i] = cost[0][i];
		prev_nodes[i] = 0;
		visited[i] = 0;
	}

	node_dists[0] = 0;
	visited[0] = 1;
	count = 1;

	while (count < num_total_nodes - 1) {
		mindistance = 9999999;

		for (i = 0; i < num_total_nodes; i++)
			if (node_dists[i] < mindistance && !visited[i]) {
				mindistance = node_dists[i];
				nextnode = i;
			}

		visited[nextnode] = 1;
		for (i = 0; i < num_total_nodes; i++)
			if (!visited[i])
				if (mindistance + cost[nextnode][i] < node_dists[i]) {
					node_dists[i] = mindistance + cost[nextnode][i];
					prev_nodes[i] = nextnode;
				}
		count++;
	}

//	// Printing the distance
//	for (i = 0; i < num_total_nodes; i++)
//		if (i != 0) {
//			printf("\nDistance from source to %d: %d", i, node_dists[i]);
//		}
	// Returns previous node id to reach each node.(For example if 0<-->2<-->3, 3rd index is = 2 (0 is the gateway))
	// with the static variable prev_nodes
}


static void compute_routing_callback(void *data){
	// Use Dijkstra algorithm to compute the optimal path routing to and from the gateway node

	// First print some node information:
	printf("routingdata:num_total_nodes = %d\n", num_total_nodes);
	print_adjacency_matrix();
	print_nodelist();

	// Returns previous node id to reach each node (saves it to the static array "prev_nodes").
	printf("status:computing_dijkstra\n");
	compute_dijkstra_routing();
	// Print the prev_nodes array:
	printf("\nroutingdata:prevnodes =");
	printf("%d",prev_nodes[0]);
	for (int i = 1; i < num_total_nodes; i++)
		printf(",%d",prev_nodes[i]);
	printf("\n");
	printf("status:sending_routes\n");
	// Start sending data to the nodes
	if(num_total_nodes>1){ // Set up the callback timer to send routing info sequentially.
		curr_route = 1;
		ctimer_set(&send_routing_info_timer,0.5*CLOCK_SECOND,send_routing_info_callback, NULL);
	}
}
static void reconnect_broadcast_send_callback(void *data){
	leds_off(LEDS_ALL);
	leds_on(LEDS_BLUE);
	node_dists = malloc(MAX_NEIGHBORS * sizeof(unsigned short));
	discovery_packet tx_disc_packet;
	linkaddr_copy(&tx_disc_packet.src, &linkaddr_node_addr);
	tx_disc_packet.disc_type = 2;
	tx_disc_packet.src_dist_to_gateway = 9999;
	packetbuf_copyfrom(&tx_disc_packet, sizeof(discovery_packet));
	broadcast_send(&broadcast);
	etimer_set(&discovery_finished_timer, CLOCK_SECOND);
}

static void send_discovery_broadcast(){
	leds_on(LEDS_RED);
	discovery_packet tx_disc_packet;
	linkaddr_copy(&tx_disc_packet.src, &linkaddr_node_addr);
	tx_disc_packet.disc_type = 0; // Initial network discovery
	packetbuf_copyfrom(&tx_disc_packet, sizeof(discovery_packet));
	broadcast_send(&broadcast);
	printf("Broadcast message sent.\n");
	discovery_broadcast_done = true;
	leds_off(LEDS_RED);
}

static void send_neighbor_data(){
	neighbor_data_packet rpacket;
	linkaddr_copy(&rpacket.src, &linkaddr_node_addr);
	rpacket.num_neighbors = num_neighbors;
	memcpy(rpacket.ntable, neighbors, sizeof(rpacket.ntable));

	wrapper_packet tx_packet;
	tx_packet.msgtype = 0; // "Neighbor info to gateway" message
	tx_packet.msg.n = rpacket;
	packetbuf_copyfrom(&tx_packet, sizeof(wrapper_packet));
	// Send to the first element in the neighbors list since it is closest to the the gateway,
	// or it is the gateway because of the delays in broadcasts.
	runicast_send(&runicast, &neighbors[0].addr, RUNICAST_MAX_RETRANSMIT); // Resend max 2 times
	printf("Neighbor data sent.\n");
}

static void send_sensor_data_packet(sensordata_packet spacket){
	wrapper_packet wpacket;
	wpacket.msgtype = 2; // Sensor data packet type
	wpacket.msg.s = spacket;
	packetbuf_copyfrom(&wpacket, sizeof(wrapper_packet));
	printf("Temperature: %d mC\nVibration: %d\nBattery Voltage: %u mV\n\n", spacket.tempval, spacket.vibration_val, spacket.mvoltval);
	printf("Sensor data sent to 0x%x%x.\n",active_nlist[0].u8[0],active_nlist[0].u8[1]);
	runicast_send(&runicast, &active_nlist[0],RUNICAST_MAX_RETRANSMIT);
}


static void add_new_node_in_gateway(newnode_data_packet npacket){
	// Add new node, or reconnect an existing node
	int node_id;
	node_id = getIdFromNodelist(npacket.src);
	//print_neighbors_table(npacket.ntable, npacket.num_neighbors);

	update_adjacency_matrix(npacket.ntable, npacket.src, npacket.num_neighbors);

	// Update prev nodes array
	int prev_id = getIdFromNodelist(npacket.conn_addr);
	prev_nodes[node_id] = prev_id;

	// Print the new information:
	printf("\n");
	printf("routingdata:num_total_nodes = %d\n", num_total_nodes);
	print_adjacency_matrix();
	print_nodelist();
	// Print the prev_nodes array:
	printf("\nroutingdata:prevnodes =");
	printf("%d",prev_nodes[0]);
	for (int i = 1; i < num_total_nodes; i++)
		printf(",%d",prev_nodes[i]);
	printf("\n");
	sensordata_times_reset();
	//check_node_alive_callback(NULL);
}
static void propagate_dist_to_tailnodes(int distance){
	// Send runicast messages to the connected tail nodes when the distance to gateway of this node is updated
	// Update them to 9999 if this node is disconnected
	if(num_active_neighbors <= 1){
		return; // If no tail nodes, just return
	}
	update_dist_packet upacket;
	linkaddr_copy(&upacket.src, &linkaddr_node_addr);
	upacket.dist = distance;
	wrapper_packet wpacket;
	wpacket.msgtype = 5;
	wpacket.msg.ud = upacket;
	packetbuf_copyfrom(&wpacket, sizeof(wrapper_packet));
	for(int i=1;i<num_active_neighbors;i++){
		runicast_send(&runicast,&active_nlist[i] ,RUNICAST_MAX_RETRANSMIT);
	}
}

static void find_best_node_connect(){
	int min_dist = 9999;
	uint8_t min_ind;
	for(int i=0;i<num_neighbors;i++){
		if(node_dists[i] < min_dist){
			min_dist = node_dists[i];
			min_ind = i;
		}
	}
	if(min_dist>=9999){
		printf("No nodes found to connect to !\n");
		current_phase = 6;
	}else{
		// Connect:
		linkaddr_copy(&active_nlist[0], &neighbors[min_ind].addr);
		send_newnode_msg();
		// No need to recalculate distance from rssi, add fix 100 as cost;
		dist_to_gateway = min_dist + 100;
		propagate_dist_to_tailnodes(dist_to_gateway);
		// Start sensor data sending phase
		current_phase = 2;
		process_start(&sensor_data_sending_process, NULL);
	}

}


static void broadcast_recv(struct broadcast_conn *c, const linkaddr_t *from) {
//	printf("Broadcast message received from 0x%x%x:[RSSI %d]\n",
//			from->u8[0], from->u8[1],
//			(int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI));

	// if current node is during the discovery phase:
	if (current_phase == 0){
		discovery_packet rx_disc_packet;
		packetbuf_copyto(&rx_disc_packet);

		// If this is a disc. broadcast during the initial set up of the network
		if(rx_disc_packet.disc_type == 0){
			// Check if it can be considered a neighbor by RSSI limit, ignore the packet if RSSI is lower than min
			if ((int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI) < MIN_NEIGHBOR_RSSI){
				printf("Low RSSI signal, not considered a neighbor!\n");
				return;
			}
			// Add to neighbors if not currently in the list
			checkAddNeighbor(rx_disc_packet);

			discovery_packet tx_disc_packet;
			linkaddr_copy(&tx_disc_packet.src, &linkaddr_node_addr);

			// Send unicast back to reply
			packetbuf_copyfrom(&tx_disc_packet, sizeof(discovery_packet));
			unicast_send(&unicast, &rx_disc_packet.src);

			if (!discovery_broadcast_done){
				// Set timer to add randomized delay to avoid collisions and then send the rebroadcast
				process_post(&routing_process,PROCESS_EVENT_MSG,0); // Post with data = 0
			}
		}
		// If this is a beacon broadcast coming from an established network,since current phase is 0 as well, this is a new node to be added
		else if(rx_disc_packet.disc_type == 1){
			// Check if it can be considered a neighbor by RSSI limit, ignore the packet if RSSI is lower than min
			if ((int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI) < MIN_NEIGHBOR_RSSI){
				printf("Low RSSI signal, not considered a neighbor!\n");
				return;
			}
			current_phase = 4; // New addition phase for a new node
			leds_off(LEDS_ALL);
			leds_on(LEDS_BLUE);
			node_dists = malloc(MAX_NEIGHBORS * sizeof(unsigned short));
			discovery_packet tx_disc_packet;
			linkaddr_copy(&tx_disc_packet.src, &linkaddr_node_addr);
			tx_disc_packet.disc_type = 2;
			tx_disc_packet.src_dist_to_gateway = 9999;
			packetbuf_copyfrom(&tx_disc_packet, sizeof(discovery_packet));
			broadcast_send(&broadcast);

			// Post to the process, set a timer there to wait for runicast responses,
			// and after it expires, find the best node and connect
			process_post(&routing_process,PROCESS_EVENT_MSG,0);
		}
	}
	// If current node is in sensor data transmit phase
	else if (current_phase == 2){
		// Check if it can be considered a neighbor by RSSI limit, ignore the packet if RSSI is lower than min
		if ((int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI) < MIN_NEIGHBOR_RSSI){
			printf("Low RSSI signal, not considered a neighbor!\n");
			return;
		}
		discovery_packet rx_disc_packet;
		packetbuf_copyto(&rx_disc_packet);
		// If this is a discovery msg from a new node to be added to an established network, respond with a runicast including cost
		if(rx_disc_packet.disc_type == 2){
			checkAddNeighbor(rx_disc_packet);
			wrapper_packet wpacket;
			discovery_packet tx_dpacket;
			linkaddr_copy(&tx_dpacket.src, &linkaddr_node_addr);
			tx_dpacket.disc_type = 2;
			tx_dpacket.src_dist_to_gateway = dist_to_gateway;
			wpacket.msgtype = 3;
			wpacket.msg.d = tx_dpacket;
			packetbuf_copyfrom(&wpacket, sizeof(wrapper_packet));
			runicast_send(&runicast, &rx_disc_packet.src,RUNICAST_MAX_RETRANSMIT);
		}

	}
	 // TODO: Add check responses
}


// Defines the behavior of a connection upon receiving data.
static void unicast_recv(struct unicast_conn *c, const linkaddr_t *from) {
	// if during the discovery phase:
	if (current_phase == 0){
		discovery_packet rx_disc_packet;
		packetbuf_copyto(&rx_disc_packet);
		// Save the src from these
		printf("Unicast message received from 0x%x%x [RSSI %d]\n",
				from->u8[0], from->u8[1],
				(int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI));

		// Check if it can be considered a neighbor by RSSI limit, ignore the packet if RSSI is lower than min
		if ((int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI) < MIN_NEIGHBOR_RSSI){
			printf("Low RSSI signal, not considered a neighbor!\n");
			return;
		}
		// Add to neighbors if not currently in the list
		checkAddNeighbor(rx_disc_packet);
	}
//	// If during the sensor data transmit phase, and sensor data packet
//	else if(current_phase == 2){
//		wrapper_packet rx_wpacket;
//		packetbuf_copyto(&rx_wpacket);
//
////		if(rx_wpacket.msgtype == 2){
////			// If this is the gateway node (Sensor data has reached the gateway node)
////			if(isGateway){
////				sensordata_packet rx_spacket;
////				rx_spacket = rx_wpacket.msg.s;
////				//printf("Received sensor data\”");
////				// Update the entry corresponding to the src node of the sensor data in sensordata_times
////				uint8_t srcindex = rx_spacket.src_ind;
////				sensordata_times[srcindex-1] = clock_seconds();
////				print_sensor_data(rx_spacket);
////			}else{
////				// If this is not the gateway, Route the signal toward the gateway
////				printf("Sensor data packet hopped\n");
////				linkaddr_t tx_addr = active_nlist[0];
////				packetbuf_copyfrom(&rx_wpacket, sizeof(wrapper_packet));
////				unicast_send(&unicast, &tx_addr);
////			}
////		}
//	}
}

static void recv_runicast(struct runicast_conn *c, const linkaddr_t *from, uint8_t seqno) {
	// Routing packets are send via runicast
	wrapper_packet rx_packet;
	packetbuf_copyto(&rx_packet);
	// If this is a neighbor info packet:
	if(rx_packet.msgtype == 0){
		if(isGateway){
			neighbor_data_packet npacket = rx_packet.msg.n;
			// Use the data in the adjacency matrix creation
			//printf("Gateway received neighbor table from 0x%x%x\n",npacket.src.u8[0],npacket.src.u8[1]);
			//print_neighbors_table(npacket.ntable, npacket.num_neighbors);

			update_adjacency_matrix(npacket.ntable, npacket.src, npacket.num_neighbors);
			//print_adjacency_matrix();
			// print_nodelist();
			// Set callback timer to end waiting for neighbor data packets, and calculate the optimal routes in the callback function
			process_post(&routing_process,PROCESS_EVENT_MSG,1); // Post with data = 1
		}else{
			// Send the incoming neighbor data towards the gateway by sending it to the first element
			// in the neighbors list since it is closest to the the gateway, or it is the gateway because of the delays in broadcasts.
			packetbuf_copyfrom(&rx_packet, sizeof(wrapper_packet));
			runicast_send(&runicast, &neighbors[0].addr,RUNICAST_MAX_RETRANSMIT);

		}
		// If this is a routing data packet from the gateway
	} else if(rx_packet.msgtype == 1){
		if((rx_packet.msg.r.hops_left == 0) && linkaddr_cmp(&rx_packet.msg.r.hops_list[0], &linkaddr_node_addr)){
			// Destination is reached
			routing_paths_packet rpacket = rx_packet.msg.r;

			//printf("Routing packet received.\n");

			memcpy(active_nlist, rpacket.nlist, sizeof(active_nlist));
			num_active_neighbors =	rpacket.num_active_neighbors;
			dist_to_gateway = rpacket.cost_to_gateway;

			node_ind = rpacket.node_index;
			//					printf("Next node to gateway is 0x%x%x \nNum. of active neighbors: %d \nLast neighbor addr:  0x%x%x \n",
			//												rpacket.nlist[0].u8[0],rpacket.nlist[0].u8[1], rpacket.num_active_neighbors,
			//												rpacket.nlist[rpacket.num_active_neighbors-1].u8[0], rpacket.nlist[rpacket.num_active_neighbors-1].u8[1]);
			printf("Next node to gateway is 0x%x%x \nNum. of active neighbors: %d \nLast neighbor addr:  0x%x%x \nNode index = %d\n",
					active_nlist[0].u8[0],active_nlist[0].u8[1], num_active_neighbors,
					active_nlist[num_active_neighbors-1].u8[0], active_nlist[num_active_neighbors-1].u8[1], rpacket.node_index);
			// Start the sensor data sending phase and the corresponding process
			//________________________________________________________________________

			current_phase = 2;
			process_start(&sensor_data_sending_process, NULL);

		}else if(rx_packet.msg.r.hops_left > 0){
			// If destination has not been reached, send it to the next hop
			rx_packet.msg.r.hops_left--;
			linkaddr_t next_hop = rx_packet.msg.r.hops_list[rx_packet.msg.r.hops_left];
			packetbuf_copyfrom(&rx_packet, sizeof(wrapper_packet));
			runicast_send(&runicast, &next_hop, RUNICAST_MAX_RETRANSMIT);
			//printf("Routing packet sent to its next hop (%d hops left)",rx_packet.msg.r.hops_left);
		}else{
			printf("\n\nThere is a problem with hops !!!\n\n");
		}

		// If sensor data packet is received:
	}else if(rx_packet.msgtype == 2){
		if(current_phase == 2){
			// If this is the gateway node (Sensor data has reached the gateway node)
			if(isGateway){
				sensordata_packet rx_spacket;
				rx_spacket = rx_packet.msg.s;
				//printf("Received sensor data\”");
				// Update the entry corresponding to the src node of the sensor data in sensordata_times
				uint8_t srcindex = getIdFromNodelist(rx_spacket.src_addr);
				sensordata_times[srcindex-1] = clock_seconds();
				print_sensor_data(rx_spacket);
			}else{
				// If this is not the gateway, Route the signal toward the gateway
				printf("Sensor data packet hopped\n");
				linkaddr_t tx_addr = active_nlist[0];
				packetbuf_copyfrom(&rx_packet, sizeof(wrapper_packet));
				runicast_send(&runicast, &tx_addr,RUNICAST_MAX_RETRANSMIT);
			}
		}
		// If this is a discovery response runicast for a node to be newly added to an established network
	}else if(rx_packet.msgtype == 3){
		discovery_packet dpacket = rx_packet.msg.d;
		checkAddNeighbor_withDist(dpacket);
	}
	//
	else if(rx_packet.msgtype == 4){
		if(isGateway){
			add_new_node_in_gateway(rx_packet.msg.newnode);
		}else{
			// If this is the node that the new node is connected to:
			if(linkaddr_cmp(&rx_packet.msg.newnode.conn_addr, &linkaddr_node_addr)){
				num_active_neighbors++;
				num_neighbors++;
				linkaddr_copy(&active_nlist[num_active_neighbors-1],&rx_packet.msg.newnode.src);
			}
			// Forward the packet toward the gateway:
			packetbuf_copyfrom(&rx_packet, sizeof(wrapper_packet));
			runicast_send(&runicast, &active_nlist[0], RUNICAST_MAX_RETRANSMIT);
		}
	}
	// If this is a distance update message
	else if(rx_packet.msgtype == 5){
		int16_t msg_rssi = (int16_t)packetbuf_attr(PACKETBUF_ATTR_RSSI);
		dist_to_gateway = rx_packet.msg.ud.dist + rssi_to_cost(msg_rssi);
		propagate_dist_to_tailnodes(dist_to_gateway);
	}
}
static void sent_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
	//printf("runicast message sent to successfully %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
}

static void timedout_runicast(struct runicast_conn *c, const linkaddr_t *to, uint8_t retransmissions){
	printf("ERROR: runicast message timed out when sending to %d.%d, retransmissions %d\n", to->u8[0], to->u8[1], retransmissions);
	if(current_phase == 2){
		dist_to_gateway = 9999;
		current_phase = 3; // Switched to failure recovery phase
		propagate_dist_to_tailnodes(9999);
		//process_exit(&sensor_data_sending_process);
		// Set up a timer to wait for the dist propagation to complete, and then send a discovery broadcast.
		// Need to process post to set up the timer, then end the sensor data sending process
		process_post(&sensor_data_sending_process,PROCESS_EVENT_MSG,0);
		//TODO: Failure recovery

	}
	leds_off(LEDS_ALL);
	leds_on(LEDS_RED);
}

// Defines the functions used as callbacks for connections.
static const struct unicast_callbacks unicast_call = {unicast_recv};

static const struct broadcast_callbacks broadcast_call = {broadcast_recv};

static const struct runicast_callbacks runicast_call = {recv_runicast,
                                                             sent_runicast,
                                                             timedout_runicast};
//------------------------ PROCESS' THREAD ------------------------
PROCESS_THREAD(routing_process, ev, data) {


	//PROCESS_EXITHANDLER(unicast_close(&unicast);)
	//PROCESS_EXITHANDLER(broadcast_close(&broadcast);)
	PROCESS_BEGIN();

	// Configure your team's channel (11 - 26).
	NETSTACK_CONF_RADIO.set_value(RADIO_PARAM_CHANNEL,15);

	/* Configure the user button */
	button_sensor.configure(BUTTON_SENSOR_CONFIG_TYPE_INTERVAL, CLOCK_SECOND);

	print_settings();
	check_for_invalid_addr();

	// Open unicast connection.
	unicast_open(&unicast, 130, &unicast_call);

	// Open broadcast connection.
	broadcast_open(&broadcast, 129, &broadcast_call);

	// Open runicast connection.
	runicast_open(&runicast, 131, &runicast_call);

	current_phase = 0; // Start with the discovery phase
	leds_off(LEDS_ALL);
	leds_on(23);


	discovery_broadcast_done = false;
	num_neighbors = 0;
	printf("Routing packet size: %d\n\n",sizeof(wrapper_packet));
	while(1) {
		PROCESS_WAIT_EVENT();

		if(ev == PROCESS_EVENT_MSG){
			if(data == 0){
				if(current_phase == 4){ // Set a timer to wait for runicast responses,
					// and after it expires, find the best node and connect
					etimer_set(&discovery_finished_timer, CLOCK_SECOND);

				}
				else{ // Regular wait to rebroadcast during initial network construction
					// Set timer to add randomized delay to avoid collisions and then send the rebroadcast
					etimer_set(&packet_delay_timer, CLOCK_SECOND*(0.7 + (0.2*random_rand()/RANDOM_RAND_MAX)));
				}
			}
			else if(data == 1){ // GATEWAY ONLY
				// Set timer to end waiting for neighbor data packets
				ctimer_set(&compute_routing_timer,5*CLOCK_SECOND,compute_routing_callback, NULL);
			}
		}
		else if(ev == PROCESS_EVENT_TIMER){
			// broadcast timer expired

			if (etimer_expired(&packet_delay_timer) && !discovery_broadcast_done){
				send_discovery_broadcast();
				// Set the timer for the end of discovery process for this node
				etimer_set(&discovery_finished_timer, 1.5*CLOCK_SECOND);
			} else if(etimer_expired(&discovery_finished_timer)){
				if(current_phase == 4){// Find the best node from the data in received runicast messages and connect to connect the new node
					find_best_node_connect();
				}
				else if(current_phase == 3){// Find the best node from the data in received runicast messages and connect for failure recovery
					find_best_node_connect();
				}
				else{
					//After some time from this node sends a discovery broadcast, we start the routing process,
					// and send the neighbors table towards the gateway node.
					current_phase = 1;
					printf("Routing phase started\n");
					leds_off(LEDS_ALL);
					leds_on(LEDS_BLUE);
					//Send only if this is not the gateway:
					if(!isGateway){
						send_neighbor_data();
					}
				}
			}

		}		// check if button was pressed
		else if(ev == sensors_event)
		{
			if(data == &button_sensor)
			{
				if( button_sensor.value(BUTTON_SENSOR_VALUE_TYPE_LEVEL) ==
						BUTTON_SENSOR_PRESSED_LEVEL ) {

					// Set up as gateway and generate first discovery packet:
					isGateway = true;
					dist_to_gateway = 0;
					// First node in the nodelist is the gateway itself
					linkaddr_copy(&nodelist[0], &linkaddr_node_addr);
					num_total_nodes = 1;
					send_discovery_broadcast();
					discovery_broadcast_done = true;
					// Allocate memory for the adjacency matrix only for the gateway			}
					adjmx = calloc(MAX_NODES, sizeof(unsigned short *));
					for (int i = 0; i < MAX_NODES; i++){
						adjmx[i] = calloc(MAX_NODES, sizeof(unsigned short));
					}
					num_total_nodes = MAX_NODES;
					//print_adjacency_matrix();
					num_total_nodes = 1;
					printf("status:ndisc\n");
					// Set the timer for the end of discovery process for the gateway node
					etimer_set(&discovery_finished_timer, 1.5*CLOCK_SECOND);

				}
			}
		}else if(ev == serial_line_event_message){
			//if(!strcmp(data,"start")){
				// Set up as gateway and generate first discovery packet:
				isGateway = true;
				dist_to_gateway = 0;
				// First node in the nodelist is the gateway itself
				linkaddr_copy(&nodelist[0], &linkaddr_node_addr);
				num_total_nodes = 1;
				send_discovery_broadcast();
				discovery_broadcast_done = true;
				// Allocate memory for the adjacency matrix only for the gateway			}
				adjmx = calloc(MAX_NODES, sizeof(unsigned short *));
				for (int i = 0; i < MAX_NODES; i++){
					adjmx[i] = calloc(MAX_NODES, sizeof(unsigned short));
				}
				num_total_nodes = MAX_NODES;
				//print_adjacency_matrix();
				printf("status:ndisc\n");
				printf("reboot\n");
				watchdog_reboot();
				num_total_nodes = 1;
				// Set the timer for the end of discovery process for the gateway node
				etimer_set(&discovery_finished_timer, 1.5*CLOCK_SECOND);

			//}
		}
	}

	PROCESS_END();
}

PROCESS_THREAD(sensor_data_sending_process, ev, data) {
	PROCESS_BEGIN();
	leds_off(LEDS_ALL);
	leds_on(LEDS_GREEN);
	/* Configure the ADC port for the vibration sensor */
	configure_node_sensors();
	// Set up the timer to periodically send data to the gateway
	etimer_set(&sensor_send_timer, 2*CLOCK_SECOND);

	// Set up the timer to periodically send beacon broadcasts to look for new nodes
	ctimer_set(&beacon_send_timer,5*CLOCK_SECOND,&beacon_send_callback, NULL);


	while(1) {
		PROCESS_WAIT_EVENT();
		if(ev == PROCESS_EVENT_TIMER){
			if(etimer_expired(&sensor_send_timer)){
//				// Get the sensor data, form it into a packet and send it
				sensordata_packet tx_spacket;
				tx_spacket = create_sensor_packet();
				send_sensor_data_packet(tx_spacket);

				// Reset the timer
				etimer_reset(&sensor_send_timer);
			}
		}
		else if(ev == PROCESS_EVENT_MSG){
			if(data == 0){
				if(current_phase == 3){ // If failure recovery process
					// Set up a timer to wait for the infinity distance to propagate towards tail,
					// then send a broadcast like we are adding a new node
					ctimer_set(&reconnect_broadcast_send_timer, CLOCK_SECOND, &reconnect_broadcast_send_callback, NULL);
					// End the sensor data process until we can reroute
					process_exit(&sensor_data_sending_process);
				}
			}
		}
	}
	PROCESS_END();
}

PROCESS_THREAD(sensor_process_gateway, ev, data) {
	PROCESS_BEGIN();
	leds_off(LEDS_ALL);
	leds_on(LEDS_GREEN);
	current_phase = 2; // Start the sensor data sending phase for the gateway node
	printf("\nstatus:sensor_receive\n");
	// Allocate and initialize the array for holding timings to use to detect
	// how much time has passed since we last received sensor data from each node.
	// Indexes are shifted by 1 since gateway node does not send sensor data (index 0 is node 1)
	sensordata_times = malloc((MAX_NODES - 1) * sizeof(unsigned int));
	// Initialize:
	for(int i=0;i<num_total_nodes-1;i++){
		sensordata_times[i] = clock_seconds();
	}

	// Print the prev_nodes array:
	printf("\nroutingdata:prevnodes =");
	printf("%d",prev_nodes[0]);
		for (int i = 1; i < num_total_nodes; i++)
			printf(",%d",prev_nodes[i]);
		printf("\n");
	// Set up timer to check the time past since a sensor data packet is received from each node
	ctimer_set(&check_node_alive_timer, 4*CLOCK_SECOND,check_node_alive_callback, NULL);

	// Set up the timer to periodically send beacon broadcasts to look for new nodes
	ctimer_set(&beacon_send_timer,5*CLOCK_SECOND,&beacon_send_callback, NULL);

	while(1) {
		PROCESS_WAIT_EVENT();
		if(ev == PROCESS_EVENT_MSG){
			printf("sdsd\n");
		}
	}
	PROCESS_END();
}
