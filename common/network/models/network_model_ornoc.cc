//network_model_ornoc.cc
//el ORNoC v.0

#include <math.h>
#include <vector>
//#include <iostream>
//#include <fstream>
using namespace std;

#include "network_model_ornoc.h"
#include "simulator.h"
#include "tile_manager.h"
#include "tile.h"
#include "config.h"
#include "log.h"
#include "utils.h"

//// static variables
bool NetworkModelOrnoc::initialized = false;

// enet
SInt32 NetworkModelOrnoc::enet_width_per_layer;
SInt32 NetworkModelOrnoc::enet_height_per_layer;

// clusters
SInt32 NetworkModelOrnoc::num_clusters;
SInt32 NetworkModelOrnoc::num_clusters_per_layer;
SInt32 NetworkModelOrnoc::num_tiles_per_layer;
SInt32 NetworkModelOrnoc::cluster_size;
SInt32 NetworkModelOrnoc::numX_clusters_per_layer;
SInt32 NetworkModelOrnoc::numY_clusters_per_layer;
SInt32 NetworkModelOrnoc::cluster_width_per_layer;
SInt32 NetworkModelOrnoc::cluster_height_per_layer;

// sub clusters
SInt32 NetworkModelOrnoc::num_access_points_per_cluster;
SInt32 NetworkModelOrnoc::num_sub_clusters_per_layer;
SInt32 NetworkModelOrnoc::numX_sub_clusters_per_layer;
SInt32 NetworkModelOrnoc::numY_sub_clusters_per_layer;
SInt32 NetworkModelOrnoc::sub_cluster_width_per_layer;
SInt32 NetworkModelOrnoc::sub_cluster_height_per_layer;

vector<NetworkModelOrnoc::Ring> NetworkModelOrnoc::rings;
vector<NetworkModelOrnoc::ClusterInfo> NetworkModelOrnoc::cluster_info_list;
NetworkModelOrnoc::ReceiveNetworkType NetworkModelOrnoc::receive_network_type;
SInt32 NetworkModelOrnoc::num_receive_networks_per_cluster;
SInt32 NetworkModelOrnoc::max_wavelengths_per_ring;
NetworkModelOrnoc::RoutingStrategy NetworkModelOrnoc::routing_strategy;
SInt32 NetworkModelOrnoc::unicast_distance_threshold;
bool NetworkModelOrnoc::contention_model_enabled;
bool NetworkModelOrnoc::verbose_output;

vector< vector<bool> > NetworkModelOrnoc::connectvitiy_matrix;
string NetworkModelOrnoc::connectivity_matrix_path;
SInt32 NetworkModelOrnoc::num_layers;

NetworkModelOrnoc::Ring::Ring(ring_id_t id_, bool clockwise_, SInt32 num_portions_, SInt32 num_wavelengths_)
{
	   id = id_;
	   clockwise = clockwise_;
	   portions.resize(num_portions_);

	   for (auto it = portions.begin(); it != portions.end(); ++it){
		   Portion& p = *it;
		   p.source_cluster_id = it - portions.begin();
		   p.target_cluster_id = (p.source_cluster_id + 1) % num_portions_;

		   for (SInt32 i = 0; i != num_wavelengths_; i++){
			   Wavelength wl = Wavelength(i, false);
			   p.wavelengths.push_back(wl);
		   }
	   }
}

NetworkModelOrnoc::NetworkModelOrnoc(Network *net, SInt32 network_id)
   : NetworkModel(net, network_id)
{
	try
	{
		_flit_width = Sim()->getCfg()->getInt("network/ornoc/flit_width");
	}
	catch (...)
	{
		 LOG_PRINT_ERROR("Could not read ORNoC frequency and flit_width parameters from cfg file");
	}

	_has_broadcast_capability = false;

	// initialize ring network topology parameters
	initializeRNetTopologyParams();

	// initialize enet, onet, and receiver network parameters
	createRNetRouterAndLinkModels();

}

NetworkModelOrnoc::~NetworkModelOrnoc()
{
	destroyRNetRouterAndLinkModels();
}

void NetworkModelOrnoc::initializeRNetTopologyParams()
{
	   if (initialized)
	      return;
	   initialized = true;

	   SInt32 num_application_tiles = Config::getSingleton()->getApplicationTiles();

	   try
	   {
		  cluster_size = Sim()->getCfg()->getInt("network/ornoc/cluster_size");
		  num_access_points_per_cluster = Sim()->getCfg()->getInt("network/ornoc/num_optical_access_points_per_cluster");
		  receive_network_type = parseReceiveNetType(Sim()->getCfg()->getString("network/ornoc/receive_network_type"));
		  num_receive_networks_per_cluster = Sim()->getCfg()->getInt("network/ornoc/num_receive_networks_per_cluster");
		  max_wavelengths_per_ring = Sim()->getCfg()->getInt("network/ornoc/max_wavelengths_per_ring");
		  routing_strategy = parseRoutingStrategy(Sim()->getCfg()->getString("network/ornoc/global_routing_strategy"));
		  unicast_distance_threshold = Sim()->getCfg()->getInt("network/ornoc/unicast_distance_threshold");
		  num_layers = Sim()->getCfg()->getInt("network/ornoc/num_layers");
		  contention_model_enabled = Sim()->getCfg()->getBool("network/ornoc/queue_model/enabled");
		  connectivity_matrix_path = Sim()->getCfg()->getString("network/ornoc/connectivity_matrix_path");
		  verbose_output = Sim()->getCfg()->getBool("network/ornoc/verbose");
	   }
	   catch (...)
	   {
		   LOG_PRINT_ERROR("Error reading ornoc parameters");
	   }

	   LOG_ASSERT_ERROR(num_access_points_per_cluster <= cluster_size,
	         "Num optical access points per cluster(%i) must be less than or equal to cluster size(%i)",
	         num_access_points_per_cluster, cluster_size);
	   LOG_ASSERT_ERROR(isPerfectSquare(num_application_tiles),
	         "Num Application Tiles(%i) must be a perfect square", num_application_tiles);
	   LOG_ASSERT_ERROR(isPower2(num_application_tiles),
	         "Num Application Tiles(%i) _num_sub_clustersmust be a power of 2", num_application_tiles);
	   LOG_ASSERT_ERROR(isPower2(cluster_size),
	         "Cluster Size(%i) must be a power of 2", cluster_size);
	   LOG_ASSERT_ERROR((num_application_tiles % cluster_size) == 0,
	         "Num Application Tiles(%i) must be a multiple of Cluster Size(%i)", num_application_tiles, cluster_size);
	   LOG_ASSERT_ERROR((num_clusters % num_layers) == 0,
	   	         "Num Clusters(%i) must be a multiple of Num Layers(%i)", num_application_tiles, cluster_size);

	   num_clusters = num_application_tiles / cluster_size;
	   LOG_ASSERT_ERROR(num_clusters > 1, "Number of Clusters(%i) must be > 1", num_clusters);

	   num_clusters_per_layer = num_clusters / num_layers;
	   LOG_ASSERT_ERROR(num_clusters_per_layer > 0, "Number of Clusters(%i) must be > 0", num_clusters);

	   num_tiles_per_layer = num_clusters_per_layer * cluster_size;

	   num_sub_clusters_per_layer = num_access_points_per_cluster;
	   LOG_ASSERT_ERROR(isPower2(num_access_points_per_cluster),
	         "Number of Optical Access Points(%i) must be a power of 2", num_access_points_per_cluster);

	   initializeClusters();
}

void NetworkModelOrnoc::createRNetRouterAndLinkModels()
{
   if (isSystemTile(_tile_id))
	  return;

  // enet router
  UInt64 enet_router_delay = 0;
  UInt32 num_flits_per_output_buffer_enet_router = 0;

  // send hub router
  UInt64 send_hub_router_delay = 0;
  UInt32 num_flits_per_output_buffer_send_hub_router = 0;

  // receive hub router
  UInt64 receive_hub_router_delay = 0;
  UInt32 num_flits_per_output_buffer_receive_hub_router = 0;

  // star net router
  //UInt64 star_net_router_delay = 0;
  //UInt32 num_flits_per_output_buffer_star_net_router = 0;

  // electrical link type
  string electrical_link_type;

  // contention model
  string contention_model_type;

  try
  {
	 // enet router
	 enet_router_delay = (UInt64) Sim()->getCfg()->getInt("network/ornoc/enet/router/delay");
	 num_flits_per_output_buffer_enet_router = Sim()->getCfg()->getInt("network/ornoc/enet/router/num_flits_per_port_buffer");

	 // send hub router
	 send_hub_router_delay = (UInt64) Sim()->getCfg()->getInt("network/ornoc/onet/send_hub/router/delay");
	 num_flits_per_output_buffer_send_hub_router = Sim()->getCfg()->getInt("network/ornoc/onet/send_hub/router/num_flits_per_port_buffer");

	 // receive hub router
	 receive_hub_router_delay = (UInt64) Sim()->getCfg()->getInt("network/ornoc/onet/receive_hub/router/delay");
	 num_flits_per_output_buffer_receive_hub_router = Sim()->getCfg()->getInt("network/ornoc/onet/receive_hub/router/num_flits_per_port_buffer");

	 // star net router
	 //star_net_router_delay = (UInt64) Sim()->getCfg()->getInt("network/ornoc/star_net/router/delay");
	 //num_flits_per_output_buffer_star_net_router = Sim()->getCfg()->getInt("network/ornoc/star_net/router/num_flits_per_port_buffer");

	 // electrical link type
	 electrical_link_type = Sim()->getCfg()->getString("network/ornoc/electrical_link_type");

	 contention_model_type = Sim()->getCfg()->getString("network/ornoc/queue_model/type");
  }
  catch (...)
  {
	 LOG_PRINT_ERROR("Could not read ANet router and link parameters from the cfg file");
  }


  // injection port
  injection_router = new RouterModel(this, _frequency, _voltage,
                                      1, 1,
                                      4, 0, _flit_width,
                                      contention_model_enabled, contention_model_type);

  // enet router
  // typical mesh router has 4 IO ports for north-south-east-west and one more IO for processor
  num_enet_router_ports = 5;

  SInt32 num_enet_router_output_ports = num_enet_router_ports;
  if (isAccessPoint(_tile_id))
     num_enet_router_output_ports += 1;

  enet_router = new RouterModel(this, _frequency, _voltage,
                                 num_enet_router_ports, num_enet_router_output_ports,
                                 num_flits_per_output_buffer_enet_router, enet_router_delay, _flit_width,
                                 contention_model_enabled, contention_model_type);

  // idle, unicast and broadcast
  // enet link
  double enet_link_length = _tile_width;
  enet_link_list.resize(num_enet_router_output_ports);
  for (SInt32 i = 0; i < num_enet_router_output_ports; i++){
     enet_link_list[i] = new ElectricalLinkModel(this, electrical_link_type,
                                                  _frequency, _voltage,
                                                  enet_link_length, _flit_width);
     assert(enet_link_list[i]->getDelay() == 1);
  }

  // instantiate optical send and receive routers
  cluster_id_t cluster_id = getClusterID(_tile_id);

  if (_tile_id == getTileIDWithOpticalHub(cluster_id)) {

	  SInt32 num_send_ports = getNumOniSendPorts(cluster_id);
	  SInt32 num_receive_ports = getNumOniReceivePorts(cluster_id);

      send_hub_router = new RouterModel(this, _frequency, _voltage,
                                         num_access_points_per_cluster, num_send_ports,
                                         num_flits_per_output_buffer_send_hub_router, send_hub_router_delay, _flit_width,
                                         contention_model_enabled, contention_model_type);

      // optical link power models
      optical_links.resize(num_send_ports);
      double waveguide_length = computeOpticalLinkLength(); // in mm

      for (SInt32 i = 0; i < num_send_ports; i++) {
    	  optical_links[i] = new OpticalLinkModel(this, num_clusters_per_layer, _frequency, _voltage, waveguide_length, _flit_width);
    	  LOG_ASSERT_ERROR(optical_links[i]->getDelay() == 3, "Optical link delay should be 3. Now %llu", optical_links[i]->getDelay());
      }

      // receive hub router models
      receive_hub_router = new RouterModel(this, _frequency, _voltage,
    		  	  	  	  	  	  	  	   num_receive_ports, num_receive_networks_per_cluster,
                                           num_flits_per_output_buffer_receive_hub_router, receive_hub_router_delay,
                                           _flit_width,
                                           contention_model_enabled, contention_model_type);

      // receive network
      if (receive_network_type == BROADCAST) {
    	  double btree_link_length = _tile_width * cluster_size;
    	  btree_link_list.resize(num_receive_networks_per_cluster);
		  for (SInt32 i = 0; i < num_receive_networks_per_cluster; i++) {
			 btree_link_list[i] = new ElectricalLinkModel(this, electrical_link_type,
														   _frequency, _voltage,
														   btree_link_length, _flit_width);
			 assert(btree_link_list[i]->getDelay() == 1);
		  }
      }
      // TODO: finish -> star network

  } // tile_id with optical hub

}


void NetworkModelOrnoc::destroyRNetRouterAndLinkModels()
{
	//TODO: ensure everything get cleaned up
	if (isSystemTile(_tile_id))
	  return;

   delete injection_router;
   delete enet_router;

   for (SInt32 i = 0; i < num_enet_router_ports; i++)
	  delete enet_link_list[i];

   if (isAccessPoint(_tile_id))
	  delete enet_link_list[num_enet_router_ports];

   if (_tile_id == getTileIDWithOpticalHub(getClusterID(_tile_id))) {
	  delete send_hub_router;

	  for (auto it = optical_links.begin(); it != optical_links.end(); it++) {
		  delete (*it);
	  }

	  delete receive_hub_router;

	  // receive network
	  if (receive_network_type == BROADCAST) {
		 for (SInt32 i = 0; i < num_receive_networks_per_cluster; i++)
			delete btree_link_list[i];
	  }
	  else {
// TODO

//		 for (SInt32 i = 0; i < num_receive_networks_per_cluster; i++) {
//			// Star Net Router
//			delete _star_net_router_list[i];
//			// Star Net Links
//			for (SInt32 j = 0; j < _cluster_size; j++)
//			   delete _star_net_link_list[i][j];
//		 }
	  }
   }
}

void NetworkModelOrnoc::routePacket(const NetPacket& pkt, queue<Hop>& next_hops)
{
	   tile_id_t pkt_sender = TILE_ID(pkt.sender);
	   tile_id_t pkt_receiver = TILE_ID(pkt.receiver);

	   if (pkt.node_type == SEND_TILE) {
	      assert(pkt_sender == _tile_id);

	      UInt64 zero_load_delay = 0;
	      UInt64 contention_delay = 0;
	      injection_router->processPacket(pkt, 0, zero_load_delay, contention_delay);

	      Hop hop(pkt, _tile_id, EMESH, Latency(zero_load_delay,_frequency), Latency(contention_delay,_frequency));
	      next_hops.push(hop);
	   }
	   else {
	      Route computed_route = computeGlobalRoute(pkt_sender, pkt_receiver);
	      if (computed_route == ENET) {
	         LOG_PRINT("Route: ENET");
	         routePacketOnENet(pkt, pkt_sender, pkt_receiver, next_hops);
	      }
	      else if (computed_route == ONET) {
	         LOG_PRINT("Route: ONET");
	         routePacketOnONet(pkt, pkt_sender, pkt_receiver, next_hops);
	      }
	   }
}

NetworkModelOrnoc::Route NetworkModelOrnoc::computeGlobalRoute(tile_id_t sender, tile_id_t receiver)
{
	//determine type of communication required between sender and receiver

   if (receiver == NetPacket::BROADCAST)
	  return ONET; // should not happen with ORNoC


   if (getLayerID(sender) == getLayerID(receiver)) {
	  if (getClusterID(sender) == getClusterID(receiver))
		  return ENET;
	  else if (routing_strategy == ISOLATED_CLUSTERS)
		  //TODO: for now any layer communication is all ENET
		  return ENET;
	  else {
		  // TODO
		  // routing_strategy == distance_based
		  // for now just ENET
		 // SInt32 num_hops_on_enet = computeNumHopsOnENet(sender, receiver);
		 //return (num_hops_on_enet <= unicast_distance_threshold) ? ENET : ONET;
		  return ENET;
	  }
   }
   else {
	 return ONET;
   }
}

void NetworkModelOrnoc::routePacketOnENet(const NetPacket& pkt, tile_id_t sender, tile_id_t receiver, queue<Hop>& next_hops)
{
   LOG_ASSERT_ERROR(receiver != NetPacket::BROADCAST, "Cannot broadcast packets on ENet");

   SInt32 cx, cy, cl, dx, dy, dl;
   computePosition(_tile_id, cx, cy, cl);
   computePosition(receiver, dx, dy, dl);

   LOG_ASSERT_ERROR(cl == dl, "ENet communication cannot take place across layers.");

   NextDest next_dest;

   if (cx > dx)
	  next_dest = NextDest(computeTileID(cx-1, cy, cl), LEFT, EMESH);
   else if (cx < dx)
	  next_dest = NextDest(computeTileID(cx+1, cy, cl), RIGHT, EMESH);
   else if (cy > dy)
	  next_dest = NextDest(computeTileID(cx, cy-1, cl), DOWN, EMESH);
   else if (cy < dy)
	  next_dest = NextDest(computeTileID(cx, cy+1, cl), UP, EMESH);
   else // (cx == dx) && (cy == dy)
	  next_dest = NextDest(_tile_id, SELF, RECEIVE_TILE);

   UInt64 zero_load_delay = 0;
   UInt64 contention_delay = 0;

   // go through router and link
   assert(0 <= next_dest._output_port && next_dest._output_port < (SInt32) enet_link_list.size());

   enet_router->processPacket(pkt, next_dest._output_port, zero_load_delay, contention_delay);
   enet_link_list[next_dest._output_port]->processPacket(pkt, zero_load_delay);

   Hop hop(pkt, next_dest._tile_id, next_dest._node_type, Latency(zero_load_delay,_frequency), Latency(contention_delay,_frequency));
   next_hops.push(hop);
}

void NetworkModelOrnoc::routePacketOnONet(const NetPacket& pkt, tile_id_t sender, tile_id_t receiver, queue<Hop>& next_hops)
{

   // packet must be routed to access point via emesh first
   if (pkt.node_type == EMESH) {
	   // make sure we are within the same cluster
	  assert(getClusterID(sender) == getClusterID(_tile_id));

	  if (isAccessPoint(_tile_id)) {
		 UInt64 zero_load_delay = 0;
		 UInt64 contention_delay = 0;

		 enet_router->processPacket(pkt, num_enet_router_ports, zero_load_delay, contention_delay);
		 enet_link_list[num_enet_router_ports]->processPacket(pkt, zero_load_delay);

		 Hop hop(pkt, getTileIDWithOpticalHub(getClusterID(_tile_id)), SEND_HUB, Latency(zero_load_delay,_frequency), Latency(contention_delay,_frequency));
		 next_hops.push(hop);
	  }
	  else {
		 tile_id_t access_point = getNearestAccessPoint(sender);
		 routePacketOnENet(pkt, sender, access_point, next_hops);
	  }
   }
   // after packet has been routed to acess point on ONet
   else if (pkt.node_type == SEND_HUB) {

	 // broadcast mode is not used in ORNoC
	 assert(receiver != NetPacket::BROADCAST);

	 UInt64 zero_load_delay = 0;
	 UInt64 contention_delay = 0;
	 NextDest next_dest;
	 vector<Ring::Portion> source_portions, receive_portions;
	 SInt32 cx, cy, cl, dx, dy, dl,
	 	 	this_cluster_id, dest_cluster_id, direct_idx;
	 direct_idx = -1;

	 computePosition(_tile_id, cx, cy, cl);
	 computePosition(receiver, dx, dy, dl);

	 this_cluster_id = getClusterID(_tile_id);
	 dest_cluster_id = getClusterID(receiver);

	 getClusterSendPortions(this_cluster_id, source_portions);
	 //getClusterReceivePortions(this_cluster_id, receive_portions);

	 if(cl == dl) {
		 // TODO: consider optical path on same layer
		 //       currently restricted
		 assert(cl != dl);
	 }
	 else {
		 // pairs of clusters in destination layer with associated send port
		 vector<pair<cluster_id_t, SInt32>> dest_clusters;

		 // obtain all destination layer's clusters ids
		 for (auto pit = source_portions.begin(); pit != source_portions.end(); pit++) {
			 if (getLayerID((*pit).target_cluster_id) == dl) {
				 SInt32 idx = distance(source_portions.begin(), pit);
				 dest_clusters.push_back(make_pair((*pit).target_cluster_id, idx));

				 // check for direct paths
				 if (dest_cluster_id == (*pit).target_cluster_id) {
					 direct_idx = idx;
					 break;
				 }
			 }
		 }

		 // there should be at least one destination cluster - user's responsibility
		 assert(dest_clusters.size() > 0);

		 // TODO: currently takes first available cluster on other layer - in future determine most efficient path
		 //       ^ in future will use first part of pair in dest_clusters
		 SInt32 outputPort = (direct_idx >= 0 ? dest_clusters[direct_idx].second : dest_clusters[0].second);

		 send_hub_router->processPacket(pkt, outputPort, zero_load_delay, contention_delay);
		 optical_links[outputPort]->processPacket(pkt, 1, zero_load_delay);
		 Hop hop(pkt, getTileIDWithOpticalHub(getClusterID(receiver)), RECEIVE_HUB, Latency(zero_load_delay,_frequency), Latency(contention_delay,_frequency));
		 next_hops.push(hop);

	 }
   }
   else if (pkt.node_type == RECEIVE_HUB) {
	  vector<tile_id_t> tile_id_list;
	  getTileIDListInCluster(getClusterID(_tile_id), tile_id_list);
	  assert(cluster_size == (SInt32) tile_id_list.size());

	  // TODO: why using sender?
	  SInt32 receive_net_id = computeReceiveNetID(sender);

	  UInt64 zero_load_delay = 0;
	  UInt64 contention_delay = 0;

	  // receive Hub Router
	  // update router event counters, get delay, update dynamic energy
	  receive_hub_router->processPacket(pkt, receive_net_id, zero_load_delay, contention_delay);

	  // for now just assume direct broadcast tree
	  // update link counters, get delay, update dynamic energy
	  btree_link_list[receive_net_id]->processPacket(pkt, zero_load_delay);

	  // for now no broadcast to all tiles
		 Hop hop(pkt, receiver, RECEIVE_TILE, Latency(zero_load_delay,_frequency), Latency(contention_delay,_frequency));
		 next_hops.push(hop);
   }
   else
   {
	  LOG_PRINT_ERROR("Unrecognized Node Type(%i)", pkt.node_type);
   }
}

//TODO
SInt32 NetworkModelOrnoc::computeNumHopsOnENet(tile_id_t sender, tile_id_t receiver)
{
	// ONLY NEEDED FOR DISTANCE DETERMINATION
	// NOT IMPLEMENTED YET
	return 0;
}

tile_id_t NetworkModelOrnoc::computeTileID(SInt32 x, SInt32 y, SInt32 l)
{
   if ((x < 0) || (y < 0) || (x >= enet_width_per_layer) || (y >= enet_height_per_layer))
      return INVALID_TILE_ID;
   else
      return ((y * enet_width_per_layer + x) + (l * num_tiles_per_layer));
}
SInt32 NetworkModelOrnoc::computeReceiveNetID(tile_id_t sender)
{
   SInt32 sending_cluster_id = getClusterID(sender);
   return (sending_cluster_id % num_receive_networks_per_cluster); // "random-ish"
}

void NetworkModelOrnoc::initializeClusters()
{
	   cluster_info_list.resize(num_clusters);

	   if (isEven(floorLog2(num_clusters_per_layer))) {
		   numX_clusters_per_layer = sqrt(num_clusters_per_layer);
		   numY_clusters_per_layer = sqrt(num_clusters_per_layer);
	   }
	   else {
		   numX_clusters_per_layer = sqrt(num_clusters_per_layer / 2);
		   numY_clusters_per_layer = sqrt(num_clusters_per_layer * 2);
	   }

	   if (isEven(floorLog2(cluster_size))) {
		   cluster_width_per_layer = sqrt(cluster_size);
		   cluster_height_per_layer = sqrt(cluster_size);
	   }
	   else {
		   cluster_width_per_layer = sqrt(cluster_size / 2);
		   cluster_height_per_layer = sqrt(cluster_size * 2);
	   }

	   enet_width_per_layer = numX_clusters_per_layer * cluster_width_per_layer;
	   enet_height_per_layer = numY_clusters_per_layer * cluster_height_per_layer;

	   // initialize cluster ids and boundaries
	   for (SInt32 l = 0; l < num_layers; l++) {
		   for (SInt32 y = 0; y < numY_clusters_per_layer; y++) {
		      for (SInt32 x = 0; x < numX_clusters_per_layer; x++) {

		         cluster_id_t cluster_id = (l * num_clusters_per_layer) + (y * numX_clusters_per_layer) + x;
		         ClusterInfo::Boundary _boundary(x * cluster_width_per_layer, (x + 1) * cluster_width_per_layer,
		                                        y * cluster_height_per_layer, (y + 1) * cluster_height_per_layer);
		         cluster_info_list[cluster_id].boundary = _boundary;
		         cluster_info_list[cluster_id].layer_id = l;
		      }
		   }
	   }

	   // sub clusters

	   if (isEven(floorLog2(num_sub_clusters_per_layer))){
	      numX_sub_clusters_per_layer = sqrt(num_sub_clusters_per_layer);
	      numY_sub_clusters_per_layer = sqrt(num_sub_clusters_per_layer);
	   }
	   else{
	      numX_sub_clusters_per_layer = sqrt(num_sub_clusters_per_layer * 2);
	      numY_sub_clusters_per_layer = sqrt(num_sub_clusters_per_layer / 2);
	   }
	   sub_cluster_width_per_layer = cluster_width_per_layer / numX_sub_clusters_per_layer;
	   sub_cluster_height_per_layer = cluster_height_per_layer / numY_sub_clusters_per_layer;

	   // initialize access point list
	   for (SInt32 i = 0; i < num_clusters; i++){
	      initializeAccessPointList(i);
	   }

	   // retrieve connectivity matrix from file
	   string homepath = getenv("HOME");
	   ifstream connectivity_fs((homepath + "/" + connectivity_matrix_path).c_str());
	   string line;

	   while(getline(connectivity_fs, line)){
		   vector<bool> row;

		   if (line.size() != (UInt32)num_clusters)
			   break;

		   for(string::size_type i = 0; i < line.size(); i++){
			   row.push_back(line[i] == '1');
		   }
		   connectvitiy_matrix.push_back(row);
	   }

	   connectivity_fs.close();

	   LOG_ASSERT_ERROR(connectvitiy_matrix.size() == (unsigned)num_clusters,
	   	         "Number of rows in connectivity matrix file (%i) must be equal to cluster size (%i)",
	   	      connectvitiy_matrix.size(), num_clusters);

	   // "stupid" algorithm (first come, first serve )
	   rings.clear();
	   rings.push_back(Ring(0, true, num_clusters, max_wavelengths_per_ring));

	   for(UInt32 i = 0; i < connectvitiy_matrix.size(); i++){

		   for(UInt32 j = 0; j < connectvitiy_matrix[i].size(); j++){

			   if(connectvitiy_matrix[i][j]){

				   for(UInt32 r = 0; r != rings.size(); ++r){

					   SInt32 wl = availableWavelength(rings[r].portions, i, j, rings[r].clockwise);

					   if ( (wl < 0) && (r == (rings.size() - 1)) ){
						   // create new ring if there is no available connection in any existing ring
						   ring_id_t ring_id = rings.size();
						   rings.push_back(Ring(ring_id, isEven(ring_id), num_clusters, max_wavelengths_per_ring));
					   }

					   if (wl > -1) {
						   UInt32 idx, eidx;
						   idx = (rings[r].clockwise) ? i : j;
						   eidx = (rings[r].clockwise) ? j : i;
						   while(idx != eidx){
							   rings[r].portions[idx].wavelengths[wl].reserved = true;
							   idx = (idx + 1) % num_clusters;
						   }
						   break;
					   }
				   }
			   }
		   }
	   }

	   // details printout
	   if (verbose_output){
		   cout << endl << "ORNoC structure:" << endl;
		   for (auto rit = rings.begin(); rit != rings.end(); rit++){
			   auto &r = *rit;
			   cout << "Ring ID: " << r.id << endl;
			   cout << "Portions:" << endl;

			   for (auto pit = r.portions.begin(); pit != r.portions.end(); pit++){
				   auto &p = *pit;
				   cout << "	Portion # " << (pit - r.portions.begin()) << endl;
				   cout << "		Reserved wavelengths: ";
				   for (auto wit = p.wavelengths.begin(); wit != p.wavelengths.end(); wit++){
					   if ((*wit).reserved)
						   cout << (*wit).wavelength_id << " ";
				   }
				   cout << endl;
			   }
		   }
	   }
}

SInt32 NetworkModelOrnoc::availableWavelength(vector<Ring::Portion>& portions, SInt32 source_id, SInt32 target_id, bool clockwise)
{
	for (SInt32 i = 0; i < max_wavelengths_per_ring; i++)
	{
		bool available = true;
		SInt32 idx, eidx;

		if (clockwise){
			idx = source_id;
			eidx = target_id;
		}else{
			idx = target_id;
			eidx = source_id;
		}

		while (idx != eidx)
		{
			if (portions[idx].wavelengths[i].reserved)
			{
				available = false;
				break;
			}
			idx = (idx + 1) % num_clusters;
		}
		if (available)
			return i;
	}
	return -1;
}

void NetworkModelOrnoc::initializeAccessPointList(SInt32 cluster_id)
{
   ClusterInfo::Boundary& boundary = cluster_info_list[cluster_id].boundary;
   for (SInt32 y = 0; y < numY_sub_clusters_per_layer; y++){
	  for (SInt32 x = 0; x < numX_sub_clusters_per_layer; x++){
		 SInt32 access_point_x = boundary.minX + (x * sub_cluster_width_per_layer) + (sub_cluster_width_per_layer / 2);
		 SInt32 access_point_y = boundary.minY + (y * sub_cluster_height_per_layer) + (sub_cluster_height_per_layer / 2);
		 tile_id_t access_point_id = access_point_y * enet_width_per_layer + access_point_x;
		 cluster_info_list[cluster_id].access_point_list.push_back(access_point_id);
	  }
   }
}

NetworkModelOrnoc::ReceiveNetworkType NetworkModelOrnoc::parseReceiveNetType(string str)
{
   if (str == "btree")
	  return BROADCAST;
   else if (str == "star")
	  return STAR_ROUTER;
   else {
	  LOG_PRINT_ERROR("Unrecognized Receive Net Type(%s)", str.c_str());
	  return (ReceiveNetworkType) -1;
   }
}

NetworkModelOrnoc::RoutingStrategy NetworkModelOrnoc::parseRoutingStrategy(string strategy)
{
	// TODO
	return ISOLATED_CLUSTERS;
}

bool NetworkModelOrnoc::isAccessPoint(tile_id_t tile_id)
{
	return (tile_id == getNearestAccessPoint(tile_id));
}

tile_id_t NetworkModelOrnoc::getNearestAccessPoint(tile_id_t tile_id)
{
   SInt32 cluster_id = getClusterID(tile_id);
   SInt32 sub_cluster_id = getSubClusterID(tile_id);
   return cluster_info_list[cluster_id].access_point_list[sub_cluster_id];
}

tile_id_t NetworkModelOrnoc::getTileIDWithOpticalHub(SInt32 cluster_id)
{
   // consider a mesh formed by the clusters
   SInt32 layer_id;
   layer_id = cluster_id / num_clusters_per_layer;

   SInt32 cluster_pos_x, cluster_pos_y;
   cluster_pos_x = cluster_id % cluster_width_per_layer;
   cluster_pos_y = (cluster_id % num_clusters_per_layer) / cluster_width_per_layer;

   SInt32 optical_hub_x, optical_hub_y;
   optical_hub_x = (cluster_pos_x * cluster_width_per_layer) + (cluster_width_per_layer / 2);
   optical_hub_y = (cluster_pos_y * cluster_height_per_layer) + (cluster_height_per_layer / 2);

   return ((optical_hub_y * enet_width_per_layer + optical_hub_x) + (layer_id * num_tiles_per_layer));
}

void
NetworkModelOrnoc::getTileIDListInCluster(SInt32 cluster_id, vector<tile_id_t>& tile_id_list)
{
	SInt32 layer_id;
	SInt32 cluster_pos_x, cluster_pos_y;
	SInt32 core_x, core_y;

	layer_id = cluster_id / num_clusters_per_layer;
	cluster_pos_x = cluster_id % cluster_width_per_layer;
	cluster_pos_y = (cluster_id % num_clusters_per_layer) / cluster_width_per_layer;

	core_x = (cluster_pos_x * cluster_width_per_layer) + (layer_id * num_tiles_per_layer);
	core_y = (cluster_pos_y * cluster_height_per_layer);

   for (SInt32 i = core_x; i < core_x + cluster_width_per_layer; i++)
   {
      for (SInt32 j = core_y; j < core_y + cluster_height_per_layer; j++)
      {
         SInt32 tile_id = (j * enet_width_per_layer + i) + (layer_id * num_tiles_per_layer);
         assert (tile_id < (SInt32) Config::getSingleton()->getApplicationTiles());
         tile_id_list.push_back(tile_id);
      }
   }
}

SInt32 NetworkModelOrnoc::getNumOniSendPorts(cluster_id_t cluster_id)
{
	SInt32 num = 0;
	for (auto rit = rings.begin(); rit != rings.end(); rit++){
	   for (auto pit = (*rit).portions.begin(); pit != (*rit).portions.end(); pit++){
		   if ((*pit).source_cluster_id == cluster_id)
			   num += 1;
	   }
	}
	return num;
}

void NetworkModelOrnoc::getClusterSendPortions(cluster_id_t cluster_id, vector<Ring::Portion>& portions)
{
	for (auto rit = rings.begin(); rit != rings.end(); rit++){
	   for (auto pit = (*rit).portions.begin(); pit != (*rit).portions.end(); pit++){
		   if ((*pit).source_cluster_id == cluster_id)
			   portions.push_back((*pit));
	   }
	}
}

SInt32 NetworkModelOrnoc::getNumOniReceivePorts(cluster_id_t cluster_id)
{
	SInt32 num = 0;
   for (auto rit = rings.begin(); rit != rings.end(); rit++){
	   for (auto pit = (*rit).portions.begin(); pit != (*rit).portions.end(); pit++){
		   if ((*pit).target_cluster_id == cluster_id)
			   num += 1;
	   }
   }
   return num;
}


void NetworkModelOrnoc::getClusterReceivePortions(cluster_id_t cluster_id, vector<Ring::Portion>& portions)
{
   for (auto rit = rings.begin(); rit != rings.end(); rit++){
	   for (auto pit = (*rit).portions.begin(); pit != (*rit).portions.end(); pit++){
		   if ((*pit).target_cluster_id == cluster_id)
			   portions.push_back((*pit));
	   }
   }
}

SInt32 NetworkModelOrnoc::getLayerID(tile_id_t tile_id)
{
	return (tile_id / num_tiles_per_layer);
}

SInt32 NetworkModelOrnoc::getClusterID(tile_id_t tile_id)
{
   // consider a mesh per layer formed by the clusters
   SInt32 cluster_mesh_width_per_layer;

   cluster_mesh_width_per_layer = enet_width_per_layer / cluster_width_per_layer;

   SInt32 tile_x, tile_y, layer_id;
   computePosition(tile_id, tile_x, tile_y, layer_id);

   SInt32 cluster_pos_x, cluster_pos_y;
   cluster_pos_x = tile_x / cluster_width_per_layer;
   cluster_pos_y = tile_y / cluster_height_per_layer;

   return ((cluster_pos_y * cluster_mesh_width_per_layer + cluster_pos_x) + (layer_id * num_clusters_per_layer));
}

SInt32 NetworkModelOrnoc::getSubClusterID(tile_id_t tile_id)
{
	SInt32 cx, cy, cl;
	computePosition(tile_id, cx, cy, cl);

	SInt32 cluster_id = getClusterID(tile_id);
	ClusterInfo::Boundary& boundary = cluster_info_list[cluster_id].boundary;

	SInt32 pos_x = (cx - boundary.minX) / sub_cluster_width_per_layer;
	SInt32 pos_y = (cy - boundary.minY) / sub_cluster_height_per_layer;
	return (pos_y * numX_sub_clusters_per_layer) + pos_x;
}

void NetworkModelOrnoc::computePosition(tile_id_t tile_id, SInt32& x, SInt32& y, layer_id_t& layer_id)
{
   x = tile_id % enet_width_per_layer;
   y = (tile_id % num_tiles_per_layer) / enet_width_per_layer;
   layer_id = tile_id / num_tiles_per_layer;
}

SInt32 NetworkModelOrnoc::computeLayerID(tile_id_t tile_id)
{
	return (tile_id / num_tiles_per_layer);
}

double NetworkModelOrnoc::computeOpticalLinkLength()
{
	//TODO - change this
	// Note that number of clusters must be 'positive' and 'power of 2'
	// 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024

	if (num_clusters_per_layer == 2)
	{
	  // Assume that optical link connects the mid-point of the clusters
	  return (cluster_height_per_layer * _tile_width);
	}
	else if (num_clusters_per_layer == 4)
	{
	  // Assume that optical link passes through mid-point of the clusters
	  return (cluster_width_per_layer * _tile_width) * (cluster_height_per_layer * _tile_width);
	}
	else if (num_clusters_per_layer == 8)
	{
	  return (cluster_width_per_layer * _tile_width) * (2 * cluster_height_per_layer * _tile_width);
	}
	else
	{
	  // Assume that optical link passes through the edges of the clusters
	  double length_rectangle = (numX_clusters_per_layer-2) * cluster_width_per_layer * _tile_width;
	  double height_rectangle = (cluster_height_per_layer*2) * _tile_width;
	  SInt32 num_rectangles = numY_clusters_per_layer/4;
	  return (num_rectangles * 2 * (length_rectangle + height_rectangle));
	}
}

