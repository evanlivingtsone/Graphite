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
SInt32 NetworkModelOrnoc::cluster_size;
SInt32 NetworkModelOrnoc::numX_clusters_per_layer;
SInt32 NetworkModelOrnoc::numY_clusters_per_layer;
SInt32 NetworkModelOrnoc::cluster_width_per_layer;
SInt32 NetworkModelOrnoc::cluster_height_per_layer;

// sub clusters
SInt32 NetworkModelOrnoc::num_access_points_per_cluster;
SInt32 NetworkModelOrnoc::num_sub_clusters;
SInt32 NetworkModelOrnoc::numX_sub_clusters;
SInt32 NetworkModelOrnoc::numY_sub_clusters;
SInt32 NetworkModelOrnoc::sub_cluster_width;
SInt32 NetworkModelOrnoc::sub_cluster_height;

vector<NetworkModelOrnoc::ClusterInfo> NetworkModelOrnoc::cluster_info_list;
NetworkModelOrnoc::ReceiveNetworkType NetworkModelOrnoc::receive_network_type;
SInt32 NetworkModelOrnoc::num_receive_networks_per_cluster;
SInt32 NetworkModelOrnoc::max_wavelengths_per_ring;
NetworkModelOrnoc::RoutingStrategy NetworkModelOrnoc::routing_strategy;
SInt32 NetworkModelOrnoc::unicast_distance_threshold;
bool NetworkModelOrnoc::contention_model_enabled;

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
	         "Num Application Tiles(%i) must be a power of 2", num_application_tiles);
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

	   num_sub_clusters = num_access_points_per_cluster;
	   LOG_ASSERT_ERROR(isPower2(num_access_points_per_cluster),
	         "Number of Optical Access Points(%i) must be a power of 2", num_access_points_per_cluster);

	   num_layers = num_clusters / num_layers;
	   LOG_ASSERT_ERROR(num_layers > 1, "Number of Layers(%i) must be > 1", num_layers);

	   enet_width_per_layer = (SInt32) floor(sqrt(cluster_size * num_layers));
	   enet_height_per_layer = enet_width_per_layer;

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
  UInt64 star_net_router_delay = 0;
  UInt32 num_flits_per_output_buffer_star_net_router = 0;

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
	 star_net_router_delay = (UInt64) Sim()->getCfg()->getInt("network/ornoc/star_net/router/delay");
	 num_flits_per_output_buffer_star_net_router = Sim()->getCfg()->getInt("network/ornoc/star_net/router/num_flits_per_port_buffer");

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

}

void NetworkModelOrnoc::destroyRNetRouterAndLinkModels()
{
	// TODO
}

void NetworkModelOrnoc::routePacket(const NetPacket& pkt, queue<Hop>& next_hops)
{

}

void NetworkModelOrnoc::initializeClusters()
{
	   cluster_info_list.resize(num_clusters);

	   if (isEven(floorLog2(num_clusters_per_layer))){
		   numX_clusters_per_layer = sqrt(num_clusters_per_layer);
		   numY_clusters_per_layer = sqrt(num_clusters_per_layer);
	   }
	   else {
		   numX_clusters_per_layer = sqrt(num_clusters_per_layer / 2);
		   numY_clusters_per_layer = sqrt(num_clusters_per_layer * 2);
	   }

	   cluster_width_per_layer = enet_width_per_layer / numX_clusters_per_layer;
	   cluster_height_per_layer = enet_height_per_layer / numY_clusters_per_layer;

	   // initialize cluster ids and boundaries
	   for (SInt32 l = 0; l < num_layers; l++){
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

	   if (isEven(floorLog2(num_sub_clusters))){
	      numX_sub_clusters = sqrt(num_sub_clusters);
	      numY_sub_clusters = sqrt(num_sub_clusters);
	   }
	   else{
	      numX_sub_clusters = sqrt(num_sub_clusters * 2);
	      numY_sub_clusters = sqrt(num_sub_clusters / 2);
	   }
	   sub_cluster_width = cluster_width_per_layer / numX_sub_clusters;
	   sub_cluster_height = cluster_height_per_layer / numY_sub_clusters;

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

		   for(string::size_type i = 0; i < line.size(); i++)
		   {
			   row.push_back(line[i] == '1');
		   }
		   connectvitiy_matrix.push_back(row);
	   }

	   connectivity_fs.close();

	   LOG_ASSERT_ERROR(connectvitiy_matrix.size() == (unsigned)num_clusters,
	   	         "Number of rows in connectivity matrix file (%i) must be equal to cluster size (%i)",
	   	      connectvitiy_matrix.size(), num_clusters);


	   // "stupid" algorithm (first come, first serve )
	   vector<Ring> rings;
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

					   if (wl > -1){
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

	   // printout of ORNoC structure
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
   for (SInt32 y = 0; y < numY_sub_clusters; y++){
	  for (SInt32 x = 0; x < numX_sub_clusters; x++){
		 SInt32 access_point_x = boundary.minX + (x * sub_cluster_width) + (sub_cluster_width / 2);
		 SInt32 access_point_y = boundary.minY + (y * sub_cluster_height) + (sub_cluster_height / 2);
		 tile_id_t access_point_id = access_point_y * enet_width_per_layer + access_point_x;
		 cluster_info_list[cluster_id].access_point_list.push_back(access_point_id);
	  }
   }
}

NetworkModelOrnoc::ReceiveNetworkType NetworkModelOrnoc::parseReceiveNetType(string str)
{
	// TODO
	return BROADCAST;
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

SInt32 NetworkModelOrnoc::getClusterID(tile_id_t tile_id)
{
   // consider a mesh formed by the clusters
   SInt32 cluster_mesh_width;
   cluster_mesh_width = enet_width_per_layer / cluster_width_per_layer;

   SInt32 tile_x, tile_y;
   computePositionOnENet(tile_id, tile_x, tile_y);

   SInt32 cluster_pos_x, cluster_pos_y;
   cluster_pos_x = tile_x / _cluster_width;
   cluster_pos_y = tile_y / _cluster_height;

   return (cluster_pos_y * cluster_mesh_width + cluster_pos_x);
}

