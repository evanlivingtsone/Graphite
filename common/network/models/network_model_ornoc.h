#pragma once

#include <vector>
#include <string>
using namespace std;

#include "queue_model.h"
#include "network.h"
#include "network_model.h"
#include "router_model.h"
#include "electrical_link_model.h"
#include "electrical_link_power_model.h"
#include "optical_link_model.h"
#include "optical_link_power_model.h"

class NetworkModelOrnoc : public NetworkModel
{
public:
	   NetworkModelOrnoc(Network *net, SInt32 network_id);
	   ~NetworkModelOrnoc();

	   void routePacket(const NetPacket &pkt, queue<Hop> &nextHops);

	   static pair<bool, vector<tile_id_t> > computeMemoryControllerPositions(SInt32 num_memory_controllers, SInt32 tile_count);
	   static pair<bool, vector<vector<tile_id_t> > > computeProcessToTileMapping();

private:
	   typedef SInt32 wavelength_id_t;
	   typedef SInt32 ring_id_t;
	   typedef SInt32 layer_id_t;
	   typedef SInt32 cluster_id_t;

	   // enumerations

	   enum NodeType{
	      EMESH = 2,  // Start at 2 always
	      SEND_HUB,
	      RECEIVE_HUB,
	   };

	   enum OutputDirection
	   {
	      SELF = 0,
	      LEFT,
	      RIGHT,
	      DOWN,
	      UP
	   };

	   enum RoutingStrategy{
		   ISOLATED_CLUSTERS = 0,
		   EFFICIENT
	   };

	   enum Route{
		   ENET = 0,
		   ONET
	   };

	   enum ReceiveNetworkType{
		   BROADCAST = 0,
		   STAR_ROUTER
	   };

	   // internal classes

	   // ring class containing portion and wavelengths
	   class Ring{
	   public:

		   class Wavelength{
		   public:
			   Wavelength(wavelength_id_t wavelength_id_, bool reserved_) : wavelength_id(wavelength_id_), reserved(reserved_) {}
			   wavelength_id_t wavelength_id;
			   bool reserved;
		   };

		   class Portion{
		   public:
			   vector<Wavelength> wavelengths;
			   cluster_id_t source_cluster_id;
			   cluster_id_t target_cluster_id;
		   };

		   Ring(ring_id_t id_, bool clockwise_, SInt32 num_portions_, SInt32 num_wavelengths_);
		   ring_id_t id;
		   bool clockwise;
		   vector<Portion> portions;
		   vector< vector<wavelength_id_t> > connectivity_matrix;
	   };

	   // cluster boundaries and access points
	   class ClusterInfo{
	   public:

		   class Boundary{
		   public:
			   Boundary() : minX(0),maxX(0), minY(0),maxY(0){}
			   Boundary(SInt32 minX_, SInt32 maxX_, SInt32 minY_, SInt32 maxY_) : minX(minX_), maxX(maxX_), minY(minY_), maxY(maxY_){}
			   ~Boundary() {}
			   SInt32 minX, maxX, minY, maxY;
		   };

		   Boundary boundary;
		   vector<tile_id_t> access_point_list;
		   layer_id_t layer_id;
	   };

	   // fields

	   static bool initialized;

	   // eNet
	   static SInt32 enet_width_per_layer;
	   static SInt32 enet_height_per_layer;

	   // clusters
	   static SInt32 num_clusters;
	   static SInt32 num_clusters_per_layer;
	   static SInt32 num_tiles_per_layer;
	   static SInt32 cluster_size;
	   static SInt32 numX_clusters_per_layer;
	   static SInt32 numY_clusters_per_layer;
	   static SInt32 cluster_width_per_layer;
	   static SInt32 cluster_height_per_layer;
	   static vector<ClusterInfo> cluster_info_list;
	   static vector<Ring> rings;

	   // sub clusters
	   static SInt32 num_access_points_per_cluster;
	   static SInt32 num_sub_clusters_per_layer;
	   static SInt32 numX_sub_clusters_per_layer;
	   static SInt32 numY_sub_clusters_per_layer;
	   static SInt32 sub_cluster_width_per_layer;
	   static SInt32 sub_cluster_height_per_layer;

	   //

	   static ReceiveNetworkType receive_network_type;
	   static SInt32 num_receive_networks_per_cluster;
	   static SInt32 max_wavelengths_per_ring;
	   static RoutingStrategy routing_strategy;
	   static SInt32 unicast_distance_threshold;
	   static bool contention_model_enabled;
	   static bool verbose_output;

	   static vector< vector<bool> > connectivity_matrix;
	   static string connectivity_matrix_path;
	   static SInt32 num_layers;

	   // injection port router
	   RouterModel* injection_router;

	   // enet router and links
	   SInt32 num_enet_router_ports;
	   RouterModel* enet_router;
	   vector<ElectricalLinkModel*> enet_link_list;

	   // send hub router
	   RouterModel* send_hub_router;

	   // receive hub router
	   RouterModel* receive_hub_router;

	   // optical links
	   vector<OpticalLinkModel*> optical_links;

	   // broadcast tree link list
	   vector<ElectricalLinkModel*> btree_link_list;

	   //star network router and link list
	   vector <RouterModel*> star_net_router_list;
	   vector<vector<ElectricalLinkModel*> > star_net_link_list;

	   // < private functions >

	   // routing functions
	   void routePacketOnENet(const NetPacket& pkt, tile_id_t sender, tile_id_t receiver, queue<Hop>& next_hops);
	   void routePacketOnONet(const NetPacket& pkt, tile_id_t sender, tile_id_t receiver, queue<Hop>& next_hops);

	   // initialization functions
	   static void initializeRNetTopologyParams();
	   static void initializeClusters();
	   static void initializeAccessPointList(SInt32 cluster_id);

	   static SInt32 availableWavelength(Ring& ring, SInt32 source_id, SInt32 target_id);

	   void createRNetRouterAndLinkModels();
	   void destroyRNetRouterAndLinkModels();

	   // output summary
	   void outputPowerSummary(ostream& out, const Time& target_completion_time);
	   void outputEventCountSummary(ostream& out);
	   void outputContentionModelsSummary(ostream& out);

	   // location functions

	   static SInt32 getNumOniSendPorts(cluster_id_t cluster_id);
	   static SInt32 getNumOniReceivePorts(cluster_id_t cluster_id);

	   static void getClusterSendPortions(cluster_id_t cluster_id, vector<Ring::Portion>& portions);
	   static void getClusterReceivePortions(cluster_id_t cluster_id, vector<Ring::Portion>& portions);

	   static bool findONetPath(cluster_id_t source_cluster_id, cluster_id_t target_cluster_id, vector<cluster_id_t>& path);

	   static cluster_id_t getClusterID(tile_id_t tile_id);
	   static SInt32 getSubClusterID(tile_id_t tile_id);

	   static void getClusterIDListInLayer(SInt32 layer_id, vector<SInt32>& cluster_id_list);
	   static void getTileIDListInCluster(cluster_id_t cluster_id, vector<tile_id_t>& tile_id_list);

	   static SInt32 getLayerID(tile_id_t tile_id);
	   static SInt32 getRingID(tile_id_t tile_id);

	   static tile_id_t getNearestAccessPoint(tile_id_t tile_id);
	   bool isAccessPoint(tile_id_t tile_id);

	   static tile_id_t getTileIDWithOpticalHub(SInt32 cluster_id);

	   static SInt32 getIndexInList(tile_id_t tile_id, vector<tile_id_t>& tile_id_list);

	   static SInt32 computeNumHopsOnENet(tile_id_t sender, tile_id_t receiver);
	   static void computePosition(tile_id_t tile_id, SInt32& x, SInt32& y, layer_id_t& layer_id);
	   static SInt32 computeLayerID(tile_id_t tile_id);
	   static tile_id_t computeTileID(SInt32 x, SInt32 y, SInt32 l);
	   static SInt32 computeReceiveNetID(tile_id_t sender);

	   // compute waveguide length
	   double computeOpticalLinkLength();

	   // routing algorithms
	   static RoutingStrategy parseRoutingStrategy(string strategy);
	   Route computeGlobalRoute(tile_id_t sender, tile_id_t receiver);
	   static ReceiveNetworkType parseReceiveNetType(string receive_net_type);


};



