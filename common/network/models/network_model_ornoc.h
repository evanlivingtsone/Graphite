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

private:
	   typedef SInt32 wavelength_id_t;
	   typedef SInt32 ring_id_t;
	   typedef SInt32 layer_id_t;
	   typedef SInt32 cluster_id_t;


	   // Enumerations
	   enum NodeType{
	      EMESH = 2,  // Start at 2 always
	      SEND_HUB,
	      RECEIVE_HUB,
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

	   // Internal classes

	   // Ring class containing portion and wavelengths
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
	   };

	   // Cluster boundaries and access points
	   class ClusterInfo{
	   public:

		   class Boundary{
		   public:
			   Boundary() : minX(0),maxX(0), minY(0),maxY(0){}
			   Boundary(SInt32 minX_, SInt32 maxX_, SInt32 minY_, SInt32 maxY_) : minX(minX_), maxX(maxX_), minY(minY_), maxY(maxY_){}
			   ~Boundary() {}
			   SInt32 minX, maxX, minY, maxY;
		   };

		   class ConnectionElement{
		   public:
			   ConnectionElement() : target_cluster_id(0), wavelength(0),reserved(false), ring_id(0) {}
			   ConnectionElement(tile_id_t target_cluster_id_, wavelength_id_t wavelength_, bool reserved_, ring_id_t ring_id_) :
				   	   	   	   	 target_cluster_id(target_cluster_id_), wavelength(wavelength_),reserved(reserved_), ring_id(ring_id_) {}
			   ~ConnectionElement() {}

			   cluster_id_t target_cluster_id;
			   wavelength_id_t wavelength;
			   bool reserved;
			   ring_id_t ring_id;
		   };

		   Boundary boundary;
		   vector<ConnectionElement> connection_element_list;
		   vector<tile_id_t> access_point_list;
		   layer_id_t layer_id;
	   };

	   // Fields

	   static bool initialized;

	   // ENet
	   static SInt32 enet_width_per_layer;
	   static SInt32 enet_height_per_layer;

	   // Clusters
	   static SInt32 num_clusters;
	   static SInt32 num_clusters_per_layer;
	   static SInt32 cluster_size;
	   static SInt32 numX_clusters_per_layer;
	   static SInt32 numY_clusters_per_layer;
	   static SInt32 cluster_width_per_layer;
	   static SInt32 cluster_height_per_layer;
	   static vector<ClusterInfo> cluster_info_list;

	   // Sub clusters
	   static SInt32 num_access_points_per_cluster;
	   static SInt32 num_sub_clusters;
	   static SInt32 numX_sub_clusters;
	   static SInt32 numY_sub_clusters;
	   static SInt32 sub_cluster_width;
	   static SInt32 sub_cluster_height;

	   //

	   static ReceiveNetworkType receive_network_type;
	   static SInt32 num_receive_networks_per_cluster;
	   static SInt32 max_wavelengths_per_ring;
	   static RoutingStrategy routing_strategy;
	   static SInt32 unicast_distance_threshold;
	   static bool contention_model_enabled;

	   static vector< vector<bool> > connectvitiy_matrix;
	   static string connectivity_matrix_path;
	   static SInt32 num_layers;

	   // Injection port router
	   RouterModel* injection_router;

	   // ENet router and links
	   SInt32 num_enet_router_ports;
	   RouterModel* enet_router;
	   vector<ElectricalLinkModel*> enet_link_list;

	   // Send hub router
	   RouterModel* send_hub_router;

	   // Receive hub router
	   RouterModel* receive_hub_router;

	   // Optical link
	   OpticalLinkModel* optical_link;

	   // Broadcast tree link list
	   vector<ElectricalLinkModel*> btree_link_list;

	   //Star network router and link list
	   vector <RouterModel*> star_net_router_list;
	   vector<vector<ElectricalLinkModel*> > star_net_link_list;

	   // < Private functions >

	   // Routing functions
	   void routePacketOnENet(const NetPacket& pkt, tile_id_t sender, tile_id_t receiver, queue<Hop>& next_hops);
	   void routePacketOnONet(const NetPacket& pkt, tile_id_t sender, tile_id_t receiver, queue<Hop>& next_hops);

	   // Initialization functions
	   static void initializeRNetTopologyParams();
	   static void initializeClusters();
	   static void initializeAccessPointList(SInt32 cluster_id);

	   static SInt32 availableWavelength(vector<Ring::Portion>& portions, SInt32 source_id, SInt32 target_id, bool clockwise);

//	   template <class Iterator>
//	   static void _availableWavelength(Iterator first, Iterator last, vector<Ring::Portion>& portions, SInt32 source_id,
//			   SInt32 target_id, bool clockwise);

	   void createRNetRouterAndLinkModels();
	   void destroyRNetRouterAndLinkModels();

	   // Output summary
	   void outputPowerSummary(ostream& out, const Time& target_completion_time);
	   void outputEventCountSummary(ostream& out);
	   void outputContentionModelsSummary(ostream& out);

	   // Location functions
	   static cluster_id_t getClusterID(tile_id_t tile_id);
	   static SInt32 getSubClusterID(tile_id_t tile_id);
	   static void getTileIDListInCluster(cluster_id_t cluster_id, vector<tile_id_t>& tile_id_list);

	   static SInt32 getLayerID(tile_id_t tile_id);
	   static SInt32 getRingID(tile_id_t tile_id);

	   static tile_id_t getNearestAccessPoint(tile_id_t tile_id);
	   bool isAccessPoint(tile_id_t tile_id);

	   static tile_id_t getTileIDWithOpticalHub(SInt32 cluster_id);

	   static SInt32 getIndexInList(tile_id_t tile_id, vector<tile_id_t>& tile_id_list);

	   static SInt32 computeNumHopsOnENet(tile_id_t sender, tile_id_t receiver);
	   static void computePositionOnENet(tile_id_t tile_id, SInt32& x, SInt32& y);
	   static tile_id_t computeTileIDOnENet(SInt32 x, SInt32 y);
	   static SInt32 computeReceiveNetID(tile_id_t sender);

	   // Compute waveguide length
	   double computeOpticalLinkLength();

	   // Routing algorithms
	   static RoutingStrategy parseRoutingStrategy(string strategy);
	   Route computeGlobalRoute(tile_id_t sender, tile_id_t receiver);
	   static ReceiveNetworkType parseReceiveNetType(string receive_net_type);


};



