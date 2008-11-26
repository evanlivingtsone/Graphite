#ifndef DRAM_DIRECTORY_H
#define DRAM_DIRECTORY_H

#include "debug.h"
#include "pin.H"
#include "dram_directory_entry.h"
#include "network.h"
#include "memory_manager.h"
#include <map>

//TODO i don't think this is used
extern LEVEL_BASE::KNOB<UINT32> g_knob_dram_access_cost;

//LIMITED_DIRECTORY Flag
//Dir(i)NB ; i = number of pointers
//if MAX_SHARERS >= total number of cores, then the directory
//collaspes into the full-mapped case.
//TODO use a knob to set this instead
//(-dms) : directory_max_sharers
//TODO provide easy mechanism to initiate a broadcast invalidation
//	static const UINT32 MAX_SHARERS = 2;
extern LEVEL_BASE::KNOB<UINT32> g_knob_dir_max_sharers;
enum shmem_req_t;

class DramDirectory
{
 private:
   //assumption: each dram_directory is tied to a given network (node), and is addressed at dram_id
	Network* the_network; 
	UINT32 num_lines;
   unsigned int bytes_per_cache_line;
   UINT32 number_of_cores;
   //key dram entries on cache_line (assumes cache_line is 1:1 to dram memory lines)
   std::map<UINT32, DramDirectoryEntry*> dram_directory_entries;
   UINT32 dram_id;

	enum CURRENT_FUNCTION {
		NONE,
		DEMOTE_OWNER,
		INVALIDATE_SHARERS,
		EVICT_SHARER
	};
	
	//state for re-entering the Directory
	shmem_req_t shmem_req_type;
	UINT32 current_requestor;
   CURRENT_FUNCTION current_function;
	ADDRINT request_ca_address;
	DramDirectoryEntry* current_dram_dir_entry;
	CacheState::cstate_t current_new_cstate;
	UINT32 current_eviction_id; //
	UINT32 numAcksToRecv; //number of acks we are still waiting for

	/* Added by George */
   UINT64 dramAccessCost;
   
	//TODO debugAssertValidStates();
	//scan the directory occasionally for invalid state configurations.
	//can that be done here? or only in the MMU since we may need cache access.

public:
	//is this a needed function?
	DramDirectoryEntry* getEntry(ADDRINT address);

   DramDirectory(UINT32 num_lines, UINT32 bytes_per_cache_line, UINT32 dram_id_arg, UINT32 num_of_cores, Network* network_arg);
   virtual ~DramDirectory();
   
   /***************************************/
	
	//receive and process request for memory_block
	void processSharedMemReq(NetPacket req_packet);
	void finishSharedMemReq();
	
	void processAck(NetPacket ack_packet);
	
	void startDemoteOwner(DramDirectoryEntry* dram_dir_entry, CacheState::cstate_t new_cstate);
	void processDemoteOwnerAck(NetPacket ack_packet);
	
	void startInvalidateSharers(DramDirectoryEntry* dram_dir_entry);
	void processInvalidateSharersAck(NetPacket ack_packet);
	
	void startEvictSharer(DramDirectoryEntry* dram_dir_entry, UINT32 eviction_id);
	void processEvictSharerAck(NetPacket ack_packet);

	//make many of these private functions
	void copyDataToDram(ADDRINT address, char* data_buffer);
	//sending another memory line to another core. rename.
	void sendDataLine(DramDirectoryEntry* dram_dir_entry, UINT32 requestor, CacheState::cstate_t new_cstate);
	
	void processWriteBack(NetPacket req_packet);
	
   void setNumberOfLines(UINT32 number_of_lines) { num_lines = number_of_lines; }

	void runDramAccessModel();
	UINT64 getDramAccessCost();
	
	/***************************************/
	
	void debugSetDramState(ADDRINT addr, DramDirectoryEntry::dstate_t dstate, vector<UINT32> sharers_list, char *d_data);
	bool debugAssertDramState(ADDRINT addr, DramDirectoryEntry::dstate_t dstate, vector<UINT32> sharers_list, char *d_data);

   /***************************************/

	void setDramMemoryLine(ADDRINT addr, char* data_buffer, UINT32 data_size);
	
	//for debug purposes
   void print();
};

#endif
