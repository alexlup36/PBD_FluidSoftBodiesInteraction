#ifndef SPATIAL_PARTITION
#define SPATIAL_PARTITION

#include <vector>
#include <map>
#include <set>

#include "Common.h"
#include "FluidParticle.h"

class SpatialPartition
{
public:

	static SpatialPartition& GetInstance()
	{
		static SpatialPartition instance;
		return instance;
	}

	void Setup();
	void ClearBuckets();
	void RegisterObject(FluidParticle* particle);
	void GetNeighbors(const FluidParticle& particle, std::vector<FluidParticle*>& nearbyParticleList);

private:
	// -----------------------------------------------------------------------------
	// Hide constructor for singleton implementation
	SpatialPartition() {};

	// Delete unneeded copy constructor and assignment operator
	SpatialPartition(SpatialPartition const&) = delete;
	void operator=(SpatialPartition const&) = delete;
	// -----------------------------------------------------------------------------

	std::map<int, std::vector<FluidParticle*>> m_Buckets;

	void GetIdForObject(const FluidParticle& particle, std::set<int>& cellIDList);
};

#endif // SPATIAL_PARTITION