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
	void RegisterObject(BaseParticle* particle);
	void GetNeighbors(const BaseParticle& particle,
		std::vector<FluidParticle*>& nearbyFluidParticleList,
		std::vector<DeformableParticle*>& nearbyDeformableParticleList);

private:
	// -----------------------------------------------------------------------------
	// Hide constructor for singleton implementation
	SpatialPartition() {};

	// Delete unneeded copy constructor and assignment operator
	SpatialPartition(SpatialPartition const&) = delete;
	void operator=(SpatialPartition const&) = delete;
	// -----------------------------------------------------------------------------

	std::map<int, std::vector<BaseParticle*>> m_Buckets;

	void GetIdForObject(const BaseParticle& particle, std::set<int>& cellIDList);
};

#endif // SPATIAL_PARTITION