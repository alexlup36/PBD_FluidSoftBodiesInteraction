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
	void GetNeighbors(BaseParticle& particle,
		std::vector<FluidParticle*>& nearbyFluidParticleList,
		std::vector<DeformableParticle*>& nearbyDeformableParticleList,
		std::vector<BaseParticle*>& allParticles);

	inline std::map<int, std::vector<BaseParticle*>>& GetBuckets() { return m_Buckets; }

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

	// Multithreading
	std::mutex m_BucketAccessMutex;
};

#endif // SPATIAL_PARTITION