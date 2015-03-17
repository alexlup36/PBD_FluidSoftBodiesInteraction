#ifndef SPATIAL_PARTITION
#define SPATIAL_PARTITION

#include <vector>
#include <map>
#include <set>

#include "Common.h"
#include "Particle.h"

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
	void RegisterObject(Particle* particle);
	void/*std::vector<Particle*>*/ GetNeighbors(const Particle& particle, std::vector<Particle*>& nearbyParticleList);

private:
	// -----------------------------------------------------------------------------
	// Hide constructor for singleton implementation
	SpatialPartition() {};

	// Delete unneeded copy constructor and assignment operator
	SpatialPartition(SpatialPartition const&) = delete;
	void operator=(SpatialPartition const&) = delete;
	// -----------------------------------------------------------------------------

	std::map<int, std::vector<Particle*>> m_Buckets;

	void/*std::vector<int>*/ GetIdForObject(const Particle& particle, std::set<int>& cellIDList);
};

#endif // SPATIAL_PARTITION