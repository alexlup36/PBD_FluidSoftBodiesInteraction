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
	SpatialPartition();
	~SpatialPartition();

	void Setup();
	void ClearBuckets();
	void RegisterObject(Particle* particle);
	void/*std::vector<Particle*>*/ GetNeighbors(const Particle& particle, std::vector<Particle*>& nearbyParticleList);

private:
	std::map<int, std::vector<Particle*>> m_Buckets;

	void/*std::vector<int>*/ GetIdForObject(const Particle& particle, std::set<int>& cellIDList);
};

#endif // SPATIAL_PARTITION