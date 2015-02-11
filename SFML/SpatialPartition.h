#ifndef SPATIAL_PARTITION
#define SPATIAL_PARTITION

#include <vector>
#include <map>

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
	std::vector<Particle*> GetNeighbors(const Particle& particle);

private:
	std::map<int, std::vector<Particle*>> m_Buckets;

	std::vector<int> GetIdForObject(const Particle& particle);
};

#endif // SPATIAL_PARTITION