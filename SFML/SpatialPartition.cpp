#include "SpatialPartition.h"

#include <math.h>

// ------------------------------------------------------------------------

void SpatialPartition::Setup()
{
	for (int i = 0; i < TOTAL_CELLS; i++)
	{
		m_Buckets.insert(std::pair<int, std::vector<BaseParticle*>>(i, std::vector<BaseParticle*>()));
	}
}

// ------------------------------------------------------------------------

void SpatialPartition::ClearBuckets()
{
	m_Buckets.clear();
}

// ------------------------------------------------------------------------

void SpatialPartition::RegisterObject(BaseParticle* particle)
{
	// Get a list of ids of the cell the current particle is in
	particle->UpdateCellIds();
	std::set<int>& cellIDsList = particle->GetCellIDsList();

#ifdef MULTITHREADING

	m_BucketAccessMutex.lock();

#endif // MULTITHREADING

	for each (auto cellId in cellIDsList)
	{
		m_Buckets[cellId].push_back(particle);
	}

#ifdef MULTITHREADING

	m_BucketAccessMutex.unlock();

#endif // MULTITHREADING

}

// ------------------------------------------------------------------------

void SpatialPartition::GetIdForObject(const BaseParticle& particle, std::set<int>& cellIDList)
{
	float fXPos = particle.LocalPosition.x;
	float fYPos = particle.LocalPosition.y;
	float fRadius = particle.Radius;

	// Top left corner
	int iCellIndex = (int)(std::floor((fXPos - fRadius) / CELL_SIZE) +
		std::floor((fYPos - fRadius) / CELL_SIZE) * CELL_COLS);
	cellIDList.insert(iCellIndex);

	// Top right corner
	iCellIndex = (int)(std::floor((fXPos + fRadius) / CELL_SIZE) +
		std::floor((fYPos - fRadius) / CELL_SIZE) * CELL_COLS);
	cellIDList.insert(iCellIndex);

	// Bottom left corner
	iCellIndex = (int)(std::floor((fXPos - fRadius) / CELL_SIZE) +
		std::floor((fYPos + fRadius) / CELL_SIZE) * CELL_COLS);
	cellIDList.insert(iCellIndex);

	// Bottom right corner
	iCellIndex = (int)(std::floor((fXPos + fRadius) / CELL_SIZE) +
		std::floor((fYPos + fRadius) / CELL_SIZE) * CELL_COLS);
	cellIDList.insert(iCellIndex);
}

// ------------------------------------------------------------------------

void SpatialPartition::GetNeighbors(BaseParticle& particle, 
	std::vector<FluidParticle*>& nearbyFluidParticleList,
	std::vector<DeformableParticle*>& nearbyDeformableParticleList,
	std::vector<BaseParticle*>& allParticles)
{
	std::set<int>& cellIDsList = particle.GetCellIDsList();

	for (std::set<int>::iterator it = cellIDsList.begin(); it != cellIDsList.end(); it++)
	{
		std::vector<BaseParticle*>& currentBucket = m_Buckets[*it];
		unsigned int iCurrentBucketSize = currentBucket.size();

		for (unsigned int index = 0; index < iCurrentBucketSize; index++)
		{
			// Get the current element
			BaseParticle* pCurrentParticle = currentBucket[index];

			if (pCurrentParticle->Index != particle.Index)
			{
				if (pCurrentParticle->ParticleType == ParticleType::FluidParticle)
				{
					nearbyFluidParticleList.push_back((FluidParticle*)pCurrentParticle);
				}
				else
				{
					nearbyDeformableParticleList.push_back((DeformableParticle*)pCurrentParticle);
				}
				allParticles.push_back(pCurrentParticle);
			}
		}
	}
}

// ------------------------------------------------------------------------