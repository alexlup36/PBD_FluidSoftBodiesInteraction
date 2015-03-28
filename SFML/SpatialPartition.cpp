#include "SpatialPartition.h"

#include <math.h>


void SpatialPartition::Setup()
{
	for (int i = 0; i < TOTAL_CELLS; i++)
	{
		m_Buckets.insert(std::pair<int, std::vector<BaseParticle*>>(i, std::vector<BaseParticle*>()));
	}
}

void SpatialPartition::ClearBuckets()
{
	m_Buckets.clear();
}

void SpatialPartition::RegisterObject(BaseParticle* particle)
{
	// Get a list of ids of the cell the current particle is in
	std::set<int> cellIDsList;
	GetIdForObject(*particle, cellIDsList);

	for each (auto cellId in cellIDsList)
	{
		m_Buckets[cellId].push_back(particle);
	}
}

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

void SpatialPartition::GetNeighbors(const BaseParticle& particle, 
	std::vector<FluidParticle*>& nearbyFluidParticleList,
	std::vector<DeformableParticle*>& nearbyDeformableParticleList)
{
	std::set<int> cellIDsList;

	GetIdForObject(particle, cellIDsList);

	// Test pre reserve memory for stl vector
	int iCountFluid = 0;
	int iCountSoft = 0;
	for each (int id in cellIDsList)
	{
		auto currentBucket = m_Buckets[id];

		for each (BaseParticle* pParticle in currentBucket)
		{
			if (pParticle->ParticleType == ParticleType::FluidParticle)
			{
				iCountFluid++;
			}
			else
			{
				iCountSoft++;
			}
		}
	}
	
	nearbyFluidParticleList.reserve(iCountFluid);
	nearbyDeformableParticleList.reserve(iCountSoft);

	int iIndex = 0;

	for each (int id in cellIDsList)
	{
		auto currentBucket = m_Buckets[id];

		for each (BaseParticle* pParticle in currentBucket)
		{
			if (pParticle->Index != particle.Index)
			{
				if (pParticle->ParticleType == ParticleType::FluidParticle)
				{
					nearbyFluidParticleList.push_back((FluidParticle*)pParticle);
				}
				else
				{
					nearbyDeformableParticleList.push_back((DeformableParticle*)pParticle);
				}
			}
		}
	}
}