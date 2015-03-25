#include "SpatialPartition.h"

#include <math.h>


void SpatialPartition::Setup()
{
	for (int i = 0; i < TOTAL_CELLS; i++)
	{
		m_Buckets.insert(std::pair<int, std::vector<FluidParticle*>>(i, std::vector<FluidParticle*>()));
	}
}

void SpatialPartition::ClearBuckets()
{
	m_Buckets.clear();

	//Setup();
}

void SpatialPartition::RegisterObject(FluidParticle* particle)
{
	// Get a list of ids of the cell the current particle is in
	std::set<int> cellIDsList;
	GetIdForObject(*particle, cellIDsList);

	for each (auto cellId in cellIDsList)
	{
		m_Buckets[cellId].push_back(particle);
	}
}

void SpatialPartition::GetIdForObject(const FluidParticle& particle, std::set<int>& cellIDList)
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

void SpatialPartition::GetNeighbors(const FluidParticle& particle, std::vector<FluidParticle*>& nearbyParticleList)
{
	std::set<int> cellIDsList;
	GetIdForObject(particle, cellIDsList);

	// Test pre reserve memory for stl vector
	int iCount = 0;
	for each (int id in cellIDsList)
	{
		auto currentBucket = m_Buckets[id];

		for each (FluidParticle* pParticle in currentBucket)
		{
			iCount++;
		}
	}
	nearbyParticleList.reserve(iCount);
	int iIndex = 0;

	for each (int id in cellIDsList)
	{
		auto currentBucket = m_Buckets[id];

		for each (FluidParticle* pParticle in currentBucket)
		{
			if (pParticle->Index != particle.Index)
			{
				nearbyParticleList.push_back(pParticle);
			}
		}
	}
}