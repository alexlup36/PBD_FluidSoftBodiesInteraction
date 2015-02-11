#include "SpatialPartition.h"

#include <math.h>


SpatialPartition::SpatialPartition()
{
}


SpatialPartition::~SpatialPartition()
{
}

void SpatialPartition::Setup()
{
	for (int i = 0; i < CELL_COLS * CELL_ROWS; i++)
	{
		m_Buckets.insert(std::pair<int, std::vector<Particle*>>(i, std::vector<Particle*>()));
	}
}

void SpatialPartition::ClearBuckets()
{
	m_Buckets.clear();

	Setup();
}

void SpatialPartition::RegisterObject(Particle* particle)
{
	// Get a list of ids of the cell the current particle is in
	std::vector<int> cellIDsList = GetIdForObject(*particle);

	for each (auto cellId in cellIDsList)
	{
		m_Buckets[cellId].push_back(particle);
	}
}

std::vector<int> SpatialPartition::GetIdForObject(const Particle& particle)
{
	std::vector<int> cellIDList;

	float fXPos = particle.GetLocalPosition().x;
	float fYPos = particle.GetLocalPosition().y;
	float fRadius = particle.GetRadius();

	// Top left corner
	int iCellIndex = (int)(std::floor((fXPos - fRadius) / CELL_SIZE) +
		std::floor((fYPos - fRadius) / CELL_SIZE) * CELL_COLS);

	// The value is not in the list
	if (std::find(cellIDList.begin(), cellIDList.end(), iCellIndex) == cellIDList.end())
	{
		cellIDList.push_back(iCellIndex);
	}

	// Top right corner
	iCellIndex = (int)(std::floor((fXPos + fRadius) / CELL_SIZE) +
		std::floor((fYPos - fRadius) / CELL_SIZE) * CELL_COLS);

	// The value is not in the list
	if (std::find(cellIDList.begin(), cellIDList.end(), iCellIndex) == cellIDList.end())
	{
		cellIDList.push_back(iCellIndex);
	}

	// Bottom left corner
	iCellIndex = (int)(std::floor((fXPos - fRadius) / CELL_SIZE) +
		std::floor((fYPos + fRadius) / CELL_SIZE) * CELL_COLS);

	// The value is not in the list
	if (std::find(cellIDList.begin(), cellIDList.end(), iCellIndex) == cellIDList.end())
	{
		cellIDList.push_back(iCellIndex);
	}

	// Bottom right corner
	iCellIndex = (int)(std::floor((fXPos + fRadius) / CELL_SIZE) +
		std::floor((fYPos + fRadius) / CELL_SIZE) * CELL_COLS);

	// The value is not in the list
	if (std::find(cellIDList.begin(), cellIDList.end(), iCellIndex) == cellIDList.end())
	{
		cellIDList.push_back(iCellIndex);
	}

	return cellIDList;
}

std::vector<Particle*> SpatialPartition::GetNeighbors(const Particle& particle)
{
	std::vector<Particle*> nearbyParticleList;

	std::vector<int> cellIDsList = GetIdForObject(particle);

	for each (int id in cellIDsList)
	{
		for each (Particle* particle in m_Buckets[id])
		{
			nearbyParticleList.push_back(particle);
		}
	}

	return nearbyParticleList;
}