#include "BaseParticle.h"

#include "SpatialPartition.h"
#include "ParticleManager.h"

int BaseParticle::ParticleGlobalIndex = 0;

BaseParticle::BaseParticle(const glm::vec2& position, unsigned int parentIndex)
{
	// Shape
	m_Shape.setPosition(sf::Vector2<float>(position.x, position.y));
	m_Shape.setRadius(PARTICLE_RADIUS);
	m_Shape.setOutlineColor(sf::Color::White);
	m_Shape.setOutlineThickness(1.0f);
	m_Shape.setFillColor(m_DefaultColor);
	m_Shape.setOrigin(m_Shape.getLocalBounds().width / 2.0f, 
	m_Shape.getLocalBounds().height / 2.0f);

	Position			= glm::vec2(position.x, position.y);
	LocalPosition		= glm::vec2(Position.x - WALL_LEFTLIMIT, Position.y - WALL_TOPLIMIT);
	PositionCorrection	= glm::vec2(0.0f);
	Velocity			= glm::vec2(0.0f, 0.0f);

	Mass		= PARTICLE_MASS;
	InverseMass	= PARTICLE_INVERSE_MASS;
	Radius		= PARTICLE_RADIUS;

	GlobalIndex = ParticleGlobalIndex++;

	m_iParentSimulationIndex = parentIndex;
}


BaseParticle::~BaseParticle()
{
}

void BaseParticle::Update()
{
	// Update the position of the shape
	m_Shape.setPosition(sf::Vector2<float>(Position.x, Position.y));
}

void BaseParticle::Draw(sf::RenderWindow& window)
{
	window.draw(m_Shape);
}

void BaseParticle::UpdateNeighbors()
{
	ParticleManager& particleMan = ParticleManager::GetInstance();
	std::vector<BaseParticle*>& particleList = particleMan.GetParticles();

	// Clear the neighbor lists
	m_FluidNeighborParticles.clear();
	m_DeformableNeighborParticles.clear();

	// Update neighbors
	std::map<int, std::vector<int>>& buckets = SpatialPartition::GetInstance().GetBuckets();

	for (int i = 0; i < m_cellIDsList.size(); i++)
	{
		std::vector<int>& currentBucket = buckets[m_cellIDsList[i]];
		unsigned int iCurrentBucketSize = currentBucket.size();

		for (unsigned int index = 0; index < iCurrentBucketSize; index++)
		{
			// Get the current element
			BaseParticle* pCurrentParticle = particleList[currentBucket[index]];

			if (pCurrentParticle->Index != Index)
			{
				if (pCurrentParticle->ParticleType == ParticleType::FluidParticle)
				{
					m_FluidNeighborParticles.push_back(pCurrentParticle->GlobalIndex);
				}
				else
				{
					m_DeformableNeighborParticles.push_back(pCurrentParticle->GlobalIndex);
				}
			}
		}
	}
}

void BaseParticle::UpdateCellIds()
{
	m_cellIDsList.resize(0);

	float iXPosMRad = (LocalPosition.x - Radius) * INVERSE_CELL_SIZE;
	float iXPosPRad = (LocalPosition.x + Radius) * INVERSE_CELL_SIZE;
	float iYPosMRad = (LocalPosition.y - Radius) * INVERSE_CELL_SIZE;
	float iYPosPRad = (LocalPosition.y + Radius) * INVERSE_CELL_SIZE;

	int iFloorXM = Floor(iXPosMRad);
	int iFloorXP = Floor(iXPosPRad);
	int iFloorYM = Floor(iYPosMRad);
	int iFloorYP = Floor(iYPosPRad);

	// Top left corner
	int iCellIndex = iFloorXM + iFloorYM * CELL_COLS;
	m_cellIDsList.push_back(iCellIndex);

	// Top right corner
	iCellIndex = iFloorXP + iFloorYM * CELL_COLS;
	if (IsUnique(iCellIndex))
	{
		m_cellIDsList.push_back(iCellIndex);
	}

	// Bottom left corner
	iCellIndex = iFloorXM + iFloorYP * CELL_COLS;
	if (IsUnique(iCellIndex))
	{
		m_cellIDsList.push_back(iCellIndex);
	}

	// Bottom right corner
	iCellIndex = iFloorXP + iFloorYP * CELL_COLS;
	if (IsUnique(iCellIndex))
	{
		m_cellIDsList.push_back(iCellIndex);
	}
}

const bool BaseParticle::IsUnique(int element) const
{
	unsigned int index = 0;
	unsigned int size = m_cellIDsList.size();

	bool unique = true;

	for (index = 0; index < size; index++)
	{
		if (element == m_cellIDsList[index])
		{
			return false;
		}
	}

	return unique;
}
