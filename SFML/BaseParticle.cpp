#include "BaseParticle.h"

#include "SpatialPartition.h"

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

	// Reset cell ids
	m_cellIDsList.clear();
}

void BaseParticle::Draw(sf::RenderWindow& window)
{
	window.draw(m_Shape);
}

void BaseParticle::UpdateNeighbors()
{
	// Clear the neighbor lists
	m_FluidNeighborParticles.clear();
	m_DeformableNeighborParticles.clear();

	// Update neighbors
	std::map<int, std::vector<BaseParticle*>>& buckets = SpatialPartition::GetInstance().GetBuckets();

	for (std::set<int>::iterator it = m_cellIDsList.begin(); it != m_cellIDsList.end(); it++)
	{
		std::vector<BaseParticle*>& currentBucket = buckets[*it];
		unsigned int iCurrentBucketSize = currentBucket.size();

		for (unsigned int index = 0; index < iCurrentBucketSize; index++)
		{
			// Get the current element
			BaseParticle* pCurrentParticle = currentBucket[index];

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
	float fXPos = LocalPosition.x;
	float fYPos = LocalPosition.y;
	float fRadius = Radius;

	// Top left corner
	int iCellIndex = (int)(std::floor((fXPos - fRadius) / CELL_SIZE) +
		std::floor((fYPos - fRadius) / CELL_SIZE) * CELL_COLS);
	m_cellIDsList.insert(iCellIndex);

	// Top right corner
	iCellIndex = (int)(std::floor((fXPos + fRadius) / CELL_SIZE) +
		std::floor((fYPos - fRadius) / CELL_SIZE) * CELL_COLS);
	m_cellIDsList.insert(iCellIndex);

	// Bottom left corner
	iCellIndex = (int)(std::floor((fXPos - fRadius) / CELL_SIZE) +
		std::floor((fYPos + fRadius) / CELL_SIZE) * CELL_COLS);
	m_cellIDsList.insert(iCellIndex);

	// Bottom right corner
	iCellIndex = (int)(std::floor((fXPos + fRadius) / CELL_SIZE) +
		std::floor((fYPos + fRadius) / CELL_SIZE) * CELL_COLS);
	m_cellIDsList.insert(iCellIndex);
}
