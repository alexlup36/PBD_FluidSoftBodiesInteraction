#ifndef BASEPARTICLE_H
#define BASEPARTICLE_H

#include "Common.h"

class BaseSimulation;

class BaseParticle
{
public:
	BaseParticle() { }
	BaseParticle(const glm::vec2& position, unsigned int parentIndex);
	virtual ~BaseParticle();

	// ------------------------------------------------------------------------
	// Public methods 
	// ------------------------------------------------------------------------

	virtual void Update();
	virtual void Draw(sf::RenderWindow& window);

	// Colors
	inline void SetDefaultColor() { m_Shape.setFillColor(m_DefaultColor); }
	inline void SetNeighborColor() { m_Shape.setFillColor(m_NeighborColor); }
	inline void SetCollisionColor() { m_Shape.setFillColor(m_CollisionColor); }

	inline bool IsColliding(const BaseParticle& other)
	{
		float fDx = Position.x - other.Position.x;
		float fDy = Position.y - other.Position.y;

		return PARTICLE_RADIUS2 > (fDx * fDx) + (fDy * fDy);
	}

	inline float DistanceToLine(const glm::vec2& p1, const glm::vec2& p2)
	{
		float p1p2Length = glm::length(p1 - p2);
		if (p1p2Length != 0.0f)
		{
			return ((p2.y - p1.y) * Position.x - (p2.x - p1.x) * Position.y + p2.x * p1.y - p2.y * p1.x) / p1p2Length;
		}
		else
		{
			return 0.0f;
		}
	}
		

	inline const unsigned int GetParentIndex() const { return m_iParentSimulationIndex; }

	// ------------------------------------------------------------------------
	// Public members
	// ------------------------------------------------------------------------

	// Particle index
	int Index;
	int GlobalIndex;

	ParticleType ParticleType;

	float Radius;
	float Mass;
	float InverseMass;

	glm::vec2 Position;
	glm::vec2 Velocity;

protected:

	// ------------------------------------------------------------------------
	// Protected members
	// ------------------------------------------------------------------------

	static int ParticleGlobalIndex;

	unsigned int m_iParentSimulationIndex;

	// Color
	sf::Color m_DefaultColor;
	sf::Color m_CollisionColor;
	sf::Color m_NeighborColor;

	// Particle shape
	sf::CircleShape m_Shape;
};

#endif // BASEPARTICLE_H