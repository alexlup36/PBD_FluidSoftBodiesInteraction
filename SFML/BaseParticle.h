#ifndef BASEPARTICLE_H
#define BASEPARTICLE_H

#include "Common.h"

#include <set>

class BaseSimulation;

enum class ParticleType
{
	FluidParticle,
	DeformableParticle,

	Invalid
};

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

	// Neighbors
	void UpdateNeighbors();
	inline std::vector<int>& GetFluidNeighbors() { return m_FluidNeighborParticles; }
	inline std::vector<int>& GetSoftNeighbors() { return m_DeformableNeighborParticles; }

	void UpdateCellIds();
	const bool IsUnique(int element) const;
	
	std::vector<int>& GetCellIDsList() { return m_cellIDsList; }

	// Colors
	inline void SetDefaultColor() { m_Shape.setFillColor(m_DefaultColor); }
	inline void SetNeighborColor() { m_Shape.setFillColor(m_NeighborColor); }
	inline void SetCollisionColor() { m_Shape.setFillColor(m_CollisionColor); }

	inline void SetDefaultColor(const sf::Color& newColor) { m_DefaultColor = newColor; }

	inline bool IsCollidingStatic(BaseParticle& other)
	{
		float fDx = Position.x - other.Position.x;
		float fDy = Position.y - other.Position.y;

		if (PARTICLE_RADIUS2 > (fDx * fDx) + (fDy * fDy))
		{
			/*SetCollisionColor();
			other.SetCollisionColor();*/
			return true;
		}
		else
		{
			/*SetDefaultColor();
			other.SetDefaultColor();*/
			return false;
		}
	}

	inline bool IsCollidingDynamicStatic( BaseParticle& dynamicCircle,
		BaseParticle& staticCircle)
	{
		glm::vec2 c1c2 = dynamicCircle.Position - staticCircle.Position;
		glm::vec2 relVel = dynamicCircle.Velocity - staticCircle.Velocity;

		float c = glm::dot(c1c2, c1c2) - PARTICLE_RADIUS2;
		if (c < 0.0f)
		{
			/*dynamicCircle.SetDefaultColor();
			staticCircle.SetDefaultColor();*/
			return true;
		}

		float a = glm::dot(relVel, relVel);
		if (a < EPS)
		{
			/*dynamicCircle.SetDefaultColor();
			staticCircle.SetDefaultColor();*/
			return false;
		}

		float b = glm::dot(relVel, c1c2);
		if (b >= 0.0f)
		{
			/*dynamicCircle.SetDefaultColor();
			staticCircle.SetDefaultColor();*/
			return false;
		}

		float d = b * b - a * c;
		if (d < 0.0f)
		{
			/*dynamicCircle.SetDefaultColor();
			staticCircle.SetDefaultColor();*/
			return false;
		}

		/*float t = (-b - sqrt(d)) / a;
		dynamicCircle.Position += dynamicCircle.Velocity * t;
		staticCircle.Position += staticCircle.Velocity * t;*/

		/*dynamicCircle.SetCollisionColor();
		staticCircle.SetCollisionColor();*/
		return true;
	}

	inline bool IsCollidingDynamic(BaseParticle& other)
	{
		glm::vec2 d = ClosestPointToPointOnLine(Position, Position + Velocity, other.Position);

		float fDx = d.x - other.Position.x;
		float fDy = d.y - other.Position.y;

		float fSqrDistance = fDx * fDx + fDy * fDy;

		if (fSqrDistance < PARTICLE_RADIUS2)
		{
			/*SetCollisionColor();
			other.SetCollisionColor();*/
			return true;
		}
		else
		{
			/*SetDefaultColor();
			other.SetDefaultColor();*/
			return false;
		}
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

	inline glm::vec2 ClosestPointToPointOnLine(const glm::vec2& p1, const glm::vec2& p2, const glm::vec2& point)
	{
		// Intersection point
		glm::vec2 c = glm::vec2(0.0f);

		// Find the equation of the line
		float fA1 = p2.y - p1.y;
		float fB1 = p1.x - p2.x;
		float fC1 = p1.x * p2.y - p2.x * p1.y;

		// Find the equation of the perpendicular which goes through the current point
		float fA2 = fA1;
		float fB2 = -fB1;
		float fC2 = fB2 * point.x + fA2 * point.y;

		// Calculate the determinant of the system for the 2 equations
		float fDet = fA1 * fA1 + fB1 * fB1;

		if (fDet == 0.0f)
		{
			// If the determinant is 0.0 the closest point on the line is the point itself
			c = Position;
		}
		else
		{
			// Cramer rule to solve the system
			c.x = (fA1 * fC1 - fB1 * fC2) / fDet;
			c.y = (fA1 * fC2 + fB1 * fC1) / fDet;
		}

		return c;
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
	glm::vec2 LocalPosition;
	glm::vec2 PredictedPosition;
	glm::vec2 PositionCorrection;
	glm::vec2 Velocity;

	float SignedDistance;
	glm::vec2 GradientSignedDistance;

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

	std::vector<int> m_FluidNeighborParticles;
	std::vector<int> m_DeformableNeighborParticles;

	// List of IDs of the cell the current particle is in
	std::vector<int> m_cellIDsList;
};

#endif // BASEPARTICLE_H