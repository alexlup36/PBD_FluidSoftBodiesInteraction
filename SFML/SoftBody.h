#ifndef SOFTBODY_H
#define SOFTBODY_H

#include "Common.h"
#include <stack>
#include <algorithm>

// ----------------------------------------------------------------------------

struct SoftBodyParticle
{
	SoftBodyParticle()
	{

	}

	SoftBodyParticle(const glm::vec2& originalPosition)
		: OriginalPosition(originalPosition) 
	{
		Position		= originalPosition;
		NewPosition		= originalPosition;
		GoalPosition	= originalPosition;

		float fVelX = 0.0f; // (float)(rand() % 200 - 100);
		float fVelY = 0.0f; // (float)(rand() % 200 - 100);
		Velocity = glm::vec2(fVelX, fVelY);

		Fixed = false;
		Mass = 1.0f;

		// Increment particle index
		m_iSoftBodyParticleIndex = m_iGlobalIndex++;
		// Set the particle radius
		m_fRadius = SOFTBODY_PARTICLE_RADIUS;

		// Set the shape properties
		m_Shape.setPosition(sf::Vector2<float>(Position.x, Position.y));
		m_Shape.setRadius(SOFTBODY_PARTICLE_RADIUS);
		m_Shape.setOutlineColor(sf::Color::Yellow);
		m_Shape.setOutlineThickness(1.0f);
		m_Shape.setFillColor(sf::Color::Green);
		sf::FloatRect rect = m_Shape.getLocalBounds();
		m_Shape.setOrigin(rect.width / 2.0f, rect.height / 2.0f);

		// Set the goal position shape properties
		m_GoalPositionShape.setPosition(sf::Vector2<float>(GoalPosition.x, GoalPosition.y));
		m_GoalPositionShape.setRadius(SOFTBODY_PARTICLE_RADIUS);
		m_GoalPositionShape.setOutlineColor(sf::Color::Cyan);
		m_GoalPositionShape.setOutlineThickness(1.0f);
		m_GoalPositionShape.setFillColor(sf::Color::Blue);
		rect = m_GoalPositionShape.getLocalBounds();
		m_GoalPositionShape.setOrigin(rect.width / 2.0f, rect.height / 2.0f);
	}

	glm::vec2 OriginalPosition;
	glm::vec2 Position;
	glm::vec2 NewPosition;
	glm::vec2 GoalPosition;
	glm::vec2 Velocity;
	bool Fixed;
	float Mass;

	void UpdateShapePosition();
	void Draw(sf::RenderWindow& window);

	void UpdateGoalShapePosition();
	void DrawGoalShape(sf::RenderWindow& window);

	inline int GetSBParticleIndex() { return m_iSoftBodyParticleIndex; }
	inline void SetControlledColor() { m_Shape.setFillColor(sf::Color::Red); }
	inline void SetDefaultColor() { m_Shape.setFillColor(sf::Color::Green); }

private:

	static int m_iGlobalIndex;
	int m_iSoftBodyParticleIndex;

	sf::CircleShape m_Shape;
	sf::CircleShape m_GoalPositionShape;

	float m_fRadius;
};

// ----------------------------------------------------------------------------

class SoftBody
{
public:
	SoftBody();

	void Update(float dt);
	void Draw(sf::RenderWindow& window);

	inline unsigned int GetParticleCount() { return m_SoftBodyParticles.size(); }
	inline void ClearSoftBodyParticleList() { m_SoftBodyParticles.clear(); }
	inline void AddSoftBodyParticle(const SoftBodyParticle& sbParticle) { m_SoftBodyParticles.push_back(sbParticle); }
	inline void SetReady(bool ready) { m_bReady = ready; }
	inline bool IsReady() { return m_bReady; }

	inline std::vector<SoftBodyParticle>& GetParticleList() { return m_SoftBodyParticles; }

private:
	bool m_bAllowFlipping;
	bool m_bVolumeConservation;
	bool m_bLinearMatch;
	bool m_bQuadraticMatch;
	bool m_bReady;
	bool m_bDrawGoalPositions;

	bool m_bConvexHull;

	float m_fStiffness;
	float m_fBeta;

	std::vector<SoftBodyParticle> m_SoftBodyParticles;

	void UpdateForces(float dt);
	void ShapeMatching(float dt);

	// Graham scan
	inline static bool SmallestY(SoftBodyParticle& p1, SoftBodyParticle& p2)
	{
		if (p1.Position.y != p2.Position.y)
		{
			return p1.Position.y > p2.Position.y;
		}
		return p1.Position.x < p2.Position.x;
	}

	inline static bool PolarOrder(SoftBodyParticle& p1, SoftBodyParticle& p2)
	{
		int order = CCWTurn(m_Pivot, p1, p2);
		if (order == 0)
		{
			return glm::length(m_Pivot.Position - p1.Position) <
				glm::length(m_Pivot.Position - p2.Position);
		}
		return order == -1;
	}

	inline static int CCWTurn(const SoftBodyParticle& p1,
		const SoftBodyParticle& p2,
		const SoftBodyParticle& p3)
	{
		int iArea = (int)((p2.Position.x - p1.Position.x) * (p3.Position.y - p1.Position.y) -
			(p2.Position.y - p1.Position.y) * (p3.Position.x - p1.Position.x));

		if (iArea > 0)
		{
			return -1;
		}
		else if (iArea < 0)
		{
			return 1;
		}

		return 0;
	}
	static SoftBodyParticle m_Pivot;

	std::stack<SoftBodyParticle*> m_ConvexHull;
};

#endif // SOFTBODY_H