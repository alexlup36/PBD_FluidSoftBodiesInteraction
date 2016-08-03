#ifndef SOFTBODYPARTICLE_H
#define SOFTBODYPARTICLE_H

#include "Common.h"

class SoftBodyParticle
{
public:
	SoftBodyParticle()
	{

	}

	SoftBodyParticle(const glm::vec2& originalPosition)
		: OriginalPosition(originalPosition)
	{
		Position = originalPosition;
		NewPosition = originalPosition;
		GoalPosition = originalPosition;

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

#endif // SOFTBODYPARTICLE_H