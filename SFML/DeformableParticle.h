#ifndef DEFORMABLEPARTICLE_H
#define DEFORMABLEPARTICLE_H

#include "BaseParticle.h"
#include "ParticleManager.h"
//#include "SoftBody.h"

class SoftBody;

class DeformableParticle : public BaseParticle
{
public:
	DeformableParticle()
	{

	}

	DeformableParticle(const glm::vec2& position, unsigned int iParentIndex)
		: BaseParticle(position, iParentIndex)
	{
		// Particle type
		ParticleType = ParticleType::DeformableParticle;

		// Color
		m_ControlledColor	= sf::Color::Red;
		m_DefaultColor		= sf::Color::Yellow;
		SetDefaultColor();

		// Position
		OriginalPosition	= position;
		NewPosition			= position;
		GoalPosition		= position;

		// Fixed
		m_bFixed = false;
		if (m_bFixed)
		{
			InverseMass = 0.0f;
		}

		// Index
		Index = DeformableParticleGlobalIndex++;

		// Goal shape properties
		m_GoalShape.setPosition(sf::Vector2<float>(GoalPosition.x, GoalPosition.y));
		m_GoalShape.setRadius(PARTICLE_RADIUS);
		m_GoalShape.setOutlineColor(sf::Color::Cyan);
		m_GoalShape.setOutlineThickness(1.0f);
		m_GoalShape.setFillColor(sf::Color::Blue);
		m_GoalShape.setOrigin(m_GoalShape.getLocalBounds().width / 2.0f,
			m_GoalShape.getLocalBounds().height / 2.0f);
	}
	
	void UpdateGoalShapePosition();
	void DrawGoalShape(sf::RenderWindow& window);

	inline void SetControlledColor() { m_Shape.setFillColor(m_ControlledColor); }

	inline bool IsFixedParticle() { return m_bFixed; }

	// ------------------------------------------------------------------------
	// Public members
	// ------------------------------------------------------------------------

	glm::vec2 OriginalPosition;
	glm::vec2 NewPosition;
	glm::vec2 GoalPosition;

private:
	static int DeformableParticleGlobalIndex;

	// ------------------------------------------------------------------------
	// Private members
	// ------------------------------------------------------------------------

	// Fixed particle infinite mass
	bool m_bFixed;

	// Color
	sf::Color m_ControlledColor;

	sf::CircleShape m_GoalShape;
};

#endif // DEFORMABLEPARTICLE_H