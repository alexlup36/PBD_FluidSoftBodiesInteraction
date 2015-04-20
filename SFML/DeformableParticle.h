#ifndef DEFORMABLEPARTICLE_H
#define DEFORMABLEPARTICLE_H

#include "BaseParticle.h"
#include "ParticleManager.h"

struct Edge
{
	DeformableParticle* Start;
	DeformableParticle* End;
};

class SoftBody;

class DeformableParticle : public BaseParticle
{
public:
	DeformableParticle()
	{

	}

	DeformableParticle(const glm::vec2& position, const sf::Color& color, unsigned int iParentIndex)
		: BaseParticle(position, iParentIndex)
	{
		// Particle type
		ParticleType = ParticleType::DeformableParticle;

		// Color
		m_ControlledColor	= sf::Color::Red;
		m_DefaultColor		= color;
		SetDefaultColor();

		// Position
		OriginalPosition	= position;
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

	virtual void Update();
	void Draw(sf::RenderWindow& window) override;
	
	void UpdateGoalShapePosition();
	void DrawGoalShape(sf::RenderWindow& window);

	float CalculateMinimumTranslationDistance();

	inline void SetControlledColor() { m_Shape.setFillColor(m_ControlledColor); }

	inline bool IsFixedParticle() { return m_bFixed; }

	inline void SetParentRef(SoftBody* parent) { m_pParentReference = parent; }
	inline SoftBody* GetParent() { return m_pParentReference; }

	// ------------------------------------------------------------------------
	// Public members
	// ------------------------------------------------------------------------

	glm::vec2 OriginalPosition;
	glm::vec2 GoalPosition;

private:
	static int DeformableParticleGlobalIndex;

	// ------------------------------------------------------------------------
	// Private members
	// ------------------------------------------------------------------------

	SoftBody* m_pParentReference;

	// Fixed particle infinite mass
	bool m_bFixed;

	// Color
	sf::Color m_ControlledColor;

	sf::CircleShape m_GoalShape;

	glm::vec2 m_ClosesPoint;
	glm::vec2 m_vIntersectionPoint;
	Edge m_ClosestEdge;
};

#endif // DEFORMABLEPARTICLE_H