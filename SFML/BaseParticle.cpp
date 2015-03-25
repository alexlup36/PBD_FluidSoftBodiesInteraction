#include "BaseParticle.h"

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

	Position = glm::vec2(position.x, position.y);
	Velocity = glm::vec2(0.0f, 0.0f);

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