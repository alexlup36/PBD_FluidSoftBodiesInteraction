#include "Particle.h"

#include <iostream>

int Particle::m_iGlobalIndex = 0;

Particle::Particle(const glm::vec2& position, float radius)
{
	m_Shape.setPosition(sf::Vector2<float>(position.x, position.y));
	m_Shape.setRadius(radius);
	m_Shape.setOutlineColor(sf::Color::Red);
	m_Shape.setOutlineThickness(1.0f);
	SetDefaultColor();

	sf::FloatRect rect = m_Shape.getLocalBounds();
	m_Shape.setOrigin(rect.width / 2.0f, rect.height / 2.0f);

	m_Position				= glm::vec2(position.x, position.y);
	m_LocalPosition			= glm::vec2(m_Position.x - WALL_LEFTLIMIT, m_Position.y - WALL_TOPLIMIT);
	m_PredictedPosition		= m_Position;

	float fVelX = 100.0f;// 0.0f;// (float)(rand() % 200 - 100);
	float fVelY = 100.0f;// 0.0f;// (float)(rand() % 200 - 100);
	m_Velocity		= glm::vec2(fVelX, fVelY);
	m_Force			= glm::vec2(0.0f, 0.0f);
	m_fMass			= 1.0f;
	m_fInvMass		= 1.0f / m_fMass;
	m_fRadius		= radius;
	m_fRestDensity  = WATER_DENSITY;
	m_fSPHDensity	= 0.0f;

	// Increment particle index
	m_iParticleIndex = m_iGlobalIndex++;

	//std::cout << "Crated particle: " << m_iParticleIndex << std::endl;
}


Particle::~Particle()
{

}

void Particle::Draw(sf::RenderWindow& window)
{
	window.draw(m_Shape);
}

void Particle::UpdateShapePosition()
{
	// Update the position of the shape
	m_Shape.setPosition(sf::Vector2<float>(m_Position.x, m_Position.y));
}
