#include "Particle.h"

#include <iostream>

int Particle::m_iGlobalIndex = 0;

Particle::Particle(const sf::Vector2f& position, float radius)
{
	m_Shape.setPosition(position);
	m_Shape.setRadius(radius);
	m_Shape.setOutlineColor(sf::Color::Red);
	m_Shape.setOutlineThickness(1.0f);
	SetDefaultColor();

	sf::FloatRect rect = m_Shape.getLocalBounds();
	m_Shape.setOrigin(rect.width / 2.0f, rect.height / 2.0f);

	m_Position				= sf::Vector2f(position.x, position.y);
	m_LocalPosition			= sf::Vector2f(m_Position.x - WALL_LEFTLIMIT, m_Position.y - WALL_TOPLIMIT);
	m_PredictedPosition		= m_Position;

	float fVelX = 0.0f;// (float)(rand() % 200 - 100);
	float fVelY = 0.0f;// (float)(rand() % 200 - 100);
	m_Velocity = sf::Vector2f(fVelX, fVelY);
	m_Force		= sf::Vector2f(0.0f, 0.0f);
	m_fMass		= 1.0f;
	m_fInvMass	= 1.0f / m_fMass;
	m_fRadius	= radius;
	m_fRestDensity  = WATER_DENSITY;
	m_fSPHDensity = 0.0f;

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
	m_Shape.setPosition(m_Position);
}
