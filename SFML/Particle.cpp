#include "Particle.h"

#include <iostream>

int Particle::m_iGlobalIndex = 0;

Particle::Particle(const sf::Vector2f& position, float radius)
{
	m_Shape.setPosition(position);
	m_Shape.setRadius(radius);
	m_Shape.setOutlineColor(sf::Color::Red);
	m_Shape.setOutlineThickness(1.0f);
	m_Shape.setFillColor(sf::Color::Green);

	sf::FloatRect rect = m_Shape.getLocalBounds();
	m_Shape.setOrigin(rect.width / 2.0f, rect.height / 2.0f);

	m_Position				= sf::Vector2f(position.x, position.y);
	m_PredictedPosition		= m_Position;

	float fVelX = (float)(rand() % 200 - 100);
	float fVelY = (float)(rand() % 200 - 100);
	m_Velocity	= sf::Vector2f(fVelX, fVelY);
	m_Force		= sf::Vector2f(0.0f, 0.0f);
	m_fMass		= (float)(rand() % 100 + 10);//10.0f;
	m_fInvMass	= 1.0f / m_fMass;
	m_fRadius	= radius;

	// Increment particle index
	m_iParticleIndex = m_iGlobalIndex++;

	//std::cout << "Crated particle: " << m_iParticleIndex << std::endl;
}


Particle::~Particle()
{
}

void Particle::Update(float dt)
{
	UpdateExternalForces(dt);
	DampVelocity();
	CalculatePredictedPosition(dt);

	// Update the position of the shape
	m_Shape.setPosition(m_Position);

	//// Wall collision -> change the direction of the velocity
	//if (IsAtLimit())
	//{
	//	// Collision left or right limit
	//	if (m_bLeft || m_bRight)
	//	{
	//		m_Velocity.x = -m_Velocity.x;
	//	}

	//	// Collision top or bottom limit
	//	if (m_bTop || m_bBottom)
	//	{
	//		m_Velocity.y = -m_Velocity.y;
	//	}

	//	// Change the color of the particle
	//	m_Shape.setFillColor(sf::Color::Blue);
	//}
	//else
	//{
	//	m_Shape.setFillColor(sf::Color::Green);
	//}
}

void Particle::Draw(sf::RenderWindow& window)
{
	window.draw(m_Shape);
}

void Particle::UpdateExternalForces(float dt)
{
	if (GRAVITY_ON)
	{
		// Update particle force
		m_Force = GRAVITATIONAL_ACCELERATION * m_fMass;

		// Add more forces

		// Update particle velocity
		m_Velocity += dt * m_fInvMass * m_Force;
	}
}

void Particle::DampVelocity()
{
	// Damp velocity
	m_Velocity.x *= VELOCITY_DAMPING;
	m_Velocity.y *= VELOCITY_DAMPING;
}

void Particle::CalculatePredictedPosition(float dt)
{
	// Update position
	m_PredictedPosition.x += dt * m_Velocity.x;
	m_PredictedPosition.y += dt * m_Velocity.y;

	// Clamp the position to the particle limits
	m_PredictedPosition.x = std::max(PARTICLE_LEFTLIMIT, std::min(m_PredictedPosition.x, PARTICLE_RIGHTLIMIT));
	m_PredictedPosition.y = std::max(PARTICLE_TOPLIMIT, std::min(m_PredictedPosition.y, PARTICLE_BOTTOMLIMIT));
}

void Particle::UpdateActualPosAndVelocity()
{

}

