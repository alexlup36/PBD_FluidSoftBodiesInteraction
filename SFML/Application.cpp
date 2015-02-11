#include "Application.h"

#include <time.h>
#include <memory>


Application::Application()
{
	srand((unsigned int)time(NULL));
}

Application::~Application()
{
}

void Application::Initialize()
{
	if (!m_Texture.loadFromFile("sprite.png"))
	{
		std::cout << "Failed to load texture file." << std::endl;
	}

	// ---------------------------------------------------------------------------
	// Spatial partition

	m_SpatialManager.Setup();
}

void Application::Update(float dt)
{
	// Reset the spatial manager
	m_SpatialManager.ClearBuckets();

	for (int index = 0; index < PARTICLE_COUNT; index++)
	{
		m_ParticleList[index].Update(dt);

		// Repopulate the spatial manager with the particles
		m_SpatialManager.RegisterObject(&m_ParticleList[index]);
	}

	// Collision with other particles
	for (int index = 0; index < PARTICLE_COUNT; index++)
	{
		// Get the neighbors of the current particle
		std::vector<Particle*> neighborList = m_SpatialManager.GetNeighbors(m_ParticleList[index]);

		for each (Particle* particle in neighborList)
		{
			// Don't check for collision with itself
			if (m_ParticleList[index].GetParticleIndex() != particle->GetParticleIndex())
			{
				// Mark the particle as a neighbor
				particle->SetAsNeighbor();

				// Check if there is a collision between particles
				if (m_ParticleList[index].IsCollision(*particle))
				{
					sf::Vector2f P1Velocity = m_ParticleList[index].GetVelocity();
					sf::Vector2f P2Velocity = particle->GetVelocity();
					float P1Mass = m_ParticleList[index].GetParticleMass();
					float P2Mass = particle->GetParticleMass();

					// Debug - print particle velocities before the collision
					/*std::cout << "Velocities before collision" << std::endl;
					std::cout << "Particle 1: ";
					PrintVector2(m_ParticleList[index].GetVelocity());
					std::cout << "Particle 2: ";
					PrintVector2(particle->GetVelocity());*/

					// Collision response

					// Calculate the unit normal and unit tangent
					sf::Vector2f normal = sf::Vector2f(P2Velocity.x - P1Velocity.x, P2Velocity.y - P1Velocity.y);
					float fNormalLength = sqrt(normal.x * normal.x + normal.y * normal.y);
					normal.x /= fNormalLength;
					normal.y /= fNormalLength;

					sf::Vector2f tangent = sf::Vector2f(-normal.y, normal.x);

					// Distance between centers
					float fDx = particle->GetPosition().x - m_ParticleList[index].GetPosition().x;
					float fDy = particle->GetPosition().y - m_ParticleList[index].GetPosition().y;
					float fDistance = sqrt(fDx * fDx + fDy * fDy);
					sf::Vector2f offset = normal * (2.0f * PARTICLE_RADIUS - fDistance + 1.0f);
					particle->SetPosition(particle->GetPosition() + offset);

					// Project the velocities on the normal and tangent direction
					float fP1n = Dot(normal, P1Velocity);
					float fP1t = Dot(tangent, P1Velocity);
					float fP2n = Dot(normal, P2Velocity);
					float fP2t = Dot(tangent, P2Velocity);

					// Calculate the components of the velocities after the collision
					float fP1nFinal = (fP1n * (P1Mass - P2Mass) + 2.0f * P2Mass * fP2n) / (P1Mass + P2Mass);
					float fP2nFinal = (fP2n * (P2Mass - P1Mass) + 2.0f * P1Mass * fP1n) / (P1Mass + P2Mass);
					float fP1tFinal = fP1t;
					float fP2tFinal = fP2t;

					// Calculate the updated velocities
					m_ParticleList[index].SetVelocity(normal * fP1nFinal + tangent * fP1tFinal);
					particle->SetVelocity(normal * fP2nFinal + tangent * fP2tFinal);

					m_ParticleList[index].SetIsColliding();
					particle->SetIsColliding();

					// Debug - print particle velocities after the collision
					/*std::cout << "Velocities after collision" << std::endl;
					std::cout << "Particle 1: ";
					PrintVector2(m_ParticleList[index].GetVelocity());
					std::cout << "Particle 2: ";
					PrintVector2(particle->GetVelocity());*/

					int x = 0;

					//std::cout << "Collision between particle: " << m_ParticleList[index].GetParticleIndex() << " and particle: " << particle->GetParticleIndex() << std::endl;
				}
			}
		}
	}
}

void Application::Draw(sf::RenderWindow& window)
{
	// Draw limits
	sf::Vertex line[] =
	{
		sf::Vertex(sf::Vector2f(WALL_LEFTLIMIT, WALL_TOPLIMIT)), // Top limit
		sf::Vertex(sf::Vector2f(WALL_RIGHTLIMIT, WALL_TOPLIMIT)),

		sf::Vertex(sf::Vector2f(WALL_LEFTLIMIT, WALL_BOTTOMLIMIT)), // Bottom limit
		sf::Vertex(sf::Vector2f(WALL_RIGHTLIMIT, WALL_BOTTOMLIMIT)),

		sf::Vertex(sf::Vector2f(WALL_LEFTLIMIT, WALL_TOPLIMIT)), // Left limit
		sf::Vertex(sf::Vector2f(WALL_LEFTLIMIT, WALL_BOTTOMLIMIT)),

		sf::Vertex(sf::Vector2f(WALL_RIGHTLIMIT, WALL_TOPLIMIT)), // Right limit
		sf::Vertex(sf::Vector2f(WALL_RIGHTLIMIT, WALL_BOTTOMLIMIT)),
	};

	window.draw(line, 8, sf::Lines);

	// Draw particles
	for (int index = 0; index < PARTICLE_COUNT; index++)
	{
		m_ParticleList[index].Draw(window);
	}
}

sf::Vector2f Application::GetRandomPosWithinLimits(float fRadius)
{
	int iXPosition = rand() % (int)(PARTICLE_RIGHTLIMIT - PARTICLE_LEFTLIMIT + PARTICLE_LEFTLIMIT);
	int iYPosition = rand() % (int)(PARTICLE_BOTTOMLIMIT - PARTICLE_TOPLIMIT + PARTICLE_TOPLIMIT);

	return sf::Vector2f((float)iXPosition, (float)iYPosition);
}

void Application::BuildParticleSystem(int iParticleCount)
{
	for (int index = 0; index < PARTICLE_COUNT; index++)
	{
		std::shared_ptr<Particle> particle = std::make_shared<Particle>(GetRandomPosWithinLimits(PARTICLE_RADIUS), PARTICLE_RADIUS);

		// Build particle list
		m_ParticleList.push_back(*particle);
	}
}