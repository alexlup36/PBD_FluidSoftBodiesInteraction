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

void Application::Initialize(sf::RenderWindow& window)
{
	if (!m_Texture.loadFromFile("sprite.png"))
	{
		std::cout << "Failed to load texture file." << std::endl;
	}

	// ---------------------------------------------------------------------------
	// Spatial partition

	m_SpatialManager.Setup();
}

void Application::Update(sf::RenderWindow& window, float dt)
{
	// Reset the spatial manager
	m_SpatialManager.ClearBuckets();

	UpdateExternalForces(dt);
	DampVelocities();
	CalculatePredictedPositions(window, dt);
	FindNeighborParticles();

	// Generate external collision constraints
	GenerateCollisionConstraints(window);

	// Store the neighbors of each particle in the current step
	std::vector<Particle*> neighbors[PARTICLE_COUNT];

	// Project constraints
	int iIteration = 0;
	while (iIteration++ < SOLVER_ITERATIONS)
	{
		for (unsigned int i = 0; i < m_ParticleList.size(); i++)
		{
			// Get current particle
			Particle currentParticle = m_ParticleList[i];

			// Get the neighbors of the current particle
			//std::vector<Particle*> neighborList;
			m_SpatialManager.GetNeighbors(currentParticle, neighbors[i]);

			// Velocity accumulator
			float accumulatorDensity = 0.0f;
			float accumulatorGradient = 0.0f;

			// Gradient for neighbors
			for (unsigned int j = 0; j < neighbors[i].size(); j++)
			{
				glm::vec2 neighborParticlePosition = neighbors[i][j]->GetPosition();

				// Calculate the vector between the particles
				glm::vec2 r = currentParticle.GetPosition() - neighborParticlePosition;
				// Use poly6 smoothing kernel
				accumulatorDensity += (Poly6(r, SMOOTHING_DISTANCE) * currentParticle.GetParticleMass());

				// Calculate the gradient of the spiky kernel
				glm::vec2 spikyGradient = -1.0f / WATER_DENSITY * SpikyGradient(r, SMOOTHING_DISTANCE);
				float spikyGradientLength = glm::length(spikyGradient);
				accumulatorGradient += (spikyGradientLength * spikyGradientLength);
			}

			// Gradient for non-neighbors
			glm::vec2 accum = glm::vec2(0.0f, 0.0f);
			for (unsigned int j = 0; j < neighbors[i].size(); j++)
			{
				// Calculate the vector between the particles
				glm::vec2 r = currentParticle.GetPosition() - neighbors[i][j]->GetPosition();

				accum += SpikyGradient(r, SMOOTHING_DISTANCE);
			}
			glm::vec2 ciGradient = accum * ONE_OVER_WATER_DENSITY;
			float ciGradientLength = glm::length(ciGradient);
			accumulatorGradient += (ciGradientLength * ciGradientLength);

			// Set the new density
			currentParticle.SetSPHDensity(accumulatorDensity);
			float fDensityConstraint = currentParticle.GetSPHDensity() / WATER_DENSITY - 1.0f;

			// Calculate lambda
			m_ParticleList[i].SetLambda(-1.0f * (fDensityConstraint / (accumulatorGradient + RELAXATION_PARAMETER)));
		}

		// Calculate the delta p for each particle based on the density constraint
		for (unsigned int i = 0; i < m_ParticleList.size(); i++)
		{
			glm::vec2 accumulatorGradient = glm::vec2(0.0f, 0.0f);

			// Gradient for neighbors
			for (unsigned int j = 0; j < neighbors[i].size(); j++)
			{
				// Calculate the gradient of the spiky kernel
				glm::vec2 spikyGradient = SpikyGradient(m_ParticleList[i].GetPosition() - neighbors[i][j]->GetPosition(), 
					SMOOTHING_DISTANCE);
				// Accumulate

				float lambda_i = m_ParticleList[i].GetLambda();
				float lambda_j = neighbors[i][j]->GetLambda();
				accumulatorGradient += (spikyGradient * (lambda_i + lambda_j));
			}

			// Add position correction for the current particle
			m_ParticleList[i].AddPositionCorrection(ONE_OVER_WATER_DENSITY * accumulatorGradient);
		}

		
		// Update container constraints
		for each (auto container_constraint in m_ContainerConstraints)
		{
			glm::vec2 particlePredictedPosition = 
				m_ParticleList[container_constraint.particleIndex].GetPredictedPosition();

			//float constraint = Dot(particlePredictedPosition - container_constraint.projectionPoint, container_constraint.normalVector);
			float constraint = glm::dot(particlePredictedPosition - container_constraint.projectionPoint, container_constraint.normalVector);

			glm::vec2 gradientDescent = container_constraint.normalVector;
			float gradienDescentLength = glm::length(gradientDescent);
			glm::vec2 dp = -constraint / (gradienDescentLength * gradienDescentLength) * gradientDescent;

			m_ParticleList[container_constraint.particleIndex].AddPositionCorrection(dp * container_constraint.stiffness_adjusted);
		}

		// For all particles calculate lambda

		// For all particles calculate dp

		// For all particles update the predicted position
	}

	// Update the actual position and velocity of the particle
	UpdateActualPosAndVelocities(dt);

	// Clear the constraint list
	m_ContainerConstraints.clear();


	//// Collision with other particles
	//for (int index = 0; index < PARTICLE_COUNT; index++)
	//{
	//	// Get the neighbors of the current particle
	//	std::vector<Particle*> neighborList = m_SpatialManager.GetNeighbors(m_ParticleList[index]);

	//	for each (Particle* particle in neighborList)
	//	{
	//		// Don't check for collision with itself
	//		if (m_ParticleList[index].GetParticleIndex() != particle->GetParticleIndex())
	//		{
	//			// Mark the particle as a neighbor
	//			particle->SetAsNeighbor();

	//			// Check if there is a collision between particles
	//			if (m_ParticleList[index].IsCollision(*particle))
	//			{
	//				glm::vec2 P1Velocity = m_ParticleList[index].GetVelocity();
	//				glm::vec2 P2Velocity = particle->GetVelocity();
	//				float P1Mass = m_ParticleList[index].GetParticleMass();
	//				float P2Mass = particle->GetParticleMass();

	//				// Debug - print particle velocities before the collision
	//				/*std::cout << "Velocities before collision" << std::endl;
	//				std::cout << "Particle 1: ";
	//				PrintVector2(m_ParticleList[index].GetVelocity());
	//				std::cout << "Particle 2: ";
	//				PrintVector2(particle->GetVelocity());*/

	//				// Collision response

	//				// Calculate the unit normal and unit tangent
	//				glm::vec2 normal = glm::vec2(P2Velocity.x - P1Velocity.x, P2Velocity.y - P1Velocity.y);
	//				float fNormalLength = sqrt(normal.x * normal.x + normal.y * normal.y);
	//				normal.x /= fNormalLength;
	//				normal.y /= fNormalLength;

	//				glm::vec2 tangent = glm::vec2(-normal.y, normal.x);

	//				// Distance between centers
	//				float fDx = particle->GetPosition().x - m_ParticleList[index].GetPosition().x;
	//				float fDy = particle->GetPosition().y - m_ParticleList[index].GetPosition().y;
	//				float fDistance = sqrt(fDx * fDx + fDy * fDy);
	//				glm::vec2 offset = normal * (2.0f * PARTICLE_RADIUS - fDistance + 1.0f);
	//				particle->SetPosition(particle->GetPosition() + offset);

	//				// Project the velocities on the normal and tangent direction
	//				float fP1n = glm::dot(normal, P1Velocity);
	//				float fP1t = glm::dot(tangent, P1Velocity);
	//				float fP2n = glm::dot(normal, P2Velocity);
	//				float fP2t = glm::dot(tangent, P2Velocity);

	//				// Calculate the components of the velocities after the collision
	//				float fP1nFinal = (fP1n * (P1Mass - P2Mass) + 2.0f * P2Mass * fP2n) / (P1Mass + P2Mass);
	//				float fP2nFinal = (fP2n * (P2Mass - P1Mass) + 2.0f * P1Mass * fP1n) / (P1Mass + P2Mass);
	//				float fP1tFinal = fP1t;
	//				float fP2tFinal = fP2t;

	//				// Calculate the updated velocities
	//				m_ParticleList[index].SetVelocity(normal * fP1nFinal + tangent * fP1tFinal);
	//				particle->SetVelocity(normal * fP2nFinal + tangent * fP2tFinal);

	//				m_ParticleList[index].SetIsColliding();
	//				particle->SetIsColliding();

	//				// Debug - print particle velocities after the collision
	//				/*std::cout << "Velocities after collision" << std::endl;
	//				std::cout << "Particle 1: ";
	//				PrintVector2(m_ParticleList[index].GetVelocity());
	//				std::cout << "Particle 2: ";
	//				PrintVector2(particle->GetVelocity());*/

	//				int x = 0;

	//				//std::cout << "Collision between particle: " << m_ParticleList[index].GetParticleIndex() << " and particle: " << particle->GetParticleIndex() << std::endl;
	//			}
	//		}
	//	}
	//}
}

void Application::Draw(sf::RenderWindow& window)
{
	// Draw particles
	for (int index = 0; index < PARTICLE_COUNT; index++)
	{
		m_ParticleList[index].Draw(window);
	}
}

glm::vec2 Application::GetRandomPosWithinLimits()
{
	int iXPosition = rand() % (int)(PARTICLE_RIGHTLIMIT - PARTICLE_LEFTLIMIT) + (int)PARTICLE_LEFTLIMIT;
	int iYPosition = rand() % (int)(PARTICLE_BOTTOMLIMIT - PARTICLE_TOPLIMIT) + (int)PARTICLE_TOPLIMIT;

	return glm::vec2((float)iXPosition, (float)iYPosition);
}

void Application::BuildParticleSystem(int iParticleCount)
{
	for (int index = 0; index < PARTICLE_COUNT; index++)
	{
		std::shared_ptr<Particle> particle = std::make_shared<Particle>(GetRandomPosWithinLimits(), PARTICLE_RADIUS);

		// Build particle list
		m_ParticleList.push_back(*particle);
	}
}

void Application::UpdateExternalForces(float dt)
{
	for (auto it = m_ParticleList.begin(); it != m_ParticleList.end(); it++)
	{
		if (GRAVITY_ON)
		{
			// Update particle force
			it->SetForce(GRAVITATIONAL_ACCELERATION * it->GetParticleMass());
		}

		// Add more forces
		// -----

		// Update particle velocity
		glm::vec2 deltaVelocity = dt * it->GetParticleMass() * it->GetForce();
		it->AddDeltaVelocity(dt * it->GetParticleMass() * it->GetForce());

		// Reset position correction
		it->ResetPositionCorrection();
	}
}

void Application::DampVelocities()
{
	// Damp velocity
	for (auto it = m_ParticleList.begin(); it != m_ParticleList.end(); it++)
	{
		it->SetVelocity(it->GetVelocity() * VELOCITY_DAMPING);
	}
}

void Application::CalculatePredictedPositions(sf::RenderWindow& window, float dt)
{
	// Calculate the predicted positions
	for (auto it = m_ParticleList.begin(); it != m_ParticleList.end(); it++)
	{
		// Update position
		it->AddDeltaPredPosition(dt * it->GetVelocity());

		// Debug
		/*sf::Vertex line[] =
		{
			sf::Vertex(it->GetPosition()),
			sf::Vertex(it->GetPredictedPosition())
		};

		window.draw(line, 2, sf::Lines);*/
	}
}

void Application::UpdateActualPosAndVelocities(float dt)
{
	for (auto it = m_ParticleList.begin(); it != m_ParticleList.end(); it++)
	{
		// Update predicted position
		it->AddDeltaPredPosition(it->GetPositionCorrection());

		if (dt != 0.0f)
		{
			// Update velocity based on the distance offset (after correcting the position)
			it->SetVelocity((it->GetPredictedPosition() - it->GetPosition()) / dt);
		}

		// Apply XSPH viscosity
		if (XSPH_VISCOSITY)
		{
			XSPH_Viscosity(*it);
		}

		// Update position
		it->SetPosition(it->GetPredictedPosition());

		// Update local position
		glm::vec2 localOffset(WALL_LEFTLIMIT, WALL_TOPLIMIT);
		it->SetLocalPosition(it->GetPosition() - localOffset);

		// Update the position of the particle shape
		it->UpdateShapePosition();
	}
}

void Application::GenerateCollisionConstraints(sf::RenderWindow& window)
{
	for (auto it = m_ParticleList.begin(); it != m_ParticleList.end(); it++)
	{
		if (it->GetPredictedPosition().x < PARTICLE_LEFTLIMIT)
		{
			float fIntersectionCoeff = (PARTICLE_LEFTLIMIT - it->GetPosition().x) / (it->GetPredictedPosition().x - it->GetPosition().x);
			glm::vec2 intersectionPoint = glm::vec2(PARTICLE_LEFTLIMIT,
				it->GetPosition().y + fIntersectionCoeff * (it->GetPredictedPosition().y - it->GetPosition().y));

			ContainerConstraint cc;
			cc.particleIndex = it->GetParticleIndex();
			cc.normalVector = glm::vec2(1.0f, 0.0f);
			cc.projectionPoint = intersectionPoint;
			cc.stiffness = 0.1f;
			cc.stiffness_adjusted = 1.0f - pow(1.0f - cc.stiffness, 1.0f / SOLVER_ITERATIONS);
			m_ContainerConstraints.push_back(cc);
		}

		if (it->GetPredictedPosition().y < PARTICLE_TOPLIMIT)
		{
			float fIntersectionCoeff = (PARTICLE_TOPLIMIT - it->GetPosition().y) / (it->GetPredictedPosition().y - it->GetPosition().y);
			glm::vec2 intersectionPoint = glm::vec2(it->GetPosition().x + fIntersectionCoeff * (it->GetPredictedPosition().x - it->GetPosition().x),
				PARTICLE_TOPLIMIT);

			ContainerConstraint cc;
			cc.particleIndex = it->GetParticleIndex();
			cc.normalVector = glm::vec2(0.0f, 1.0f);
			cc.projectionPoint = intersectionPoint;
			cc.stiffness = 0.1f;
			cc.stiffness_adjusted = 1.0f - pow(1.0f - cc.stiffness, 1.0f / SOLVER_ITERATIONS);
			m_ContainerConstraints.push_back(cc);
		}

		if (it->GetPredictedPosition().x > PARTICLE_RIGHTLIMIT)
		{
			float fIntersectionCoeff = (PARTICLE_RIGHTLIMIT - it->GetPosition().x) / (it->GetPredictedPosition().x - it->GetPosition().x);
			glm::vec2 intersectionPoint = glm::vec2(PARTICLE_RIGHTLIMIT,
				it->GetPosition().y + fIntersectionCoeff * (it->GetPredictedPosition().y - it->GetPosition().y));

			ContainerConstraint cc;
			cc.particleIndex = it->GetParticleIndex();
			cc.normalVector = glm::vec2(-1.0f, 0.0f);
			cc.projectionPoint = intersectionPoint;
			cc.stiffness = 0.1f;
			cc.stiffness_adjusted = 1.0f - pow(1.0f - cc.stiffness, 1.0f / SOLVER_ITERATIONS);
			m_ContainerConstraints.push_back(cc);
		}

		if (it->GetPredictedPosition().y > PARTICLE_BOTTOMLIMIT)
		{
			if (it->GetPredictedPosition().y - it->GetPosition().y != 0.0f)
			{
				float fIntersectionCoeff = (PARTICLE_BOTTOMLIMIT - it->GetPosition().y) / (it->GetPredictedPosition().y - it->GetPosition().y);
				glm::vec2 intersectionPoint = glm::vec2(it->GetPosition().x + fIntersectionCoeff * (it->GetPredictedPosition().x - it->GetPosition().x),
					PARTICLE_BOTTOMLIMIT);

				ContainerConstraint cc;
				cc.particleIndex = it->GetParticleIndex();
				cc.normalVector = glm::vec2(0.0f, -1.0f);
				cc.projectionPoint = intersectionPoint;
				cc.stiffness = 0.1f;
				cc.stiffness_adjusted = 1.0f - pow(1.0f - cc.stiffness, 1.0f / SOLVER_ITERATIONS);
				m_ContainerConstraints.push_back(cc);
			}
		}
	}

	//// Collision with other particles
	//for (int index = 0; index < PARTICLE_COUNT; index++)
	//{
	//	// Get the neighbors of the current particle
	//	//std::vector<Particle*> neighborList = m_SpatialManager.GetNeighbors(m_ParticleList[index]);
	//	std::vector<Particle*> neighborList;
	//	m_SpatialManager.GetNeighbors(m_ParticleList[index], neighborList);

	//	// If there are no neighbors set the color to default
	//	/*if (neighborList.size() == 0)
	//	{
	//		m_ParticleList[index].SetDefaultColor();
	//	}
	//	else
	//	{
	//		m_ParticleList[index].SetAsNeighborColor();
	//	}*/

	//	for each (Particle* particle in neighborList)
	//	{
	//		// Mark the particle as a neighbor
	//		//particle->SetAsNeighborColor();

	//		// Check if there is a collision between particles
	//		if (m_ParticleList[index].IsCollision(*particle))
	//		{
	//			/*particle->SetIsCollidingColor();
	//			m_ParticleList[index].SetIsCollidingColor();*/
	//		}
	//	}
	//}
}

void Application::FindNeighborParticles()
{
	for (int index = 0; index < PARTICLE_COUNT; index++)
	{
		// Repopulate the spatial manager with the particles
		m_SpatialManager.RegisterObject(&m_ParticleList[index]);
	}
}

void Application::XSPH_Viscosity(Particle& particle)
{
	// XSPH viscosity

	/*std::cout << "Initial velocity: ";
	PrintVector2(currentParticle.GetVelocity());*/

	// Get the neighbors of the current particle
	std::vector<Particle*> neighborList;
	m_SpatialManager.GetNeighbors(particle, neighborList);

	// Velocity accumulator
	glm::vec2 accumulatorVelocity = glm::vec2(0.0f, 0.0f);

	for each (Particle* pNeighborParticle in neighborList)
	{
		// Calculate the vector between the particles
		glm::vec2 r = particle.GetPosition() - pNeighborParticle->GetPosition();
		// Use poly6 smoothing kernel
		accumulatorVelocity += (Poly6(r, SMOOTHING_DISTANCE) * (pNeighborParticle->GetVelocity() - particle.GetVelocity()));
	}

	// Add the accumulated velocity to implement XSPH
	particle.AddDeltaVelocity(XSPHParam * accumulatorVelocity);

	/*std::cout << "Velocity after XSPH viscosity: ";
	PrintVector2(currentParticle.GetVelocity());*/
}
