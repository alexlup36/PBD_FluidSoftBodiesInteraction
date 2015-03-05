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
	FindNeighborParticles(); // might need to be done in the iterations loop

	// Get the neighbors for all particles in the current update step
	std::vector<Particle*> neighbors[PARTICLE_COUNT];
	for (unsigned int iParticleIndex = 0; iParticleIndex < PARTICLE_COUNT; iParticleIndex++)
	{
		m_SpatialManager.GetNeighbors(m_ParticleList[iParticleIndex], neighbors[iParticleIndex]);
	}

	// Project constraints
	int iIteration = 0;
	while (iIteration++ < SOLVER_ITERATIONS)
	{
		if (FLUID_SIMULATION)
		{
			// ------------------------------------------------------------------------

			// For all particles calculate density constraint
			for (unsigned int iParticleIndex = 0; iParticleIndex < PARTICLE_COUNT; iParticleIndex++)
			{
				ComputeParticleConstraint(m_ParticleList[iParticleIndex], neighbors[iParticleIndex]);
			}

			// ------------------------------------------------------------------------

			// For all particles calculate lambda
			for (unsigned int iParticleIndex = 0; iParticleIndex < PARTICLE_COUNT; iParticleIndex++)
			{
				ComputeLambda(m_ParticleList[iParticleIndex], neighbors[iParticleIndex]);
			}

			// ------------------------------------------------------------------------

			// For all particles calculate the position correction - dp
			for (unsigned int iParticleIndex = 0; iParticleIndex < PARTICLE_COUNT; iParticleIndex++)
			{
				ComputePositionCorrection(m_ParticleList[iParticleIndex], neighbors[iParticleIndex]);
			}

			// ------------------------------------------------------------------------

			// For all particles update the predicted position
			for (auto it = m_ParticleList.begin(); it != m_ParticleList.end(); it++)
			{
				it->AddDeltaPredPosition(it->GetPositionCorrection());
			}

			// ------------------------------------------------------------------------
		}
		
		if (PBD_COLLISION)
		{
			// Generate external collision constraints
			GenerateCollisionConstraints(window);

			// Update container constraints Position based
			for each (auto container_constraint in m_ContainerConstraints)
			{
				glm::vec2 particlePredictedPosition =
					m_ParticleList[container_constraint.particleIndex].GetPredictedPosition();

				float constraint = glm::dot(particlePredictedPosition - container_constraint.projectionPoint, container_constraint.normalVector);

				glm::vec2 gradientDescent = container_constraint.normalVector;
				float gradienDescentLength = glm::length(gradientDescent);
				glm::vec2 dp = -constraint / (gradienDescentLength * gradienDescentLength) * gradientDescent;

				m_ParticleList[container_constraint.particleIndex].AddDeltaPredPosition(dp * container_constraint.stiffness_adjusted);
			}
		}
		
		// ------------------------------------------------------------------------
	}

	// Update the actual position and velocity of the particle
	UpdateActualPosAndVelocities(dt);

	if (!PBD_COLLISION)
	{
		// Collision detection against the container 
		for (auto it = m_ParticleList.begin(); it != m_ParticleList.end(); it++)
		{
			// Wall collision response
			glm::vec2 currentVelocity = it->GetVelocity();
			
			if (it->GetPredictedPosition().x < PARTICLE_LEFTLIMIT || it->GetPredictedPosition().x > PARTICLE_RIGHTLIMIT)
			{
				it->SetVelocity(glm::vec2(-currentVelocity.x, currentVelocity.y));
			}

			if (it->GetPredictedPosition().y < PARTICLE_TOPLIMIT || it->GetPredictedPosition().y > PARTICLE_BOTTOMLIMIT)
			{
				it->SetVelocity(glm::vec2(currentVelocity.x, -currentVelocity.y));
			}

			// Clamp position inside the container
			glm::vec2 currentPosition = it->GetPosition();
			currentPosition.x = glm::clamp(currentPosition.x, PARTICLE_LEFTLIMIT, PARTICLE_RIGHTLIMIT);
			currentPosition.y = glm::clamp(currentPosition.y, PARTICLE_TOPLIMIT, PARTICLE_BOTTOMLIMIT);
			it->SetPosition(currentPosition);
		}
	}
	
	// Clear the constraint list
	m_ContainerConstraints.clear();
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
	float fWidthDistance = PARTICLE_RIGHTLIMIT - PARTICLE_LEFTLIMIT;
	float fHeightDistance = PARTICLE_BOTTOMLIMIT - PARTICLE_TOPLIMIT;

	float fHorizontalOffset = 200.0f;
	float fVerticalOffset = 50.0f;

	float fDx = (fWidthDistance - 2.0f * fHorizontalOffset) / PARTICLE_WIDTH_COUNT;
	float fDy = (fHeightDistance - 2.0f * fVerticalOffset) / PARTICLE_HEIGHT_COUNT;

	glm::vec2 currentPosition = glm::vec2(PARTICLE_LEFTLIMIT + fHorizontalOffset, PARTICLE_TOPLIMIT + fVerticalOffset);

	for (int iLine = 0; iLine < PARTICLE_WIDTH_COUNT; iLine++)
	{
		for (int jColumn = 0; jColumn < PARTICLE_HEIGHT_COUNT; jColumn++)
		{
			std::shared_ptr<Particle> particle = std::make_shared<Particle>(currentPosition, PARTICLE_RADIUS);

			// Update current position X
			currentPosition.x += fDx;

			// Build particle list
			m_ParticleList.push_back(*particle);
		}

		// Update current position Y
		currentPosition.y += fDy;
		currentPosition.x = PARTICLE_LEFTLIMIT + fHorizontalOffset;
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

		// Update particle velocity
		glm::vec2 deltaVelocity = dt * it->GetParticleMass() * it->GetForce();
		it->AddDeltaVelocity(dt * it->GetParticleMass() * it->GetForce());
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
	}
}

void Application::UpdateActualPosAndVelocities(float dt)
{
	for (auto it = m_ParticleList.begin(); it != m_ParticleList.end(); it++)
	{
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

	// Get the neighbors of the current particle
	std::vector<Particle*> neighborList;
	m_SpatialManager.GetNeighbors(particle, neighborList);

	// Velocity accumulator
	glm::vec2 accumulatorVelocity = glm::vec2(0.0f);

	for each (Particle* pNeighborParticle in neighborList)
	{
		// Use poly6 smoothing kernel
		accumulatorVelocity += (Poly6Kernel(particle.GetPredictedPosition(), pNeighborParticle->GetPredictedPosition()) * (particle.GetVelocity() - pNeighborParticle->GetVelocity()));
	}

	// Add the accumulated velocity to implement XSPH
	particle.AddDeltaVelocity(XSPHParam * accumulatorVelocity);
}

// ------------------------------------------------------------------------

void Application::ComputeParticleConstraint(Particle& particle, std::vector<Particle*>& pNeighborList)
{
	// Calculate the particle density using the standard SPH density estimator
	float fAcc = 0.0f;

	// Accumulate density resulting from particle-neighbor interaction
	for (unsigned int i = 0; i < pNeighborList.size(); i++)
	{
		// For the current neighbor calculate the Poly6 kernel value using the vector between the 
		// current particle and the current neighbor
		fAcc += Poly6Kernel(particle.GetPredictedPosition(), pNeighborList[i]->GetPredictedPosition());
	}

	// Update the particle SPH density
	particle.SetSPHDensity(fAcc);

	// Calculate and update the particle density constraint value
	particle.SetDensityConstraint(particle.GetSPHDensity() * INVERSE_WATER_RESTDENSITY - 1.0f);
}

// ------------------------------------------------------------------------

glm::vec2 Application::ComputeParticleGradientConstraint(Particle& particle, Particle& neighbor, std::vector<Particle*>& pParticleNeighborList)
{
	// Calculate the gradient of the constraint function - Monaghan 1992
	// SPH recipe for the gradient of the constraint function with respect
	// to a particle k

	// If the particle k is a neighboring particle
	if (particle.GetParticleIndex() == neighbor.GetParticleIndex()) // k = i
	{
		// Accumulator for the gradient
		glm::vec2 acc = glm::vec2(0.0f);

		for (unsigned int i = 0; i < pParticleNeighborList.size(); i++)
		{
			// Get the current neighbor particle
			Particle* p = pParticleNeighborList[i];

			// Calculate the sum of all gradients between the particle and its neighbors
			acc += SpikyKernelGradient(particle.GetPredictedPosition(), p->GetPredictedPosition());
		}

		acc *= INVERSE_WATER_RESTDENSITY;

		return acc;
	}
	else // k = j Particle k is not a neighboring particle
	{
		// Calculate the gradient 
		glm::vec2 gradient = SpikyKernelGradient(particle.GetPredictedPosition(), neighbor.GetPredictedPosition());
		gradient *= (-1.0f * INVERSE_WATER_RESTDENSITY);

		return gradient;
	}
}

// ------------------------------------------------------------------------

void Application::ComputeLambda(Particle& particle, std::vector<Particle*>& pParticleNeighborList)
{
	float acc = 0.0f;

	// k = i
	glm::vec2 gradient = ComputeParticleGradientConstraint(particle, particle, pParticleNeighborList);
	float fGradientLength = glm::length(gradient);
	acc += fGradientLength * fGradientLength;

	// k = j
	for (unsigned int i = 0; i < pParticleNeighborList.size(); i++)
	{
		glm::vec2 grad = ComputeParticleGradientConstraint(particle, *pParticleNeighborList[i], pParticleNeighborList);
		float fGradLength = glm::length(grad);
		acc += fGradLength * fGradLength;
	}

	// Calculate the lambda value for the current particle
	particle.SetLambda((-1.0f) * particle.GetDensityConstraint() / (acc + RELAXATION_PARAMETER));
}

// ------------------------------------------------------------------------

void Application::ComputePositionCorrection(Particle& particle, std::vector<Particle*>& pParticleNeighborList)
{
	glm::vec2 acc = glm::vec2(0.0f);

	// Calculate the delta position using the gradient of the kernel and the lambda values for each particle
	for (unsigned int i = 0; i < pParticleNeighborList.size(); i++)
	{
		// Get the current particle
		Particle* pCurrentNeighborParticle = pParticleNeighborList[i];

		glm::vec2 gradient = SpikyKernelGradient(particle.GetPredictedPosition(), pCurrentNeighborParticle->GetPredictedPosition());

		// Add an artificial pressure term which improves the particle distribution, creates surface tension, and
		// lowers the neighborhood requirements of traditional SPH
		if (ARTIFICIAL_PRESSURE_TERM)
		{
			acc += gradient * (particle.GetLambda() + pCurrentNeighborParticle->GetLambda() + ComputeArtificialPressureTerm(particle, *pCurrentNeighborParticle));
		}
		else
		{
			acc += gradient * (particle.GetLambda() + pCurrentNeighborParticle->GetLambda());
		}
	}

	// Scale the acc by the inverse of the rest density
	particle.SetPositionCorrection(acc * INVERSE_WATER_RESTDENSITY /* * particle.mass */);
}

// ------------------------------------------------------------------------

float Application::ComputeArtificialPressureTerm(const Particle& p1, const Particle& p2)
{
	// Calculate an artificial pressure term which solves the problem of a particle having to few
	// neighbors which results in negative pressure. The ARTIFICIAL_PRESSURE constant is the value
	// of the kernel function at some fixed point inside the smoothing radius 
	float fKernelValue = Poly6Kernel(p1.GetPredictedPosition(), p2.GetPredictedPosition());

	return (-0.1f) * pow(fKernelValue * INVERSE_ARTIFICIAL_PRESSURE, 4.0f);
}

// ------------------------------------------------------------------------