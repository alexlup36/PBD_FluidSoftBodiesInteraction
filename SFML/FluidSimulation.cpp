#include "FluidSimulation.h"

#include <time.h>
#include <memory>
#include "MarchingSquares.h"

// ------------------------------------------------------------------------

void FluidSimulation::Update(sf::RenderWindow& window, float dt)
{
	// Reset the spatial manager
	SpatialPartition::GetInstance().ClearBuckets();
	 
	UpdateExternalForces(dt);
	DampVelocities();
	CalculatePredictedPositions(window, dt);

	// Project constraints
	int iIteration = 0;

	while (iIteration++ < SOLVER_ITERATIONS)
	{
		if (FLUID_SIMULATION)
		{
			// ------------------------------------------------------------------------
			
			FindNeighborParticles();

#ifdef MULTITHREADING
			m_ThreadPool->wait();
#endif // MULTITHREADING

			// For the current particle get the lists of neighbors
			for (unsigned int iParticleIndex = 0; iParticleIndex < m_ParticleList.size(); iParticleIndex++)
			{
				m_ParticleList[iParticleIndex]->UpdateNeighbors();
			}

			// ------------------------------------------------------------------------

			// For all particles calculate density constraint
			for (unsigned int iParticleIndex = 0; iParticleIndex < m_ParticleList.size(); iParticleIndex++)
			{
				ComputeParticleConstraint(m_ParticleList[iParticleIndex]);
			}

			// ------------------------------------------------------------------------

			// For all particles calculate lambda
			for (unsigned int iParticleIndex = 0; iParticleIndex < PARTICLE_COUNT; iParticleIndex++)
			{
				ComputeLambda(m_ParticleList[iParticleIndex]);
			}

			// ------------------------------------------------------------------------

			// For all particles calculate the position correction - dp
			for (unsigned int iParticleIndex = 0; iParticleIndex < PARTICLE_COUNT; iParticleIndex++)
			{
				ComputePositionCorrection(m_ParticleList[iParticleIndex]);
			}

			// ------------------------------------------------------------------------

			// For all particles update the predicted position
			for (auto it = m_ParticleList.begin(); it != m_ParticleList.end(); it++)
			{
				FluidParticle* pCurrentParticle = *it;

				pCurrentParticle->PredictedPosition += pCurrentParticle->PositionCorrection;
			}

			// ------------------------------------------------------------------------

			// Handle collision against deformable particles

			// Get global particle list size
			unsigned int deformableParticlesCount = m_ParticleManager->GetDeformableParticles().size();
			
			// Particle-particle collision detection and response
			for (unsigned int iFluidParticleIndex = 0; iFluidParticleIndex < m_ParticleList.size(); iFluidParticleIndex++)
			{
				// Get the current fluid particle
				FluidParticle* pCurrentFluidParticle = m_ParticleList[iFluidParticleIndex];

				// Get the no of deformable particles which are neighbors to the current fluid particle
				unsigned int iDeformableParticleNeighborCount = m_ParticleList[iFluidParticleIndex]->GetSoftNeighbors().size();
				std::vector<int>& deformableParticleNeighborList = m_ParticleList[iFluidParticleIndex]->GetSoftNeighbors();

				// Go through all the deformable particles neighbors and check for collisions
				for (unsigned iDeformableParticleIndex = 0; 
					iDeformableParticleIndex < iDeformableParticleNeighborCount;
					iDeformableParticleIndex++)
				{
					// Get the current soft particle
					BaseParticle* pCurrentSoftParticle = m_ParticleManager->GetParticle(deformableParticleNeighborList[iDeformableParticleIndex]);

					// Check if there is a collision between particles
					if (pCurrentSoftParticle->IsColliding(*pCurrentFluidParticle))
					{
						glm::vec2 p1p2 = pCurrentSoftParticle->Position - pCurrentFluidParticle->Position;
						float fDistance = glm::length(p1p2);

						glm::vec2 fDp1 = -0.5f * (fDistance - PARTICLE_RADIUS_TWO) * (p1p2) / fDistance;
						glm::vec2 fDp2 = -fDp1;

						// Position correction due to interaction with fluid particle
						pCurrentSoftParticle->PositionCorrection += fDp1 * PBDSTIFFNESS_ADJUSTED;

						pCurrentFluidParticle->PredictedPosition += fDp2 * PBDSTIFFNESS_ADJUSTED;
					}
				}
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
					m_ParticleList[container_constraint.particleIndex]->PredictedPosition;

				float constraint = glm::dot(particlePredictedPosition - container_constraint.projectionPoint, container_constraint.normalVector);

				glm::vec2 gradientDescent = container_constraint.normalVector;
				float gradienDescentLength = glm::length(gradientDescent);
				glm::vec2 dp = -constraint / (gradienDescentLength * gradienDescentLength) * gradientDescent;

				m_ParticleList[container_constraint.particleIndex]->PredictedPosition += dp * container_constraint.stiffness_adjusted;
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
			FluidParticle* pCurrentParticle = *it;

			// Wall collision response
			glm::vec2 currentVelocity = pCurrentParticle->Velocity;
			
			if (pCurrentParticle->PredictedPosition.x < PARTICLE_LEFTLIMIT || pCurrentParticle->PredictedPosition.x > PARTICLE_RIGHTLIMIT)
			{
				pCurrentParticle->Velocity = glm::vec2(-currentVelocity.x, currentVelocity.y);
			}

			if (pCurrentParticle->PredictedPosition.y < PARTICLE_TOPLIMIT || pCurrentParticle->PredictedPosition.y > PARTICLE_BOTTOMLIMIT)
			{
				pCurrentParticle->Velocity = glm::vec2(currentVelocity.x, -currentVelocity.y);
			}

			// Clamp position inside the container
			glm::vec2 currentPosition = pCurrentParticle->Position;
			currentPosition.x = glm::clamp(currentPosition.x, PARTICLE_LEFTLIMIT + 1.0f, PARTICLE_RIGHTLIMIT - 1.0f);
			currentPosition.y = glm::clamp(currentPosition.y, PARTICLE_TOPLIMIT + 1.0f, PARTICLE_BOTTOMLIMIT - 1.0f);
			pCurrentParticle->Position = currentPosition;
		}
	}
	
	// Clear the constraint list
	m_ContainerConstraints.clear();
}

// ------------------------------------------------------------------------

void FluidSimulation::Draw(sf::RenderWindow& window)
{
	if (FLUIDRENDERING_PARTICLE)
	{
		// Draw particles
		for (int index = 0; index < PARTICLE_COUNT; index++)
		{
			m_ParticleList[index]->Draw(window);
		}
	}
	
	if (FLUIDRENDERING_MARCHINGSQUARES)
	{
		MarchingSquares::GetInstance().ProcessMarchingSquares(this, window);
	}
}

// ------------------------------------------------------------------------

glm::vec2 FluidSimulation::GetRandomPosWithinLimits()
{
	int iXPosition = rand() % (int)(PARTICLE_RIGHTLIMIT - PARTICLE_LEFTLIMIT) + (int)PARTICLE_LEFTLIMIT;
	int iYPosition = rand() % (int)(PARTICLE_BOTTOMLIMIT - PARTICLE_TOPLIMIT) + (int)PARTICLE_TOPLIMIT;

	return glm::vec2((float)iXPosition, (float)iYPosition);
}

// ------------------------------------------------------------------------

void FluidSimulation::BuildParticleSystem(const glm::vec2& startPosition, 
	const sf::Color& color, 
	int iParticleCount)
{
	// Initialize the particle manager
	m_ParticleManager = &ParticleManager::GetInstance();

	float fWidthDistance = PARTICLE_RIGHTLIMIT - PARTICLE_LEFTLIMIT;
	float fHeightDistance = PARTICLE_BOTTOMLIMIT - PARTICLE_TOPLIMIT;

	float fDx = 3.0f * PARTICLE_RADIUS;
	float fDy = 3.0f * PARTICLE_RADIUS;

	glm::vec2 currentPosition = glm::vec2(PARTICLE_LEFTLIMIT + startPosition.x, PARTICLE_TOPLIMIT + startPosition.y);

	for (int iLine = 0; iLine < PARTICLE_WIDTH_COUNT; iLine++)
	{
		for (int jColumn = 0; jColumn < PARTICLE_HEIGHT_COUNT; jColumn++)
		{
			FluidParticle* particle = new FluidParticle(currentPosition, color, GetSimulationIndex());

			// Update current position X
			currentPosition.x += fDx;

			// Build particle list
			m_ParticleList.push_back(particle);

			// Add particle to the global list
			m_ParticleManager->AddGlobalParticle(particle);
		}

		// Update current position Y
		currentPosition.y += fDy;
		currentPosition.x = PARTICLE_LEFTLIMIT + startPosition.x;
	}

#ifdef MULTITHREADING

	unsigned int iThreadCount = 6;
	m_ThreadPool = std::make_unique<boost::threadpool::pool>(iThreadCount);

	// Create a list of tasks
	for (unsigned int iThreadIndex = 0; iThreadIndex < iThreadCount; iThreadIndex++)
	{
		// Repopulate the spatial manager with the fluid particles

		// Calculate the start and end index to process for the current thread
		int iStep = m_ParticleList.size() / iThreadCount;

		int iStartIndex = iStep * iThreadIndex;
		int iEndIndex = iStep * (iThreadIndex + 1);

		if (iThreadIndex == iThreadCount - 1)
		{
			iEndIndex = m_ParticleList.size();
		}

		// Initialize task
		taskList.push_back(boost::bind(&FluidSimulation::RegisterFluidObject,
			this,
			iStartIndex,
			iEndIndex));
	}

#endif // MULTITHREADING
	
}

// ------------------------------------------------------------------------

void FluidSimulation::UpdateExternalForces(float dt)
{
	for (auto it = m_ParticleList.begin(); it != m_ParticleList.end(); it++)
	{
		// Update particle velocity
		if (GRAVITY_ON)
		{
			FluidParticle* pCurrentParticle = *it;

			pCurrentParticle->Velocity += dt * pCurrentParticle->Mass * (GRAVITATIONAL_ACCELERATION * pCurrentParticle->Mass);
		}
	}
}

// ------------------------------------------------------------------------

void FluidSimulation::DampVelocities()
{
	// Damp velocity
	for (auto it = m_ParticleList.begin(); it != m_ParticleList.end(); it++)
	{
		FluidParticle* pCurrentParticle = *it;

		pCurrentParticle->Velocity = pCurrentParticle->Velocity * VELOCITY_DAMPING;
	}
}

// ------------------------------------------------------------------------

void FluidSimulation::CalculatePredictedPositions(sf::RenderWindow& window, float dt)
{
	// Calculate the predicted positions
	for (auto it = m_ParticleList.begin(); it != m_ParticleList.end(); it++)
	{
		FluidParticle* pCurrentParticle = *it;

		// Update position
		pCurrentParticle->PredictedPosition += dt * pCurrentParticle->Velocity;
	}
}

// ------------------------------------------------------------------------

void FluidSimulation::UpdateActualPosAndVelocities(float dt)
{
	for (unsigned int iParticleIndex = 0; iParticleIndex < m_ParticleList.size(); iParticleIndex++)
	{
		FluidParticle& currentParticle = *m_ParticleList[iParticleIndex];

		if (dt != 0.0f)
		{
			// Update velocity based on the distance offset (after correcting the position)
			currentParticle.Velocity = (currentParticle.PredictedPosition - currentParticle.Position) / dt;
		}

		// Apply XSPH viscosity
		if (XSPH_VISCOSITY)
		{
			XSPH_Viscosity(&currentParticle);
		}

		// Update position
		currentParticle.Position = currentParticle.PredictedPosition;

		// Update local position
		glm::vec2 localOffset(WALL_LEFTLIMIT, WALL_TOPLIMIT);
		currentParticle.LocalPosition = currentParticle.Position - localOffset;

		// Update the position of the particle shape
		currentParticle.Update();
	}
}

// ------------------------------------------------------------------------

void FluidSimulation::GenerateCollisionConstraints(sf::RenderWindow& window)
{
	for (auto it = m_ParticleList.begin(); it != m_ParticleList.end(); it++)
	{
		FluidParticle* currentParticle = *it;

		if (currentParticle->PredictedPosition.x < PARTICLE_LEFTLIMIT)
		{
			float fIntersectionCoeff = (PARTICLE_LEFTLIMIT - currentParticle->Position.x) / (currentParticle->PredictedPosition.x - currentParticle->Position.x);
			glm::vec2 intersectionPoint = glm::vec2(PARTICLE_LEFTLIMIT,
				currentParticle->Position.y + fIntersectionCoeff * (currentParticle->PredictedPosition.y - currentParticle->Position.y));

			ContainerConstraint cc;
			cc.particleIndex = currentParticle->Index;
			cc.normalVector = glm::vec2(1.0f, 0.0f);
			cc.projectionPoint = intersectionPoint;
			cc.stiffness = PBDSTIFFNESS;
			cc.stiffness_adjusted = PBDSTIFFNESS_ADJUSTED;
			m_ContainerConstraints.push_back(cc);
		}

		if (currentParticle->PredictedPosition.y < PARTICLE_TOPLIMIT)
		{
			float fIntersectionCoeff = (PARTICLE_TOPLIMIT - currentParticle->Position.y) / (currentParticle->PredictedPosition.y - currentParticle->Position.y);
			glm::vec2 intersectionPoint = glm::vec2(currentParticle->Position.x + fIntersectionCoeff * (currentParticle->PredictedPosition.x - currentParticle->Position.x),
				PARTICLE_TOPLIMIT);

			ContainerConstraint cc;
			cc.particleIndex = currentParticle->Index;
			cc.normalVector = glm::vec2(0.0f, 1.0f);
			cc.projectionPoint = intersectionPoint;
			cc.stiffness = PBDSTIFFNESS;
			cc.stiffness_adjusted = PBDSTIFFNESS_ADJUSTED;
			m_ContainerConstraints.push_back(cc);
		}

		if (currentParticle->PredictedPosition.x > PARTICLE_RIGHTLIMIT)
		{
			float fIntersectionCoeff = (PARTICLE_RIGHTLIMIT - currentParticle->Position.x) / (currentParticle->PredictedPosition.x - currentParticle->Position.x);
			glm::vec2 intersectionPoint = glm::vec2(PARTICLE_RIGHTLIMIT,
				currentParticle->Position.y + fIntersectionCoeff * (currentParticle->PredictedPosition.y - currentParticle->Position.y));

			ContainerConstraint cc;
			cc.particleIndex = currentParticle->Index;
			cc.normalVector = glm::vec2(-1.0f, 0.0f);
			cc.projectionPoint = intersectionPoint;
			cc.stiffness = PBDSTIFFNESS;
			cc.stiffness_adjusted = PBDSTIFFNESS_ADJUSTED;
			m_ContainerConstraints.push_back(cc);
		}

		if (currentParticle->PredictedPosition.y > PARTICLE_BOTTOMLIMIT)
		{
			if (currentParticle->PredictedPosition.y - currentParticle->Position.y != 0.0f)
			{
				float fIntersectionCoeff = (PARTICLE_BOTTOMLIMIT - currentParticle->Position.y) / (currentParticle->PredictedPosition.y - currentParticle->Position.y);
				glm::vec2 intersectionPoint = glm::vec2(currentParticle->Position.x + fIntersectionCoeff * (currentParticle->PredictedPosition.x - currentParticle->Position.x),
					PARTICLE_BOTTOMLIMIT);

				ContainerConstraint cc;
				cc.particleIndex = currentParticle->Index;
				cc.normalVector = glm::vec2(0.0f, -1.0f);
				cc.projectionPoint = intersectionPoint;
				cc.stiffness = PBDSTIFFNESS;
				cc.stiffness_adjusted = PBDSTIFFNESS_ADJUSTED;
				m_ContainerConstraints.push_back(cc);
			}
		}
	}
}

// ------------------------------------------------------------------------

void FluidSimulation::FindNeighborParticles()
{

#ifdef MULTITHREADING

	for (unsigned int i = 0; i < taskList.size(); i++)
	{
		m_ThreadPool->schedule(taskList[i]);
	}

#else

	for (unsigned int index = 0; index < m_ParticleList.size(); index++)
	{
		// Repopulate the spatial manager with the particles
		SpatialPartition::GetInstance().RegisterObject(m_ParticleList[index]);
	}

	for (unsigned int index = 0; index < ParticleManager::GetInstance().GetDeformableParticles().size(); index++)
	{
		// Repopulate the spatial manager with the particles
		SpatialPartition::GetInstance().RegisterObject((BaseParticle*)m_ParticleManager->GetDeformableParticle(index));
	}

#endif // MULTITHREADING

}

// ------------------------------------------------------------------------

void FluidSimulation::XSPH_Viscosity(FluidParticle* particle)
{
	std::vector<int>& fluidNeighborList = particle->GetFluidNeighbors();
	unsigned int fluidNeighborCount = fluidNeighborList.size();

	// XSPH viscosity

	// Velocity accumulator
	glm::vec2 accumulatorVelocity = glm::vec2(0.0f);

	for (unsigned int i = 0; i < fluidNeighborCount; i++)
	{
		// Get the current neighbor particle
		FluidParticle* p = (FluidParticle*)m_ParticleManager->GetParticle(fluidNeighborList[i]);

		// Use poly6 smoothing kernel
		accumulatorVelocity += (Poly6Kernel(particle->PredictedPosition, p->PredictedPosition) * (particle->Velocity - p->Velocity));
	}

	// Add the accumulated velocity to implement XSPH
	particle->Velocity += XSPHParam * accumulatorVelocity;
}

// ------------------------------------------------------------------------

void FluidSimulation::ComputeParticleConstraint(FluidParticle* particle)
{
	// Calculate the particle density using the standard SPH density estimator
	float fAccFluid = 0.0f;
	float fAccSoft = 0.0f;
	float fSampleDensityDifference = 200000.0f;

	std::vector<int>& fluidNeighborList = particle->GetFluidNeighbors();
	std::vector<int>& softNeighborList = particle->GetSoftNeighbors();
	unsigned int fluidNeighborCount = fluidNeighborList.size();
	unsigned int softNeighborCount = softNeighborList.size();

	for (unsigned int i = 0; i < fluidNeighborCount; i++)
	{
		fAccFluid += Poly6Kernel(particle->PredictedPosition, m_ParticleManager->GetParticle(fluidNeighborList[i])->PredictedPosition);
	}

	for (unsigned int i = 0; i < softNeighborCount; i++)
	{
		fAccSoft += Poly6Kernel(particle->PredictedPosition, m_ParticleManager->GetParticle(softNeighborList[i])->PredictedPosition);
	}

	// Update the particle SPH density
	particle->SPHDensity = fAccFluid + fSampleDensityDifference * fAccSoft;

	// Calculate and update the particle density constraint value
	particle->DensityConstraint = particle->SPHDensity * INVERSE_WATER_RESTDENSITY - 1.0f;
}

// ------------------------------------------------------------------------

glm::vec2 FluidSimulation::ComputeParticleGradientConstraint(FluidParticle* particle, FluidParticle* neighbor)
{
	// Calculate the gradient of the constraint function - Monaghan 1992
	// SPH recipe for the gradient of the constraint function with respect
	// to a particle k

	std::vector<int>& fluidNeighborList = particle->GetFluidNeighbors();
	unsigned int fluidNeighborCount = fluidNeighborList.size();

	// If the particle k is a neighboring particle
	if (particle->Index == neighbor->Index) // k = i
	{
		// Accumulator for the gradient
		glm::vec2 acc = glm::vec2(0.0f);

		for (unsigned int i = 0; i < fluidNeighborCount; i++)
		{
			// Get the current neighbor particle
			FluidParticle* p = (FluidParticle*)m_ParticleManager->GetParticle(fluidNeighborList[i]);

			// Calculate the sum of all gradients between the particle and its neighbors
			acc += SpikyKernelGradient(particle->PredictedPosition, p->PredictedPosition);
		}

		acc *= INVERSE_WATER_RESTDENSITY;

		return acc;
	}
	else // k = j Particle k is not a neighboring particle
	{
		// Calculate the gradient 
		glm::vec2 gradient = SpikyKernelGradient(particle->PredictedPosition, neighbor->PredictedPosition);
		gradient *= (-1.0f * INVERSE_WATER_RESTDENSITY);

		return gradient;
	}
}

// ------------------------------------------------------------------------

void FluidSimulation::ComputeLambda(FluidParticle* particle)
{
	std::vector<int>& fluidNeighborList = particle->GetFluidNeighbors();
	unsigned int fluidNeighborCount = fluidNeighborList.size();

	float acc = 0.0f;

	// k = i
	glm::vec2 gradient = ComputeParticleGradientConstraint(particle, particle);
	float fGradientLength = glm::length(gradient);
	acc += fGradientLength * fGradientLength;

	// k = j
	for (unsigned int i = 0; i < fluidNeighborCount; i++)
	{
		glm::vec2 grad = ComputeParticleGradientConstraint(particle, (FluidParticle*)m_ParticleManager->GetParticle(fluidNeighborList[i]));
		float fGradLength = glm::length(grad);
		acc += fGradLength * fGradLength;
	}

	// Calculate the lambda value for the current particle
	particle->Lambda = (-1.0f) * particle->DensityConstraint / (acc + RELAXATION_PARAMETER);
}

// ------------------------------------------------------------------------

void FluidSimulation::ComputePositionCorrection(FluidParticle* particle)
{
	std::vector<int>& fluidNeighborList = particle->GetFluidNeighbors();
	unsigned int fluidNeighborCount = fluidNeighborList.size();

	glm::vec2 acc = glm::vec2(0.0f);

	// Calculate the delta position using the gradient of the kernel and the lambda values for each particle
	for (unsigned int i = 0; i < fluidNeighborCount; i++)
	{
		// Get the current particle
		FluidParticle* pCurrentNeighborParticle = (FluidParticle*)m_ParticleManager->GetParticle(fluidNeighborList[i]);

		glm::vec2 gradient = SpikyKernelGradient(particle->PredictedPosition, pCurrentNeighborParticle->PredictedPosition);

		// Add an artificial pressure term which improves the particle distribution, creates surface tension, and
		// lowers the neighborhood requirements of traditional SPH
		if (ARTIFICIAL_PRESSURE_TERM)
		{
			acc += gradient * (particle->Lambda + pCurrentNeighborParticle->Lambda + ComputeArtificialPressureTerm(particle, pCurrentNeighborParticle));
		}
		else
		{
			acc += gradient * (particle->Lambda + pCurrentNeighborParticle->Lambda);
		}
	}

	// Scale the acc by the inverse of the rest density
	particle->PositionCorrection = acc * INVERSE_WATER_RESTDENSITY /* * particle.mass */;
}

// ------------------------------------------------------------------------

float FluidSimulation::ComputeArtificialPressureTerm(const FluidParticle* p1, const FluidParticle* p2)
{
	// Calculate an artificial pressure term which solves the problem of a particle having to few
	// neighbors which results in negative pressure. The ARTIFICIAL_PRESSURE constant is the value
	// of the kernel function at some fixed point inside the smoothing radius 
	float fKernelValue = Poly6Kernel(p1->PredictedPosition, p2->PredictedPosition);

	return (-0.1f) * pow(fKernelValue * INVERSE_ARTIFICIAL_PRESSURE, 4.0f);
}


// ------------------------------------------------------------------------
// Multithreading helper methods ------------------------------------------
// ------------------------------------------------------------------------

void FluidSimulation::RegisterFluidObject(int iStartIndex, int iEndIndex)
{
	// Do the actual update
	for (int i = iStartIndex; i < iEndIndex; i++)
	{
		// Repopulate the spatial manager with the particles
		SpatialPartition::GetInstance().RegisterObject(m_ParticleList[i]);
	}

	return;
}

void FluidSimulation::RegisterDeformableObject(int iStartIndex, int iEndIndex)
{
	// Do the actual update
	for (int i = iStartIndex; i < iEndIndex; i++)
	{
		// Repopulate the spatial manager with the particles
		SpatialPartition::GetInstance().RegisterObject((BaseParticle*)m_ParticleManager->GetDeformableParticle(i));
	}
}

// ------------------------------------------------------------------------