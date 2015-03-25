#include "SoftBody.h"
#include "Mat2Utility.h"

#include "SpatialPartition.h"


int SoftBody::SoftBodyIndex = 0;

SoftBody::SoftBody()
{
	m_bAllowFlipping			= true;
	m_bVolumeConservation		= false;
	m_bLinearMatch				= true;
	m_bQuadraticMatch			= false;

	m_bConvexHullInitialized	= false;
	m_bDrawConvexHull			= false;
	m_bBezierCurve				= false;

	m_bReady					= false;
	m_bDrawGoalPositions		= false;

	m_fBeta = 0.0f;

	m_iIndex = SoftBodyIndex++;
}

void SoftBody::Update(float dt)
{
	if (m_bReady)
	{
		// Graham scan
		if (!m_bConvexHullInitialized && m_bDrawConvexHull)
		{
			m_bConvexHullInitialized = true;
			GrahamScan::InitializeSingleton(m_ParticlesList);
		}

		if (m_bBezierCurve)
		{
			// Calculate the array of points which form the bezier curve
			m_BezierCurve.CalculateMulticurveBezierPoints(m_BezierPoints);
		}
		
		// Update external forces
		UpdateForces(dt);

		// Project positions
		ShapeMatching(dt);

		// Project constraints
		int iIteration = 0;
		while (iIteration++ < SOLVER_ITERATIONS)
		{
			// Get global particle list size
			unsigned int globalListSize = ParticleManager::GetInstance().GlobalParticleListSize();
			// Particle-particle collision detection and response
			for (unsigned int i = 0; i < globalListSize; i++)
			{
				for (unsigned int j = 0; j < globalListSize; j++)
				{
					BaseParticle* p1 = ParticleManager::GetInstance().GetParticle(i);
					BaseParticle* p2 = ParticleManager::GetInstance().GetParticle(j);

					if (p1->ParticleType == ParticleType::FluidParticle &&
						p2->ParticleType == ParticleType::FluidParticle)
					{
						continue;
					}

					// Make sure it's not the same particle
					if (p1->GlobalIndex != p2->GlobalIndex)
					{
						// Make sure it's not within the same simulation
						if (p1->GetParentIndex() != p2->GetParentIndex())
						{
							// Check if there is a collision between particles
							if (p1->IsColliding(*p2))
							{
								glm::vec2 p1p2 = p1->Position - p2->Position;
								float fDistance = glm::length(p1p2);

								glm::vec2 fDp1 = -0.5f * (fDistance - PARTICLE_RADIUS_TWO) * (p1p2) / fDistance;
								glm::vec2 fDp2 = -fDp1;

								p1->Position += fDp1 * PBDSTIFFNESS_ADJUSTED;
								p2->Position += fDp2 * PBDSTIFFNESS_ADJUSTED;
							}
						}
					}
				}
			}
		}

		Integrate(dt);

		// Update the position of the bezier points
		m_BezierCurve.UpdateBezierPoints(m_ParticlesList);
	}
}

void SoftBody::Draw(sf::RenderWindow& window)
{	
	if (m_bReady)
	{
		// Draw bezier curves which links together all the soft body particles
		if (m_bBezierCurve)
		{
			// Draw lines between the points in the array
			m_BezierCurve.DrawBezierCurve(window, m_BezierPoints);
		}
		
		// Draw convex hull using the GrahamScan algorithm
		if (m_bDrawConvexHull)
		{
			GrahamScan::GetInstance()->Draw(window);
		}

		// Draw the goal positions for all the particles
		if (m_bDrawGoalPositions)
		{
			// Draw the goal shapes
			for (unsigned int iIndex = 0; iIndex < m_ParticlesList.size(); iIndex++)
			{
				m_ParticlesList[iIndex]->DrawGoalShape(window);
			}
		}

		// Draw all the particles during the setup stage
		for (unsigned int iIndex = 0; iIndex < m_ParticlesList.size(); iIndex++)
		{
			m_ParticlesList[iIndex]->Draw(window);
		}
	}
	else
	{
		// Draw all the particles during the setup stage
		for (unsigned int iIndex = 0; iIndex < m_ParticlesList.size(); iIndex++)
		{
			m_ParticlesList[iIndex]->Draw(window);
		}
	}
}

void SoftBody::ShapeMatching(float dt)
{
	// Project particle position
	if (m_ParticlesList.size() <= 1)
	{
		return;
	}

	float fTotalMass = 0.0f;
	glm::vec2 centerOfMass = glm::vec2(0.0f);
	glm::vec2 centerOfMass0 = glm::vec2(0.0f);

	unsigned int iIndex = 0;

	// Calculate the center of mass for the original cloud configuration and the current cloud configuration
	for (iIndex = 0; iIndex < m_ParticlesList.size(); iIndex++)
	{
		DeformableParticle& currentParticle = *m_ParticlesList[iIndex];

		float fTempMass = currentParticle.Mass;
		if (currentParticle.IsFixedParticle())
		{
			fTempMass *= 100.0f;
		}
		fTotalMass		+= fTempMass;
		centerOfMass	+= currentParticle.NewPosition * fTempMass;
		centerOfMass0	+= currentParticle.OriginalPosition * fTempMass;
	}
	centerOfMass /= fTotalMass;
	centerOfMass0 /= fTotalMass;


	glm::mat2 Aqq = glm::mat2(0.0f);
	glm::mat2 Apq = glm::mat2(0.0f);
	for (iIndex = 0; iIndex < m_ParticlesList.size(); iIndex++)
	{
		DeformableParticle& currentParticle = *m_ParticlesList[iIndex];
		float fMass = currentParticle.Mass;

		glm::vec2 p = currentParticle.NewPosition - centerOfMass;
		glm::vec2 q = currentParticle.OriginalPosition - centerOfMass0;

		// Apq
		Apq[0][0] += fMass * p.x * q.x;
		Apq[0][1] += fMass * p.x * q.y;
		Apq[1][0] += fMass * p.y * q.x;
		Apq[1][1] += fMass * p.y * q.y;
		// Aqq
		Aqq[0][0] += fMass * q.x * q.x;
		Aqq[0][1] += fMass * q.x * q.y;
		Aqq[1][0] += fMass * q.y * q.x;
		Aqq[1][1] += fMass * q.y * q.y;
	}

	// Prevent flipping
	float detApq = glm::determinant(Apq);

	if (!m_bAllowFlipping && detApq < 0.0f)
	{
		Apq[0][1] = -Apq[0][1];
		Apq[1][1] = -Apq[1][1];
	}

	// Polar decomposition
	glm::mat2 R = glm::mat2(0.0f);
	glm::mat2 S = glm::mat2(0.0f);
	PolarDecomposition(Apq, R, S);

	if (m_bLinearMatch)
	{
		glm::mat2 A = glm::mat2(0.0f);

		float detAqq = glm::determinant(Aqq);
		if (detAqq != 0.0f)
		{
			glm::mat2 AqqInverse = glm::inverse(Aqq);
			A = Apq * AqqInverse;
		}

		if (m_bVolumeConservation)
		{
			float detA = glm::determinant(A);
			if (detA != 0.0f)
			{
				detA = 1.0f / sqrt(fabs(detA));
				if (detA > 2.0f) detA = 2.0f;
				A *= detA;
			}
		}

		glm::mat2 T = R * (1.0f - m_fBeta) + A * m_fBeta;

		for (iIndex = 0; iIndex < m_ParticlesList.size(); iIndex++)
		{
			DeformableParticle& currentParticle = *m_ParticlesList[iIndex];

			if (currentParticle.IsFixedParticle()) continue;

			currentParticle.GoalPosition = centerOfMass + T * (currentParticle.OriginalPosition - centerOfMass0);

			m_fStiffness = dt / SOFTBODY_STIFFNESS_VALUE;
			currentParticle.NewPosition += SOFTBODY_STIFFNESS_VALUE * (currentParticle.GoalPosition - currentParticle.NewPosition);

			if (m_bDrawGoalPositions)
			{
				currentParticle.UpdateGoalShapePosition();
			}
		}
	}
}

void SoftBody::Integrate(float dt)
{
	// Integrate
	float dt1 = 1.0f / dt;

	for (unsigned int iIndex = 0; iIndex < m_ParticlesList.size(); iIndex++)
	{
		DeformableParticle& currentParticle = *m_ParticlesList[iIndex];

		currentParticle.Velocity = (currentParticle.NewPosition - currentParticle.Position) * dt1;
		currentParticle.Position = currentParticle.NewPosition;

		currentParticle.Update();
	}
}

void SoftBody::UpdateCollision(float dt)
{
	unsigned int iIndex;
	for (iIndex = 0; iIndex < m_ParticlesList.size(); iIndex++)
	{
		DeformableParticle& currentParticle = *m_ParticlesList[iIndex];

		if (currentParticle.NewPosition.x < SOFTBODYPARTICLE_LEFTLIMIT || currentParticle.NewPosition.x > SOFTBODYPARTICLE_RIGHTLIMIT)
		{
			currentParticle.NewPosition.x = currentParticle.Position.x - currentParticle.Velocity.x * dt * SOFTBODY_RESTITUTION_COEFF;
			currentParticle.NewPosition.y = currentParticle.Position.y;
		}

		if (currentParticle.NewPosition.y < SOFTBODYPARTICLE_TOPLIMIT || currentParticle.NewPosition.y > SOFTBODYPARTICLE_BOTTOMLIMIT)
		{
			currentParticle.NewPosition.y = currentParticle.Position.y - currentParticle.Velocity.y * dt * SOFTBODY_RESTITUTION_COEFF;
			currentParticle.NewPosition.x = currentParticle.Position.x;
		}

		currentParticle.NewPosition.x = glm::clamp(currentParticle.NewPosition.x, SOFTBODYPARTICLE_LEFTLIMIT, SOFTBODYPARTICLE_RIGHTLIMIT);
		currentParticle.NewPosition.y = glm::clamp(currentParticle.NewPosition.y, SOFTBODYPARTICLE_TOPLIMIT, SOFTBODYPARTICLE_BOTTOMLIMIT);
	}
}

void SoftBody::UpdateForces(float dt)
{
	// Update external forces
	if (GRAVITY_ON)
	{
		unsigned int iIndex;

		for (iIndex = 0; iIndex < m_ParticlesList.size(); iIndex++)
		{
			DeformableParticle& currentParticle = *m_ParticlesList[iIndex];

			// Add gravity
			if (currentParticle.IsFixedParticle()) continue;
			currentParticle.Velocity += GRAVITATIONAL_ACCELERATION * dt;
			currentParticle.NewPosition = currentParticle.Position + currentParticle.Velocity * dt;
			currentParticle.GoalPosition = currentParticle.OriginalPosition;
		}
	}

	// Update container collision
	UpdateCollision(dt);
}

bool ScanlineSortY(glm::vec2& p1, glm::vec2& p2)
{
	if (p1.y != p2.y)
	{
		return p1.y < p2.y;
	}
	return p1.x < p2.x;
}

bool ScanlineSortX(glm::vec2& p1, glm::vec2& p2)
{
	return p1.x <= p2.x;
}

void SoftBody::SetReady(bool ready)
{
	// Add the first particle add the end of the array to get a smooth bezier contour
	m_BezierCurve.AddBezierPoint(m_ParticlesList[0]->Position);

	if (!m_bBezierCurve)
	{
		m_BezierPoints.clear();

		// Calculate the array of points which form the bezier curve
		m_BezierCurve.CalculateMulticurveBezierPoints(m_BezierPoints);

		// Sort points
		std::sort(m_BezierPoints.begin(), m_BezierPoints.end(), ScanlineSortY);

		bool bDraw = false;
		glm::vec2 startingPoint = m_BezierPoints[0];

		for (unsigned int i = 0; i < m_BezierPoints.size() - 1; i++)
		{
			if (m_BezierPoints[i].y - startingPoint.y > 2.0f * PARTICLE_RADIUS)
			{
				bDraw = true;
				startingPoint = m_BezierPoints[i];
			}

			if (bDraw)
			{
				glm::vec2 currentPos = m_BezierPoints[i];
				glm::vec2 endPos = m_BezierPoints[i + 1];
				while (currentPos.x < endPos.x)
				{
					// Create a soft body particle
					DeformableParticle* sbParticle = new DeformableParticle(currentPos, m_iIndex);

					// Move to the right
					currentPos.x += 2 * PARTICLE_RADIUS;

					// Add the newly created particle to the soft-body collection
					AddSoftBodyParticle(*sbParticle);
				}

				bDraw = false;
			}
		}
	}

	m_bReady = ready;
}