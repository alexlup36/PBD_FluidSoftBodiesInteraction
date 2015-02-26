#include "SoftBody.h"
#include "Mat2Utility.h"

int SoftBodyParticle::m_iGlobalIndex = 0;
SoftBodyParticle SoftBody::m_Pivot;

SoftBody::SoftBody()
{
	m_bAllowFlipping		= true;
	m_bVolumeConservation	= true;
	m_bLinearMatch			= true;
	m_bQuadraticMatch		= false;

	m_bConvexHull			= false;

	m_bReady				= false;
	m_bDrawGoalPositions	= false;

	m_fBeta = 0.0f;
}

void SoftBody::Update(float dt)
{
	if (m_bReady)
	{
		// Calculate the convex hull using the graham scan algorithm
		if (m_bConvexHull == false)
		{
			m_bConvexHull = true;

			std::sort(m_SoftBodyParticles.begin(), m_SoftBodyParticles.end(), SmallestY);
			m_Pivot = m_SoftBodyParticles[0];

			std::sort(m_SoftBodyParticles.begin() + 1, m_SoftBodyParticles.end(), PolarOrder);

			m_ConvexHull.push(&m_SoftBodyParticles[0]);
			m_ConvexHull.push(&m_SoftBodyParticles[1]);

			for (unsigned int i = 2; i < m_SoftBodyParticles.size(); i++)
			{
				SoftBodyParticle* top = m_ConvexHull.top();
				m_ConvexHull.pop();

				while (CCWTurn(*m_ConvexHull.top(), *top, m_SoftBodyParticles[i]) != -1)
				{
					top = m_ConvexHull.top();
					m_ConvexHull.pop();
				}

				m_ConvexHull.push(top);
				m_ConvexHull.push(&m_SoftBodyParticles[i]);
			}
		}

		// Update all particles

		// Update external forces
		if (GRAVITY_ON)
		{
			unsigned int iIndex;

			for (iIndex = 0; iIndex < m_SoftBodyParticles.size(); iIndex++)
			{
				SoftBodyParticle& currentParticle = m_SoftBodyParticles[iIndex];

				// Add gravity
				if (currentParticle.Fixed) continue;
				currentParticle.Velocity += GRAVITATIONAL_ACCELERATION * dt;
				currentParticle.NewPosition = currentParticle.Position + currentParticle.Velocity * dt;
				currentParticle.GoalPosition = currentParticle.OriginalPosition;
			}
		}

		// Collision detection against the container
		unsigned int iIndex;
		for (iIndex = 0; iIndex < m_SoftBodyParticles.size(); iIndex++)
		{
			SoftBodyParticle& currentParticle = m_SoftBodyParticles[iIndex];

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

		// Project positions
		ShapeMatching(dt);

		// Integrate
		float dt1 = 1.0f / dt;

		for (unsigned int iIndex = 0; iIndex < m_SoftBodyParticles.size(); iIndex++)
		{
			SoftBodyParticle& currentParticle = m_SoftBodyParticles[iIndex];

			currentParticle.Velocity = (currentParticle.NewPosition - currentParticle.Position) * dt1;
			currentParticle.Position = currentParticle.NewPosition;

			currentParticle.UpdateShapePosition();
		}
	}
}

void SoftBody::Draw(sf::RenderWindow& window)
{
	//if (m_bReady)
	//{
		// Draw all the particles
		for (unsigned int iIndex = 0; iIndex < m_SoftBodyParticles.size(); iIndex++)
		{
			m_SoftBodyParticles[iIndex].Draw(window);
		}
	//}
	
	if (m_bReady)
	{
		// Draw the convex hull
		int iConvexHullSize = m_ConvexHull.size();

		std::stack<SoftBodyParticle*> tempStack = m_ConvexHull;
		std::vector<sf::Vertex> lines;
		SoftBodyParticle* top = nullptr;

		while (tempStack.size() > 0)
		{
			top = tempStack.top();
			tempStack.pop();

			lines.push_back(sf::Vertex(sf::Vector2f(top->Position.x, top->Position.y)));
			if (tempStack.size() != 0)
			{
				lines.push_back(sf::Vertex(sf::Vector2f(tempStack.top()->Position.x, tempStack.top()->Position.y)));
			}
			else
			{
				lines.push_back(sf::Vertex(sf::Vector2f(lines[0].position.x, lines[0].position.y)));
			}
		}
		window.draw(&lines[0], iConvexHullSize * 2, sf::Lines);

		// Draw the goal positions for all the particles
		if (m_bDrawGoalPositions)
		{
			// Draw the goal shapes
			for (unsigned int iIndex = 0; iIndex < m_SoftBodyParticles.size(); iIndex++)
			{
				m_SoftBodyParticles[iIndex].DrawGoalShape(window);
			}
		}
	}
}

void SoftBody::ShapeMatching(float dt)
{
	// Project particle position
	if (m_SoftBodyParticles.size() <= 1)
	{
		return;
	}

	float fTotalMass = 0.0f;
	glm::vec2 centerOfMass = glm::vec2(0.0f);
	glm::vec2 centerOfMass0 = glm::vec2(0.0f);

	unsigned int iIndex = 0;

	// Calculate the center of mass for the original cloud configuration and the current cloud configuration
	for (iIndex = 0; iIndex < m_SoftBodyParticles.size(); iIndex++)
	{
		SoftBodyParticle& currentParticle = m_SoftBodyParticles[iIndex];

		float fTempMass = currentParticle.Mass;
		if (currentParticle.Fixed)
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
	for (iIndex = 0; iIndex < m_SoftBodyParticles.size(); iIndex++)
	{
		SoftBodyParticle& currentParticle = m_SoftBodyParticles[iIndex];
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
		A = Apq * glm::inverse(Aqq);

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

		for (iIndex = 0; iIndex < m_SoftBodyParticles.size(); iIndex++)
		{
			SoftBodyParticle& currentParticle = m_SoftBodyParticles[iIndex];

			if (currentParticle.Fixed) continue;

			currentParticle.GoalPosition = centerOfMass + T * (currentParticle.OriginalPosition - centerOfMass0);

			m_fStiffness = dt / SOFTBODY_STIFFNESS_VALUE;
			currentParticle.NewPosition += m_fStiffness * (currentParticle.GoalPosition - currentParticle.NewPosition);

			if (m_bDrawGoalPositions)
			{
				currentParticle.UpdateGoalShapePosition();
			}
		}
	}
}

// ----------------------------------------------------------

void SoftBodyParticle::UpdateShapePosition()
{
	// Update the position of the shape
	m_Shape.setPosition(sf::Vector2f(Position.x, Position.y));
}

void SoftBodyParticle::Draw(sf::RenderWindow& window)
{
	window.draw(m_Shape);
}

void SoftBodyParticle::UpdateGoalShapePosition()
{
	// Update the goal position of the shape
	m_GoalPositionShape.setPosition(sf::Vector2f(GoalPosition.x, GoalPosition.y));
}

void SoftBodyParticle::DrawGoalShape(sf::RenderWindow& window)
{
	window.draw(m_GoalPositionShape);
}
