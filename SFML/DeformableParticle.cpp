#include "DeformableParticle.h"
#include "GrahamScan.h"
#include "BaseSimulation.h"
#include "SoftBody.h"
#include "SimulationManager.h"

int DeformableParticle::DeformableParticleGlobalIndex = 0;


void DeformableParticle::UpdateGoalShapePosition()
{
	// Update the goal position of the shape
	m_GoalShape.setPosition(sf::Vector2f(GoalPosition.x, GoalPosition.y));
}

void DeformableParticle::DrawGoalShape(sf::RenderWindow& window)
{
	window.draw(m_GoalShape);

	// Debug test drawing
	//DrawLine(window, intersectionPoint, Position, sf::Color::Green);
}

float DeformableParticle::CalculateMinimumTranslationDistance()
{
	float fMinDistance = 0.0f;
	bool bClosestEdgeFound = false;

	std::vector<SoftBody*> softBodyList = SimulationManager::GetInstance().GetSoftBodySimulationList();

	for each (SoftBody* pSoftBody in softBodyList)
	{
		// Make sure it's not the same as the current instance
		if (pSoftBody->GetSimulationIndex() != GetParent()->GetSimulationIndex())
		{
			// Get the edge list
			std::vector<Edge> edgeList = pSoftBody->GetConvexHull().GetEdgeList();

			fMinDistance = std::numeric_limits<float>::max();

			// Get the closest edge
			for (unsigned int i = 0; i < edgeList.size(); i++)
			{
				float fDistanceToEdge = DistanceToLine(edgeList[i].Start->Position, edgeList[i].End->Position);

				if (abs(fDistanceToEdge) < abs(fMinDistance))
				{
					fMinDistance = fDistanceToEdge;
					m_ClosestEdge = edgeList[i];

					bClosestEdgeFound = true;
				}
			}
		}
	}

	if (bClosestEdgeFound)
	{
		// Signed distance
		SignedDistance = fMinDistance;

		// Signed distance gradient
		glm::vec2 startToPoint = Position - m_ClosestEdge.Start->Position;
		glm::vec2 edgeNormalized = glm::normalize(m_ClosestEdge.End->Position - m_ClosestEdge.Start->Position);
		m_vIntersectionPoint = m_ClosestEdge.Start->Position + glm::dot(startToPoint, edgeNormalized) * edgeNormalized;
	}
	else
	{
		SignedDistance = 0.0f;
	}
	
	if (m_vIntersectionPoint != glm::vec2(0.0f))
	{
		glm::vec2 intersectionToPosition = m_vIntersectionPoint - Position;
		GradientSignedDistance = glm::normalize(intersectionToPosition);
	}
	else
	{
		GradientSignedDistance = glm::vec2(0.0f);
	}
	
	return SignedDistance;
}

void DeformableParticle::Update()
{
	// Call base update
	BaseParticle::Update();
}
