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
}

float DeformableParticle::CalculateMinimumTranslationDistance()
{
	float fMinDistance = 0.0f;
	SignedDistance = 0.0f;

	std::vector<SoftBody*> softBodyList = SimulationManager::GetInstance().GetSoftBodySimulationList();

	for each (SoftBody* pSoftBody in softBodyList)
	{
		// Make sure it's not the same as the current instance
		if (pSoftBody->GetSimulationIndex() != GetParent()->GetSimulationIndex())
		{
			// Get the edge list
			std::vector<Edge> edgeList = pSoftBody->GetConvexHull().GetEdgeList();

			// Particle is inside the convex hull => push it outside
			if (IsPointInsidePolygon(Position, edgeList))
			{
				fMinDistance = std::numeric_limits<float>::max();

				// Get the closest edge
				for (unsigned int i = 0; i < edgeList.size(); i++)
				{
					float fDistanceToEdge = DistanceToLine(edgeList[i].Start->Position, edgeList[i].End->Position);

					if (fDistanceToEdge < fMinDistance)
					{
						glm::vec2 point = ClosestPointToPointOnLine(edgeList[i].Start->Position, edgeList[i].End->Position, Position);

						// Signed distance
						SignedDistance = fDistanceToEdge;
						m_vIntersectionPoint = point;
					}
				}
			}
		}
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

void DeformableParticle::Draw(sf::RenderWindow& window)
{
	BaseParticle::Draw(window);
}
