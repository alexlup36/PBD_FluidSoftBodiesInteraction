#include "FluidParticle.h"
#include "SimulationManager.h"
#include "SoftBody.h"

int FluidParticle::FluidParticleGlobalIndex = 0;

float FluidParticle::CalculateMinimumTranslationDistance()
{
	float fMinDistance = 0.0f;

	SignedDistance = 0.0f;

	std::vector<SoftBody*>& softBodyList = SimulationManager::GetInstance().GetSoftBodySimulationList();

	for each (SoftBody* pSoftBody in softBodyList)
	{
		// Get the edge list
		std::vector<Edge>& edgeList = pSoftBody->GetConvexHull().GetEdgeList();

		// Particle is inside the convex hull => push it outside
		if (DeformableParticle::IsPointInsidePolygon(Position, edgeList))
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

void FluidParticle::Draw(sf::RenderWindow& window)
{
	BaseParticle::Draw(window);
}
