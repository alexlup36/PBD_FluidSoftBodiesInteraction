#include "FluidParticle.h"
#include "SimulationManager.h"
#include "SoftBody.h"

int FluidParticle::FluidParticleGlobalIndex = 0;

float FluidParticle::CalculateMinimumTranslationDistance()
{
	float fMinDistance = 0.0f;
	bool bClosestEdgeFound = false;

	std::vector<SoftBody*> softBodyList = SimulationManager::GetInstance().GetSoftBodySimulationList();

	for each (SoftBody* pSoftBody in softBodyList)
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

				m_ClosesPoint = ClosestPointToPointOnLine(m_ClosestEdge.Start->Position, m_ClosestEdge.End->Position, Position);
			}
		}
	}

	if (bClosestEdgeFound && IsBetween(m_ClosestEdge.Start->Position, m_ClosestEdge.End->Position, m_ClosesPoint))
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

void FluidParticle::Draw(sf::RenderWindow& window)
{
	BaseParticle::Draw(window);

	/*if (SignedDistance < 0.0f && GlobalIndex == 800)
	{
		DrawLine(window, m_ClosesPoint, Position, sf::Color::Red);
	}*/
}
