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
	//unsigned int iSizeDefList = ParticleManager::GetInstance().GetDeformableParticles().size();

	//for (unsigned int iGlobalIndexDef = 0; iGlobalIndexDef < iSizeDefList; iGlobalIndexDef++)
	//{
	//	// Get the current particle
	//	DeformableParticle* otherParticle = ParticleManager::GetInstance().GetDeformableParticle(iGlobalIndexDef);
	//	// Make sure it's not the same as the current instance
	//	if (otherParticle->Index != Index)
	//	{
	//		// Get the edge list
	//		std::vector<Edge> edgeList = otherParticle->GetParent()->GetConvexHull().GetEdgeList();

	//		fMinDistance = std::numeric_limits<float>::max();

	//		// Get the closest edge
	//		for (unsigned int i = 0; i < edgeList.size(); i++)
	//		{
	//			float fDistanceToEdge = DistanceToLine(edgeList[i].Start->Position, edgeList[i].End->Position);

	//			if (abs(fDistanceToEdge) < abs(fMinDistance))
	//			{
	//				fMinDistance = fDistanceToEdge;
	//				m_ClosestEdge = edgeList[i];
	//			}
	//		}
	//	}
	//}

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
				}
			}
		}
	}

	// Signed distance
	SignedDistance = fMinDistance;

	// Signed distance gradient
	glm::vec2 startToPoint = Position - m_ClosestEdge.Start->Position;
	glm::vec2 edgeNormalized = glm::normalize(m_ClosestEdge.End->Position - m_ClosestEdge.Start->Position);
	intersectionPoint = m_ClosestEdge.Start->Position + glm::dot(startToPoint, edgeNormalized) * edgeNormalized;

	glm::vec2 intersectionToPosition = intersectionPoint - Position;
	if (intersectionToPosition != glm::vec2(0.0f))
	{
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
