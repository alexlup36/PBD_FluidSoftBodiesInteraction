#ifndef SOFTBODY_H
#define SOFTBODY_H

#include "Common.h"
#include <algorithm>
#include "GrahamScan.h"
#include "BezierCurve.h"
#include "DeformableParticle.h"
#include "BaseSimulation.h"

class SoftBody : public BaseSimulation
{
public:
	SoftBody();

	void Update(float dt);
	void Draw(sf::RenderWindow& window);
	void SetReady(bool ready);

	inline unsigned int GetParticleCount() { return m_ParticlesList.size(); }
	inline void ClearSoftBodyParticleList() { m_ParticlesList.clear(); }
	inline void AddSoftBodyParticle(DeformableParticle& deformableParticle)
	{
		m_ParticlesList.push_back(&deformableParticle);

		// The current instance to the global list of particles
		ParticleManager::GetInstance().AddGlobalParticle(&deformableParticle);
		
		// Add the position of the particle to the list of point for bezier curve representation
		m_BezierCurve.AddBezierPoint(deformableParticle.Position);
	}

	inline bool IsReady() { return m_bReady; }

	inline std::vector<DeformableParticle*>& GetParticleList() { return m_ParticlesList; }

private:
	bool m_bAllowFlipping;
	bool m_bVolumeConservation;
	bool m_bLinearMatch;
	bool m_bQuadraticMatch;
	bool m_bReady;
	bool m_bDrawGoalPositions;

	bool m_bConvexHullInitialized;
	bool m_bDrawConvexHull;
	bool m_bBezierCurve;

	int m_iIndex;
	static int SoftBodyIndex;

	float m_fStiffness;
	float m_fBeta;

	std::vector<glm::vec2> m_BezierPoints;
	BezierCurve m_BezierCurve;

	std::vector<DeformableParticle*> m_ParticlesList;

	void ShapeMatching(float dt);
	void Integrate(float dt);
	void UpdateCollision(float dt);
	void UpdateForces(float dt);

	// Sort function
	friend extern bool ScanlineSortY(glm::vec2& p1, glm::vec2& p2);
	friend extern bool ScanlineSortX(glm::vec2& p1, glm::vec2& p2);
};

#endif // SOFTBODY_H