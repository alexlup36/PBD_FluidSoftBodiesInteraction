#ifndef SOFTBODY_H
#define SOFTBODY_H

#include "Common.h"
#include <algorithm>
#include "GrahamScan.h"
#include "BezierCurve.h"
#include "SoftBodyParticle.h"

class SoftBody
{
public:
	SoftBody();

	void Update(float dt);
	void Draw(sf::RenderWindow& window);

	inline unsigned int GetParticleCount() { return m_SoftBodyParticles.size(); }
	inline void ClearSoftBodyParticleList() { m_SoftBodyParticles.clear(); }
	inline void AddSoftBodyParticle(const SoftBodyParticle& sbParticle) 
	{
		m_SoftBodyParticles.push_back(sbParticle); 
		
		// Add the position of the particle to the list of point for bezier curve representation
		BezierCurve::GetInstance().AddBezierPoint(sbParticle.Position);
	}
	inline void SetReady(bool ready)
	{
		// Add the first particle add the end of the array to get a smooth bezier contour
		BezierCurve::GetInstance().AddBezierPoint(m_SoftBodyParticles[0].Position);

		m_bReady = ready; 
	}
	inline bool IsReady() { return m_bReady; }

	inline std::vector<SoftBodyParticle>& GetParticleList() { return m_SoftBodyParticles; }

private:
	bool m_bAllowFlipping;
	bool m_bVolumeConservation;
	bool m_bLinearMatch;
	bool m_bQuadraticMatch;
	bool m_bReady;
	bool m_bDrawGoalPositions;

	bool m_bConvexHull;
	bool m_bDrawConvexHull;
	bool m_bBezierCurve;

	float m_fStiffness;
	float m_fBeta;

	std::vector<SoftBodyParticle> m_SoftBodyParticles;

	void ShapeMatching(float dt);
	void Integrate(float dt);
	void UpdateCollision(float dt);
	void UpdateForces(float dt);
};

#endif // SOFTBODY_H