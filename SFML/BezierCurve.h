#ifndef BEZIERCURVE_H
#define BEZIERCURVE_H

#include "Common.h"
#include "SoftBodyParticle.h"

class BezierCurve
{
public:
	static BezierCurve& GetInstance()
	{
		static BezierCurve instance;
		return instance;
	}

	inline void AddBezierPoint(const glm::vec2& point) { m_BezierParticleList.push_back(point); }
	inline void UpdateBezierPoints(const std::vector<SoftBodyParticle>& softBodyParticleList)
	{
		for (unsigned int iIndex = 0; iIndex < softBodyParticleList.size(); iIndex++)
		{
			m_BezierParticleList[iIndex] = softBodyParticleList[iIndex].Position;
		}
		m_BezierParticleList[softBodyParticleList.size()] = softBodyParticleList[0].Position;
	}

	// Bezier calculations
	void CalculateMulticurveBezierPoints(std::vector<glm::vec2>& finalPoints);
	// Draw the bezier curve by drawing lines between the given array of points 
	void DrawBezierCurve(sf::RenderWindow& window, const std::vector<glm::vec2>& bezierPoints);

	void DrawLine(sf::RenderWindow& window,
		const glm::vec2& p1,
		const glm::vec2& p2);

private:
	// Hide constructor for singleton implementation
	BezierCurve() {};

	// Delete unneeded copy constructor and assignment operator
	BezierCurve(BezierCurve const&) = delete;
	void operator=(BezierCurve const&) = delete;

	// Methods

	// Calculate the final control point using the initial control point so that the curve goes through the input control point
	void CalculateControlPointPosition(const glm::vec2& p0,
		const glm::vec2& controlPoint,
		const glm::vec2& p2,
		glm::vec2& finalControlPoint);

	void QuadraticBezier(const glm::vec2& p0,
		const glm::vec2& p1,
		const glm::vec2& p2,
		float t,
		glm::vec2& finalPoint);

	const float BEZIERSTEP = 0.01f;
	std::vector<glm::vec2> m_BezierParticleList;
};

#endif // BEZIERCURVE_H