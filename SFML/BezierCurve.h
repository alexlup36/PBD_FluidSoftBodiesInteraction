#ifndef BEZIERCURVE_H
#define BEZIERCURVE_H

#include "Common.h"
#include "DeformableParticle.h"

class BezierCurve
{

public:

	BezierCurve() {};

	inline void AddBezierPoint(const glm::vec2& point) { m_BezierParticleList.push_back(point); }

	// Bezier calculations
	void CalculateMulticurveBezierPoints(std::vector<glm::vec2>& finalPoints);
	// Draw the bezier curve by drawing lines between the given array of points 
	void DrawBezierCurve(sf::RenderWindow& window, const std::vector<glm::vec2>& bezierPoints);
	// Update the points in the curve
	void UpdateBezierPoints(const std::vector<DeformableParticle*>& deformableParticleList);

private:
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