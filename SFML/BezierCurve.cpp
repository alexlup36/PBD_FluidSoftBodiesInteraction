#include "BezierCurve.h"

void BezierCurve::CalculateMulticurveBezierPoints(std::vector<glm::vec2>& finalPoints)
{
	finalPoints.clear();

	glm::vec2 p0, p1, midPoint, currentPoint, tempPoint;
	float midx = 0.0f, midy = 0.0f;

	currentPoint = m_BezierParticleList[0];

	for (unsigned int i = 1; i < m_BezierParticleList.size() - 2; i++)
	{
		p0 = m_BezierParticleList[i];
		p1 = m_BezierParticleList[i + 1];
		midPoint.x = (p0.x + p1.x) * 0.5f;
		midPoint.y = (p0.y + p1.y) * 0.5f;

		// Calculate the array of points which form the bezier curve
		for (float t = 0.0f; t <= 1.0f; t += BEZIERSTEP)
		{
			glm::vec2 newControlPoint;
			CalculateControlPointPosition(currentPoint, p0, midPoint, newControlPoint);
			QuadraticBezier(currentPoint, newControlPoint, midPoint, t, tempPoint);
			finalPoints.push_back(tempPoint);
		}

		currentPoint = midPoint;
	}

	p0 = m_BezierParticleList[m_BezierParticleList.size() - 2];
	p1 = m_BezierParticleList[m_BezierParticleList.size() - 1];
	// Calculate the array of points which form the bezier curve
	for (float t = 0.0f; t <= 1.0f; t += BEZIERSTEP)
	{
		glm::vec2 newControlPoint;
		CalculateControlPointPosition(currentPoint, p0, p1, newControlPoint);
		QuadraticBezier(currentPoint, newControlPoint, p1, t, tempPoint);
		finalPoints.push_back(tempPoint);
	}
}

void BezierCurve::DrawBezierCurve(sf::RenderWindow& window, const std::vector<glm::vec2>& bezierPoints)
{
	for (unsigned int i = 0; i < bezierPoints.size() - 1; i++)
	{
		DrawLine(window, bezierPoints[i], bezierPoints[i + 1], sf::Color::Red);
	}
}

void BezierCurve::CalculateControlPointPosition(const glm::vec2& p0,
	const glm::vec2& controlPoint,
	const glm::vec2& p2,
	glm::vec2& finalControlPoint)
{
	finalControlPoint.x = controlPoint.x * 2.0f - (p0.x + p2.x) / 2.0f;
	finalControlPoint.y = controlPoint.y * 2.0f - (p0.y + p2.y) / 2.0f;
}

void BezierCurve::QuadraticBezier(const glm::vec2& p0,
	const glm::vec2& p1,
	const glm::vec2& p2,
	float t,
	glm::vec2& finalPoint)
{
	float tdiff = 1.0f - t;
	float tdiff2 = tdiff * tdiff;
	float t2 = t * t;

	finalPoint.x = tdiff2 * p0.x + tdiff * 2.0f * t * p1.x + t2 * p2.x;
	finalPoint.y = tdiff2 * p0.y + tdiff * 2.0f * t * p1.y + t2 * p2.y;
}

void BezierCurve::UpdateBezierPoints(const std::vector<DeformableParticle*>& deformableParticleList)
{
	for (unsigned int iIndex = 0; iIndex < deformableParticleList.size(); iIndex++)
	{
		m_BezierParticleList[iIndex] = deformableParticleList[iIndex]->Position;
	}
	m_BezierParticleList[deformableParticleList.size()] = deformableParticleList[0]->Position;
}