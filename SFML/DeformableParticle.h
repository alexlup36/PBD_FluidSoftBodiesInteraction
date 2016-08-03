#ifndef DEFORMABLEPARTICLE_H
#define DEFORMABLEPARTICLE_H

#include "BaseParticle.h"
#include "ParticleManager.h"

class SoftBody;

struct Edge
{
	DeformableParticle* Start;
	DeformableParticle* End;
};

class DeformableParticle : public BaseParticle
{
public:
	DeformableParticle()
	{

	}

	DeformableParticle(const glm::vec2& position, const sf::Color& color, unsigned int iParentIndex)
		: BaseParticle(position, iParentIndex)
	{
		// Particle type
		ParticleType = ParticleType::DeformableParticle;

		// Color
		m_ControlledColor	= sf::Color::Red;
		m_DefaultColor		= color;
		SetDefaultColor();
		m_Shape.setOutlineThickness(0.0f);

		// Position
		OriginalPosition	= position;
		GoalPosition		= position;

		// Fixed
		m_bFixed = false;
		if (m_bFixed)
		{
			InverseMass = 0.0f;
		}

		// Index
		Index = DeformableParticleGlobalIndex++;

		// Goal shape properties
		m_GoalShape.setPosition(sf::Vector2<float>(GoalPosition.x, GoalPosition.y));
		m_GoalShape.setRadius(PARTICLE_RADIUS);
		m_GoalShape.setOutlineColor(sf::Color::Cyan);
		m_GoalShape.setOutlineThickness(1.0f);
		m_GoalShape.setFillColor(sf::Color::Blue);
		m_GoalShape.setOrigin(m_GoalShape.getLocalBounds().width / 2.0f,
			m_GoalShape.getLocalBounds().height / 2.0f);
	}

	virtual void Update();
	void Draw(sf::RenderWindow& window) override;
	
	void UpdateGoalShapePosition();
	void DrawGoalShape(sf::RenderWindow& window);

	float CalculateMinimumTranslationDistance();

	inline void SetControlledColor() { m_Shape.setFillColor(m_ControlledColor); }

	inline bool IsFixedParticle() { return m_bFixed; }
	inline void SetFixedParticle(const glm::vec2& pos) 
	{ 
		m_bFixed = true; 
		PredictedPosition = pos;
	}
	inline void ReleaseParticle()
	{
		m_bFixed = false;
	}

	inline void SetParentRef(SoftBody* parent) { m_pParentReference = parent; }
	inline SoftBody* GetParent() { return m_pParentReference; }

	static bool IsPointInsidePolygon(const glm::vec2& point, const std::vector<Edge>& edgeList)
	{
		// Axis aligned bounding box
		float xMin, yMin, xMax, yMax;;
		xMin = yMin = std::numeric_limits<float>::max();
		xMax = yMax = std::numeric_limits<float>::min();

		for each (Edge e in edgeList)
		{
			float startXEdge = e.Start->Position.x;
			float endXEdge = e.End->Position.x;
			float startYEdge = e.Start->Position.y;
			float endYEdge = e.End->Position.y;

			// X
			if (startXEdge < xMin)
			{
				xMin = startXEdge;
			}
			if (startXEdge > xMax)
			{
				xMax = startXEdge;
			}
			if (endXEdge < xMin)
			{
				xMin = endXEdge;
			}
			if (endXEdge > xMax)
			{
				xMax = endXEdge;
			}

			// Y
			if (startYEdge < yMin)
			{
				yMin = startYEdge;
			}
			if (startYEdge > yMax)
			{
				yMax = startYEdge;
			}
			if (endYEdge < yMin)
			{
				yMin = endYEdge;
			}
			if (endYEdge > yMax)
			{
				yMax = endYEdge;
			}
		}

		// Check against bounding box
		if (point.x < xMin || point.x > xMax || point.y < yMin || point.y > yMax)
		{
			return false;
		}

		// Build ray
		float xStartRay = xMin - EPS;
		float yStartRay = point.y;
		float xEndRay = point.x;
		float yEndRay = point.y;

		unsigned int intersections = 0;

		// Check each edge for intersection
		for each (Edge e in edgeList)
		{
			if (Intersecting(xStartRay, yStartRay, xEndRay, yEndRay,
				e.Start->Position.x, e.Start->Position.y, e.End->Position.x, e.End->Position.y))
			{
				intersections++;
			}
		}

		// even hits outside, odd hits inside
		if (intersections % 2 == 0)
		{
			return false;
		}
		else
		{
			return true;
		}
	}

	// ------------------------------------------------------------------------
	// Public members
	// ------------------------------------------------------------------------

	glm::vec2 OriginalPosition;
	glm::vec2 GoalPosition;

private:
	static int DeformableParticleGlobalIndex;

	// ------------------------------------------------------------------------
	// Private members
	// ------------------------------------------------------------------------

	SoftBody* m_pParentReference;

	// Fixed particle infinite mass
	bool m_bFixed;

	// Color
	sf::Color m_ControlledColor;

	sf::CircleShape m_GoalShape;

	glm::vec2 m_vIntersectionPoint;
};

#endif // DEFORMABLEPARTICLE_H