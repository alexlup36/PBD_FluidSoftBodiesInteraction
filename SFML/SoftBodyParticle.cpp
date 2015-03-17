#include "SoftBodyParticle.h"

int SoftBodyParticle::m_iGlobalIndex = 0;

void SoftBodyParticle::UpdateShapePosition()
{
	// Update the position of the shape
	m_Shape.setPosition(sf::Vector2f(Position.x, Position.y));
}

void SoftBodyParticle::Draw(sf::RenderWindow& window)
{
	window.draw(m_Shape);
}

void SoftBodyParticle::UpdateGoalShapePosition()
{
	// Update the goal position of the shape
	m_GoalPositionShape.setPosition(sf::Vector2f(GoalPosition.x, GoalPosition.y));
}

void SoftBodyParticle::DrawGoalShape(sf::RenderWindow& window)
{
	window.draw(m_GoalPositionShape);
}