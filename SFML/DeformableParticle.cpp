#include "DeformableParticle.h"

int DeformableParticle::DeformableParticleGlobalIndex = 0;


void DeformableParticle::UpdateGoalShapePosition()
{
	// Update the goal position of the shape
	m_GoalShape.setPosition(sf::Vector2f(GoalPosition.x, GoalPosition.y));
}

void DeformableParticle::DrawGoalShape(sf::RenderWindow& window)
{
	window.draw(m_GoalShape);
}