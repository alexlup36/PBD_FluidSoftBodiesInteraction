#include "Common.h"

//std::vector<BaseParticle*> ParticleList;

void DrawLine(sf::RenderWindow& window,
	const glm::vec2& p1,
	const glm::vec2& p2)
{
	sf::Vertex line[] =
	{
		sf::Vertex(sf::Vector2f(p1.x, p1.y)),
		sf::Vertex(sf::Vector2f(p2.x, p2.y))
	};

	window.draw(line, 2, sf::Lines);
}