#include "Common.h"

void DrawLine(sf::RenderWindow& window,
	const glm::vec2& p1,
	const glm::vec2& p2,
	const sf::Color& color)
{
	sf::Vertex line[] =
	{
		sf::Vertex(sf::Vector2f(p1.x, p1.y), color),
		sf::Vertex(sf::Vector2f(p2.x, p2.y), color)
	};

	window.draw(line, 2, sf::Lines);
}