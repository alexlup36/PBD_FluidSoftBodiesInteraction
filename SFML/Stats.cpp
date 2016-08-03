#include "Stats.h"


Stats::Stats(const sf::Font& font,
	float xPos, float yPos,
	unsigned int size,
	const sf::Color& color)
{
	m_Stats.setFont(font);
	m_Stats.setColor(color);
	m_Stats.setCharacterSize(size);
	m_Stats.setPosition(sf::Vector2f(xPos, yPos));
}