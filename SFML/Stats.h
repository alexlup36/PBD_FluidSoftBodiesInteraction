#ifndef STATS_H
#define STATS_H

#include "Common.h"

class Stats
{
public:
	Stats(const sf::Font& font,
		float xPos, float yPos,
		unsigned int size,
		const sf::Color& color);
	
	inline void ClearString() { m_Stats.setString(""); }
	inline void SetString(const std::string& text) { m_Stats.setString(text); }
	inline void Draw(sf::RenderWindow& window) { window.draw(m_Stats); }

private:
	sf::Text m_Stats;
};

#endif // STATS_H