#include "Quadtree.h"


Quadtree::Quadtree()
	: m_iCurrentDepth(0)
{
	// Initialize the bounds of the node
	m_Bounds = sf::FloatRect(WALL_LEFTLIMIT,
		WALL_TOPLIMIT,
		WALL_RIGHTLIMIT - WALL_LEFTLIMIT,
		WALL_BOTTOMLIMIT - WALL_TOPLIMIT);
}

Quadtree::Quadtree(int level, const sf::FloatRect& bounds)
	: m_iCurrentDepth(level), m_Bounds(bounds)
{

}

void Quadtree::Clear()
{
	for (unsigned int i = 0; i < m_ChildNodes.size(); i++)
	{
		if (m_ChildNodes[i] != nullptr)
		{
			// Recursive call
			m_ChildNodes[i]->Clear();

			// Delete child node
			delete m_ChildNodes[i];
			m_ChildNodes[i] = nullptr;
		}
	}
}

void Quadtree::Split()
{
	float fLeft = m_Bounds.left;
	float fTop = m_Bounds.top;
	float fSubWidth = m_Bounds.width / 2.0f;
	float fSubHeight = m_Bounds.height / 2.0f;

	// Top right child - 0
	sf::FloatRect topRightRect = sf::FloatRect(fLeft + fSubWidth, fTop, fSubWidth, fSubHeight);
	m_ChildNodes.push_back(new Quadtree(m_iCurrentDepth + 1, topRightRect));
	// Top left child - 1
	sf::FloatRect topLeftRect = sf::FloatRect(fLeft, fTop, fSubWidth, fSubHeight);
	m_ChildNodes.push_back(new Quadtree(m_iCurrentDepth + 1, topLeftRect));
	// Bottom left child - 2
	sf::FloatRect bottomLeftRect = sf::FloatRect(fLeft, fTop + fSubHeight, fSubWidth, fSubHeight);
	m_ChildNodes.push_back(new Quadtree(m_iCurrentDepth + 1, bottomLeftRect));
	// Bottom right child - 3
	sf::FloatRect bottomRightRect = sf::FloatRect(fLeft + fSubWidth, fTop + fSubHeight, fSubWidth, fSubHeight);
	m_ChildNodes.push_back(new Quadtree(m_iCurrentDepth + 1, bottomRightRect));
}

int Quadtree::GetIndex(const Particle& particle)
{
	int iIndex = -1;

	float fVerticalMidpoint = m_Bounds.left + m_Bounds.width / 2.0f;
	float fHorizontalMidpoint = m_Bounds.top + m_Bounds.height / 2.0f;

	// Object fits completely within the top half
	bool bTopHalf = particle.GetPosition().y + particle.GetRadius() < fHorizontalMidpoint;
	// Object fits completely within the bottom half
	bool bBottomHalf = particle.GetPosition().y - particle.GetRadius() > fHorizontalMidpoint;

	// Object fits completely within the left quadrants
	//if ()

	return iIndex;
}
