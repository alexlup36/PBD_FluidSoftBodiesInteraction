#include "GrahamScan.h"

DeformableParticle* GrahamScan::m_Pivot = nullptr;
GrahamScan* GrahamScan::theInstance = nullptr;

GrahamScan::GrahamScan(std::vector<DeformableParticle*>& softBodyParticleList)
{
	// Calculate the convex hull using the graham scan algorithm
	std::sort(softBodyParticleList.begin(), softBodyParticleList.end(), SmallestY);

	std::sort(softBodyParticleList.begin() + 1, softBodyParticleList.end(), PolarOrder);

	m_ConvexHull.push(softBodyParticleList[0]);
	m_ConvexHull.push(softBodyParticleList[1]);

	for (unsigned int i = 2; i < softBodyParticleList.size(); i++)
	{
		DeformableParticle* top = m_ConvexHull.top();
		m_ConvexHull.pop();

		while (m_ConvexHull.size() > 0 && CCWTurn(*m_ConvexHull.top(), *top, *softBodyParticleList[i]) != -1)
		{
			top = m_ConvexHull.top();
			m_ConvexHull.pop();
		}
		
		m_ConvexHull.push(top);
		m_ConvexHull.push(softBodyParticleList[i]);
	}
}

void GrahamScan::Draw(sf::RenderWindow& window)
{
	// Draw the convex hull
	std::stack<DeformableParticle*> tempStack = m_ConvexHull;
	int iConvexHullSize = m_ConvexHull.size();
	std::vector<sf::Vertex> lines;
	DeformableParticle* top = nullptr;

	while (tempStack.size() > 0)
	{
		top = tempStack.top();
		tempStack.pop();

		lines.push_back(sf::Vertex(sf::Vector2f(top->Position.x, top->Position.y)));
		if (tempStack.size() != 0)
		{
			lines.push_back(sf::Vertex(sf::Vector2f(tempStack.top()->Position.x, tempStack.top()->Position.y)));
		}
		else
		{
			lines.push_back(sf::Vertex(sf::Vector2f(lines[0].position.x, lines[0].position.y)));
		}
	}
	window.draw(&lines[0], iConvexHullSize * 2, sf::Lines);
}

// Graham scan util - friends
bool SmallestY(DeformableParticle* p1, DeformableParticle* p2)
{
	if (p1->Position.y != p2->Position.y)
	{
		return p1->Position.y > p2->Position.y;
	}
	return p1->Position.x < p2->Position.x;
}

bool PolarOrder(DeformableParticle* p1, DeformableParticle* p2)
{
	int order = GrahamScan::CCWTurn(*GrahamScan::m_Pivot, *p1, *p2);
	if (order == 0)
	{
		return glm::length(GrahamScan::m_Pivot->Position - p1->Position) <
			glm::length(GrahamScan::m_Pivot->Position - p2->Position);
	}
	return order == -1;
}