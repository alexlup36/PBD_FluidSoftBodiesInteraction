#include "GrahamScan.h"

DeformableParticle* GrahamScan::m_Pivot = nullptr;


void GrahamScan::Initialize(std::vector<DeformableParticle*>& deformableParticleList)
{
	while (!m_ConvexHull.empty())
	{
		m_ConvexHull.pop();
	}

	// Calculate the convex hull using the graham scan algorithm
	std::sort(deformableParticleList.begin(), deformableParticleList.end(), SmallestY);

	m_Pivot = deformableParticleList[0];

	std::sort(deformableParticleList.begin() + 1, deformableParticleList.end(), PolarOrder);

	m_ConvexHull.push(deformableParticleList[0]);
	m_ConvexHull.push(deformableParticleList[1]);

	for (unsigned int i = 2; i < deformableParticleList.size(); i++)
	{
		DeformableParticle* top = m_ConvexHull.top();
		m_ConvexHull.pop();

		while (m_ConvexHull.size() > 0 && CCWTurn(*m_ConvexHull.top(), *top, *deformableParticleList[i]) != -1)
		{
			top = m_ConvexHull.top();
			m_ConvexHull.pop();
		}

		m_ConvexHull.push(top);
		m_ConvexHull.push(deformableParticleList[i]);
	}

	// Initialize edges
	std::stack<DeformableParticle*> tempStack = m_ConvexHull;
	int iConvexHullSize = m_ConvexHull.size();
	DeformableParticle* top = nullptr;
	m_EdgeList.clear();

	while (tempStack.size() > 0)
	{
		top = tempStack.top();
		tempStack.pop();

		Edge edge;
		edge.Start = top;
	
		if (tempStack.size() != 0)
		{
			edge.End = tempStack.top();
		}
		else
		{
			edge.End = m_EdgeList[0].Start;
		}

		m_EdgeList.push_back(edge);
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

int CCWTurn(const DeformableParticle& p1,
	const DeformableParticle& p2,
	const DeformableParticle& p3)
{
	int iArea = (int)((p2.Position.x - p1.Position.x) * (p3.Position.y - p1.Position.y) -
		(p2.Position.y - p1.Position.y) * (p3.Position.x - p1.Position.x));

	if (iArea > 0)
	{
		return -1;
	}
	else if (iArea < 0)
	{
		return 1;
	}

	return 0;
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
	int order = CCWTurn(*GrahamScan::m_Pivot, *p1, *p2);
	if (order == 0)
	{
		return glm::length(GrahamScan::m_Pivot->Position - p1->Position) <
			glm::length(GrahamScan::m_Pivot->Position - p2->Position);
	}
	return order == -1;
}