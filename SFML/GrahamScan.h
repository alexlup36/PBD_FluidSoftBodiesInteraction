#ifndef GRAHAMSCAN_H
#define GRAHAMSCAN_H

#include "DeformableParticle.h"
#include <stack>

struct Edge;

class GrahamScan
{

public:

	GrahamScan() {};

	void Initialize(std::vector<DeformableParticle*>& deformableParticleList);

	void Draw(sf::RenderWindow& window);

	// Graham scan
	friend extern bool SmallestY(DeformableParticle* p1, DeformableParticle* p2);
	friend extern bool PolarOrder(DeformableParticle* p1, DeformableParticle* p2);
	friend extern int CCWTurn(const DeformableParticle& p1,
		const DeformableParticle& p2,
		const DeformableParticle& p3);

	const std::vector<Edge>& GetEdgeList() { return m_EdgeList; }

private:

	static DeformableParticle* m_Pivot;
	std::stack<DeformableParticle*> m_ConvexHull;
	std::vector<Edge> m_EdgeList;
};

#endif // GRAHAMSCAN_H