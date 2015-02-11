#ifndef QUADTREE_H
#define QUADTREE_H

#include "Common.h"
#include "Particle.h"

class Quadtree
{
public:
	Quadtree();
	Quadtree(int level, const sf::FloatRect& bounds);

	void Clear();

private:
	int m_iMaxObjects;	// Max objects in the node before it splits
	int m_iMaxDepth;	// Max depth a node can be

	int m_iCurrentDepth;					// Current depth level
	std::vector<Particle*> m_ObjectList;	// Objects stored in the quad tree
	std::vector<Quadtree*> m_ChildNodes;	// List of the sub nodes
	sf::FloatRect m_Bounds;					// 2D space that this node occupies

	// Splits the node into 4 child nodes
	void Split();
	// Get the node index where the particle belongs to
	int GetIndex(const Particle& particle);
};

#endif // QUADTREE_H