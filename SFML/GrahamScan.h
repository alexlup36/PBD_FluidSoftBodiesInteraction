#include "DeformableParticle.h"
#include <stack>

class GrahamScan
{

public:

	static void InitializeSingleton(std::vector<DeformableParticle*>& deformableParticleList)
	{
		if (theInstance == nullptr)
		{
			theInstance = new GrahamScan(deformableParticleList);
		}
	}

	static GrahamScan* GetInstance()
	{
		if (theInstance != nullptr)
		{
			return theInstance;
		}

		return nullptr;
	}

	void Draw(sf::RenderWindow& window);

	// Graham scan
	friend extern bool SmallestY(DeformableParticle* p1, DeformableParticle* p2);
	friend extern bool PolarOrder(DeformableParticle* p1, DeformableParticle* p2);

	static int CCWTurn(const DeformableParticle& p1,
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

private:

	static GrahamScan* theInstance;

	// Hide constructor for singleton implementation
	GrahamScan() {};
	GrahamScan(std::vector<DeformableParticle*>& deformableParticleList);
	
	// Delete unneeded copy constructor and assignment operator
	GrahamScan(GrahamScan const&) = delete;
	void operator=(GrahamScan const&) = delete;

	static DeformableParticle* m_Pivot;
	std::stack<DeformableParticle*> m_ConvexHull;
};