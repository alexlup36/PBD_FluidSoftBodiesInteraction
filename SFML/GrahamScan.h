#include "SoftBodyParticle.h"
#include <stack>

class GrahamScan
{

public:

	static void InitializeSingleton(std::vector<SoftBodyParticle>& softBodyParticleList)
	{
		if (theInstance == nullptr)
		{
			theInstance = new GrahamScan(softBodyParticleList);
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
	friend extern bool SmallestY(SoftBodyParticle& p1, SoftBodyParticle& p2);
	friend extern bool PolarOrder(SoftBodyParticle& p1, SoftBodyParticle& p2);

	static int CCWTurn(const SoftBodyParticle& p1,
		const SoftBodyParticle& p2,
		const SoftBodyParticle& p3)
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
	GrahamScan(std::vector<SoftBodyParticle>& softBodyParticleList);
	
	// Delete unneeded copy constructor and assignment operator
	GrahamScan(GrahamScan const&) = delete;
	void operator=(GrahamScan const&) = delete;

	static SoftBodyParticle m_Pivot;
	std::stack<SoftBodyParticle*> m_ConvexHull;
};