#ifndef MARCHINGSQUARES_H
#define MARCHINGSQUARES_H

#include "Common.h"
#include "FluidParticle.h"
#include "FluidSimulation.h"

struct Cell
{
	// Samples for the corners of the cell
	float SampleTopRight;
	float SampleTopLeft;
	float SampleBottomLeft;
	float SampleBottomRight;

	// Intersections with edges
	glm::vec2 LeftEdgeIntersection;
	glm::vec2 RightEdgeIntersection;
	glm::vec2 BottomEdgeIntersection;
	glm::vec2 TopEdgeIntersection;

	// Line type
	int m_iLineType;
};

class MarchingSquares
{
public:
	static MarchingSquares& GetInstance()
	{
		static MarchingSquares instance;
		return instance;
	}

	// Update and draw marching squares
	void ProcessMarchingSquares(FluidSimulation* fluidSim, sf::RenderWindow& window);

private:
	// --------------------------------------------------------------------------------

	// Hide constructor for singleton implementation
	MarchingSquares() 
	{
		m_Map.resize(MAPHEIGHT);
		for (unsigned int i = 0; i < MAPHEIGHT; i++)
		{
			m_Map[i].resize(MAPWIDTH);
		}
	};

	// Delete unneeded copy constructor and assignment operator
	MarchingSquares(MarchingSquares const&) = delete;
	void operator=(MarchingSquares const&) = delete;

	// --------------------------------------------------------------------------------

	// Calculate the intensity
	float SamplePoint(FluidSimulation* sim, unsigned int x, unsigned int y);

	// Methods
	inline float InvSqrt(float x)
	{
		float xhalf = 0.5f * x;
		int i = *(int*)&x;					// store floating-point bits in integer
		i = 0x5f3759df - (i >> 1);			// initial guess for Newton's method
		x = *(float*)&i;					// convert new bits into float
		x = x * (1.5f - xhalf * x * x);     // One round of Newton's method 
		return x;
	}
	inline float CalculateEquation(const FluidParticle* particle, unsigned int x, unsigned int y)
	{
		float fParticleX = particle->Position.x;
		float fParticleY = particle->Position.y;

		float invSqrt = InvSqrt((x - fParticleX) * (x - fParticleX) + (y - fParticleY) * (y - fParticleY));
		return 0.5f * particle->Radius * invSqrt;
	}

	// Members

	// Constants

	// Uniform box division fluid rendering using marching squares
	const unsigned int BOXSIZE = (unsigned int)PARTICLE_RADIUS;
	const unsigned int MAPHEIGHT = (unsigned int)(WindowResolution.y / BOXSIZE) - 1;
	const unsigned int MAPWIDTH = (unsigned int)(WindowResolution.x / BOXSIZE) - 1;

	// Store the information for the uniform box division in a 2d array of cells
	std::vector<std::vector<Cell>> m_Map;
};


#endif // MARCHINGSQUARES_H