#ifndef APPLICATION_H
#define APPLICATION_H

#include <iostream>
#include <vector>
#include <memory>

#include "Common.h"
#include "FluidParticle.h"
#include "DeformableParticle.h"
#include "SpatialPartition.h"
#include "BaseSimulation.h"

// Multithreading
#ifdef MULTITHREADING
#include <boost/threadpool.hpp>
#endif // MULTITHREADING


class FluidSimulation : public BaseSimulation
{
public:

	struct ContainerConstraint 
	{ 
		int particleIndex;	
		float stiffness; 
		float stiffness_adjusted; 
		glm::vec2 normalVector;
		glm::vec2 projectionPoint;
	};

	FluidSimulation() 
	{
		srand((unsigned int)time(NULL));

		// Spatial partition
		SpatialPartition::GetInstance().Setup();

		// Set simulation type
		SimType = SimulationType::FluidSimulation;
	};

	void Update(sf::RenderWindow& window, float dt);
	void Draw(sf::RenderWindow& window);

	void BuildParticleSystem(const glm::vec2& startPosition, const sf::Color& color, int iParticleCount);

	glm::vec2 GetRandomPosWithinLimits();
	inline const std::vector<FluidParticle*>& GetFluidParticleList() { return m_ParticleList; }

private:

	// -------------------------------------------------------------------------------
	// Multithreading ----------------------------------------------------------------
	// -------------------------------------------------------------------------------

#ifdef MULTITHREADING
	
	std::unique_ptr<boost::threadpool::pool> m_ThreadPool;
	typedef boost::function<void()> Task;
	std::vector<Task> taskList;

#endif // MULTITHREADING

	// -------------------------------------------------------------------------------
	// Member variables --------------------------------------------------------------
	// -------------------------------------------------------------------------------

	std::vector<FluidParticle*> m_ParticleList;
	std::vector<ContainerConstraint> m_ContainerConstraints;

	ParticleManager* m_ParticleManager;

	void DrawContainer(sf::RenderWindow& window);

	void UpdateExternalForces(float dt);
	void DampVelocities();

	void CalculatePredictedPositions(sf::RenderWindow& window, float dt);
	
	void FindNeighborParticles();
	void RegisterFluidObject(int iStartIndex, int iEndIndex);
	void RegisterDeformableObject(int iStartIndex, int iEndIndex);

	void UpdateActualPosAndVelocities(float dt);
	void GenerateCollisionConstraints(sf::RenderWindow& window);
	void XSPH_Viscosity(FluidParticle* particle);

	// ------------------------------------------------------------------------

	void ComputeParticleConstraint(FluidParticle* particle);

	// ------------------------------------------------------------------------

	glm::vec2 ComputeParticleGradientConstraint(FluidParticle* particle, FluidParticle* neighbor);

	// ------------------------------------------------------------------------

	void ComputeLambda(FluidParticle* particle);

	// ------------------------------------------------------------------------

	void ComputePositionCorrection(FluidParticle* particle);

	// ------------------------------------------------------------------------

	float ComputeArtificialPressureTerm(const FluidParticle* p1, const FluidParticle* p2);

	// ------------------------------------------------------------------------

	float Poly6Kernel(const glm::vec2& pi, const glm::vec2& pj) 
	{
		float rLength = glm::length(pi - pj);
		float rLength2 = rLength * rLength;
		
		// Poly6 kernel is 0 for r<=h (lenght<=smoothing distance)
		if (rLength > SMOOTHING_DISTANCE || rLength == 0.0f)
		{
			return 0.0f;
		}
		
		float diff = SMOOTHING_DISTANCE2 - rLength2;
		return POLY6COEFF * diff * diff * diff;
	}

	// ------------------------------------------------------------------------

	glm::vec2 Poly6KernelGradient(const glm::vec2& pi, const glm::vec2& pj)
	{
		glm::vec2 r = pi - pj;
		float rLength = glm::length(r);
		float rLength2 = rLength * rLength;

		// Poly6 kernel is 0 for r<=h (lenght<=smoothing distance)
		if (rLength > SMOOTHING_DISTANCE || rLength == 0.0f)
		{
			return glm::vec2(0.0f);
		}

		float diff = SMOOTHING_DISTANCE2 - rLength2;
		return (-1.0f) * SIXPOLY6COEFF * r * diff * diff;
	}

	// ------------------------------------------------------------------------

	glm::vec2 SpikyKernelGradient(const glm::vec2& pi, const glm::vec2& pj)
	{
		glm::vec2 r = pi - pj;
		float rLength = glm::length(r);

		if (rLength > SMOOTHING_DISTANCE || rLength == 0.0f)
		{
			return glm::vec2(0.0f);
		}

		float diff = SMOOTHING_DISTANCE - rLength;

		return (SPIKYGRADCOEFF * diff * diff * 1.0f / (rLength + 0.0001f)) * r;
	}

	// ------------------------------------------------------------------------

	inline void PrintVector2(const glm::vec2& v) { std::cout << "x = " << v.x << " y = " << v.y << std::endl; }
};



#endif // APPLICATION_H