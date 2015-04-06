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

		m_ThreadPool = nullptr;
	};

	void Update(sf::RenderWindow& window, float dt);
	void Draw(sf::RenderWindow& window);

	void BuildParticleSystem(const glm::vec2& startPosition, const sf::Color& color);
#ifdef MULTITHREADING
	void SetupMultithread();

	void AddFluidParticles(const glm::vec2& position, const sf::Color& color);
#endif // MULTITHREADING

	glm::vec2 GetRandomPosWithinLimits();
	inline const std::vector<FluidParticle*>& GetFluidParticleList() { return m_ParticleList; }

private:

	// -------------------------------------------------------------------------------
	// Multithreading ----------------------------------------------------------------
	// -------------------------------------------------------------------------------

#ifdef MULTITHREADING
	
	std::unique_ptr<boost::threadpool::pool> m_ThreadPool;
	typedef boost::function<void()> Task;
	std::vector<Task> LambdaTaskList;
	std::vector<Task> PositionCorrectionTaskList;
	std::vector<Task> ParticleConstratinTaskList;
	std::vector<Task> MinTransDistanceTaskList;

#endif // MULTITHREADING

	// -------------------------------------------------------------------------------
	// Member variables --------------------------------------------------------------
	// -------------------------------------------------------------------------------

	// Constants
	const bool XSPH_VISCOSITY = true;
	const bool ARTIFICIAL_PRESSURE_TERM = true;
	const bool FLUIDRENDERING_PARTICLE = true;
	const bool FLUIDRENDERING_MARCHINGSQUARES = false;
	const bool PBD_COLLISION = false;

	const int PARTICLE_WIDTH_COUNT = 40;
	const int PARTICLE_HEIGHT_COUNT = 40;

	const int PARTICLE_WIDTH_NEW = 20;
	const int PARTICLE_HEIGHT_NEW = 20;

	const float VELOCITY_DAMPING = 0.999f;

	const float XSPHParam = 0.05f;

	// Constants used for SPH
	const float SMOOTHING_DISTANCE = CELL_SIZE;
	const float SMOOTHING_DISTANCE2 = CELL_SIZE * CELL_SIZE;
	const float SMOOTHING_DISTANCE9 = CELL_SIZE * CELL_SIZE * CELL_SIZE *
		CELL_SIZE * CELL_SIZE * CELL_SIZE *
		CELL_SIZE * CELL_SIZE * CELL_SIZE;
	const float PI = 3.14159265359f;
	const float POLY6COEFF = 315.0f / 64.0f / PI / SMOOTHING_DISTANCE9;
	const float SIXPOLY6COEFF = 6.0f * POLY6COEFF;
	const float WATER_RESTDENSITY = 1000.0f;
	const float INVERSE_WATER_RESTDENSITY = 1.0f / WATER_RESTDENSITY;
	const float ARTIFICIAL_PRESSURE = POLY6COEFF * std::pow(SMOOTHING_DISTANCE2 - 0.1f * SMOOTHING_DISTANCE2, 3.0f);
	const float INVERSE_ARTIFICIAL_PRESSURE = 1.0f / ARTIFICIAL_PRESSURE;
	const float SMOOTHING_DISTANCE6 = CELL_SIZE * CELL_SIZE * CELL_SIZE * CELL_SIZE * CELL_SIZE * CELL_SIZE;
	const float SPIKYGRADCOEFF = 45.0f / PI / SMOOTHING_DISTANCE6;

	// PBF constant
	const float RELAXATION_PARAMETER = 0.0000075f;

	// -------------------------------------------------------------------------------

	std::vector<FluidParticle*> m_ParticleList;
	std::vector<ContainerConstraint> m_ContainerConstraints;

	ParticleManager* m_ParticleManager;

	void DrawContainer(sf::RenderWindow& window);

	void UpdateExternalForces(float dt);
	void DampVelocities();

	void CalculatePredictedPositions(sf::RenderWindow& window, float dt);
	
	void FindNeighborParticles();

	void UpdateActualPosAndVelocities(float dt);
	void GenerateCollisionConstraints(sf::RenderWindow& window);
	void XSPH_Viscosity(FluidParticle* particle);

	// ------------------------------------------------------------------------

	void ComputeParticleConstraint(FluidParticle* particle);
	glm::vec2 ComputeParticleGradientConstraint(FluidParticle* particle, FluidParticle* neighbor);
	void ComputeLambda(FluidParticle* particle);
	void ComputePositionCorrection(FluidParticle* particle);
	float ComputeArtificialPressureTerm(const FluidParticle* p1, const FluidParticle* p2);
	void ContainerCollisionUpdate();

	// ------------------------------------------------------------------------

#ifdef MULTITHREADING

	void ComputeParticleConstraintMultithread(int iStartIndex, int iEndIndex);
	void ComputeLambdaMultithread(int iStartIndex, int iEndIndex);
	void PositionCorrectionMultithread(int iStartIndex, int iEndIndex);
	void ComputeMTDMultithread(int iStartIndex, int iEndIndex);

#endif // MULTITHREADING

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