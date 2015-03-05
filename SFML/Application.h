#ifndef APPLICATION_H
#define APPLICATION_H

#include <iostream>
#include <vector>

#include "Common.h"
#include "Particle.h"
#include "SpatialPartition.h"

class Application
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

	Application();
	~Application();

	void Initialize(sf::RenderWindow& window);
	void Update(sf::RenderWindow& window, float dt);
	void Draw(sf::RenderWindow& window);

	void BuildParticleSystem(int iParticleCount);

	glm::vec2 GetRandomPosWithinLimits();

private:
	sf::Texture m_Texture;

	std::vector<Particle> m_ParticleList;

	std::vector<ContainerConstraint> m_ContainerConstraints;

	SpatialPartition m_SpatialManager;

	void DrawContainer(sf::RenderWindow& window);

	void UpdateExternalForces(float dt);
	void DampVelocities();
	void CalculatePredictedPositions(sf::RenderWindow& window, float dt);
	void FindNeighborParticles();
	void UpdateActualPosAndVelocities(float dt);
	void GenerateCollisionConstraints(sf::RenderWindow& window);
	void XSPH_Viscosity(Particle& particle);

	// ------------------------------------------------------------------------

	void ComputeParticleConstraint(Particle& particle, std::vector<Particle*>& pNeighborList);

	// ------------------------------------------------------------------------

	glm::vec2 ComputeParticleGradientConstraint(Particle& particle, Particle& neighbor, std::vector<Particle*>& pParticleNeighborList);

	// ------------------------------------------------------------------------

	void ComputeLambda(Particle& particle, std::vector<Particle*>& pParticleNeighborList);

	// ------------------------------------------------------------------------

	void ComputePositionCorrection(Particle& particle, std::vector<Particle*>& pParticleNeighborList);

	// ------------------------------------------------------------------------

	float ComputeArtificialPressureTerm(const Particle& p1, const Particle& p2);

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