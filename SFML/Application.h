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

	inline float Poly6(const glm::vec2& r, float h) 
	{
		float rLength = sqrt(r.x * r.x + r.y * r.y);
		
		if (0 <= rLength && rLength <= h)
		{
			float diff = h * h - rLength * rLength;
			return POLY6COEFF * diff * diff * diff;
		}
		else
		{
			return 0.0f;
		}
	}

	inline glm::vec2 SpikyGradient(const glm::vec2& r, float h)
	{
		float rLength = sqrt(r.x * r.x + r.y * r.y);
		float diff = h - rLength;

		return SPIKYGRADCOEFF * diff * diff * glm::vec2(r.x / rLength, r.y / rLength);
	}

	inline void PrintVector2(const glm::vec2& v) { std::cout << "x = " << v.x << " y = " << v.y << std::endl; }
};



#endif // APPLICATION_H