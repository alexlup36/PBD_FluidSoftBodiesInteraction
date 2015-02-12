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
	Application();
	~Application();

	void Initialize(sf::RenderWindow& window);
	void Update(sf::RenderWindow& window, float dt);
	void Draw(sf::RenderWindow& window);

	void BuildParticleSystem(int iParticleCount);

	sf::Vector2f GetRandomPosWithinLimits();

private:
	sf::Texture m_Texture;

	std::vector<Particle> m_ParticleList;

	SpatialPartition m_SpatialManager;

	void DrawContainer(sf::RenderWindow& window);

	void UpdateExternalForces(float dt);
	void DampVelocities();
	void CalculatePredictedPositions(sf::RenderWindow& window, float dt);
	void FindNeighborParticles();
	void UpdateActualPosAndVelocities(float dt);
	void GenerateCollisionConstraints(sf::RenderWindow& window);

	inline float Dot(const sf::Vector2f& v1, const sf::Vector2f& v2) { return v1.x * v2.x + v1.y * v2.y; }
	inline void PrintVector2(const sf::Vector2f& v) { std::cout << "x = " << v.x << " y = " << v.y << std::endl; }
};



#endif // APPLICATION_H