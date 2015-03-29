#ifndef FLUIDPARTICLE_H
#define FLUIDPARTICLE_H

#include "BaseParticle.h"
#include "ParticleManager.h"

class FluidSimulation;

class FluidParticle : public BaseParticle
{
public:
	FluidParticle(const glm::vec2& position, unsigned int iParentIndex)
		: BaseParticle(position, iParentIndex)
	{
		// Index
		Index = FluidParticleGlobalIndex++;

		// Color
		m_DefaultColor = sf::Color::Blue;
		SetDefaultColor();
		m_Shape.setOutlineThickness(0.0f);

		PredictedPosition	= Position;

		Velocity = glm::vec2(0.0f, 0.0f);

		// Fluid properties
		DensityConstraint	= 0.0f;
		SPHDensity			= 0.0f;
		Lambda				= 0.0f;

		// Particle type
		ParticleType = ParticleType::FluidParticle;
	}

	// ------------------------------------------------------------------------
	// Public members
	// ------------------------------------------------------------------------

	float SPHDensity;
	float DensityConstraint;
	float Lambda;

private:
	static int FluidParticleGlobalIndex;
};

#endif // FLUIDPARTICLE_H