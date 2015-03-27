#ifndef COMMON_H
#define COMMON_H

#include <SFML/Graphics.hpp>
#include <math.h>

#include <glm/vec2.hpp>
#include <glm/mat2x2.hpp>
#include <glm/glm.hpp>
#include <glm/common.hpp>

#include <iostream>

#include <vector>

#define EPS 0.001f

// ------------------------------------------------------------------------
// Enum definition
// ------------------------------------------------------------------------

enum class ParticleType
{
	FluidParticle,
	DeformableParticle,

	Invalid
};

enum class SimulationType
{
	FluidSimulation,
	SoftBodySimulation,

	Invalid
};


// Window
const sf::Vector2i WindowResolution = sf::Vector2i(1366, 768);
const float HorizontalOffset		= 100.0f;
const float VerticalOffsetTop		= 150.0f;
const float VerticalOffsetBottom	= 50.0f; 

// Simulation time
const float TICKS_PER_SECOND	= 30.0f;
const float SKIP_TICKS			= 1.0f / TICKS_PER_SECOND;
const float FIXED_DELTA			= 1.0f / 30.0f;
const int MAX_FRAMESKIP			= 1;
const int SPEEDMULTIPLIER		= 2;

const int PARTICLE_WIDTH_COUNT		= 30;
const int PARTICLE_HEIGHT_COUNT		= 30;
const int PARTICLE_COUNT			= PARTICLE_WIDTH_COUNT * PARTICLE_HEIGHT_COUNT;
const float PARTICLE_RADIUS			= 4.0f;
const float PARTICLE_RADIUS_TWO		= PARTICLE_RADIUS + PARTICLE_RADIUS;
const float PARTICLE_RADIUS2		= PARTICLE_RADIUS_TWO * PARTICLE_RADIUS_TWO;
const float PARTICLE_MASS			= 1.0f;
const float PARTICLE_INVERSE_MASS	= 1.0f / PARTICLE_MASS;

// Container dimensions
const float CONTAINER_WIDTH		= WindowResolution.x - 2.0f * HorizontalOffset;
const float CONTAINER_HEIGHT	= WindowResolution.y - (VerticalOffsetTop + VerticalOffsetBottom);

// Spatial partitioning
const float CELL_SIZE	= 4.0f * PARTICLE_RADIUS;
const float CELL_COLS	= std::trunc(CONTAINER_WIDTH / CELL_SIZE);
const float CELL_ROWS	= std::trunc(CONTAINER_HEIGHT / CELL_SIZE);
const float TOTAL_CELLS = CELL_COLS * CELL_ROWS;

// Fluid limits
const float WALL_LEFTLIMIT			= HorizontalOffset;
const float WALL_RIGHTLIMIT			= WALL_LEFTLIMIT + CELL_COLS * CELL_SIZE;
const float WALL_TOPLIMIT			= VerticalOffsetTop;
const float WALL_BOTTOMLIMIT		= WALL_TOPLIMIT + CELL_ROWS * CELL_SIZE;

// Container corners
const glm::vec2 ContainerBottomLeft		= glm::vec2(WALL_LEFTLIMIT, WALL_BOTTOMLIMIT);
const glm::vec2 ContainerTopLeft		= glm::vec2(WALL_LEFTLIMIT, WALL_TOPLIMIT);
const glm::vec2 ContainerBottomRight	= glm::vec2(WALL_RIGHTLIMIT, WALL_BOTTOMLIMIT);
const glm::vec2 ContainerTopRight		= glm::vec2(WALL_RIGHTLIMIT, WALL_TOPLIMIT);

// Particle limits
const float PARTICLE_LEFTLIMIT		= WALL_LEFTLIMIT + PARTICLE_RADIUS + 1.0f;
const float PARTICLE_RIGHTLIMIT		= WALL_RIGHTLIMIT - PARTICLE_RADIUS - 1.0f;
const float PARTICLE_TOPLIMIT		= WALL_TOPLIMIT + PARTICLE_RADIUS + 1.0f;
const float PARTICLE_BOTTOMLIMIT	= WALL_BOTTOMLIMIT - PARTICLE_RADIUS - 1.0f;

const glm::vec2 GRAVITATIONAL_ACCELERATION(0.0f, 9.81f);
const bool GRAVITY_ON						= true;
const bool XSPH_VISCOSITY					= true;
const bool ARTIFICIAL_PRESSURE_TERM			= true;
const bool FLUID_SIMULATION					= true;
const bool FLUIDRENDERING_PARTICLE			= true;
const bool FLUIDRENDERING_MARCHINGSQUARES	= false;
const bool PBD_COLLISION					= false;
const bool SOFTBODY_SIMULATION				= true;

// Physics constants
const float VELOCITY_DAMPING = 0.999f;

// Solver iterations
const int SOLVER_ITERATIONS = 3;

// Constants used for SPH
const float XSPHParam					= 0.05f;
const float SMOOTHING_DISTANCE			= CELL_SIZE;
const float SMOOTHING_DISTANCE2			= CELL_SIZE * CELL_SIZE;
const float SMOOTHING_DISTANCE9			= CELL_SIZE * CELL_SIZE * CELL_SIZE *
	CELL_SIZE * CELL_SIZE * CELL_SIZE *
	CELL_SIZE * CELL_SIZE * CELL_SIZE;
const float PI							= 3.14159265359f;
const float POLY6COEFF					= 315.0f / 64.0f / PI / SMOOTHING_DISTANCE9;
const float SIXPOLY6COEFF				= 6.0f * POLY6COEFF;
const float WATER_RESTDENSITY			= 1000.0f;
const float INVERSE_WATER_RESTDENSITY	= 1.0f / WATER_RESTDENSITY;
const float ARTIFICIAL_PRESSURE			= POLY6COEFF * std::pow(SMOOTHING_DISTANCE2 - 0.1f * SMOOTHING_DISTANCE2, 3.0f);
const float INVERSE_ARTIFICIAL_PRESSURE = 1.0f / ARTIFICIAL_PRESSURE;
const float SMOOTHING_DISTANCE6			= CELL_SIZE * CELL_SIZE * CELL_SIZE * CELL_SIZE * CELL_SIZE * CELL_SIZE;
const float SPIKYGRADCOEFF				= 45.0f / PI / SMOOTHING_DISTANCE6;

// PBF constant
const float RELAXATION_PARAMETER = 0.000001f;

// PBD constants
const float PBDSTIFFNESS			= 0.1f;
const float PBDSTIFFNESS_ADJUSTED	= 1.0f - pow(1.0f - PBDSTIFFNESS, 1.0f / SOLVER_ITERATIONS);

// ------------------------------------------------------------------------------
// Soft body constants
// ------------------------------------------------------------------------------
const float SOFTBODY_RESTITUTION_COEFF	= 0.9f;
const float SOFTBODY_STIFFNESS_VALUE	= 0.2f; // 0.01f - almost rigid 1.0f - 100.0f elastic

const float SOFTBODYPARTICLE_LEFTLIMIT		= WALL_LEFTLIMIT + PARTICLE_RADIUS;
const float SOFTBODYPARTICLE_RIGHTLIMIT		= WALL_RIGHTLIMIT - PARTICLE_RADIUS;
const float SOFTBODYPARTICLE_TOPLIMIT		= WALL_TOPLIMIT + PARTICLE_RADIUS;
const float SOFTBODYPARTICLE_BOTTOMLIMIT	= WALL_BOTTOMLIMIT - PARTICLE_RADIUS;

// Uniform box division fluid rendering using marching squares
const unsigned int BOXSIZE		= (unsigned int)PARTICLE_RADIUS;
const unsigned int MAPHEIGHT	= (unsigned int)(WindowResolution.y / BOXSIZE) - 1;
const unsigned int MAPWIDTH		= (unsigned int)(WindowResolution.x / BOXSIZE) - 1;

// Methods
void DrawLine(sf::RenderWindow& window,
	const glm::vec2& p1,
	const glm::vec2& p2,
	const sf::Color& color);

#endif // COMMON_H
