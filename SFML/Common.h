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

// Multithreading
#include <thread>
#include <mutex>
#include <condition_variable>
#define MULTITHREADING

// SSE1
#include <emmintrin.h>

#define EPS 0.001f

// ----------------------------------------------------------------------------
// Methods --------------------------------------------------------------------
// ----------------------------------------------------------------------------

void DrawLine(sf::RenderWindow& window,
	const glm::vec2& p1,
	const glm::vec2& p2,
	const sf::Color& color);

// ----------------------------------------------------------------------------

float QuakeLength(const glm::vec2& v);

// ----------------------------------------------------------------------------

inline int Floor(float f)
{
	// SSE1 instructions for float->int
	return _mm_cvtt_ss2si(_mm_load_ss(&f));
}

// ----------------------------------------------------------------------------

inline float SquareRootFloat(float number) {
	long i;
	float x, y;
	const float f = 1.5F;

	x = number * 0.5F;
	y = number;
	i = *(long *)&y;
	i = 0x5f3759df - (i >> 1);
	y = *(float *)&i;
	y = y * (f - (x * y * y));
	y = y * (f - (x * y * y));
	return number * y;
}

// ----------------------------------------------------------------------------

// ----------------------------------------------------------------------------
// Constants ------------------------------------------------------------------
// ----------------------------------------------------------------------------


// Window
const sf::Vector2i WindowResolution = sf::Vector2i(1920, 1080);
const float HorizontalOffset		= 100.0f;
const float VerticalOffsetTop		= 150.0f;
const float VerticalOffsetBottom	= 50.0f; 

// Simulation time
const float TICKS_PER_SECOND	= 30.0f;
const float SKIP_TICKS			= 1.0f / TICKS_PER_SECOND;
const float FIXED_DELTA			= 1.0f / 30.0f;
const int MAX_FRAMESKIP			= 1;
const int SPEEDMULTIPLIER		= 2;

const float PARTICLE_RADIUS			= 3.0f;
const float PARTICLE_RADIUS_TWO		= PARTICLE_RADIUS + PARTICLE_RADIUS;
const float PARTICLE_RADIUS2		= PARTICLE_RADIUS_TWO * PARTICLE_RADIUS_TWO;
const float PARTICLE_MASS			= 1.0f;
const float PARTICLE_INVERSE_MASS	= 1.0f / PARTICLE_MASS;

// Container dimensions
const float CONTAINER_WIDTH		= WindowResolution.x - 2.0f * HorizontalOffset;
const float CONTAINER_HEIGHT	= WindowResolution.y - (VerticalOffsetTop + VerticalOffsetBottom);

// Spatial partitioning
const float CELL_SIZE			= 4.0f * PARTICLE_RADIUS;
const float INVERSE_CELL_SIZE	= 1.0f / CELL_SIZE;
const int CELL_COLS				= Floor(CONTAINER_WIDTH / CELL_SIZE);
const int CELL_ROWS				= Floor(CONTAINER_HEIGHT / CELL_SIZE);
const int TOTAL_CELLS			= CELL_COLS * CELL_ROWS;

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
const bool GRAVITY_ON			= true;
const bool FLUID_SIMULATION		= true;
const bool SOFTBODY_SIMULATION	= true;

// Solver iterations
const int SOLVER_ITERATIONS = 3;

// PBD constants
const float PBDSTIFFNESS			= 0.1f;
const float PBDSTIFFNESS_ADJUSTED	= 1.0f - pow(1.0f - PBDSTIFFNESS, 1.0f / SOLVER_ITERATIONS);


#endif // COMMON_H
