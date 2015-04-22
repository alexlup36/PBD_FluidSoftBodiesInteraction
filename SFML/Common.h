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

inline bool IsBetween(const glm::vec2& p1, const glm::vec2& p2, const glm::vec2& x)
{
	return abs(glm::length(p1 - p2) - glm::length(x - p1) - glm::length(x - p2)) <= EPS;
}

// ----------------------------------------------------------------------------

// http://stackoverflow.com/questions/217578/point-in-polygon-aka-hit-test
inline bool Intersecting(float start1X, float start1Y, float end1X, float end1Y,
	float start2X, float start2Y, float end2X, float end2Y)
{
	float d1, d2;
	float a1, a2, b1, b2, c1, c2;

	// Convert vector 1 to a line (line 1) of infinite length.
	// We want the line in linear equation standard form: A*x + B*y + C = 0
	a1 = end1Y - start1Y;
	b1 = start1X - end1X;
	c1 = (end1X * start1Y) - (start1X * end1Y);

	// Every point (x,y), that solves the equation above, is on the line,
	// every point that does not solve it, is either above or below the line.
	// We insert (x1,y1) and (x2,y2) of vector 2 into the equation above.
	d1 = (a1 * start2X) + (b1 * start2Y) + c1;
	d2 = (a1 * end2X) + (b1 * end2Y) + c1;

	// If d1 and d2 both have the same sign, they are both on the same side of
	// our line 1 and in that case no intersection is possible. Careful, 0 is
	// a special case, that's why we don't test ">=" and "<=", but "<" and ">".
	if (d1 > 0 && d2 > 0) return false;
	if (d1 < 0 && d2 < 0) return false;

	// We repeat everything above for vector 2.
	// We start by calculating line 2 in linear equation standard form.
	a2 = end2Y - start2Y;
	b2 = start2X - end2X;
	c2 = (end2X * start2Y) - (start2X * end2Y);

	// Calculate d1 and d2 again, this time using points of vector 1
	d1 = (a2 * start1X) + (b2 * start1Y) + c2;
	d2 = (a2 * end1X) + (b2 * end1Y) + c2;

	// Again, if both have the same sign (and neither one is 0),
	// no intersection is possible.
	if (d1 > 0 && d2 > 0) return false;
	if (d1 < 0 && d2 < 0) return false;

	// If we get here, only three possibilities are left. Either the two
	// vectors intersect in exactly one point or they are collinear
	// (they both lie both on the same infinite line), in which case they
	// may intersect in an infinite number of points or not at all.
	if ((a1 * b2) - (a2 * b1) == 0.0f) return false;

	// If they are not collinear, they must intersect in exactly one point.
	return true;
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

inline sf::Color GetRandomColor()
{
	unsigned int r = rand() % 255;
	unsigned int g = rand() % 255;
	unsigned int b = rand() % 255;

	return sf::Color(r, g, b, 255);
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

// Simulation time;
const float TICKS_PER_SECOND	= 60.0f;
const float FIXED_DELTA			= 1.0f / TICKS_PER_SECOND;
const int MAX_FRAMESKIP			= 1;
const int SPEEDMULTIPLIER		= 1;

const float PARTICLE_RADIUS			= 3.0f;
const float PARTICLE_RADIUS_TWO		= PARTICLE_RADIUS + PARTICLE_RADIUS;
const float PARTICLE_RADIUS2		= PARTICLE_RADIUS_TWO * PARTICLE_RADIUS_TWO;
const float PARTICLE_MASS			= 2.0f;
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

const float PBDSTIFFNESSFLUIDCONTAINER = 0.8f;
const float PBDSTIFFNESS_ADJUSTEDFLUIDCONTAINTER = 1.0f - pow(1.0f - PBDSTIFFNESSFLUIDCONTAINER, 1.0f / SOLVER_ITERATIONS);

const float PBDSTIFFNESSSB			= 0.9;
const float PBDSTIFFNESS_ADJUSTEDSB = 1.0f - pow(1.0f - PBDSTIFFNESSSB, 1.0f / SOLVER_ITERATIONS);


#endif // COMMON_H
