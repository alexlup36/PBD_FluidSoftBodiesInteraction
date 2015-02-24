#ifndef COMMON_H
#define COMMON_H

#include <SFML/Graphics.hpp>
#include <math.h>

#include <glm/vec2.hpp>
#include <glm/mat2x2.hpp>
#include <glm/glm.hpp>
#include <glm/common.hpp>

#define EPS 0.001f

const int PARTICLE_COUNT = 300;
const float PARTICLE_RADIUS = 2.0f;

// Container dimensions
const float CONTAINER_WIDTH = 1100.0f;
const float CONTAINER_HEIGHT = 550;

// Spatial partitioning
const float CELL_SIZE = 3685.5f * pow((float)PARTICLE_COUNT, -0.464f);
const float CELL_COLS = std::trunc(CONTAINER_WIDTH / CELL_SIZE);
const float CELL_ROWS = std::trunc(CONTAINER_HEIGHT / CELL_SIZE);

// Fluid limits
const float WALL_LEFTLIMIT			= 100.0f;
const float WALL_RIGHTLIMIT			= WALL_LEFTLIMIT + CELL_COLS * CELL_SIZE;
const float WALL_TOPLIMIT			= 200.0f;
const float WALL_BOTTOMLIMIT		= WALL_TOPLIMIT + CELL_ROWS * CELL_SIZE;

// Container corners
const glm::vec2 ContainerBottomLeft		= glm::vec2(WALL_LEFTLIMIT, WALL_BOTTOMLIMIT);
const glm::vec2 ContainerTopLeft		= glm::vec2(WALL_LEFTLIMIT, WALL_TOPLIMIT);
const glm::vec2 ContainerBottomRight	= glm::vec2(WALL_RIGHTLIMIT, WALL_BOTTOMLIMIT);
const glm::vec2 ContainerTopRight		= glm::vec2(WALL_RIGHTLIMIT, WALL_TOPLIMIT);

// Particle limits
const float PARTICLE_LEFTLIMIT		= WALL_LEFTLIMIT + PARTICLE_RADIUS;
const float PARTICLE_RIGHTLIMIT		= WALL_RIGHTLIMIT - PARTICLE_RADIUS;
const float PARTICLE_TOPLIMIT		= WALL_TOPLIMIT + PARTICLE_RADIUS;
const float PARTICLE_BOTTOMLIMIT	= WALL_BOTTOMLIMIT - PARTICLE_RADIUS;

// Forces
const glm::vec2 GRAVITATIONAL_ACCELERATION(0.0f, 9.81f);
const bool GRAVITY_ON = true;
const bool XSPH_VISCOSITY = true;

// Physics constants
const float VELOCITY_DAMPING = 0.99999f;

// Solver iterations
const int SOLVER_ITERATIONS = 5;

// Constants used for SPH
const float XSPHParam = 0.05f;
const float SMOOTHING_DISTANCE = CELL_SIZE;
const float SMOOTHING_DISTANCE9 = CELL_SIZE * CELL_SIZE * CELL_SIZE *
	CELL_SIZE * CELL_SIZE * CELL_SIZE *
	CELL_SIZE * CELL_SIZE * CELL_SIZE;
const float PI = 3.14159265359f;
const float POLY6COEFF = 315.0f / 64.0f / PI / SMOOTHING_DISTANCE9;
const float WATER_DENSITY = 1000.0f; // kg/m3
const float ONE_OVER_WATER_DENSITY = 1.0f / WATER_DENSITY;

const float SMOOTHING_DISTANCE6 = CELL_SIZE * CELL_SIZE * CELL_SIZE *
	CELL_SIZE * CELL_SIZE * CELL_SIZE;
const float SPIKYGRADCOEFF = 45.0f / PI / SMOOTHING_DISTANCE6;

// PBF constant
const float RELAXATION_PARAMETER = 0.001f;

#endif // COMMON_H
