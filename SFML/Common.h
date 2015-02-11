#ifndef COMMON_H
#define COMMON_H

#include <SFML/Graphics.hpp>
#include <math.h>

const int PARTICLE_COUNT = 5;
const float PARTICLE_RADIUS = 30.0f;

// Container dimensions
const float CONTAINER_WIDTH = 1100.0f;
const float CONTAINER_HEIGHT = 550;

// Spatial partitioning
const float CELL_SIZE = PARTICLE_RADIUS + 30.0f;
const float CELL_COLS = std::trunc(CONTAINER_WIDTH / CELL_SIZE);
const float CELL_ROWS = std::trunc(CONTAINER_HEIGHT / CELL_SIZE);

// Fluid limits
const float WALL_LEFTLIMIT			= 100.0f;
const float WALL_RIGHTLIMIT			= WALL_LEFTLIMIT + CELL_COLS * CELL_SIZE;
const float WALL_TOPLIMIT			= 200.0f;
const float WALL_BOTTOMLIMIT		= WALL_TOPLIMIT + CELL_ROWS * CELL_SIZE;

// Particle limits
const float PARTICLE_LEFTLIMIT		= WALL_LEFTLIMIT + PARTICLE_RADIUS + 1.0f;
const float PARTICLE_RIGHTLIMIT		= WALL_RIGHTLIMIT - PARTICLE_RADIUS - 1.0f;
const float PARTICLE_TOPLIMIT		= WALL_TOPLIMIT + PARTICLE_RADIUS + 1.0f;
const float PARTICLE_BOTTOMLIMIT	= WALL_BOTTOMLIMIT - PARTICLE_RADIUS - 1.0f;

// Forces
const sf::Vector2f GRAVITATIONAL_ACCELERATION(0.0f, 9.81f);
const bool GRAVITY_ON = false;

// Physics constants
const float VELOCITY_DAMPING		= 0.99999f;

// Solver iterations
const int SOLVER_ITERATIONS = 3;

#endif // COMMON_H
