#include "BaseSimulation.h"

unsigned int BaseSimulation::GlobalSimulationCounter = 0;

BaseSimulation::BaseSimulation()
{
	m_iSimulationIndex = GlobalSimulationCounter++;
}


BaseSimulation::~BaseSimulation()
{
}
