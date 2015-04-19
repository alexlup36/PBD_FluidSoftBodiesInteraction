#ifndef BASESIMULATION_H
#define BASESIMULATION_H

#include "Common.h"

enum class SimulationType
{
	FluidSimulation,
	SoftBodySimulation,

	Invalid
};

class BaseSimulation
{
public:
	BaseSimulation();
	virtual ~BaseSimulation();

	inline const int GetSimulationIndex() const { return m_iSimulationIndex; }

	virtual void InputUpdate(float delta, int navigation);

	// ------------------------------------------------------------------------
	// Public members
	// ------------------------------------------------------------------------

	SimulationType SimType;

protected:
	unsigned int m_iSimulationIndex;

private:
	static unsigned int GlobalSimulationCounter;
};

#endif // BASESIMULATION_H