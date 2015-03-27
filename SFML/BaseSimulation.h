#ifndef BASESIMULATION_H
#define BASESIMULATION_H

#include "Common.h"

class BaseSimulation
{
public:
	BaseSimulation();
	virtual ~BaseSimulation();

	inline const int GetSimulationIndex() const { return m_iSimulationIndex; }

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