#ifndef SIMULATIONMANAGER_H
#define SIMULATIONMANAGER_H

#include "Common.h"
#include "BaseSimulation.h"

class FluidSimulation;
class SoftBody;

class SimulationManager
{
public:

	static SimulationManager& GetInstance()
	{
		static SimulationManager instance;
		return instance;
	}

	inline void AddSimulation(BaseSimulation* pSimulation)
	{
		// Based on the simulation type add it to the corresponding vector
		if (pSimulation->SimType == SimulationType::FluidSimulation)
		{
			m_FluidSimulationList.push_back((FluidSimulation*)pSimulation);
		}
		if (pSimulation->SimType == SimulationType::SoftBodySimulation)
		{
			m_SoftBodyList.push_back((SoftBody*)pSimulation);
		}

		// Add the simulation to the global list of simulations
		m_SimulationList.push_back(pSimulation);
	}

	inline std::vector<BaseSimulation*>&	GetSimulationList()			{ return m_SimulationList; }
	inline std::vector<FluidSimulation*>&	GetFluidSimulationList()	{ return m_FluidSimulationList; }
	inline std::vector<SoftBody*>&			GetSoftBodySimulationList() { return m_SoftBodyList; }

private:
	// -----------------------------------------------------------------------------
	// Hide constructor for singleton implementation
	SimulationManager() {};

	// Delete unneeded copy constructor and assignment operator
	SimulationManager(SimulationManager const&) = delete;
	void operator=(SimulationManager const&) = delete;
	// -----------------------------------------------------------------------------

	std::vector<BaseSimulation*>	m_SimulationList;
	std::vector<FluidSimulation*>	m_FluidSimulationList;
	std::vector<SoftBody*>			m_SoftBodyList;
};

#endif // SIMULATIONMANAGER_H