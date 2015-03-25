#ifndef BASESIMULATION_H
#define BASESIMULATION_H

class BaseSimulation
{
public:
	BaseSimulation();
	virtual ~BaseSimulation();

	inline const int GetSimulationIndex() const { return m_iSimulationIndex; }

protected:
	unsigned int m_iSimulationIndex;

private:
	static unsigned int GlobalSimulationCounter;
};

#endif // BASESIMULATION_H