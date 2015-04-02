#ifndef PARTICLEMANAGER_H
#define PARTICLEMANAGER_H

#include <vector>

#include "Common.h"
#include "BaseParticle.h"

class DeformableParticle;
class FluidParticle;

class ParticleManager
{
public:
	static ParticleManager& GetInstance()
	{
		static ParticleManager instance;
		return instance;
	}

	~ParticleManager()
	{
		for (unsigned int i = 0; i < m_ParticleList.size(); i++)
		{
			delete m_ParticleList[i];
		}
	};

	inline const void AddGlobalParticle(BaseParticle* pParticle) 
	{ 
		// Add the particle to the list of base particles
		m_ParticleList.push_back(pParticle); 

		// Add the particle to the corresponding list based on type
		if (pParticle->ParticleType == ParticleType::DeformableParticle)
		{
			m_DeformableParticleList.push_back((DeformableParticle*)pParticle);
		}
		if (pParticle->ParticleType == ParticleType::FluidParticle)
		{
			m_FluidParticleList.push_back((FluidParticle*)pParticle);
		}
	}
	inline std::vector<BaseParticle*> GetParticles() { return m_ParticleList; }
	inline unsigned int GlobalParticleListSize() { return m_ParticleList.size(); }	
	inline BaseParticle* GetParticle(int iIndex) { return m_ParticleList[iIndex]; }

	inline std::vector<FluidParticle*> GetFluidParticles() { return m_FluidParticleList; }
	inline FluidParticle* GetFluidParticle(int iIndex) { return m_FluidParticleList[iIndex]; }

	inline std::vector<DeformableParticle*> GetDeformableParticles() { return m_DeformableParticleList; }
	inline DeformableParticle* GetDeformableParticle(int iIndex) { return m_DeformableParticleList[iIndex]; }

private:
	// --------------------------------------------------------------------------------

	// Hide constructor for singleton implementation
	ParticleManager()
	{
	};

	// Delete unneeded copy constructor and assignment operator
	ParticleManager(ParticleManager const&) = delete;
	void operator=(ParticleManager const&) = delete;

	// --------------------------------------------------------------------------------

	std::vector<BaseParticle*> m_ParticleList;
	std::vector<DeformableParticle*> m_DeformableParticleList;
	std::vector<FluidParticle*> m_FluidParticleList;
};

#endif // PARTICLEMANAGER_H