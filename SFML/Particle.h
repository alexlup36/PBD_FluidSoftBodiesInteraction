#ifndef PARTICLE_H
#define PARTICLE_H

#include "Common.h"

class Particle
{
public:
	Particle(const glm::vec2& position, float radius);
	~Particle();

	void Draw(sf::RenderWindow& window);
	void UpdateShapePosition();

	// Getters and setters
	inline void SetPosition(const glm::vec2& newPosition) { m_Position = newPosition; }
	inline void AddDeltaPosition(const glm::vec2& deltaPosition) { m_Position += deltaPosition; }
	inline const glm::vec2 GetPosition() const { return m_Position; }

	inline void SetPredictedPosition(const glm::vec2& newPredPos) { m_PredictedPosition = newPredPos; }
	inline void AddDeltaPredPosition(const glm::vec2& deltaPredPosition) { m_PredictedPosition += deltaPredPosition; }
	inline const glm::vec2 GetPredictedPosition() const { return m_PredictedPosition; }

	inline const glm::vec2 GetLocalPosition() const { return m_LocalPosition; }
	inline void SetLocalPosition(const glm::vec2& newLocalPosition) { m_LocalPosition = newLocalPosition; }

	inline const glm::vec2 GetPositionCorrection() const { return m_PositionCorrection; }
	inline void SetPositionCorrection(const glm::vec2& positionOffset) { m_PositionCorrection = positionOffset; }

	inline void SetVelocity(const glm::vec2& newVelocity) { m_Velocity = newVelocity; }
	inline void AddDeltaVelocity(const glm::vec2& deltaVelocity) { m_Velocity += deltaVelocity; }
	inline glm::vec2 GetVelocity() const { return m_Velocity; }

	inline void SetForce(const glm::vec2& newForce) { m_Force = newForce; }
	inline glm::vec2 GetForce() const { return m_Force; }

	inline float GetRadius() const { return m_fRadius; }

	inline float GetSPHDensity() const { return m_fSPHDensity; }
	inline void SetSPHDensity(float newDensity) { m_fSPHDensity = newDensity; }

	inline float GetLambda() const { return m_fLambda; }
	inline void SetLambda(float newLambda) { m_fLambda = newLambda; }

	inline float GetDensityConstraint() const { return m_fDensityConstraint; }
	inline void SetDensityConstraint(float newDensityConstraint) { m_fDensityConstraint = newDensityConstraint; }

	inline bool IsAtLimit()
	{
		m_bLeft = m_Position.x <= PARTICLE_LEFTLIMIT;
		m_bRight = m_Position.x >= PARTICLE_RIGHTLIMIT;
		m_bTop = m_Position.y >= PARTICLE_BOTTOMLIMIT;
		m_bBottom = m_Position.y <= PARTICLE_TOPLIMIT;

		return m_bLeft || m_bRight || m_bTop || m_bBottom;
	}

	inline bool IsCollision(const Particle& other)
	{
		float radiusSum = other.GetRadius() + m_fRadius;
		float fDx = m_Position.x - other.GetPosition().x;
		float fDy = m_Position.y - other.GetPosition().y;

		return radiusSum * radiusSum > (fDx * fDx) + (fDy * fDy);
	}

	inline void SetDefaultColor() { m_Shape.setFillColor(sf::Color::Blue); }
	inline void SetIsCollidingColor() { m_Shape.setFillColor(sf::Color::Red); }
	inline void SetAsNeighborColor() { m_Shape.setFillColor(sf::Color::Magenta); }

	inline int GetParticleIndex() const { return m_iParticleIndex; }

	inline float GetParticleMass() const { return m_fMass; }

private:
	// Limits
	bool m_bLeft;
	bool m_bRight;
	bool m_bTop;
	bool m_bBottom;

	static int m_iGlobalIndex;
	int m_iParticleIndex;

	sf::CircleShape m_Shape;
	float m_fRadius;

	glm::vec2 m_LocalPosition;
	glm::vec2 m_Position;
	glm::vec2 m_PredictedPosition;
	glm::vec2 m_PositionCorrection;
	glm::vec2 m_Velocity;
	glm::vec2 m_Force;
	float m_fSPHDensity;
	float m_fDensityConstraint;
	float m_fMass;
	float m_fInvMass;

	// Position based fluid
	float m_fLambda;
};

#endif // PARTICLE_H