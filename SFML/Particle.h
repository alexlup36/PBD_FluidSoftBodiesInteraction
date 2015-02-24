#ifndef PARTICLE_H
#define PARTICLE_H

#include "Common.h"

class Particle
{
public:
	Particle(const sf::Vector2f& position, float radius);
	~Particle();

	void Draw(sf::RenderWindow& window);
	void UpdateShapePosition();

	// Getters and setters
	inline void SetPosition(const sf::Vector2f& newPosition) { m_Position = newPosition; }
	inline void AddDeltaPosition(const sf::Vector2f& deltaPosition) { m_Position += deltaPosition; }
	inline const sf::Vector2f GetPosition() const { return m_Position; }

	inline void SetPredictedPosition(const sf::Vector2f& newPredPos) { m_PredictedPosition = newPredPos; }
	inline void AddDeltaPredPosition(const sf::Vector2f& deltaPredPosition) { m_PredictedPosition += (deltaPredPosition); }
	inline const sf::Vector2f GetPredictedPosition() const { return m_PredictedPosition; }

	inline const sf::Vector2f GetLocalPosition() const { return m_LocalPosition; }
	inline void SetLocalPosition(const sf::Vector2f& newLocalPosition) { m_LocalPosition = newLocalPosition; }

	inline const sf::Vector2f GetPositionCorrection() const { return m_PositionCorrection; }
	inline void AddPositionCorrection(const sf::Vector2f& positionOffset) { m_PositionCorrection += positionOffset; }
	inline void ResetPositionCorrection() { m_PositionCorrection.x = 0.0f; m_PositionCorrection.y = 0.0f; }

	inline void SetVelocity(const sf::Vector2f& newVelocity) { m_Velocity = newVelocity; }
	inline void AddDeltaVelocity(const sf::Vector2f& deltaVelocity) { m_Velocity += deltaVelocity; }
	inline sf::Vector2f GetVelocity() const { return m_Velocity; }

	inline void SetForce(const sf::Vector2f& newForce) { m_Force = newForce; }
	inline sf::Vector2f GetForce() const { return m_Force; }

	inline float GetRadius() const { return m_fRadius; }

	inline float GetRestDensity() const { return m_fRestDensity; }
	inline float GetSPHDensity() const { return m_fSPHDensity; }
	inline void SetSPHDensity(float newDensity) { m_fSPHDensity = newDensity; }

	inline float GetLambda() const { return m_fLambda; }
	inline void SetLambda(float newLambda) { m_fLambda = newLambda; }

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

	inline void SetDefaultColor() { m_Shape.setFillColor(sf::Color::Green); }
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

	sf::Vector2f m_LocalPosition;
	sf::Vector2f m_Position;
	sf::Vector2f m_PredictedPosition;
	sf::Vector2f m_PositionCorrection;
	sf::Vector2f m_Velocity;
	sf::Vector2f m_Force;
	float m_fRestDensity;
	float m_fSPHDensity;
	float m_fMass;
	float m_fInvMass;

	// Position based fluid
	float m_fLambda;
};

#endif // PARTICLE_H