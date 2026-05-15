#include <Vortrix/Vortrix.h>

#include "Core/Profiler.h"

#include "Particle.h"

namespace vx::Particles
{

	void Particle::SemiImplicitIntegrate(float dt)
	{
		//VPHX_PROFILE_FUNCTION();
		if (mInverseMass <= 0.0)
			return;

		//Semi implicit intergtor
		// vel += acc * time "m/s = m/s/s * s"
		// pos += vel * time "m = m/s * s"
		mAcceleration = mAccumlatedForce * mInverseMass;
		mVelocity += mAcceleration * dt;
		mPosition += mVelocity * dt;

		//Numerical damping 
		float damping = static_cast<float>(1.0) - mDamping;
		mVelocity *= VxPow(damping, dt);

		ClearAccumlator();
	}

	void Particle::EulerIntegrate(float dt)
	{
		if (mInverseMass <= 0.0)
			return;

		//Semi implicit intergtor
		// vel += acc * time "m/s = m/s/s * s"
		// pos += vel * time "m = m/s * s"
		mPosition += mVelocity * dt;
		mAcceleration = mAccumlatedForce * mInverseMass;
		mVelocity += mAcceleration * dt;

		//Numerical damping 
		float damping = static_cast<float>(1.0) - mDamping;
		mVelocity *= VxPow(damping, dt);

		ClearAccumlator();
	}

	void Particle::AddForce(const Vec3& force)
	{
		mAccumlatedForce += force;
	}

	void Particle::AddImpluse(const Vec3& impluse)
	{
		if (HasFiniteMass())
			mVelocity += impluse * mInverseMass;
	}

	void Particle::ApplyGravity(const Vec3& gravity)
	{
		if (HasFiniteMass())
			mAccumlatedForce += mMass * gravity * mGravityScale;
	}

	inline void Particle::SetGravityScale(float scale)
	{
		mGravityScale = VxClamp(scale, static_cast<float>(0.0), static_cast<float>(0.0));
	}

	void Particle::ClearAccumlator()
	{
		mAccumlatedForce = Vec3::Zero();
	}
} //VPHX namespace