#pragma once
#include "ConstraintInterface.h"

namespace vx::Particles
{
	class Particle;
	class SpringConstraint : public IConstraintInterface
	{
		Particle* mParticleA = nullptr;
		Particle* mParticleB = nullptr;

		float mRestLength;                         //[meters]
		float mStiffness;							//[constant]
		//[0.0 = none, 1.0 = critical damping]
		float mDampingRatio;							//[constant]    <-- prevent jitterness

	public:
		SpringConstraint() = default;
		SpringConstraint(const SpringConstraint&) = default;
		//if damping not set / equal/less -1 forces system to compute optimal best for stability value
		SpringConstraint(Particle* particle_a, Particle* particle_b, float rest_length, float stiffness, float critical_damp_ratio = 0.5);

		virtual void UpdateSolver(float time_step) override;
		virtual void DebugGizmos(DebugGizmosRenderer* debug_renderer = nullptr);

		inline void SetParticles(Particle* particle_a, Particle* particle_b) {
			mParticleA = particle_a;
			mParticleB = particle_b;
		}
		inline void SetParticleA(Particle* particle) {
			mParticleA = particle;
		}
		inline void SetParticleB(Particle* particle) {
			mParticleB = particle;
		}
	};

}




