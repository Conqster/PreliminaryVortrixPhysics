#pragma once
#include <Vortrix/Vortrix.h>
#include "ConstraintInterface.h"



//TODO(Conqster): Remove This gravity needs to implicit not explicit, and each particle can define scale
namespace vx::Particles
{
	class Particle;
	class ParticleGravitySolver : public IConstraintInterface
	{
	private:
		using ParticlePool = std::vector<Particle>;
	public:
		ParticleGravitySolver() = default;
		ParticleGravitySolver(ParticlePool* particles, const Vec3& gravity = Vec3(0.0f, -9.8f, 0.0f)) :
			mParticles(particles), mGravity(gravity) {
		}
		virtual void UpdateSolver(float time_step) override;
		virtual void DebugGizmos(class DebugGizmosRenderer* debug_renderer = nullptr);

		void SetParticle(ParticlePool* p_particle_pool);
		void SetGravity(const Vec3& gravity);

	private:
		Vec3 mGravity = Vec3(0.0f, -9.8f, 0.0f);					// acceleration due gravity [ms^-2]
		ParticlePool* mParticles = nullptr;
	};
} //VPHX namespace