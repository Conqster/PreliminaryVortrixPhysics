#include "ParticleGravitySolver.h"
#include "Vortrix/Particles/Particle.h"
#include "Vortrix/Visuals/Renderers.h"


namespace vx::Particles
{
	void ParticleGravitySolver::UpdateSolver(float time_step)
	{
		for (auto& p : *mParticles)
		{
			if (&p && p.HasFiniteMass())
			{
				//f(N->kgms^-2) = ma = mg 
				p.AddForce(p.GetMass() * mGravity);
			}
		}
	}


	void ParticleGravitySolver::DebugGizmos(DebugGizmosRenderer* debug_renderer)
	{
		if (debug_renderer)
		{
			//gravity acceleration direction rather its magnitude
			for (auto& p : *mParticles)
			{
#define DEBUG_STATIC 0
#if !DEBUG_STATIC
				if (&p && !p.HasFiniteMass())
					continue;
#endif // DEBUG_STATIC

				debug_renderer->DrawLine(p.GetPosition(), p.GetPosition() +
#define DEBUG_ONLY_DIRECTION 1
#if DEBUG_ONLY_DIRECTION
					mGravity.Normalised(), Colour(Vec3::Right()));
#else
					mGravity, Colour(Vec3::Right()));
#endif // DEBUG_ONLY_DIRECTION
			}
		}
	}

	void ParticleGravitySolver::SetParticle(ParticlePool* p_particle_pool)
	{
		mParticles = p_particle_pool;
	}

	void ParticleGravitySolver::SetGravity(const Vec3& gravity)
	{
		mGravity = gravity;
	}

}