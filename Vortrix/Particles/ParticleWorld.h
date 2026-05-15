#pragma once
#include <vector>
#include "Constraints/ConstraintInterface.h"
#include "Constraints/ParticleGravitySolver.h"


#include "Collision/CollisionSolver.h"

#include <array>
#include "Constraints/SpringConstraint.h"

//
// VortrixPhysics/
// -Vortrix/
//		-Core/
//			Core.h
//			.....
//		-Particle/
//			-Collision/	
//			-Constraints/
//			-ParticleWorld.h
//			-ParticleWorld.cpp
//			-Particle.cpp
//			-Particle.h
//		-Maths/
//		-Collision/	
//		-Constraints/
//		-Body/
//		-Renderer/
//		-Geometry/
//		-PhysicsWorld.h
//		-PhysicsWorld.cpp
//		-Vortrix.h
// -SampleFramework/
//		-Application/
//		-Window/
//		-Input/
//		-Renderer/
//			-Shader.cpp
//			-Shader.h
//			-Renderer.h
//			-Renderer.cpp
//			-TExtture.cpp
//			-TExtture.h
//			-Camera.h
//		-SampleFramework.h
//

namespace vx
{
	enum class EIntegationType : uint8_t
	{
		Euler,
		SemiImplicit,
	};

	class Body;

	namespace Particles
	{
		constexpr uint32_t kParticleLimit = 1024;
		constexpr uint32_t kConstraintSolverLimit = 50;

		class Particle;
		class SpringConstraint;


		class VX_API ParticleWorld
		{
		public:
			ParticleWorld(uint32_t max_particle = kParticleLimit,
				const Vec3& gravity = Vec3(0.0f, -9.85f, 0.0f),
				bool b_enable_gravity = true);
			~ParticleWorld();

	
			// Lifecycle

			static void CreateSimpleSampleWorld(ParticleWorld* world);

			void StepSimulation(float time_step);
			void OnDebug(class DebugGizmosRenderer* debug_renderer);

			void AddParticle(const Particle& p);
			Particle* CreateParticle(const Vec3& pos, const float mass = 1.0f,
				const float gravity_scale = 1.0f, const float damping = /*0.95f*/0.55f);


			SpringConstraint* CreateSpringConstraint(Particle* p0,
				Particle* p1, float rest_length = 0.55f,
				float stiffness = 850, float critical_damp_ratio = 0.7f);
			//float stiffness = 200, float critical_damp_ratio = 1.55);


			std::vector<Particle>& GetParticles() { return mParticles; }
			ConstraintSolverPool& GetConstraintSolver() { return *mContraintSolvers; }

			int GetParticleCount() { return int(mParticles.size()); }

			Vec3 GetWorldGravity() const { return mGravity; }
			void SetWorldGravity(const Vec3& axis) { mGravity = axis; }

			Plane GetGroundCollisionPlane();
			void SetGroundCollisionPlane(const Vec3& nor, float origin_offset);

			EIntegationType GetIntegrationType() const { return mParticleIntegationType; }
			void SetIntegrationType(EIntegationType type) { mParticleIntegationType = type; }
		private:
			void Initialise(uint32_t max_particle = kParticleLimit,
				const Vec3& gravity = Vec3(0.0f, -9.85f, 0.0f),
				bool b_enable_gravity = true);
			std::vector<Particle> mParticles;

			ConstraintSolverPool* mContraintSolvers = nullptr;
			bool bApplyGravity = true;

			uint32_t mConstraintSolverCount = 0;
			std::array<SpringConstraint, kConstraintSolverLimit> mConstraintSolverBuffer{};



			//Collision
			Collision::ContactSolverManager mContactSolverManager;
			Collision::GroundContactDetection mGroundContact;
			Collision::Particle2PartilceCollisionDetection mParticle2ParticleContact;

		private:
			uint32_t mParticleCount = 0;
			uint32_t mMaxParticle = kParticleLimit;

			Vec3 mGravity = Vec3(0.0f, 1.0f, 0.0f);

			EIntegationType mParticleIntegationType = EIntegationType::SemiImplicit;

			const bool ParticleLimitReached() const;
		};
	}// namespace Particles
	
} // namespace VPHX