#include <Vortrix/Vortrix.h>
#include "ParticleWorld.h"

#include "Particle.h"

#include "Constraints/SpringConstraint.h"

#include <utility>
#include "Vortrix/Core/Profiler.h"

#include "Collision/Shapes/Shape.h"
#include "PhysicsWorld.h"

namespace vx::Particles
{

	ParticleWorld::ParticleWorld(uint32_t max_particle, const Vec3& gravity,
		bool b_enable_gravity) :
		mParticles({}),
		mContraintSolvers(nullptr),
		bApplyGravity(b_enable_gravity),
		mConstraintSolverCount(0),
		mContactSolverManager({}),
		mGroundContact({}),
		mParticle2ParticleContact({}),
		mParticleCount(0),
		mMaxParticle(kParticleLimit),
		mGravity(gravity)
	{
		Initialise(max_particle, gravity, b_enable_gravity);
	}


	ParticleWorld::~ParticleWorld()
	{
		//deallocate memory
		delete mContraintSolvers;

		mParticles.clear();
	}

	void ParticleWorld::Initialise(uint32_t max_particle,
		const Vec3& gravity,
		bool b_enable_gravity)
	{
		max_particle = std::min(max_particle, kParticleLimit);
		mParticles.reserve(static_cast<size_t>(max_particle));
		mParticleCount = 0;


		mGravity = gravity;

		//collision 
		Plane plane = Plane::CreateFromPointAndNormal(Vec3::Zero(), Vec3(0.707f, 0.707f, 0.0f));
		mGroundContact.Initialise(&mParticles/*, plane*/);
		mParticle2ParticleContact.Initialise(&mParticles);

		mContraintSolvers = new ConstraintSolverPool();

		bApplyGravity = b_enable_gravity;

		VX_LOG_DEBUG(mGravity.Length());
	}

	void ParticleWorld::CreateSimpleSampleWorld(ParticleWorld* world)
	{
		if (world == nullptr)
			return;

		float y = 5.0f;

		Particle* anchor = world->CreateParticle(Vec3(-2.5f, y, 0.0f), 0.0f);//anchor
		auto p0 = world->CreateParticle(Vec3(-1.0f, y, 0.0f), 3.0f);

		SpringConstraint* p2p_spring_solver = world->CreateSpringConstraint(anchor, p0);

		auto prev_particle = p0;
		Particle* test_particle = nullptr;
		for (int i = 0; i < 4; i++)
		{
			auto new_particle = world->CreateParticle(Vec3(-3.0f - i, y, Random::Float(-5.0f, 5.0f)), 3.0f + static_cast<float>(i));

			world->CreateSpringConstraint(prev_particle, new_particle);
			prev_particle = new_particle;

			if (i == 1)
				test_particle = new_particle;
		}

		auto anchor_1 = world->CreateParticle(Vec3(2.0f, 5.0f, 0.0f), 0.0f);
		world->CreateSpringConstraint(prev_particle, anchor_1);

		if (test_particle)
		{
			auto p3 = world->CreateParticle(Vec3(2.0f, 5.0f, 0.0f), 5.0f);
			world->CreateSpringConstraint(test_particle, p3);
		}


		//real_ft stiffness = 100.0f;
		//real_ft damping = 0.5;///0.9f;
		//std::vector<Particle*> vertices;

		//real half_size = 0.5f;

		//const Vec3 world_offset = Vec3::Right() * 5.0f + Vec3::Up() * 2.0f;

		//for(int x = -1; x <= 1; x+=2)
		//	for(int y = -1; y <= 1; y+=2)
		//		for (int z = -1; z <= 1; z += 2)
		//		{
		//			Vec3 new_vec = Vec3(
		//				x * half_size,
		//				y * half_size,
		//				z * half_size
		//			);

		//			new_vec += world_offset;
		//			vertices.emplace_back(world->CreateParticle(new_vec, 1.0f));
		//		}


		//std::vector<std::pair<int, int>> edges;
		//const size_t count = vertices.size();
		//for (size_t i = 0; i < count; ++i)
		//	for (size_t j = i+1; j < count; ++j)
		//		edges.emplace_back(i, j);


		//for (const auto& p : edges)
		//{
		//	//static_assert((p.first < 8) && p.second < 8);

		//	Particle* a = vertices[p.first];
		//	Particle* b = vertices[p.second];

		//	world->CreateSpringConstraint(a, b, 1.0f, stiffness, damping);

		//}


		//world->CreateSpringConstraint(vertices[0], vertices[1], 1.0f, 400, 10.3f);
		//world->CreateSpringConstraint(vertices[1], vertices[2], 1.0f, 400, 10.3f);
		//world->CreateSpringConstraint(vertices[2], vertices[3], 1.0f, 400, 10.3f);
		//world->CreateSpringConstraint(vertices[3], vertices[4], 1.0f, 400, 10.3f);
		//world->CreateSpringConstraint(vertices[4], vertices[5], 1.0f, 400, 10.3f);
		//world->CreateSpringConstraint(vertices[5], vertices[2], 1.0f, 400, 10.3f);
		//															   
		//world->CreateSpringConstraint(vertices[5], vertices[6], 1.0f, 400, 10.3f);
		//world->CreateSpringConstraint(vertices[6], vertices[7], 1.0f, 400, 10.3f);
		//world->CreateSpringConstraint(vertices[7], vertices[4], 1.0f, 400, 10.3f);
		//world->CreateSpringConstraint(vertices[7], vertices[0], 1.0f, 400, 10.3f);
		//world->CreateSpringConstraint(vertices[6], vertices[1], 1.0f, 400, 10.3f);
	}




	void ParticleWorld::StepSimulation(float time_step)
	{
		VX_PROFILE_FUNCTION();

		//Collision detection & resolution 
		std::vector<Collision::CollisionContactInfo> world_contact_infos;
		//reserve a size dynamically based on the number of particles
		unsigned int contact_counts = mParticles.size() * 2;
		world_contact_infos.reserve(contact_counts);
		unsigned int detection_count = mGroundContact.Intersections(world_contact_infos, contact_counts);
		detection_count = mParticle2ParticleContact.Intersections(world_contact_infos, contact_counts);
		//Resolve detections
		mContactSolverManager.ResolveContacts(world_contact_infos, time_step);


		mContraintSolvers->UpdateSolvers(time_step);


		{
			//VPHX_PROFILE_SCOPE("ParticleIntergateStep", nullptr, true);
			for (auto& p : mParticles)
			{

				if (bApplyGravity)
					p.ApplyGravity(mGravity);

				switch (mParticleIntegationType)
				{
				case vx::EIntegationType::Euler: p.EulerIntegrate(time_step);
					break;
				case vx::EIntegationType::SemiImplicit: p.SemiImplicitIntegrate(time_step); 
					break;
				default: VX_ASSERT_WARN(false, "Unknown Intergration type!!!.");
					break;
				}
			}
		}
	}

	void ParticleWorld::OnDebug(DebugGizmosRenderer* debug_renderer)
	{
		mContraintSolvers->OnDebugGizmos(debug_renderer);
	}

	inline const bool ParticleWorld::ParticleLimitReached() const
	{
 		return (mMaxParticle >= kParticleLimit) && (mParticles.size() >= mMaxParticle);
	}

	void ParticleWorld::AddParticle(const Particle& p)
	{
		if (ParticleLimitReached())
			return;

		mParticles.push_back(p);

	}

	Particle* ParticleWorld::CreateParticle(const Vec3& pos, const float mass, const float gravity_scale, const float damping)
	{
		if (ParticleLimitReached())
			return nullptr;

		mParticles.push_back(Particle(pos, mass, gravity_scale, damping));
		return &mParticles.back();
	}

	SpringConstraint* ParticleWorld::CreateSpringConstraint(Particle* p0, Particle* p1, float rest_length, float stiffness, float critical_damp_ratio)
	{
		if (mConstraintSolverCount >= kConstraintSolverLimit)
			return nullptr;

		mConstraintSolverBuffer[mConstraintSolverCount] = SpringConstraint(p0, p1, rest_length, stiffness, critical_damp_ratio);
		auto solver = &mConstraintSolverBuffer[mConstraintSolverCount];
		mConstraintSolverCount++;

		mContraintSolvers->Add(solver);
		return solver;
	}


	Plane ParticleWorld::GetGroundCollisionPlane()
	{
		return mGroundContact.GetCollsionPlane();
	}

	void ParticleWorld::SetGroundCollisionPlane(const Vec3& nor, float origin_offset)
	{
		mGroundContact.SetCollsionPlane(nor, origin_offset);
	}
}// namespace VPHX
