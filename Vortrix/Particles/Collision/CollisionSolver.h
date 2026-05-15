#pragma once

#include <Vortrix/Vortrix.h>
#include <vector>

#include "Vortrix/Geometry/Plane.h"

//detection collision 
//pair bodies
//get list of body pairs

namespace vx
{
	namespace Particles
	{
		class Particle;
		namespace Collision
		{
			using ParticlePool = std::vector<Particle>;
			/// <summary>
			/// Collision pair 
			/// </summary>
			struct CollisionContactInfo
			{
				Particle* a = nullptr;
				Particle* b = nullptr; //could be null if colliding with walls 

				Vec3 normal;

				//contact depth
				float peneration;
			};



			class ContactSolverManager
			{
			public:
				void ResolveContacts(std::vector<CollisionContactInfo>& world_contact_infos, float time_step);

			private:
				void ResolveContact(CollisionContactInfo& contact_info, float time_step);
			};


			class DetectionSolver
			{
			public:
				virtual unsigned int Intersections(std::vector<CollisionContactInfo>& world_contact_infos, float contact_limits) = 0;
			};




			class GroundContactDetection : public DetectionSolver
			{
			private:
				ParticlePool* mParticlePool = nullptr;

				//distance before making assumption that partcle is close enough to detect
				//collsion 
				float mTolerance = 0.1f;

				Plane mGroundPlane{ Vec3::Up() };
			public:
				void Initialise(ParticlePool* particle_pool_pointer, Vec3 plane_nor = Vec3::Up(), float plane_offset = 0.0f, float tolerance = 0.1f)
				{
					Initialise(particle_pool_pointer, Plane(plane_nor, plane_offset), tolerance);
				}

				inline void Initialise(ParticlePool* particle_pool_pointer, const Plane& plane, float tolerance = 0.1f)
				{
					mParticlePool = particle_pool_pointer;
					mTolerance = tolerance;

					mGroundPlane = plane;
				}

				virtual unsigned int Intersections(std::vector<CollisionContactInfo>& world_contact_infos, float contact_limits);

				const Plane GetCollsionPlane() const;
				void SetCollsionPlane(const Vec3& nor, float orgin_offset);
				void SetCollsionNormal(const Vec3& nor);
				void SetCollsionOriginOffset(float orgin_offset);
			};


			class Particle2PartilceCollisionDetection : public DetectionSolver
			{
			private:
				ParticlePool* mParticlePool = nullptr;

				//distance before making assumption that partcle is close enough to detect
				//collsion 
				float mTolerance = 0.1f;
			public:
				void Initialise(ParticlePool* particle_pool_pointer, float tolerance = 0.1f)
				{
					mParticlePool = particle_pool_pointer;
					mTolerance = tolerance;
				}

				virtual unsigned int Intersections(std::vector<CollisionContactInfo>& world_contact_infos, float contact_limits);
			};

		}

	} // namespace ParticleCollision
}// namespace VPHX