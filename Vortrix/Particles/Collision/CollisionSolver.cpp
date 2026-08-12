#include "CollisionSolver.h"
#include "Vortrix/Particles/Particle.h"


#include <tuple>

namespace vx
{
	namespace Particles::Collision
	{
		void ContactSolverManager::ResolveContacts(std::vector<CollisionContactInfo>& world_contact_infos, float time_step)
		{
			//solve all contact detected
			for (auto& contact : world_contact_infos)
				ResolveContact(contact, time_step);
		}

		void ContactSolverManager::ResolveContact(CollisionContactInfo& contact_info, float time_step)
		{
			if (!contact_info.a)
				return;

			auto& bodyA = contact_info.a;
			auto& bodyB = contact_info.b;

			const float invA = bodyA->GetInverseMass();
			const float invB = ((bodyB) ? bodyB->GetInverseMass() : 0.0);


			const float total_inv_mass = invA + invB;

			if (total_inv_mass <= kEpsilon)
				return;

			if (contact_info.peneration < 1e-5f)
				return;

			const Vec3 nor = contact_info.normal;
			const float penetration = contact_info.peneration;

			const Vec3 correction = nor * (penetration / total_inv_mass);
			bodyA->mPosition += correction * invA;
			if (bodyB)
				bodyB->mPosition -= correction * invB;


			const Vec3 velA = bodyA->GetVelocity();
			const Vec3 velB = ((bodyB) ? bodyB->GetVelocity() : Vec3(0.0f));
			Vec3 rel_vel = velA - velB;
			const float rel_vel_along_nor = rel_vel.Dot(nor);

			//If they're seperating (positive relative vel along norm using this convention
			//then dont apply closing impluse
			//NB: if your normal convention is reversed, invert this test
			if (rel_vel_along_nor > 0.0f)
				return;


			float restitution = static_cast<float>(0.66f);
			const float restitution_vel_threhold = static_cast<float>(0.5f);
			if (VxAbs(rel_vel_along_nor) < restitution_vel_threhold)
				restitution = 0.0f;


			//j = (-(1 + e) * relative velocity along normal) / (inverse mass A + B)
			float j = (-(1.0f + restitution) * rel_vel_along_nor) / (total_inv_mass);

			if (!std::isfinite(j))
				j = 0.0f;

			const Vec3 impluse = nor * j;

			contact_info.a->AddImpluse(impluse);
			if (contact_info.b)
				contact_info.b->AddImpluse(-impluse);
		}





		unsigned int GroundContactDetection::Intersections(std::vector<CollisionContactInfo>& world_contact_infos, float contact_limits)
		{
			if (!mParticlePool)
				return 0;

			unsigned int detection_count = 0;

			for (auto& p : *mParticlePool)
			{

				//assumed that the ground y is at 0 
				//and tolerance is the closeness to 0
				//if (p.Position().Y() <= mTolerance)
				const float dist = mGroundPlane.SignedDistance(p.GetPosition());
				const float penetration = p.GetRadius() - dist;

				if (penetration > 0.0f)
				{
					CollisionContactInfo contact_info;
					contact_info.a = &p;
					contact_info.b = nullptr;  //against ground or wall

					contact_info.normal = mGroundPlane.GetNormal();
					contact_info.peneration = penetration;

					//test 
					//Particle* pt = new Particle(sample_pos, 0.0f);
					//contact_info.b = pt;
					world_contact_infos.push_back(contact_info);

					//early out if contact_limit as been exhausted 
					if (++detection_count >= contact_limits)
						return detection_count;
				}

			}
			return detection_count;
		}

		const Plane GroundContactDetection::GetCollsionPlane() const
		{
			return mGroundPlane;
		}

		void GroundContactDetection::SetCollsionPlane(const Vec3& nor, float origin_offset)
		{
			mGroundPlane = Plane(nor, origin_offset);
		}

		inline void GroundContactDetection::SetCollsionNormal(const Vec3& nor)
		{
			mGroundPlane.SetNormal(nor);
		}

		inline void GroundContactDetection::SetCollsionOriginOffset(float orgin_offset)
		{
			mGroundPlane.SetConstant(orgin_offset);
		}


		unsigned int Particle2PartilceCollisionDetection::Intersections(std::vector<CollisionContactInfo>& world_contact_infos, float contact_limits)
		{

			unsigned int detection_count = 0;

			for (int i = 0; i < mParticlePool->size(); ++i)
			{

				const Particle& bodyA = (*mParticlePool)[i];
				for (int j = i + 1; j < mParticlePool->size(); ++j)
				{
					if (i == j)
					{
						printf("Error \n");
						continue;
					}

					const Particle& bodyB = (*mParticlePool)[j];

					const Vec3 delta = bodyA.mPosition - bodyB.mPosition;
					const float dist_sq = delta.LengthSq();
					const float combined_radius = bodyA.GetRadius() + bodyB.GetRadius();

					if (dist_sq < combined_radius * combined_radius)
					{
						const float dist = VxSqrt(std::max(dist_sq, kEpsilon));

						const Vec3 nor = (dist > kEpsilon) ? (delta / dist) : Vec3::Up();

						const float penetration = combined_radius - dist;

						CollisionContactInfo ci;
						ci.a = const_cast<Particle*>(&bodyA);
						ci.b = const_cast<Particle*>(&bodyB);
						ci.normal = nor;
						ci.peneration = penetration;
						world_contact_infos.push_back(ci);

						++detection_count;

						//early out if contact_limit as been exhausted 
						if (detection_count >= contact_limits)
							return detection_count;
					}



				}

			}
			return detection_count;
		}
	} // namespace ParticleCollision
} //namespcae VPHX