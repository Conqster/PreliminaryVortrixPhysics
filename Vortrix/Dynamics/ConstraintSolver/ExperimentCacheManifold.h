#pragma once
//
//#include "Maths/VortrixMaths.h"
//
//#include "Dynamics/Body/BodyID.h"
//
//
#include "Collision/ContactManifold.h"
#include "Dynamics/Body/Body.h"
//
//
//namespace vx {
//
//
//	
//
//	struct CacheContactPoint
//	{
//		Float3 localPoint0;
//		Float3 localPoint1;
//
//		float totalNormalLambda;
//		float totalTangentLambda[2];
//	};
//
//	class CacheManifold
//	{
//	public:
//		BodyID GetBodyA() const { return mBody0; }
//		BodyID GetBodyB() const { return mBody1; }
//
//		uint32 PointCount() const { return mContactPoints; }
//
//		Float3 Normal() const { return mNormal; }
//		Float3 RelativePosition() const { return mRelativePosition; }
//
//		/// caller should ensure that id0 is less id1 
//		struct Pair
//		{
//			Pair(uint32 id0, uint32 id1, uint32 _point_count) : 
//				body0_id(id0), body1_id(id1), point_count(_point_count){ }
//
//			Pair(BodyID body0, BodyID body1, uint32 _point_count) :
//				body0_id(body0.Value()), body1_id(body1.Value()), point_count(_point_count){ }
//
//			uint64 Hash()
//			{
//				(uint64(body0_id) << 32) | (uint64(body1_id) >> 24) | uint64(point_count);
//			}
//
//			uint32 body0_id;
//			uint32 body1_id;
//			uint32 point_count;
//		};
//
//		static Pair GetPairKey(BodyID body0, BodyID body1)
//		{
//			return Pair(body0, body1, 3);
//		}
//		//for now
//		std::array<CacheContactPoint, ContactManifold::kMaxPoints> mContactPoints{};
//	private:
//		//union 
//		//{
//		//	struct Pair 
//		//	{
//		//		uint32 body0_id;
//		//		uint32 body1_id;
//		//		uint32 point_count;
//		//	};
//		//	BodyID mBody0;
//		//	BodyID mBody1;
//		//	uint32 mContactPoints;
//		//};
//
//		BodyID mBody0;
//		BodyID mBody1;
//		uint32 mContactPoints;
//
//		Float3 mNormal;
//		Float3 mRelativePosition;
//
//		int mContacts = 0;
//
//
//		Pair GetPair() { return Pair(mBody0, mBody1, mContacts); }
//	};
//
//
//
//	void ValidateManifold(ContactManifold _manifold)
//	{
//		///mapped hash
//
//		CacheManifold cache;
//		Body* a = nullptr;
//		Body* b = nullptr;
//
//		Vec3 normal = Vec3::Up();
//
//		if (_manifold.PointCount() != cache.PointCount())
//			return;
//
//		//axis / normal threshold 
//		float n_treshold = kEpsilon;
//		if (_manifold.normal.Dot(Vec3::LoadFloat3Raw(cache.Normal())) < n_treshold)
//			return;
//
//
//		float eps_sq = kEpsilon * kEpsilon;
//		Quat qA = _manifold.a->Orientation();
//		Vec3 tA = _manifold.a->Position();
//		Quat qB = _manifold.b->Orientation();
//		Vec3 tB = _manifold.b->Position();
//
//
//		//relative postion 
//		Vec3 rel_pos = tB - tA;
//		if ((rel_pos - Vec3::LoadFloat3Raw(cache.RelativePosition()).LengthSq()) > eps_sq)
//			return;
//
//
//		for (int i = 0; i < _manifold.PointCount(); ++i)
//		{
//			Vec3 pAl = qA.InverseRotate(_manifold.Points()[i].pointA - tA);
//			Vec3 disp = pAl - Vec3::LoadFloat3Raw(cache.mContactPoints[i].localPoint0);
//			float dist_sq = disp.LengthSq();
//			if (VxAbs(dist_sq) > eps_sq)
//				return;
//
//			Vec3 pBl = qB.InverseRotate(_manifold.Points()[i].pointA - tB);
//			disp = pBl - Vec3::LoadFloat3Raw(cache.mContactPoints[i].localPoint1);
//			dist_sq = disp.LengthSq();
//			if (VxAbs(dist_sq) > eps_sq)
//				return;
//		}
//
//
//	}
//}//namespace vx



/// jolt checked 
/// 
/// local points if are clpose enought so get lamba
/// 
/// 




#include "Dynamics/Body/BodyID.h"

#include "Vortrix/Maths/VortrixMaths.h"
#include "Core/HashMap.h"


namespace vx{






	struct BodyPair
	{
		BodyID bodyA;
		BodyID bodyB;

		BodyPair() = default;
		BodyPair(BodyID _a, BodyID _b) : bodyA(_a), bodyB(_b){}


		static BodyPair Create(BodyID a, BodyID b)
		{
			if (a > b)
				std::swap(a, b);
			return { a,b };
		}

		void Sort() { if (bodyA > bodyB) std::swap(bodyA, bodyB); }

		uint64 Hash() const
		{
			return (uint64(bodyA.Value()) << 32) | uint64(bodyB.Value());
		}

		/// collision check 
		bool operator == (const BodyPair& rhs) const { return bodyA == rhs.bodyA && bodyB == rhs.bodyB; }
		bool operator < (const BodyPair& rhs) const { return Hash() < rhs.Hash(); }
	};



	struct CacheContactPoint
	{
		Float3 localPoint0;
		Float3 localPoint1;

		float totalNormalLambda;
		float totalTangentLambda[2];
	};

	class CacheManifold
	{
	public:
		BodyID GetBodyA() const { return mBody0; }
		BodyID GetBodyB() const { return mBody1; }
	
		uint32 PointCount() const { return mContacts; }
	
		Float3 Normal() const { return mNormal; }

		std::array<CacheContactPoint, 4> mContactPoints{};

	
		BodyID mBody0;
		BodyID mBody1;
		uint32 mContacts;
	
		Float3 mNormal;
	};


	template<typename Key_T>
	struct DefaultMapHasher
	{
		
		size_t operator()(const Key_T& k) const noexcept
		{
			return k.Hash();
		}
	};


	//using ManifoldMap = HashMap<BodyIDPairPoint, CachedManifold>;
	using ManifoldMap = HashMap<BodyPair, CacheManifold, DefaultMapHasher<BodyPair>>;
	using ManifoldMapEntry = ManifoldMap::Entry;



	/// for testing not acual constrain 
	struct ContactPointConstraint
	{
		///use pointer to point cached buffer
		CacheContactPoint* cacheLocalPoint = nullptr;

		struct ConstraintAxis //ConstraintRow
		{
			Float3 axis;
			float effMass = 0.0f; //inv_mass + point_inv_mass(due rotation)
			float totalLamda = 0.0f;//lagrange multiplier / total accumulated impluse along axis

			Float3 r0XAxis{ 0.0f }; //relative point0 Cross axis 
			//angular factor
			Float3 invIr0XAxis{ 0.0f }; //body0 inv world inertia multiply r0XAxis

			Float3 r1XAxis{ 0.0f }; //relative point1 Cross axis 
			//angular factor
			Float3 invIr1XAxis{ 0.0f }; //body1 inv world inertia multiply r1XAxis
			float bias;
		};

		ConstraintAxis normal{};
		ConstraintAxis lateralTangent[2]{};


		//void SetupAxesVelocityConstraints(const Body& body0, const Body& body1, const Vec3& world_pos0,
		//	const Vec3& world_pos1, const Vec3& normal, float e, float mu, const ContactConstraintAxesSetting& settings);

		//void GetTangentBasis(const Vec3& normal, Vec3& out_tangent1, Vec3& out_tangent2)
		//{
		//	out_tangent1 = normal.NormalisedPerpendicular();
		//	out_tangent2 = normal.Cross(out_tangent1);
		//}
	};
	/// for testing not acual constrain 
	struct ContactConstraint
	{
#if CONTACT_USE_SOLVERBODY
		SolverBodyIndex body0;
		SolverBodyIndex body1;
#else
		Body* body0 = nullptr;
		Body* body1 = nullptr;
#endif // CONTACT_USE_SOLVERBODY


		Float3 normal{ 0.0f };

		float friction = 0.0f;
		float restitution = 0.0f;


		std::array<ContactPointConstraint, ContactManifold::kMaxPoints> contactPoints{};
		int numConstraintPoints = 0;
	};


	void Test()
	{

		ManifoldMap mCache[2];
		uint32 cache_write;

		ContactManifold manifold{nullptr, nullptr};
		BodyPair key = BodyPair::Create(manifold.a->GetID(), manifold.b->GetID());




		//current manifold map 
		ManifoldMap& write_manifold_cache = mCache[cache_write];
		ManifoldMapEntry new_manifold_entry = write_manifold_cache.Create(key);
		//later support to have a static array cache space 
		if (!new_manifold_entry.Valid())
			return;

		CacheManifold* new_manifold = &new_manifold_entry.Value();

		uint32 num_contact_pt = manifold.PointCount();

		//set some values
		new_manifold->mBody0 = manifold.a->GetID();
		new_manifold->mBody1 = manifold.b->GetID();
		new_manifold->mContacts = num_contact_pt;

		/// since body 2 is less dominates to 1 either static if static is part of 
		/// participating body
		Vec3 norBl = manifold.b->GetOrientation().InverseRotate(manifold.normal);
		norBl.Store(new_manifold->mNormal);


		//read manifold map 
		ManifoldMap& read_manifold_cache = mCache[cache_write^1];
		ManifoldMapEntry old_manifold_entry = read_manifold_cache.Find(key);

		const CacheContactPoint* cache_pt_start;
		uint32 cache_pt_count;
		//persistent
		if (old_manifold_entry.Valid())
		{

			CacheManifold* old_manifold = &old_manifold_entry.Value();
			cache_pt_count = old_manifold->PointCount();
			cache_pt_start = old_manifold->mContactPoints.data();
		}
		else
		{
			//not persistent, new 
			cache_pt_start = nullptr;
			cache_pt_count = 0;
		}

		///Alocate new constraint, this is just sample 
		//ContactConstraint& constraint;// = mConstraints[idx];
		ContactConstraint constraint;// = mConstraints[idx];

		Quat qA = manifold.a->GetOrientation();
		Vec3 tA = manifold.a->GetPosition();
		Quat qB = manifold.b->GetOrientation();
		Vec3 tB = manifold.b->GetPosition();


		/// re addressing the crietia of contact manifold 
		/// using the manifold normal is good but restrictive
		/// if both local point are approx hence, it meant 
		/// computing the normal axes along both local point 
		/// int world space, meant that the axes would be approx equal 
		/// 
		/// returning in loop, means that this is a hard restriction and 
		/// extra work waste, and less predictable. and would state that 
		/// the cache point order need to meet the new manifold 
		/// but a rare scenario whrn the body rotate but a different point 
		/// align with the new this small not prefect point would still be fine as 
		/// we are saying that we applied lambda to the local point in last frame 
		/// and it align with new frame point constraint
		/// 
		/// and using nnumber of point is also restricting if for a box, 
		/// last frame 4 point are in contact a there is a small micro movement that 
		/// only 3 point are valid this is good to continue and warm start with lambda for the 3 points

		for (int i = 0; i < int(num_contact_pt); ++i)
		{
			const ManifoldPoint& mp = manifold.Points()[i];

			Vec3 p0_ls = qA.InverseRotate(mp.pointA - tA);
			Vec3 p1_ls = qB.InverseRotate(mp.pointB - tB);


			///constraint point constraitn part
			ContactPointConstraint& point_constraint = constraint.contactPoints[i];

			//check if close to any contact pt if any
			bool was_close = false;
			for (const CacheContactPoint* cache_pt = cache_pt_start;
				cache_pt < (cache_pt_start + cache_pt_count); cache_pt++)
			{
				if (p0_ls.IsApprox(Vec3::LoadFloat3Raw(cache_pt->localPoint0)) &&
					p1_ls.IsApprox(Vec3::LoadFloat3Raw(cache_pt->localPoint1)))
				{
					point_constraint.normal.totalLamda = cache_pt->totalNormalLambda;
					point_constraint.lateralTangent[0].totalLamda = cache_pt->totalTangentLambda[0];
					point_constraint.lateralTangent[1].totalLamda = cache_pt->totalTangentLambda[1];

					was_close = true;
					break;
				}
			}

			if (!was_close)
			{
				point_constraint.normal.totalLamda = 0.0f;
				point_constraint.lateralTangent[0].totalLamda = 0.0f;
				point_constraint.lateralTangent[1].totalLamda = 0.0f;
			}

			/// now only copy the local points 
			CacheContactPoint& cp = new_manifold->mContactPoints[i];

			p0_ls.Store(cp.localPoint0);
			p1_ls.Store(cp.localPoint1);

			//solving constraint point also points to the cache 
			point_constraint.cacheLocalPoint = &cp;


			//setup constact point constraint axes velocity propertices
		}
	}

	void WarmStart(ContactConstraint& contact_constraint)
	{
		float inv_mass0 = contact_constraint.body0->InverseMass();
		Vec3 lin_vel0 = contact_constraint.body0->GetLinearVelocity();
		Vec3 ang_vel0 = contact_constraint.body0->GetAngularVelocity();

		float inv_mass1 = contact_constraint.body1->InverseMass();
		Vec3 lin_vel1 = contact_constraint.body1->GetLinearVelocity();
		Vec3 ang_vel1 = contact_constraint.body1->GetAngularVelocity();
		for (int i = 0; i < contact_constraint.numConstraintPoints; ++i)
		{
			ContactPointConstraint& cpt_c = contact_constraint.contactPoints[i];

			///normal
			float impluse = cpt_c.normal.totalLamda;

			lin_vel0 -= impluse * inv_mass0 * Vec3::LoadFloat3Raw(cpt_c.normal.axis);
			ang_vel0 -= impluse * Vec3::LoadFloat3Raw(cpt_c.normal.invIr0XAxis);

			lin_vel1 += impluse * inv_mass1 * Vec3::LoadFloat3Raw(cpt_c.normal.axis);
			ang_vel1 += impluse * Vec3::LoadFloat3Raw(cpt_c.normal.invIr1XAxis);

			//tangents 
			for (int i = 0; i < 2; ++i)
			{
				impluse = cpt_c.lateralTangent[i].totalLamda;

				lin_vel0 -= impluse * inv_mass0 * Vec3::LoadFloat3Raw(cpt_c.lateralTangent[i].axis);
				ang_vel0 -= impluse * Vec3::LoadFloat3Raw(cpt_c.lateralTangent[i].invIr0XAxis);

				lin_vel1 += impluse * inv_mass1 * Vec3::LoadFloat3Raw(cpt_c.lateralTangent[i].axis);
				ang_vel1 += impluse * Vec3::LoadFloat3Raw(cpt_c.lateralTangent[i].invIr1XAxis);
			}



			///OR linear
			/// maybe dot product
			//Vec3 cummulated_axis = Vec3::LoadFloat3Raw(cpt_c.normal.axis) +
			//	Vec3::LoadFloat3Raw(cpt_c.lateralTangent[0].axis) +
			//	Vec3::LoadFloat3Raw(cpt_c.lateralTangent[1].axis);
			//cummulated_axis.Normalise();

			//Vec3 cummulates_axis_w = Vec3::LoadFloat3Raw(cpt_c.normal.invIr0XAxis) +
			//	Vec3::LoadFloat3Raw(cpt_c.lateralTangent[0].invIr0XAxis) +
			//	Vec3::LoadFloat3Raw(cpt_c.lateralTangent[1].invIr0XAxis);

			//float cummulated_lambda = cpt_c.normal.totalLamda +
			//	cpt_c.lateralTangent[0].totalLamda +
			//	cpt_c.lateralTangent[1].totalLamda;

			//Vec3 v = cummulated_lambda * inv_mass0 * cummulated_axis;
			//Vec3 w = cummulated_lambda * cummulates_axis_w;


			//lin_vel0 -= v;
			//ang_vel0 -= w;

			//lin_vel1 += v;
			//ang_vel1 += w;

		}

		contact_constraint.body0->SetLinearVelocity(lin_vel0);
		contact_constraint.body0->SetAngularVelocity(ang_vel0);

		contact_constraint.body1->SetLinearVelocity(lin_vel1);
		contact_constraint.body1->SetAngularVelocity(ang_vel1);
	}

	void WriteBackImplusesManifoldCache(ContactConstraint& contact_constraint)
	{

		for (int i = 0; i < contact_constraint.numConstraintPoints; ++i)
		{
			ContactPointConstraint& cpt_c = contact_constraint.contactPoints[i];
			CacheContactPoint* cache = cpt_c.cacheLocalPoint;
			if (cache)
			{
				cache->totalNormalLambda = cpt_c.normal.totalLamda;
				cache->totalTangentLambda[0] = cpt_c.lateralTangent[0].totalLamda;
				cache->totalTangentLambda[1] = cpt_c.lateralTangent[1].totalLamda;
			}
		}
	}



} //nmespace 
