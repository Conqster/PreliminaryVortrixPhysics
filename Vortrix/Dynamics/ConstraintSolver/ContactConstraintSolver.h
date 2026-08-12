#pragma once

#include "Vortrix/PhysicsWorldSettings.h"

#include "Vortrix/Collision/ContactManifold.h"
#include "Vortrix/Core/Profiler.h"

#include <array>
#include <algorithm>

#include "CombineFrictionRestitution.h"

#define CONTACT_USE_SOLVERBODY 1

#if CONTACT_USE_SOLVERBODY
#include "Vortrix/Dynamics/SolverBodyIndex.h"
#include "Vortrix/Dynamics/Body/BodyManager.h"
#endif // CONTACT_USE_SOLVERBODY


#include "Vortrix/Core/HashMap.h"

#include "Vortrix/Dynamics/Body/BodyID.h"





namespace vx {


	/////Some research 
	/// PGS (Projected Gauss Seidel) 
	/// TGS (Temporal Gauss Seidel) 
	/// 
	/// PGS: when maximum performance is required of for simpler
	///		 simulations where high accuracy of TGS is not necessary 
	/// TGS: When accuracy and stabliity are paramount, such as in complex
	///		machinery, ragdoll, or robotics simulations. 


	/// Jolts process simplified
	/// 
	/// going with the knowlegde, Contact Manifold are points in world space (i.e relative to manifold offset)
	/// NB:		- if persistent constraint are not reused directly, 
	///			a new one is created but total lamba per axis are filled.
	/// 
	/// create a key with {body id, sub shape id}
	/// determine the number of contact points 
	/// 
	/// transform world space normal -> body 2 local space which is usually static 
	/// combine coeffients (mu, e) friction and restitution
	/// 
	/// get contact points in last frame cache entry; stored as "CacheContactPoint"
	///	start = value
	/// which includes the count/num of points then 
	/// end = start + points count
	/// 
	/// if found call listener ContactPersisted, else ContactAdded
	/// also flag contact as presisted is found preventing ContactRemove to remove this contact
	/// 
	/// Create a new constraint see @func0
	/// 
	/// Notify island builder  see @func1
	/// 
	/// Compute scaled mass and inertia
	///  store constraint inv mass1
	///  store constraint inv inertia for rotation
	/// 
	/// Compute tangents 1 & 2 to world normal
	/// 
	/// loop through num of contact points
	///		convert manifolds offset -> world space 
	///		convert world space -> bodies local space
	///		
	///		loop through cache contact
	///			if local space points close to last frame set running lambda's then break
	///		
	///		create new point for constraint as cached contafct point (local space)
	/// 
	/// @func0:
	///		add body0 & 1
	///		add sort 
	///		add world normal
	///		add combined friction 
	///		add inv inertia scale 1
	///		add inv inertia scale 2
	///		add num of contact points 
	///		return constraint
	/// end func0; 
	/// 
	/// @func1:
	///		use bodies index in active bodies internal to store min as contact links
	///		return
	/// end func1
	/// 
	/// @func2: line 40;https://github.com/jrouwe/JoltPhysics/blob/master/Jolt/Physics/Constraints/ContactConstraintManager.cpp
	///		collision pt p -> average world position to manifold world position1 & 2
	///		compute p relative to bodies r1 & r2
	///		compute point relative velcoity COM
	///		
	///		normal axis project rel velocity (Dot(rel vel, world space nor))
	///		penetration Dot(ws1 - ws2, ws_nor) {>0 penetrating, <0 seperatuing}
	/// 
	/// 
	///		...
	///		mNormalAxisConstriant -> @func3: (calculate contraint properties) 
	///		
	///		if combined friction mu > 0.0	
	///			surface_vel
	///			tangent axis project sur_vel  (Dot(world space tangent1, surface_vel)) /////(Dot(surface_vel, world space tangent))
	///			tangent axis project sur_vel  (Dot(world space tangent2, surface_vel)) /////(Dot(surface_vel, world space tangent))
	///			
	///			mTangentAxisConstriant[0] -> @func3: (calculate contraint properties) 
	///			mTangentAxisConstriant[1] -> @func3: (calculate contraint properties) 
	/// end func2
	/// 
	/// 
	/// 
	/// @func3: (calculate contraint properties); line 119; https://github.com/jrouwe/JoltPhysics/blob/master/Jolt/Physics/Constraints/ConstraintPart/ContactConstraintPart.h#L119
	///			compute inv effective mass K = J M^-1 J^T
	///			
	///			inv_eff_mass;
	///			
	///			if body1 not static
	///				r1_X_axis = Cross(r1, axis) <--- stored 
	///				if body1 dyn
	///					I_r1_X_axis = inv_I1.Multiply3x3(r1_X_axis) <-- store
	///					inv_eff_mass = inv_mass1 + Dot(I_r1_X_axis, r1_X_axis)
	///				else inv_eff_mass = 0;
	///			else inv_eff_mass = 0;
	/// 
	///			if body2 not static
	///				r2_X_axis = Cross(r2, axis) <--- stored 
	///				if body2 dyn
	///					I_r2_X_axis = inv_I2.Multiply3x3(r2_X_axis) <-- store
	///					inv_eff_mass += inv_mass12 + Dot(I_r2_X_axis, r2_X_axis)
	/// 
	///			if inv_eff_mass is 0
	///				disactivate
	///			else
	///				mEffectiveMass = 1.0f/inv_eff_mass
	/// end func3
	/// 

	class DebugGizmosRenderer;
	struct ContactManifold;
	

	class ContactConstraintSolver
	{
	public:
		ContactConstraintSolver() = default;
		~ContactConstraintSolver()
		{
			delete[] mConstraints;
			delete[] mStepWriteManifoldCache;
			delete[] mCacheContactPoint;
		}

		void Init(uint32 max_constraints);

		/// per frame transient allocation
		VX_INLINE void PreFrameSetup(const PhysicsWorldSettings& phys_setting)
		{
			VX_PROFILE_FUNCTION();
			//might zero out constraint buffer 
			if (mNumConstraints > 0)
				std::memset(mConstraints, 0, mNumConstraints * sizeof(ContactConstraint));
			mNumConstraints = 0;

			if (mWriteManifoldCacheIdx > 0)
				std::memset(mStepWriteManifoldCache, 0, mWriteManifoldCacheIdx * sizeof(CachedManifold));
			mWriteManifoldCacheIdx = 0;

			mStats.StepReset();
		}

		void SetupContactConstraint(const ContactManifold& manifold, const struct CollisionContext& ctx);

		uint32 MaxConstraints() const { return mMaxConstraints; }
		
		void SetupContactConstraint2(const ContactManifold& manifold, const struct CollisionContext& ctx);
		/// attempting to write a thread safe version for multi threading
		void SetupContactConstraint2Mt(const ContactManifold& manifold, const struct CollisionContext& ctx);
		void WarmStart();


#if CONTACT_USE_SOLVERBODY

#else
		void SolveVelocityConstraint(const struct SolverSettings& settings)
		{
			VX_PROFILE_FUNCTION();
			SolverContactManifold(settings);
		}

		void SolvePositionConstraint(const SolverSettings& settings)
		{
			VX_PROFILE_FUNCTION();
			for (int i = 0; i < settings.positionIterations; ++i)
			{

				for (uint32 contact_idx = 0; contact_idx < mNumConstraints; ++contact_idx)
				{
					PositionalCorrection(mConstraints[contact_idx],
						settings.baumgarte, settings.positionCorrectionSlop,
						settings.positionCorrectionGlobalLimits[0], settings.positionCorrectionGlobalLimits[1],
						settings.positionCorrectionBodyLimitScale);
				}
			}
		}
#endif // CONTACT_USE_SOLVERBODY

		void SetFrictionCombineMode(ECombineMode mode) { mCombinedFrictionMode = mode; }
		void SetRestitutionCombineMode(ECombineMode mode) { mCombinedRestitutionMode = mode; }

		ECombineMode GetFrictionCombineMode() const { return mCombinedFrictionMode; }
		ECombineMode GetRestitutionCombineMode() const { return mCombinedRestitutionMode;}


		void SetPhysicsContext(PhysicsStepContext* ctx) { mPhysicsContext = ctx; }
		struct ContactConstraintSolverStat
		{
			int numContactConstraints = 0;
			int numPersistentContact = 0;

			float totalKineticWork = 0.0f; //joules
			float totalStepKineticWork = 0.0f; //joules
			float maxAttainedKineticWork = 0.0f; //joules (kgm^2/s^2

			//end < start
			float totalStepWorkLoss = 0.0f;
			float totalStepWorkGain = 0.0f;

			float totalNorLambda = 0.0f;
			float totalTanLambda = 0.0f;
			float totalBiTanLambda = 0.0f;


			int actualPointCounts = 0;
			int actualPersistentPointCounts = 0;

			VX_INLINE void StepReset()
			{
				numContactConstraints = 0;
				numPersistentContact = 0;

				totalStepKineticWork = 0.0f;
				totalStepWorkLoss = 0.0f;
				totalStepWorkGain = 0.0f;

				totalNorLambda = 0.0f;
				totalTanLambda = 0.0f;
				totalBiTanLambda = 0.0f;

				actualPointCounts = 0;
				actualPersistentPointCounts = 0;
			}
		};
		const ContactConstraintSolverStat& GetStats() const { return mStats; }

		void DebugDraw(DebugGizmosRenderer* debug_renderer, const class ConstraintSolver* constraint_solver, const BodyManager* body_manager, const DrawSettings& settings) const;
	private:
		PhysicsStepContext* mPhysicsContext = nullptr;
		ContactConstraintSolverStat mStats;
		const static int kMaxPoints = 4; //<-- waste of memory objects like 
		/// sphere, vertex - face contacts only uses one point 
		/// meaning extra work to fetch point and waste of sizeof(Point) * 3/4
		/// using a dynamic array (std:::vector) also meant:
		/// 
		/// allocating and deallocating mem multiple times per frame
		/// if reserve mem, another extra waste of 3/4 of size of Point
		/// 
		/// All this creates TLB misses L3 cache thrashing
		/// 
		/// jolt type optimisation uint8* arena and 
		/// constraints uses offset to next constaint / location in buffer
		/// 
		/// for instance, 
		/// complex shape / Box plane constact 4 points (std::array) 
		/// for sphere/box loading 4x more data into CPU cache than necessary

		int mMaxConstraints = 128;
		/// Not efficient 
		static constexpr uint kConstraintLimit = 16384;


		struct ContactConstraintAxesSetting
		{
			DebugGizmosRenderer* debug_renderer = nullptr;
			float restitutionThreshold = 0.50f;
			float positionCorrectionSlop = 0.02f;
			float baumgarte = 0.2f;// 0.45f; //0.1-0.8
			float timeStep = 1.0f / 60.0f;
			float maxSpeed = 2.0f;//2m/s
			bool debugDrawAxes = false;
		}settings;


		/// at the start of add constraint buffer, was checked to see 
		/// if contact is persistence 
		/// then have the since points are store sequenctally 
		/// use hash to get value 
		/// start = value
		/// which includes the count/num of points then 
		/// end = start + points count
		/// 
		/// jolt uses local space as the the cache validation check 
		/// each point is checked not just the contact manifold and there lamda is retrived
		struct CacheContactPoint
		{
			Float3 localPoint0;
			Float3 localPoint1;

			float totalNormalLambda;
			float totalTangentLambda[2];
		};

		/// SolverContactPoint
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

			void SetupAxesVelocityConstraints(const Body& body0, const Body& body1, const Vec3& world_pos0, 
				const Vec3& world_pos1, const Vec3& normal, float e, float mu, const ContactConstraintAxesSetting& settings);

			//void GetTangentBasis(const Vec3& normal, Vec3& out_tangent1, Vec3& out_tangent2)
			//{
			//	out_tangent1 = normal.NormalisedPerpendicular();
			//	out_tangent2 = normal.Cross(out_tangent1);
			//}
		};

		/// what is the best caching method
		///
		/// what is required 
		public:
		class ContactConstraint
		{
		public:
#if CONTACT_USE_SOLVERBODY
			void SetBodies(SolverBodyIndex body0, SolverBodyIndex body1)
#else
			void SetBodies(Body* body0, Body* body1)
#endif // CONTACT_USE_SOLVERBODY
			{
				mBody0 = body0;
				mBody1 = body1;
			}

#if CONTACT_USE_SOLVERBODY
			SolverBodyIndex BodyA() const { return mBody0; }
			SolverBodyIndex BodyB() const { return mBody1; }
#else
			Body* BodyA() const { return mBody0; }
			Body* BodyB() const { return mBody1; }
#endif // CONTACT_USE_SOLVERBODY

			Vec3 Normal() const { return Vec3::LoadFloat3Raw(mNormal); }

			float FrictionCoeff() const { return mFriction; }
			float RestitutionCoeff() const { return mRestitution; }

			uint32 NumConstraintPoints() const { return numConstraintPoints; }

			ContactPointConstraint& PointConstraint(uint32 i)
			{
				return contactPoints[i];
			}

			ContactPointConstraint* PointConstraintPtr()
			{
				return contactPoints.data();
			}

			const ContactPointConstraint* PointConstraintPtr() const
			{
				return contactPoints.data();
			}
			
			const ContactPointConstraint& PointConstraint(uint32 i) const
			{
				return contactPoints[i];
			}

			void Normal(const Vec3& nor) { nor.Store(mNormal); }
			void FrictionCoeff(float coeff) { mFriction = coeff; }
			void RestitutionCoeff(float coeff) { mRestitution = coeff; }

			ContactPointConstraint* CreatePointConstraint()
			{
				VX_ASSERT_WARN_RETURN(numConstraintPoints < ContactManifold::kMaxPoints, nullptr, "Max Contact Constraint Point excessed!!");
				return &contactPoints[numConstraintPoints++];
			}

		private:
#if CONTACT_USE_SOLVERBODY
			SolverBodyIndex mBody0;
			SolverBodyIndex mBody1;
#else
			Body* body0 = nullptr;
			Body* body1 = nullptr;
#endif // CONTACT_USE_SOLVERBODY

			/// world space normal
			Float3 mNormal{0.0f};

			float mFriction = 0.0f;
			float mRestitution = 0.0f;

			std::array<ContactPointConstraint, ContactManifold::kMaxPoints> contactPoints{};
			uint32 numConstraintPoints = 0;

		};
		static_assert(std::is_trivially_copyable_v<ContactConstraint>, "must be copyable using memset");

		ContactConstraint* GetContactConstraint(uint32 idx) const
		{
			VX_ASSERT(idx < mNumConstraints);
			return &mConstraints[idx];
		}
		private:

		/// sphere, vertex - face contacts only uses one point 
		/// meaning extra work to fetch point and waste of sizeof(Point) * 3/4
		/// using a dynamic array (std:::vector) also meant:
		/// 
		/// allocating and deallocating mem multiple times per frame
		/// if reserve mem, another extra waste of 3/4 of size of Point
		/// 
		/// All this creates TLB misses L3 cache thrashing
		/// 
		/// jolt type optimisation uint8* arena and 
		/// constraints uses offset to next constaint / location in buffer
		/// 
		/// for instance, 
		/// complex shape / Box plane constact 4 points (std::array) 
		/// for sphere/box loading 4x more data into CPU cache than necessary
		//std::array<ContactConstraint, kMaxConstraints> mConstraints;
		ContactConstraint* mConstraints = nullptr;
		//std::array<ContactConstraint, kMaxConstraints> mCacheConstraints;
		//uint32 mNumConstraints = 0; ///current frame constraint to solve 
		std::atomic<uint32> mNumConstraints;

		std::atomic<uint32> mCacheContactPointHead = 0;
		uint32 mMaxCacheContactPoints = 1024;
		/// at start tail is equal to the total count available 
		/// or maybe half of total to make as its double linear buffer
		/// 
		/// after first step head = last + 1 from last step (i.e current location) 
		/// tail end or start of last step (i.e last step/read manifold cache)
		std::atomic<uint32> mCacheContactPointTail = mMaxCacheContactPoints;
		CacheContactPoint* mCacheContactPoint = nullptr;


		ECombineMode mCombinedFrictionMode = ECombineMode::SquareRoot;
		ECombineMode mCombinedRestitutionMode = ECombineMode::Maximum;


#pragma region NEW CACHING


		struct BodyPair
		{
			BodyID bodyA;
			BodyID bodyB;

			BodyPair() = default;
			BodyPair(BodyID _a, BodyID _b) : bodyA(_a), bodyB(_b) {}


			static BodyPair Create(BodyID a, BodyID b)
			{
				if (a > b)
					std::swap(a, b);
				return { a,b };
			}

			void Sort() { if (bodyA > bodyB) std::swap(bodyA, bodyB); }

			uint64 Hash() const
			{
				return (uint64(bodyA.ID()) << 32) | uint64(bodyB.ID());
			}

			/// collision check 
			bool operator == (const BodyPair& rhs) const { return bodyA == rhs.bodyA && bodyB == rhs.bodyB; }
			bool operator < (const BodyPair& rhs) const { return Hash() < rhs.Hash(); }
		};

		class CachedManifold
		{
		public:
			CachedManifold() = default;
			CachedManifold(BodyID id0, BodyID id1, uint32 num_contact) : 
				mBody0(id0), mBody1(id1), mNumContacts(num_contact){ }


			BodyPair CreatePairKey() const { return BodyPair::Create(mBody0, mBody1); }

			BodyID GetBodyA() const { return mBody0; }
			BodyID GetBodyB() const { return mBody1; }

			uint32 NumPoints() const { return mNumContacts; }

			Float3 Normal() const { return mNormal; }

			CacheContactPoint* ContactPointPtr() 
			{ 
#if TEST_CONTACT_CONSTRAINT_MT
				return mContactPoints;
#else
				return mContactPoints.data(); 
#endif // TEST_CONTACT_CONSTRAINT_MT

			}

			void SetContactPointPtr(CacheContactPoint* head)
			{
#if TEST_CONTACT_CONSTRAINT_MT
				mContactPoints = head;
#endif // TEST_CONTACT_CONSTRAINT_MT
			}

			Float3 mNormal;

			bool mPersistent = false;
		private:
			BodyID mBody0;
			BodyID mBody1;
			uint32 mNumContacts = 0;

#if TEST_CONTACT_CONSTRAINT_MT
			CacheContactPoint* mContactPoints = nullptr;
#else
			//point to first cache point, then end = mContactPoints + mNumContact
			std::array<CacheContactPoint, 4> mContactPoints{};
#endif // TEST_CONTACT_CONSTRAINT_MT

		};

		using ManifoldMap = HashMap<BodyPair, CachedManifold>;
		using ManifoldMapEntry = ManifoldMap::Entry;
		ManifoldMap mManifoldCache[2];
		uint32 mManifoldWriteCache = 0;

		/// index head for writing into write manifold cache
		/// before tranferring into hash map 
		std::atomic<uint32> mWriteManifoldCacheIdx = 0;
		CachedManifold* mStepWriteManifoldCache = nullptr;

		CachedManifold* CreateNewManifold(const BodyPair key, BodyID a_id, BodyID b_id, uint32 num_contact_pts);

	public:

		static void WriteBackImpluseCache(ContactConstraint& contact_constraint)
		{
			for (ContactPointConstraint* cpt_c = contact_constraint.PointConstraintPtr(),
				*cpt_c_end = contact_constraint.PointConstraintPtr() + contact_constraint.NumConstraintPoints();
				cpt_c < cpt_c_end; ++cpt_c)
			{
				CacheContactPoint* cache = cpt_c->cacheLocalPoint;
				if (cache)
				{
					cache->totalNormalLambda = cpt_c->normal.totalLamda;
					cache->totalTangentLambda[0] = cpt_c->lateralTangent[0].totalLamda;
					cache->totalTangentLambda[1] = cpt_c->lateralTangent[1].totalLamda;
				}
			}
		}


		static void WriteBackImplusesManifoldCache(ContactConstraint* contact_constraints, size_t count)
		{
			for (ContactConstraint* cc = contact_constraints, *cc_end = contact_constraints + count; cc < cc_end; ++cc)
			{
				WriteBackImpluseCache(*cc);
			}
		}


		ContactConstraint* ContactConstraintsPtr() { return mConstraints; }

		uint32 NumContactConstraints() const { return mNumConstraints; }


		void FinaliseWriteManifoldCache()
		{
			for (CachedManifold* manifold_cache = mStepWriteManifoldCache;
				manifold_cache < (mStepWriteManifoldCache + mWriteManifoldCacheIdx); manifold_cache++)
			{
				BodyPair key = manifold_cache->CreatePairKey();
				mManifoldCache[mManifoldWriteCache].Emplace(key, std::move(*manifold_cache));
			}
		}
		//bool ValidateManifoldContactTransfer(CachedManifold& manifold, const BodyManager& body_manager)
		//{
		//	auto& body0 = body_manager.GetBody(manifold.GetBodyA());
		//	auto& body1= body_manager.GetBody(manifold.GetBodyB());

		//	//if bodies are dyn and awake next frame should generate manifold 

		//	if (body0.IsDynamic() && body0.IsAwake() ||
		//		body1.IsDynamic() && body1.IsAwake())
		//		return false;

		//	//only linear displacement
		//	Vec3 dispW = body1.Position() - body0.Position();
		//
		//	float tolerance = VxSqr(0.02); //2 cm
		//	return VxAbs(dispW.LengthSq()) < VxAbs(manifold.mRelativeDistanceSq) + tolerance;
		//}
		void FinaliseStepManifoldCache(const BodyManager& body_manager)
		{
			mManifoldWriteCache ^= 1;

			///old read/new write
			ManifoldMap& read_manifold_cache = mManifoldCache[mManifoldWriteCache];
			ManifoldMap& _manifold_cache = mManifoldCache[mManifoldWriteCache^1];

			mStats.numPersistentContact = 0;
			for (auto& e : read_manifold_cache)
			{
				CachedManifold& manifold = e.second;
				mStats.numPersistentContact++;
				if (manifold.mPersistent)
				{
					//if(ValidateManifoldContactTransfer(manifold, body_manager))
					//{
					//	auto& v = _manifold_cache.Create(e.first, manifold).Value();
					//	v.mPersistent = true;
					//}

					for (CacheContactPoint* ccp = manifold.ContactPointPtr(),
						*ccp_end = manifold.ContactPointPtr() + manifold.NumPoints();
						ccp < ccp_end; ++ccp)
					{
						mStats.totalNorLambda = ccp->totalNormalLambda;
						mStats.totalTanLambda = ccp->totalTangentLambda[0];
						mStats.totalBiTanLambda = ccp->totalTangentLambda[1];
					}
				}
			}


			read_manifold_cache.Clear();
		}

#pragma endregion

		
	
	private:
#pragma region Helper functions
		Mat44 BuildTangentBasisMatrix(const Vec3& normal)
		{
			const Vec3 n = normal;
			Vec3 t1, t2;

			//vector basis
			GetTangentBasis(n, nullptr, t1, t2);

			Mat44 M(1.0f);
			//M.SetIdentity();

			M.SetColumn3(0, n);
			M.SetColumn3(1, t1);
			M.SetColumn3(2, t2);

			return M;
		}

		static void GetTangentBasis(const Vec3& normal, const Vec3* rel_vel, Vec3& out_tangent1, Vec3& out_tangent2)
		{
			out_tangent1 = normal.NormalisedPerpendicular();
			if(rel_vel)
			{
				Vec3 tang_vel = *rel_vel -  normal * rel_vel->Dot(normal);
				if (tang_vel.LengthSq() > kEpsilon)
					out_tangent1 = tang_vel.Normalised();
			}

			out_tangent2 = normal.Cross(out_tangent1);
		}
#pragma endregion

		
#if CONTACT_USE_SOLVERBODY

		static void SolveVelocityConstraint(ContactConstraint& constraint, SolverBody& sbA, SolverBody& sbB);
	public:

		static void WarmStart(const ContactConstraint& contact_constraint, struct SolverBody& body0, SolverBody& body1);
		static void WarmStart(ContactConstraint* contact_constraints, size_t count, SolverBody* bodies);


		void SolveVelocityConstraint(SolverBody* bodies);
		void SolveVelocityConstraint(const uint32* constraint_start_idx, uint32 count, SolverBody* bodies);
		void SolvePositionCorrections(SolverBody* bodies, BodyManager& body_manager, float baumgarte, float slop, float min_limit, float max_limit, float limit_scale);
#else
		void SolverContactManifold(const SolverSettings& phy_settings);
		void PositionalCorrection(ContactConstraint& constraint, float baumgarte, float slop, float min_limit, float max_limit, float limit_scale);
#endif // CONTACT_USE_SOLVERBODY
	};
	
}