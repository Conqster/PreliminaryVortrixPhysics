#pragma once

#include "Vortrix/PhysicsWorldSettings.h"

#include "Collision/ContactManifold.h"
#include "Core/Profiler.h"

#include <array>
#include <algorithm>

#include "CombineFrictionRestitution.h"

#define CONTACT_USE_SOLVERBODY 0

#if CONTACT_USE_SOLVERBODY
#include "Dynamics/SolverBodyIndex.h"
#include "Dynamics/Body/BodyManager.h"
#endif // CONTACT_USE_SOLVERBODY




class DebugGizmosRenderer;
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



	struct ContactManifold;
	

	class ContactConstraintSolver
	{
	public:
		ContactConstraintSolver() = default;
		~ContactConstraintSolver()
		{
			delete[] mConstraints;
			delete[] mCachePoints;
		}

		void Init(uint32 max_constraints);

		/// per frame transient allocation
		VX_INLINE void PreFrameSetup(const PhysicsWorldSettings& phys_setting)
		{
			//might zero out constraint buffer 
			if (mNumConstraints > 0)
				std::memset(mConstraints, 0, mNumConstraints * sizeof(ContactConstraint));
			mNumConstraints = 0;

			if (mNumCachePoints > 0)
				std::memset(mCachePoints, 0, mNumCachePoints * sizeof(CacheContactConstraint));
			mNumCachePoints = 0;


			mStats.StepReset();
		}

		void SetupContactConstraint(const ContactManifold& manifold, const struct CollisionContext& ctx);
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

			VX_INLINE void StepReset()
			{
				numContactConstraints = 0;
				numPersistentContact = 0;

				totalStepKineticWork = 0.0f;
				totalStepWorkLoss = 0.0f;
				totalStepWorkGain = 0.0f;
			}
		};
		const ContactConstraintSolverStat& GetStats() const { return mStats; }

		void DebugDraw(DebugGizmosRenderer* debug_renderer, const DrawSettings& settings) const;
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


		//struct ContactPairID
		//{

		//private:
		//	BodyID mBody0;
		//	BodyID mBody1;
		//};

		class CacheContactConstraint
		{
		public:
			std::array<CacheContactPoint, 4> contactPoints;
			

			VX_INLINE int GetHash() const
			{
				//VX_ASSERT_WARN_RETURN(mBody0ID.IsValid() && mBody1ID.IsValid(),
				//	0, "Invalid Contact cache body invalid id");
			
				//Hash32Bit(5);
				//return Hash32Bit(((int)mBody0ID.Value() | ((int)mBody1ID.Value() << 16)));
			}

		private:

			VX_INLINE int Hash32Bit(int key)
			{
				// Thomas Wang's hash
				key += ~(key << 15);
				key ^= (key >> 10);
				key += (key << 3);
				key ^= (key >> 6);
				key += ~(key << 11);
				key ^= (key >> 16);
				return key;
			}

			//BodyID mBody0ID{};
			//BodyID mBody1ID{};
		};
		static_assert(std::is_trivially_copyable_v<CacheContactConstraint>, "must be copyable using memset");

		//std::array<CacheContactConstraint, kMaxConstraints> mCachePoints;
		CacheContactConstraint* mCachePoints = nullptr;
		int mNumCachePoints = 0;



		/// SolverContactPoint
		struct ContactConstraintPoint 
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

		struct ContactConstraint
		{
#if CONTACT_USE_SOLVERBODY
			SolverBodyIndex body0;
			SolverBodyIndex body1;
#else
			Body* body0 = nullptr;
			Body* body1 = nullptr;
#endif // CONTACT_USE_SOLVERBODY


			Float3 normal{0.0f};

			float friction = 0.0f;
			float restitution = 0.0f;

			//cache data
			float invMass0 = 0.0f;
			float invMass1 = 0.0f;

			std::array<ContactConstraintPoint, ContactManifold::kMaxPoints> contactPoints{};
			int numContacts = 0;
		};
		static_assert(std::is_trivially_copyable_v<ContactConstraint>, "must be copyable using memset");

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
		uint32 mNumConstraints = 0; ///current frame constraint to solve 

		ECombineMode mCombinedFrictionMode = ECombineMode::SquareRoot;
		ECombineMode mCombinedRestitutionMode = ECombineMode::Maximum;



		/// experimental cache 
		struct CacheContraint
		{

		};



		
	
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
	public:
		void SolveVelocityConstraint(struct SolverBody* bodies);
		void SolvePositionCorrections(SolverBody* bodies, BodyManager& body_manager, float baumgarte, float slop, float min_limit, float max_limit, float limit_scale);
#else
		void SolverContactManifold(const SolverSettings& phy_settings);
		void PositionalCorrection(ContactConstraint& constraint, float baumgarte, float slop, float min_limit, float max_limit, float limit_scale);
#endif // CONTACT_USE_SOLVERBODY
	};
	
}