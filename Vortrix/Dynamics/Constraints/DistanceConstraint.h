#pragma once
#include "Vortrix/Dynamics/Body/Body.h"
#include "Vortrix/Visuals/Renderers.h"

#include "Vortrix/Dynamics/ConstraintSolverRow.h"

#include "Constraint.h"

#include "SpringSettings.h"

namespace vx {


	struct DistanceConstraintSettings
	{
		Vec3 localAnchorA = Vec3(0.0f);
		Vec3 localAnchorB = Vec3(0.0f);

		float minDist = 1.0f;
		float maxDist = 1.0f;

		float frequency = 0.0f;
		float dampingRatio = 0.0f;
	};

	class DistanceConstraint : public Constraint
	{
	public:
		EConstraintType Type() const override { return EConstraintType::Distance; }
		const char* TypeName() const override { return "Distance Constraint"; }

		DistanceConstraint() : Constraint(nullptr, nullptr), 
			mLocalAnchorA(Vec3(0.0f)), mLocalAnchorB(Vec3(0.0f)),
			mMinDistance(0.0f), mMaxDistance(0.0f)
		{
			mFlags = EConstraintFlags::SolveVelocity;// | ~EConstraintFlags::SolvePosition;
		}

		DistanceConstraint(Body* bodyA, Body* bodyB, const DistanceConstraintSettings& settings);

		void SetLocalAnchorA(const Vec3& position) { mLocalAnchorA = position; }
		void SetLocalAnchorB(const Vec3& position) { mLocalAnchorB = position; }

		Vec3 GetLocalAnchorA() const { return mLocalAnchorA; }
		Vec3 GetLocalAnchorB() const { return mLocalAnchorB; }

		float GetAccumulatedLambda() const { return mAccumulatedLambda; }


		void SetDistance(float min_dist, float max_dist)
		{
			VX_ASSERT_WARN(min_dist <= max_dist);
			mMinDistance = min_dist;
			mMaxDistance = max_dist;
		}

		float GetMinDistance() const { return mMinDistance; }
		float GetMaxDistance() const { return mMaxDistance; }


		void SetSpringFrequency(float freq)
		{
			mSpring.mFrequency = freq;
			if (freq > 0.0f)
				mFlags &= ~EConstraintFlags::SolvePosition;
			else
				mFlags |= EConstraintFlags::SolvePosition;
		}

		void SetSpringDampingRatio(float ratio)
		{
			/// iff ratio is not zero, 
			/// freq needs to be zero
			/// 
			/// 
			VX_ASSERT_WARN((mSpring.mFrequency != 0.0f) || (ratio == 0.0f), 
				"Damping ratio is set but frequency is zero; value is ignored until spring frequncy > 0");
			mSpring.mDampingRatio = ratio;
		}

		float GetSpringFrequency() const { return mSpring.mFrequency; }
		float GetSpringDampingRatio() const { return mSpring.mDampingRatio; }

		virtual uint32 PrepSolver(ConstraintSolver* solver, const PhysicsStepContext& ctx) override;
		/// essentailly used for commiting back accumulated lambda
		/// based on constraints policy
		virtual void CommitSolverState(const Linear1DRow& row) override
		{
			mAccumulatedLambda = row.lambda;
		}


		virtual void SolvePositionConstraint(float dt, float baumgarte) override;


		void DrawConstraintBounds(DebugGizmosRenderer* debug_renderer, const Vec3& rAw, const Vec3& rBw, Colour col) const;

		virtual void DebugGizmos(DebugGizmosRenderer* debug_renderer, const NonContactConstraintDrawSettings& draw_settings) const override;
		void QuickSolve(float dt, int velocity_iteration = 8);

		virtual void GetRowCounts(/*const PhysicsStepContext& ctx, */uint32& o_1D_rows, uint32& o_3D_rows) override
		{
			o_1D_rows = 1;
			o_3D_rows = 0;
		}
	private:
		/// lets say this are points in body local frame
		Vec3 mLocalAnchorA = Vec3(0.0f);
		Vec3 mLocalAnchorB = Vec3(0.0f);


		float mMinDistance;
		float mMaxDistance;


		SpringSettings mSpring{};

		//presisent state (warm starting)
		float mAccumulatedLambda = 0.0f;


		Vec3 mWorldAnchorA = Vec3(0.0f);
		Vec3 mWorldAnchorB = Vec3(0.0f);
		Vec3 mWorldAxis = Vec3(0.f);

	private: 

		bool RequiresPositionCorrection()
		{
			return Contains(EConstraintFlags::SolvePosition, mFlags);
		}

		Vec3 ComputeConstraintPropertiesDisplacement(Vec3& o_rA, Vec3& o_rB);
		void BuildDistanceJacobian(Linear1DRow* o_row, float dt);
		Rigid1DConstraint BuildRigidConstraint();
	};
} //namespace vx