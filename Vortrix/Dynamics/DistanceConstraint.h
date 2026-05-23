#pragma once
#include "Body/Body.h"
#include "SampleFramework/Renderer/DebugGizmosRenderer.h"

#include "ConstraintSolverRow.h"

#include "Constraint.h"
#include "ConstraintCoordinator.h"

#include "SpringSettings.h"

namespace vx {

	class DistanceConstraint : public Constraint
	{
	public:

		DistanceConstraint() 
		{
			mFlags = EConstraintFlags::SolveVelocity;// | ~EConstraintFlags::SolvePosition;
		}
		Body* mBodyA = nullptr;
		Body* mBodyB = nullptr;

		/// lets say this are points in body local frame
		Vec3 mLocalAnchorA = Vec3(0.0f);
		Vec3 mLocalAnchorB = Vec3(0.0f);

		//presisent state (warm starting)
		float mAccumulatedLambda = 0.0f;

		float mMinDistance;
		float mMaxDistance;


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
			VX_ASSERT_WARN((mSpring.mFrequency != 0.0f) || (ratio == 0.0f), 
				"Damping ratio is set but frequency is zero; value is ignored until spring frequncy > 0");
			mSpring.mDampingRatio = ratio;
		}

		float GetSpringFrequency() const { return mSpring.mFrequency; }
		float GetSpringDampingRatio() const { return mSpring.mDampingRatio; }

		virtual bool PrepSolver(ConstraintSolver* solver, const PhysicsStepContext& ctx) override;
		/// essentailly used for commiting back accumulated lambda
		/// based on constraints policy
		virtual void CommitSolverState(const Linear1DRow& row) override
		{
			mAccumulatedLambda = row.lambda;
		}


		virtual void SolvePositionConstraint(float dt, float baumgarte) override;


		void DrawConstraintBounds(DebugGizmosRenderer* debug_renderer, const Vec3& rAw, const Vec3& rBw) const;

		virtual void DebugGizmos(DebugGizmosRenderer* debug_renderer) const override;
		void QuickSolve(float dt);

	private:
		SpringSettings mSpring;
		Vec3 mWorldAnchorA = Vec3(0.0f);
		Vec3 mWorldAnchorB = Vec3(0.0f);
		Vec3 mWorldAxis = Vec3(0.f);

	private: 

		bool RequiresPositionCorrection()
		{
			return Contains(EConstraintFlags::SolvePosition, mFlags);
		}

		Vec3 ComputeConstraintPropertiesDisplacement(Vec3& o_rA, Vec3& o_rB);
		Linear1DRow BuildDistanceJacobian(float dt);
		Rigid1DConstraint BuildRigidConstraint();

	};
} //namespace vx