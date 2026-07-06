#pragma once
#include "Vortrix/Dynamics/Body/Body.h"

#include "Vortrix/Visuals/Renderers.h"


#include "Constraint.h"


namespace vx {


	enum class EConstraintFrame
	{
		Local,
		World
	};

	struct PointConstraintSettings
	{
		PointConstraintSettings() = default;
		PointConstraintSettings(const Vec3& _anchorA, const Vec3& _anchorB, EConstraintFrame frame) :
			anchorA(_anchorA), anchorB(_anchorB), anchorPointFrame(frame) {}

		Vec3 anchorA;
		Vec3 anchorB;
		EConstraintFrame anchorPointFrame = EConstraintFrame::World;

		/// Velocity bias is used to nudge/steer linear velocity 
		/// towards target to reduce error, prevent saggyness
		/// and encourges rigidity
		bool enableVelocityBias = true;
		/// the linear error compute for velocity bias 
		/// treshold before bias is allowed
		/// so treshold could come in handle for building
		/// to bodies where the constraint would cause an overlap 
		/// without a collision filter this would cause bodies to dance 
		/// about eachother but with a treahold approx to the average body/shape from 
		/// achor to surface 
		float errorTreshold = kEpsilon;
	};




	class PointConstraint : public Constraint
	{
	public:
		EConstraintType Type() const override { return EConstraintType::Point; }
		const char* TypeName() const override { return "Point Constraint"; }
		PointConstraint() : Constraint(nullptr, nullptr)
		{
			mFlags = EConstraintFlags::SolveVelocity;// | ~EConstraintFlags::SolvePosition;
		}

		PointConstraint(Body* bodyA, Body* bodyB, const PointConstraintSettings& settings);

		bool mHasVelocityBias = true;
		float mErrorTreshold = kEpsilon;

		void SetLocalAnchorA(const Vec3& position) { mLocalAnchorA = position; }
		void SetLocalAnchorB(const Vec3& position) { mLocalAnchorB = position; }

		Vec3 GetLocalAnchorA() const { return mLocalAnchorA; }
		Vec3 GetLocalAnchorB() const { return mLocalAnchorB; }

		Vec3 GetAccumulatedLambda() const { return mAccumulatedLambda; }

		virtual bool PrepSolver(ConstraintSolver* solver, const PhysicsStepContext& ctx) override;

		/// essentailly used for commiting back accumulated lambda
		/// based on constraints policy
		virtual void CommitSolverState(const Linear1DRow& row) override;

		virtual void SolvePositionConstraint(float dt, float baumgarte) override;


		void DrawConstraintBounds(DebugGizmosRenderer* debug_renderer, const Vec3& rAw, const Vec3& rBw, Colour col) const;

		virtual void DebugGizmos(DebugGizmosRenderer* debug_renderer, const NonContactConstraintDrawSettings& draw_settings) const override;

		void QuickSolveUnified3DJacobian(float dt, int velocity_iteration = 8, int position_iteration = 2, float baumgarte = 0.2f);
		void QuickSolveSplit1DJacobians(float dt, int velocity_iteration = 8, int position_iteration = 2, float baumgarte = 0.2f);



		virtual void GetRowCounts(/*const PhysicsStepContext& ctx, */uint32& o_1D_rows, uint32& o_3D_rows) override
		{
			//if (ctx.unified3D) 
			//{
			//	o_1D_rows = 0;
			//	o_3D_rows = 1;
			//}
			//else
			//{
			o_1D_rows = 3;
			o_3D_rows = 0;
			//}
		}

		
	private:

		Vec3 ComputeConstraintPropertiesDisplacement(Vec3& o_rA, Vec3& o_rB);

		void BuildSplit1DJacobians(Linear1DRow* rows, float dt);
		void BuildAxis1DJacobian(Linear1DRow* row, float accumulated_lambda, const Vec3& axis, const Vec3& rA, const Vec3& rB, const Vec3& dispW, float dt);



		bool RequiresPositionCorrection()
		{
			return Contains(EConstraintFlags::SolvePosition, mFlags);
		}

		/// lets say this are points in body local frame
		Vec3 mLocalAnchorA = Vec3(0.0f);
		Vec3 mLocalAnchorB = Vec3(0.0f);

		//presisent state (warm starting)
		Vec3 mAccumulatedLambda = Vec3(0.0f);

	};
} //namespace vx