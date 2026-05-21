#pragma once
#include "ConstraintSolverRow.h"

#include "Constraint.h"
#include "DistanceConstraint.h"

#include "PhysicsWorldSettings.h"
#include "Body/BodyManager.h"

class DebugGizmosRenderer;

namespace vx {



	class ConstraintCoordinator
	{
	public:
		void AddConstraint(Constraint* constraint)
		{
			mConstraints.push_back(constraint);
		}

		//void AddConstraint(const DistanceConstraint& joint)
		//{
		//	AddConstraint(new DistanceConstraint(joint));
		//}

		template<typename T>
		T* AddConstraintT(const T& joint)
		{
			T* _j = new T(joint);
			AddConstraint(_j);
			return _j;
		}

		std::vector<Constraint*>& GetConstraints() { return mConstraints; }

		void PrepConstraintSolving(ConstraintSolver& solver, float dt)
		{
			for (auto& c : mConstraints)
				c->PrepSolver(&solver, dt);
		}


		void DebugGizmos(DebugGizmosRenderer* debug_renderer)
		{
			for (auto& c : mConstraints)
				c->DebugGizmos(debug_renderer);
		}
	private: 
		//for now vector 
		std::vector<Constraint*> mConstraints;
	};


	/// later have a solver builder (which includes island bases) 
	class ConstraintSolver
	{
	public:
		void AddLinearRow(const Linear1DRow& row)
		{
			mLinear1DRows.push_back(row);
		}
		
		static void SolverVelocityLinear1DRow(Linear1DRow& row, const PhysicsStepContext& ctx)
		{
			if (row.effectiveMass <= 0.0f)
				return;

			//may be have a base TwoBodyConstraint
			///later solver body not actual body
			Body& bodyA = ctx.bodyManager->GetBody(row.bodyA); //context.BodyManager().GetBody(solver_row.bodyA)
			Body& bodyB = ctx.bodyManager->GetBody(row.bodyB);

			bool dyn_a = bodyA.IsDynamic();
			bool dyn_b = bodyB.IsDynamic();


			///this is the work of solver body and not direct for Body
			Vec3 lin_velA = bodyA.GetLinearVelocity();
			Vec3 lin_velB = bodyB.GetLinearVelocity();
			Vec3 ang_velA = bodyA.GetAngularVelocity();
			Vec3 ang_velB = bodyB.GetAngularVelocity();
			float inv_massA = bodyA.GetInverseMass();
			float inv_massB = bodyB.GetInverseMass();


			//load data
			Vec3 rAXn = Vec3::LoadFloat3Raw(row.angularA);
			Vec3 invIrAXn = Vec3::LoadFloat3Raw(row.invIAngularA);

			Vec3 rBXn = Vec3::LoadFloat3Raw(row.angularB);
			Vec3 invIrBXn = Vec3::LoadFloat3Raw(row.invIAngularB);


			//then later in constraint solver 
			for (int i = 0; i < 20; ++i)
			{

				//jacobian 
				float jv;
				if (dyn_a && dyn_b) ///if constexpr (
					jv = (lin_velA - lin_velB).Dot(row.axis);
				else if (dyn_a)
					jv = lin_velA.Dot(row.axis);
				else if (dyn_b)
					jv = (-lin_velB).Dot(row.axis);
				else
				{
					VX_LOG_ERROR("Static vs static this should not be possible");
					jv = 0.0f;
				}

				if (dyn_a)
					jv += rAXn.Dot(ang_velA);
				if (dyn_b)
					jv -= rBXn.Dot(ang_velB);

				//float actual_bias = 
				float lambda = (jv - row.bias) * row.effectiveMass;

				float old_lambda = row.lambda;
				//ensure non negative
				//bilateral constraint
				row.lambda += lambda;
				row.lambda = VxClamp(old_lambda + lambda, row.minLambda, row.maxLambda);
				//updated jn
				float impluse = row.lambda - old_lambda;

				//store changes
				if (dyn_a)
				{
					lin_velA -= impluse * inv_massA * row.axis;
					ang_velA -= impluse * invIrAXn;
				}
				if (dyn_b)
				{
					lin_velB += impluse * inv_massB * row.axis;
					ang_velB += impluse * invIrBXn;
				}

			}
			//write back to body 
			bodyA.SetLinearVelocity(lin_velA);
			bodyA.SetAngularVelocity(ang_velA);

			bodyB.SetLinearVelocity(lin_velB);
			bodyB.SetAngularVelocity(ang_velB);
		}

		/// static for future multothreading
		static void SolverVelocityLinear1DRows(Linear1DRow* rows, size_t begin_offset, size_t count, const PhysicsStepContext& ctx)
		{
			//for (Linear1DRow** r = rows, **r_end = rows + count; r < r_end; ++r)
			for (Linear1DRow* r = rows, *r_end = rows + count; r < r_end; ++r)
				SolverVelocityLinear1DRow(*r, ctx);
		}


		void SolverAll(const PhysicsStepContext& ctx)
		{
			ConstraintSolver::SolverVelocityLinear1DRows(mLinear1DRows.data(), 0, mLinear1DRows.size(), ctx);
		}
	private:
		//vectot for now
		std::vector<Linear1DRow> mLinear1DRows;
	};




} //namespace vx