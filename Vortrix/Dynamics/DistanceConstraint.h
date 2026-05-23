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
		Body* mBodyA = nullptr;
		Body* mBodyB = nullptr;

		/// lets say this are points in body local frame
		Vec3 mLocalAnchorA = Vec3(0.0f);
		Vec3 mLocalAnchorB = Vec3(0.0f);

		//presisent state (warm starting)
		float mAccumulatedLambda = 0.0f;

		float mMinDistance;
		float mMaxDistance;

		SpringSettings mSpring;

		virtual bool PrepSolver(ConstraintSolver* solver, const PhysicsStepContext& ctx) override;
		/// essentailly used for commiting back accumulated lambda
		/// based on constraints policy
		virtual void CommitSolverState(const Linear1DRow& row) override
		{
			mAccumulatedLambda = row.lambda;
		}

		virtual void SolvePositionConstraint(float dt, float baumgarte) override
		{
			VX_LOG_WARN("Solve Position Constraint not implemented yet");
		}

		//Quick 
		void QuickSolve(float dt)
		{
			if (!mBodyA || !mBodyB)
				return;


			//Linear1DRow solver_row = SetupDistanceJacobian(dt);
			Linear1DRow solver_row = BuildDistanceJacobian(dt);

			if (solver_row.effMass <= 0.0f)
				return;

			Body& bodyA = *mBodyA; //context.BodyManager().GetBody(solver_row.bodyA)
			Body& bodyB = *mBodyB;

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
			Vec3 rAXn = Vec3::LoadFloat3Raw(solver_row.rAXn);
			Vec3 invIrAXn = Vec3::LoadFloat3Raw(solver_row.invIrAXn);

			Vec3 rBXn = Vec3::LoadFloat3Raw(solver_row.rBXn);
			Vec3 invIrBXn = Vec3::LoadFloat3Raw(solver_row.invIrBXn);



				//jacobian 
			float jv;
			if (dyn_a && dyn_b) ///if constexpr (
				jv = (lin_velA - lin_velB).Dot(solver_row.axis);
			else if (dyn_a)
				jv = lin_velA.Dot(solver_row.axis);
			else if (dyn_b)
				jv = (-lin_velB).Dot(solver_row.axis);
			else
			{
				VX_LOG_ERROR("Static vs static this should not be possible");
				jv = 0.0f;
			}

			if (dyn_a)
				jv += rAXn.Dot(ang_velA);
			if (dyn_b)
				jv -= rBXn.Dot(ang_velB);


			float lambda = (jv - solver_row.bias) * solver_row.effMass;

			float old_lambda = solver_row.lambda;
			//ensure non negative
			//bilateral constraint
			solver_row.lambda += lambda;
			solver_row.lambda = VxClamp(old_lambda + lambda, solver_row.minLambda, solver_row.maxLambda);
			//updated jn
			float impluse = solver_row.lambda - old_lambda;

			//store changes
			if (dyn_a)
			{
				lin_velA -= impluse * inv_massA * solver_row.axis;
				ang_velA -= impluse * invIrAXn;
			}
			if (dyn_b)
			{
				lin_velB += impluse * inv_massB * solver_row.axis;
				ang_velB += impluse * invIrBXn;
			}

			//write back to body 
			bodyA.SetLinearVelocity(lin_velA);
			bodyA.SetAngularVelocity(ang_velA);

			bodyB.SetLinearVelocity(lin_velB);
			bodyB.SetAngularVelocity(ang_velB);
		}


		void DrawConstraintBounds(DebugGizmosRenderer* debug_renderer, const Vec3& rAw, const Vec3& rBw) const
		{
			//achorbound a
			AABB boundA = AABB(0.25);
			boundA.Translate(rAw);

			AABB boundB = AABB(0.25);
			boundB.Translate(rBw);

			boundA.Merge(boundB);
			debug_renderer->DrawAABB(boundA, Colour::sMagenta);
		}




		virtual void DebugGizmos(DebugGizmosRenderer* debug_renderer) const override
		{
			if (debug_renderer && mBodyA && mBodyB)
			{

				const float mA = 1.0f / mBodyA->GetInverseMass();
				const float mB = 1.0f / mBodyB->GetInverseMass();
				//split rest length based on mass contribution 
				float total_mass = mA + mB;
				//distribute force if total mass is not too small else split
				float ratio = (total_mass > kEpsilon) ? (mB / total_mass) : 0.5;


				Vec3 rA = mBodyA->GetOrientation().Rotate(mLocalAnchorA);
				Vec3 rB = mBodyB->GetOrientation().Rotate(mLocalAnchorB);

				Vec3 rAw = rA + mBodyA->GetPosition();
				Vec3 rBw = rB + mBodyB->GetPosition();

				Vec3 disp = rBw - rAw;
				float curr_dist = disp.Length();

				Vec3 nor = (curr_dist > kEpsilon) ? disp / curr_dist : Vec3::Up();

				Colour line_col = Colour(0.3f);
				if (mMinDistance != mMaxDistance)
				{
					if (curr_dist >= mMaxDistance)
						line_col = Colour(1.0f, 0.3f, 0.3f);
					else if (curr_dist <= mMinDistance)
						line_col = Colour(0.3f, 0.6f, 1.0f);
				}
				//else
					//line_col = Colour(0.0f, 1.0f, 0.0f);

				debug_renderer->DrawLine(rAw, rBw, line_col);

				//DrawConstraintBounds(debug_renderer, rAw, rBw);
				debug_renderer->DrawSphere<4, 4>(rAw, 0.085f, Colour(1.0f, 0.2f, 0.2f));
				debug_renderer->DrawSphere<4, 4>(rBw, 0.085f, Colour(0.2f, 1.0f, 0.6f));

				//boundaries
				if (mMinDistance == mMaxDistance)
				{
					debug_renderer->DrawLine(rAw, (rAw + (nor * static_cast<float>(mMaxDistance * ratio))), Colour(1.0f, 1.0f, 0.0f));
					debug_renderer->DrawLine(rBw, (rBw + (-nor * static_cast<float>(mMaxDistance * (1 - ratio)))), Colour(0.0f, 1.0f, 0.0f));
				}
				else
				{
					Vec3 min_limit_ptA = rAw + (nor * (mMinDistance * ratio));
					Vec3 min_limit_ptB = rBw - (nor * (mMinDistance * (1.0f - ratio)));

					Vec3 max_limit_ptA = rAw + (nor * (mMaxDistance * ratio));
					Vec3 max_limit_ptB = rBw - (nor * (mMaxDistance * (1.0f - ratio)));


					constexpr bool k_draw_box = true;

					if constexpr (k_draw_box)
					{
						bool wire_frame = true;
						////later support pt 
						AABB aabb(0.125);
						aabb.Translate(min_limit_ptA);
						debug_renderer->DrawAABB(aabb, Colour(0.3f, 0.6f, 1.0f), wire_frame);
						aabb.Reset();
						aabb = AABB(0.125);
						aabb.Translate(min_limit_ptB);
						debug_renderer->DrawAABB(aabb, Colour(0.3f, 0.6f, 1.0f), wire_frame);

						aabb.Reset();
						aabb = AABB(0.125);
						aabb.Translate(max_limit_ptA);
						debug_renderer->DrawAABB(aabb, Colour(1.0f, 1.0f, 0.0f), wire_frame);
						aabb.Reset();
						aabb = AABB(0.125);
						aabb.Translate(max_limit_ptB);
						debug_renderer->DrawAABB(aabb, Colour(1.0f, 1.0f, 0.0f), wire_frame);
					}
					else
					{
						debug_renderer->DrawSphere(min_limit_ptA, 0.125, Colour(0.3f, 0.6f, 1.0f));
						debug_renderer->DrawSphere(min_limit_ptB, 0.125, Colour(0.3f, 0.6f, 1.0f));

						debug_renderer->DrawSphere(max_limit_ptA, 0.125, Colour(1.0f, 1.0f, 0.0f));
						debug_renderer->DrawSphere(max_limit_ptB, 0.125, Colour(1.0f, 1.0f, 0.0f));
					}
				}
			}
		}



		private: 

			Linear1DRow BuildDistanceJacobian(float dt)
			{
				Linear1DRow row;

				//lets take into consideration that 
				// that the achor point is not COM
				Vec3 rA = mBodyA->GetOrientation().Rotate(mLocalAnchorA);
				Vec3 rB = mBodyB->GetOrientation().Rotate(mLocalAnchorB);

				Vec3 rAw = rA + mBodyA->GetPosition();
				Vec3 rBw = rB + mBodyB->GetPosition();


				Vec3 dispW = rBw - rAw;
				Vec3 nor = dispW.Normalised();
				row.axis = nor;

				bool bodyA_nonstatic = !mBodyA->IsStatic();
				bool bodyB_nonstatic = !mBodyB->IsStatic();


				float inv_eff_mass = 0.0f;
				if (bodyA_nonstatic)
				{
					Vec3 rAXn = rA.Cross(nor);
					Vec3 invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);

					rAXn.Store(row.rAXn);
					invIrAXn.Store(row.invIrAXn);

					inv_eff_mass += mBodyA->GetInverseMass() + invIrAXn.Dot(rAXn);
				}

				if (bodyB_nonstatic)
				{
					Vec3 rBXn = rB.Cross(nor);
					Vec3 invIrBXn = mBodyB->ComputeInvInteriaWorld().Multiply3x3(rBXn);

					rBXn.Store(row.rBXn);
					invIrBXn.Store(row.invIrBXn);

					inv_eff_mass += mBodyB->GetInverseMass() + invIrBXn.Dot(rBXn);
				}

				float error = 0.0f;
				float curr_dist = dispW.Length();
				//compute error and limits
				//bilateral propagation
				if (mMinDistance == mMaxDistance)
				{
					error = curr_dist - mMaxDistance;
					row.minLambda = -kMaxf;
					row.maxLambda = kMaxf;
				}
				else if (curr_dist >= mMaxDistance)
				{
					///max limit breached 
					error = curr_dist - mMaxDistance;
					row.minLambda = -kMaxf;
					row.maxLambda = 0.0f;
				}
				else if (curr_dist <= mMinDistance)
				{
					error = curr_dist - mMinDistance;
					row.minLambda = 0.0f;
					row.maxLambda = kMaxf;
				}
				else
				{
					row.effMass = 0.0f;
					return row;
				}



				mSpring.ComputeProperties(dt, inv_eff_mass, error, 0.0f, row.effMass, row.bias, row.gamma);

				///later when figure out, caching implmentation for warm start etc
				row.lambda = mAccumulatedLambda;
				return row;




				///// stiffness
				//float k = 0.0f;
				///// damping
				//float c = 0.0f;

				//float eff_mass = (inv_eff_mass > 0.0f) ? 1.0f / inv_eff_mass : 0.0f;

				//if (mSpring.FrequencyDampingTuning())
				//{
				//	float omega = 2.0f * kVxPi * mSpring.frequency;
				//	k = eff_mass * VxSqr(omega);
				//	c = 2.0f * eff_mass * mSpring.dampingRatio * omega;
				//}

				//////might want to authour this externally 
				////float beta = 0.3f; //hard constraint
				////float gamma = 0.0f;
				////if (k > 0.0f)
				////{
				////	/// CFM = 1 / (hk+c)
				////	gamma = 1.0f / (c + dt * k);
				////	/// ERP = hk/(hk+c)
				////	beta = (dt * k) / (c + dt * k);
				////}

				////gamma += mSpring.softness;
				////row.effMass = 1.0f / (inv_eff_mass + gamma);
				//////row.bias = beta * error / dt;
				////row.bias = (beta / dt) * error;
				////row.gamma = gamma;


				///// C -> error
				///// mSpring.softness is external extra softness
				///// generally equal to zero 
				///// actual softness from Soft Constraints: Reinventing The Spring - Erin Catto - GDC 2011
				///// softness(gamma) = 1.0f / (h(hk+c)
				///// stiffness k
				///// h dt
				///// c damping 
				//float gamma = 0.0f;
				//float beta = 0.3f; //hard constraint
				//if (k > 0)
				//{
				//	gamma = 1.0f / (dt * (dt * k + c));
				//	beta = (dt * k) * gamma;
				//}

				//row.effMass = 1.0f / (inv_eff_mass + gamma);
				//float bias = beta * error / dt;
				//row.bias = bias;
				//row.gamma = gamma;

				/////later when figure out, caching implmentation for warm start etc
				//row.lambda = mAccumulatedLambda;


				//return row;
			}
	};
} //namespace vx