#pragma once
#include "Body/Body.h"
#include "SampleFramework/Renderer/DebugGizmosRenderer.h"

#include "ConstraintSolverRow.h"

#include "Constraint.h"
#include "ConstraintCoordinator.h"

namespace vx {


	enum class ESpringTuningMode
	{
		StiffnessSoftness,
		FrequencyDamping
	};

	struct SpringSettings
	{
		ESpringTuningMode tunningMode = ESpringTuningMode::StiffnessSoftness;

		union /// A[stiffness]/B[freq]
		{
			float stiffness = 0.0f; /// [N/m] hookes constant 0, hard 
			float frequency;// = 0.0f;  ///[Hz]
		};

		/// A[stiffness
		float damping = 0.0f; /// [Ns/m] damping coeeff

		/// B[freq]
		float dampingRatio = 1.0f; ///1.0f critical damping, 0.0f infinte bounces 


		float softness = 0.0f;

		bool FrequencyDampingTuning() const
		{
			return tunningMode == ESpringTuningMode::FrequencyDamping
				&& frequency > 0.0f;
		}


		bool StiffnessSoftnessTuning() const
		{
			return tunningMode == ESpringTuningMode::StiffnessSoftness
				&& stiffness > 0.0f;
		}
	};



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

		virtual bool PrepSolver(ConstraintSolver* solver, float dt) override;
		/// essentailly used for commiting back accumulated lambda
		/// based on constraints policy
		virtual void CommitSolverState(const Linear1DRow& row) override
		{

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

			Linear1DRow SetupDistanceJacobian2(float dt)
			{


				Linear1DRow row;

				row.bodyA = mBodyA->GetID();
				row.bodyB = mBodyB->GetID();

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

					rAXn.Store(row.angularA);
					invIrAXn.Store(row.invIAngularA);

					inv_eff_mass += mBodyA->GetInverseMass() + invIrAXn.Dot(rAXn);
				}

				if (bodyB_nonstatic)
				{
					Vec3 rBXn = rB.Cross(nor);
					Vec3 invIrBXn = mBodyB->ComputeInvInteriaWorld().Multiply3x3(rBXn);

					rBXn.Store(row.angularB);
					invIrBXn.Store(row.invIAngularB);

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
					row.effectiveMass = 0.0f;
					return row;
				}


				/// stiffness
				float k = 0.0f;
				/// damping
				float c = 0.0f;

				if (mSpring.FrequencyDampingTuning())
				{
					float omega = 2.0f * kVxPi * mSpring.frequency;
					k = omega * omega;
					c = 2.0f * mSpring.dampingRatio * omega;
				}
				else if (mSpring.StiffnessSoftnessTuning())
				{
					k = mSpring.stiffness;
					c = mSpring.damping;
				}

				float beta = 0.2f; //hard constraint
				float gamma = 0.0f;
				if (k > 0.0f)
				{
					gamma = 1.0f / (dt * (c + dt * k));
					beta = (dt * k) / (c + dt * k);
				}

				gamma += mSpring.softness;
				row.effectiveMass = 1.0f / (inv_eff_mass + gamma);
				row.bias = beta * error / dt;

				///later when figure out, caching implmentation for warm start etc
				row.lambda = 0.0f;

				return row;
			}
	};
} //namespace vx