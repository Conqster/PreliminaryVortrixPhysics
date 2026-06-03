#pragma once
#include "Body/Body.h"
#include "SampleFramework/Renderer/DebugGizmosRenderer.h"


#include "Constraint.h"
#include "ConstraintCoordinator.h"


namespace vx {


	enum class EConstraintFrame
	{
		Local,
		World
	};

	struct PointConstraintSettings
	{
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

		PointConstraint()
		{
			mFlags = EConstraintFlags::SolveVelocity;// | ~EConstraintFlags::SolvePosition;
		}

	
		PointConstraint(Body* bodyA, Body* bodyB, const PointConstraintSettings& settings)
		{
			VX_ASSERT(bodyA && bodyB, "invalid constraint body pairs");

			mBodyA = bodyA;
			mBodyB = bodyB;

			if (settings.anchorPointFrame == EConstraintFrame::World)
			{
				///anchor A should be equal B 
				//VX_ASSERT_WARN(settings.anchorA == settings.anchorB);

				mLocalAnchorA = bodyA->GetOrientation().InverseRotate(settings.anchorA - bodyA->GetPosition());
				mLocalAnchorB = bodyB->GetOrientation().InverseRotate(settings.anchorB - bodyB->GetPosition());
			}
			else
			{
				mLocalAnchorA = settings.anchorA;
				mLocalAnchorB = settings.anchorB;
			}

			mHasVelocityBias = settings.enableVelocityBias;
			mErrorTreshold = settings.errorTreshold;
		}

		bool mHasVelocityBias = true;
		float mErrorTreshold = kEpsilon;

		void SetLocalAnchorA(const Vec3& position) { mLocalAnchorA = position; }
		void SetLocalAnchorB(const Vec3& position) { mLocalAnchorB = position; }

		Vec3 GetLocalAnchorA() const { return mLocalAnchorA; }
		Vec3 GetLocalAnchorB() const { return mLocalAnchorB; }

		Body GetBodyA() const { return *mBodyA; }
		Body GetBodyB() const { return *mBodyB; }

		Vec3 GetAccumulatedLambda() const { return mAccumulatedLambda; }

		virtual bool PrepSolver(ConstraintSolver* solver, const PhysicsStepContext& ctx) override
		{
			return false;
		}
		/// essentailly used for commiting back accumulated lambda
		/// based on constraints policy
		virtual void CommitSolverState(const Linear1DRow& row) override
		{
			mAccumulatedLambda[0] = row.lambda;
		}


		virtual void SolvePositionConstraint(float dt, float baumgarte) override
		{

		}

		//void ResolveOffset()
		//{
		//	Vec3 rA, rB;
		//	rA = mBodyA->GetOrientation().Rotate(mLocalAnchorA);
		//	rB = mBodyB->GetOrientation().Rotate(mLocalAnchorB);

		//	Vec3 rAw = rA + mBodyA->GetPosition();
		//	Vec3 rBw = rB + mBodyB->GetPosition();

		//	if (rAw.IsApprox(rBw))
		//		return;

		//	//use average 
		//	Vec3 pt = rAw + rBw;
		//	pt *= 0.5f;




		//	mLocalAnchorA = mBodyA->GetOrientation().InverseRotate(pt - mBodyA->GetPosition());
		//	mLocalAnchorB = mBodyB->GetOrientation().InverseRotate(pt - mBodyB->GetPosition());
		//}


		void DrawConstraintBounds(DebugGizmosRenderer* debug_renderer, const Vec3& rAw, const Vec3& rBw, Colour col) const;

		virtual void DebugGizmos(DebugGizmosRenderer* debug_renderer, const NonContactConstraintDrawSettings& draw_settings) const override
		{
			Vec3 rA, rB;
			rA = mBodyA->GetOrientation().Rotate(mLocalAnchorA);
			rB = mBodyB->GetOrientation().Rotate(mLocalAnchorB);

			Vec3 rAw = rA + mBodyA->GetPosition();
			Vec3 rBw = rB + mBodyB->GetPosition();

			//debug_renderer->DrawAACross(rAw, &Colour(1.0f, 0.2f, 0.2f), 1, 0.085f);
			//debug_renderer->DrawAACross(rBw, &Colour(0.2f, 1.0f, 0.4f), 1, 0.085f);

			debug_renderer->DrawAACross(rAw, &Colour(1.0f, 0.2f, 0.2f), 1, 1.5f);
			debug_renderer->DrawAACross(rBw, &Colour(0.2f, 1.0f, 0.4f), 1, 1.5f);

			//error
			if (!rAw.IsApprox(rBw))
				debug_renderer->DrawLine(rAw, rBw, Colour::sRed);
		}
		Vec3 mLastStepBias;
		void QuickSolve(float dt)
		{
			float baumgarte = 0.2f;
			//lets take into consideration that 
			// that the achor point is not COM
			Vec3 rA = mBodyA->GetOrientation().Rotate(mLocalAnchorA);
			Vec3 rB = mBodyB->GetOrientation().Rotate(mLocalAnchorB);

			bool bodyA_nonstatic = !mBodyA->IsStatic();
			bool bodyB_nonstatic = !mBodyB->IsStatic();

			Mat44 invIA_x_rAx = Mat44(0.0f);
			Mat44 invIB_x_rBx = Mat44(0.0f);

			Mat44 inv_eff_M = Mat44(0.0f);


			Vec3 bias;
			if (mHasVelocityBias)
			{
				Vec3 pA = mBodyA->GetPosition() + rA;
				Vec3 pB = mBodyB->GetPosition() + rB;

				Vec3 error = pB - pA;
				bias = (Vec3::Greater(error.Abs(), Vec3(mErrorTreshold))) ? error / dt : Vec3(0.0f);

				mLastStepBias = bias;
				bias = baumgarte * bias;
			}
			else
				bias.ToZero();

			float inv_massA = 0.0f;
			float inv_massB = 0.0f;

			if (bodyA_nonstatic)
			{
				Mat44 invIA = mBodyA->ComputeInvInteriaWorld();

				Mat44 rAx = Mat44::SkewSymmetric3x3(rA);
				invIA_x_rAx = invIA.Multiply3x3(rAx);

				inv_massA = mBodyA->GetInverseMass();
				inv_eff_M = rAx.Multiply3x3(invIA).Multiply3x3RightTransposed(rAx);
			}

			if (bodyB_nonstatic)
			{
				Mat44 invIB = mBodyB->ComputeInvInteriaWorld();

				Mat44 rBx = Mat44::SkewSymmetric3x3(rB);
				invIB_x_rBx = invIB.Multiply3x3(rBx);

				inv_massB = mBodyB->GetInverseMass();
				inv_eff_M = inv_eff_M.Add(rBx.Multiply3x3(invIB).Multiply3x3RightTransposed(rBx));
			}

			if (!bodyA_nonstatic && !bodyB_nonstatic)
				return;

			float point_inv_eff_mass = inv_massA + inv_massB;

			inv_eff_M = inv_eff_M.Add(Mat44::Scale(point_inv_eff_mass));

			const float det = inv_eff_M.Determinant3x3();

			Vec3 Mx = inv_eff_M.GetAxisX();
			Vec3 My = inv_eff_M.GetAxisY();
			Vec3 Mz = inv_eff_M.GetAxisZ();
			const float det2 = Mx.ScalarTriple(My, Mz);

			VX_ASSERT_WARN(VxApprox(det,det2), "det");
			//if (VxAbs(det) < kEpsilon)
			if (det == 0.0f)
				return;

			Mat44 eff_M = inv_eff_M.Inverse3x3();

			bool dyn_a = mBodyA->IsDynamic();
			bool dyn_b = mBodyB->IsDynamic();

			Vec3 lin_velA = mBodyA->GetLinearVelocity();
			Vec3 lin_velB = mBodyB->GetLinearVelocity();
			
			Vec3 ang_velA = mBodyA->GetAngularVelocity();
			Vec3 ang_velB = mBodyB->GetAngularVelocity();

			Vec3 acc_lambda = Vec3(0.0f);
			//velocity iteration
			for (int i = 0; i < 8; ++i)
			{

				Vec3 impluse = eff_M.Multiply3x3(lin_velA 
					- rA.Cross(ang_velA) 
					- lin_velB 
					+ rB.Cross(ang_velB)
				- bias);
				//Vec3 Jv = (lin_velA - lin_velB)
				//	+ rA.Cross(ang_velA)
				//	- rB.Cross(ang_velB);

				//Vec3 impluse = -eff_M.Multiply3x3(Jv);
			
				acc_lambda += impluse;

				if (dyn_a)
				{
					lin_velA -= impluse * inv_massA;
					ang_velA -= invIA_x_rAx.Multiply3x3(impluse);
				}
				if (dyn_b)
				{
					lin_velB += impluse * inv_massB;
					ang_velB += invIB_x_rBx.Multiply3x3(impluse);
				}
			}

			//write back to body 
			mBodyA->SetLinearVelocity(lin_velA);
			mBodyA->SetAngularVelocity(ang_velA);

			mBodyB->SetLinearVelocity(lin_velB);
			mBodyB->SetAngularVelocity(ang_velB);


			//return;
			///cache for position constrint props
			Mat44 invIA = mBodyA->ComputeInvInteriaWorld();
			Mat44 invIB = mBodyB->ComputeInvInteriaWorld();
			for (int i = 0; i < 2; ++i)
			{
				//Vec3 separation = (Vec3(mBodyB->GetPosition() - mBodyA->GetPosition()) + mLocalAnchorB - mLocalAnchorA);

				Vec3 rA = mBodyA->GetOrientation().Rotate(mLocalAnchorA);
				Vec3 rB = mBodyB->GetOrientation().Rotate(mLocalAnchorB);

				invIA_x_rAx = Mat44(0.0f);
				invIB_x_rBx = Mat44(0.0f);

				//recompute properties
				if (bodyA_nonstatic)
				{
					Mat44 rAx = Mat44::SkewSymmetric3x3(rA);
					invIA_x_rAx = invIA.Multiply3x3(rAx);
				}
				if (bodyB_nonstatic)
				{
					Mat44 rBx = Mat44::SkewSymmetric3x3(rB);
					invIB_x_rBx = invIB.Multiply3x3(rBx);
				}

				///recompute error 
				Vec3 pA = mBodyA->GetPosition() + rA;
				Vec3 pB = mBodyB->GetPosition() + rB;
				Vec3 separation = pB - pA;

				Vec3 axis = separation.Normalised();
				Vec3 new_err = separation - (axis * mErrorTreshold);
				if(Vec3::Greater(new_err.Abs(), Vec3(kEpsilon)))
				//if (!separation.IsZero())
				//if(Vec3::Greater(separation.Abs(), axis * mErrorTreshold))
				{
					separation = new_err;

					Vec3 lambda = eff_M.Multiply3x3(-baumgarte * separation);

					if (dyn_a)
					{
						Vec3 x = lambda * inv_massA;
						mBodyA->ApplyLinearDisplacement(-x);
						mBodyA->ApplyAngularDisplacement(-(invIA_x_rAx.Multiply3x3(lambda)));
					}
					if (dyn_b)
					{
						Vec3 x = lambda * inv_massB;
						mBodyB->ApplyLinearDisplacement(x);
						mBodyB->ApplyAngularDisplacement(invIB_x_rBx.Multiply3x3(lambda));
					}
				}
			}
		}



		Vec3 ComputeConstraintPropertiesDisplacement(Vec3& o_rA, Vec3& o_rB)
		{
			//lets take into consideration that 
			// that the achor point is not COM
			o_rA = mBodyA->GetOrientation().Rotate(mLocalAnchorA);
			o_rB = mBodyB->GetOrientation().Rotate(mLocalAnchorB);

			Vec3 pA = o_rA + mBodyA->GetPosition();
			Vec3 pB = o_rB + mBodyB->GetPosition();

			return pB - pA;
		}

		void Build3_1DJacobianRow(Linear1DRow* rows, float dt)
		{
			Vec3 rA, rB;
			Vec3 dispW = ComputeConstraintPropertiesDisplacement(rA, rB);

			rows[0] = Build1DJacobianRow(0, Vec3::Right(), rA, rB, dispW, dt);
			rows[1] = Build1DJacobianRow(1, Vec3::Up(), rA, rB, dispW, dt);
			rows[2] = Build1DJacobianRow(2, Vec3::Forward(), rA, rB, dispW, dt);
		}


		Linear1DRow Build1DJacobianRow(int axis_idx, const Vec3& axis, 
			const Vec3& rA, const Vec3& rB, const Vec3& dispW, float dt)
		{
			Linear1DRow row;

			row.axis = axis;
			//lets take into consideration that 
			// that the achor point is not COM

			bool bodyA_nonstatic = !mBodyA->IsStatic();
			bool bodyB_nonstatic = !mBodyB->IsStatic();


			float inv_eff_mass = 0.0f;

			if (bodyA_nonstatic)
			{
				//Vec3 rAXn = Mat44::SkewSymmetric3x3(rA).GetColumn3(axis_idx); //essentially rA cross, along prticular axis
				Vec3 rAXn = rA.Cross(axis);
				Vec3 invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);

				rAXn.Store(row.rAXn);
				invIrAXn.Store(row.invIrAXn);

				inv_eff_mass += mBodyA->GetInverseMass() + invIrAXn.Dot(rAXn);
			}

			if (bodyB_nonstatic)
			{
				//Vec3 rBXn = Mat44::SkewSymmetric3x3(rB).GetColumn3(axis_idx); //essentially rA cross, along prticular axis
				Vec3 rBXn = rB.Cross(axis);
				Vec3 invIrBXn = mBodyB->ComputeInvInteriaWorld().Multiply3x3(rBXn);

				rBXn.Store(row.rBXn);
				invIrBXn.Store(row.invIrBXn);

				inv_eff_mass += mBodyB->GetInverseMass() + invIrBXn.Dot(rBXn);
			}

			if (!bodyA_nonstatic && !bodyB_nonstatic)
			{
				row.effMass = 0.0f;
				row.lambda = 0.0f;
				return row;
			}


			row.effMass = 1.0f / inv_eff_mass;


			if (mHasVelocityBias)
			{
				Vec3 error = dispW;

				//project error along axis 
				float axis_error = error.Dot(axis);
				float bias = (VxAbs(axis_error) > mErrorTreshold) ? axis_error / dt : 0.f;
				//bias = axis_error / dt;

				mLastStepBias[axis_idx] = bias;
				row.bias = 0.2f * bias;
			}
			else
				row.bias = 0.0f;


			//mSpring.ComputeProperties(dt, inv_eff_mass, error, 0.0f, row.effMass, row.bias, row.gamma);

			///later when figure out, caching implmentation for warm start etc
			row.lambda = mAccumulatedLambda[axis_idx];
			return row;
		}


		void QuickSolve1DJacobianRow(float dt)
		{
			Linear1DRow rows[3];
			Build3_1DJacobianRow(rows, dt);



			bool dyn_a = mBodyA->IsDynamic();
			bool dyn_b = mBodyB->IsDynamic();

			Vec3 lin_velA = mBodyA->GetLinearVelocity();
			Vec3 lin_velB = mBodyB->GetLinearVelocity();

			Vec3 ang_velA = mBodyA->GetAngularVelocity();
			Vec3 ang_velB = mBodyB->GetAngularVelocity();

			float inv_massA = mBodyA->GetInverseMass();
			float inv_massB = mBodyB->GetInverseMass();

			//velocity iteration
			for (int i = 0; i < 8; ++i)
			{
				///
				///effective mass along X axis 

				for(int x =0;x<3;++x)
				{
					Vec3 axis = rows[x].axis.Normalised();

					Vec3 rAXaxis = Vec3::LoadFloat3Raw(rows[x].rAXn);
					Vec3 rBXaxis = Vec3::LoadFloat3Raw(rows[x].rBXn);

					Vec3 invIrAXaxis = Vec3::LoadFloat3Raw(rows[x].invIrAXn);
					Vec3 invIrBXaxis = Vec3::LoadFloat3Raw(rows[x].invIrBXn);

					float jv;
					if (dyn_a && dyn_b) ///if constexpr (
						jv = (lin_velA - lin_velB).Dot(axis);
					else if (dyn_a)
						jv = lin_velA.Dot(axis);
					else if (dyn_b)
						jv = (-lin_velB).Dot(axis);
					else
					{
						VX_LOG_ERROR("Static vs static this should not be possible");
						jv = 0.0f;
					}

					if (dyn_a)
						jv += rAXaxis.Dot(ang_velA);
					if (dyn_b)
						jv -= rBXaxis.Dot(ang_velB);


					float impluse = (jv - rows[x].bias) * rows[x].effMass;
					if (dyn_a)
					{
						lin_velA -= impluse * inv_massA * axis;
						ang_velA -= impluse * invIrAXaxis;
					}
					if (dyn_b)
					{
						lin_velB += impluse * inv_massB * axis;
						ang_velB += impluse * invIrBXaxis;
					}
				}
			}

			//write back to body 
			mBodyA->SetLinearVelocity(lin_velA);
			mBodyA->SetAngularVelocity(ang_velA);

			mBodyB->SetLinearVelocity(lin_velB);
			mBodyB->SetAngularVelocity(ang_velB);

			///cache for position constrint props
	
			bool bodyA_nonstatic = !mBodyA->IsStatic();
			bool bodyB_nonstatic = !mBodyB->IsStatic();

			Mat44 invIA = mBodyA->ComputeInvInteriaWorld();
			Mat44 invIB = mBodyB->ComputeInvInteriaWorld();

			float baumgarte = 0.2f;
			for (int i = 0; i < 2; ++i)
			{
				//Vec3 separation = (Vec3(mBodyB->GetPosition() - mBodyA->GetPosition()) + mLocalAnchorB - mLocalAnchorA);


				Vec3 axes[3] = { Vec3::Right(), Vec3::Up(), Vec3::Forward() };

				for (int j = 0; j < 3; ++j)
				{
					Vec3 axis = axes[i];

					Vec3 rA, rB;
					Vec3 dispW = ComputeConstraintPropertiesDisplacement(rA, rB);
					float seperation_along_axis = axis.Dot(dispW);
					//if (seperation_along_axis < kEpsilon)
					//	continue;

					Vec3 invIArAXn = Vec3(0.0f);
					Vec3 invIBrBXn = Vec3(0.0f);

					float inv_eff_mass = 0.0f;
					if (bodyA_nonstatic)
					{
						Vec3 rAXn = rA.Cross(axis);
						invIArAXn = invIA.Multiply3x3(rAXn);

						inv_eff_mass += mBodyA->GetInverseMass() + invIArAXn.Dot(rAXn);
					}
					if (bodyB_nonstatic)
					{
						Vec3 rBXn = rB.Cross(axis);
						invIBrBXn = invIB.Multiply3x3(rBXn);

						inv_eff_mass += mBodyB->GetInverseMass() + invIBrBXn.Dot(rBXn);
					}

					float lambda = -(1.0f / inv_eff_mass) * baumgarte * seperation_along_axis;

					if (dyn_a)
					{
						Vec3 x = lambda * inv_massA * axis;
						mBodyA->ApplyLinearDisplacement(-x);
						mBodyA->ApplyAngularDisplacement(-lambda * invIArAXn);
					}
					if (dyn_b)
					{
						Vec3 x = lambda * inv_massB * axis;
						mBodyB->ApplyLinearDisplacement(x);
						mBodyB->ApplyAngularDisplacement(lambda * invIBrBXn);
					}
				}

			}
		}

		void QuickSolveEach1D(float dt)
		{
			float baumgarte = 0.2f;
			//lets take into consideration that 
			// that the achor point is not COM
			Vec3 rA = mBodyA->GetOrientation().Rotate(mLocalAnchorA);
			Vec3 rB = mBodyB->GetOrientation().Rotate(mLocalAnchorB);

			bool bodyA_nonstatic = !mBodyA->IsStatic();
			bool bodyB_nonstatic = !mBodyB->IsStatic();

			Mat44 invIA_x_rAx = Mat44(0.0f);
			Mat44 invIB_x_rBx = Mat44(0.0f);

			Mat44 inv_eff_M = Mat44(0.0f);


			Vec3 bias;
			if (mHasVelocityBias)
			{
				Vec3 pA = mBodyA->GetPosition() + rA;
				Vec3 pB = mBodyB->GetPosition() + rB;

				Vec3 error = pB - pA;
				bias = (Vec3::Greater(error.Abs(), Vec3(mErrorTreshold))) ? error / dt : Vec3(0.0f);

				mLastStepBias = bias;
				bias = baumgarte * bias;
			}
			else
				bias.ToZero();

			float inv_massA = 0.0f;
			float inv_massB = 0.0f;

			if (bodyA_nonstatic)
			{
				Mat44 invIA = mBodyA->ComputeInvInteriaWorld();

				Mat44 rAx = Mat44::SkewSymmetric3x3(rA);
				invIA_x_rAx = invIA.Multiply3x3(rAx);

				inv_massA = mBodyA->GetInverseMass();
				inv_eff_M = rAx.Multiply3x3(invIA).Multiply3x3RightTransposed(rAx);
			}

			if (bodyB_nonstatic)
			{
				Mat44 invIB = mBodyB->ComputeInvInteriaWorld();

				Mat44 rBx = Mat44::SkewSymmetric3x3(rB);
				invIB_x_rBx = invIB.Multiply3x3(rBx);

				inv_massB = mBodyB->GetInverseMass();
				inv_eff_M = inv_eff_M.Add(rBx.Multiply3x3(invIB).Multiply3x3RightTransposed(rBx));
			}

			if (!bodyA_nonstatic && !bodyB_nonstatic)
				return;

			float point_inv_eff_mass = inv_massA + inv_massB;

			inv_eff_M = inv_eff_M.Add(Mat44::Scale(point_inv_eff_mass));

			const float det = inv_eff_M.Determinant3x3();

			Vec3 Mx = inv_eff_M.GetAxisX();
			Vec3 My = inv_eff_M.GetAxisY();
			Vec3 Mz = inv_eff_M.GetAxisZ();
			const float det2 = Mx.ScalarTriple(My, Mz);

			VX_ASSERT_WARN(VxApprox(det, det2), "det");
			//if (VxAbs(det) < kEpsilon)
			if (det == 0.0f)
				return;

			Mat44 eff_M = inv_eff_M.Inverse3x3();

			bool dyn_a = mBodyA->IsDynamic();
			bool dyn_b = mBodyB->IsDynamic();

			Vec3 lin_velA = mBodyA->GetLinearVelocity();
			Vec3 lin_velB = mBodyB->GetLinearVelocity();

			Vec3 ang_velA = mBodyA->GetAngularVelocity();
			Vec3 ang_velB = mBodyB->GetAngularVelocity();

			Vec3 acc_lambda = Vec3(0.0f);


			//velocity iteration
			for (int i = 0; i < 8; ++i)
			{
				///
				///effective mass along X axis 

				Vec3 axis = eff_M.GetAxisX().Normalised();
				float eff_mass = eff_M.GetAxisX().Length();

				Vec3 rAXaxis = rA.Cross(axis);
				Vec3 rBXaxis = rB.Cross(axis);

				Vec3 invIrAXaxis = invIA_x_rAx.GetAxisX();
				Vec3 invIrBXaxis = invIB_x_rBx.GetAxisX();

				float jv;
				if (dyn_a && dyn_b) ///if constexpr (
					jv = (lin_velA - lin_velB).Dot(axis);
				else if (dyn_a)
					jv = lin_velA.Dot(axis);
				else if (dyn_b)
					jv = (-lin_velB).Dot(axis);
				else
				{
					VX_LOG_ERROR("Static vs static this should not be possible");
					jv = 0.0f;
				}

				if (dyn_a)
					jv += rAXaxis.Dot(ang_velA);
				if (dyn_b)
					jv -= rBXaxis.Dot(ang_velB);


				float impluse = (jv - bias.X()) * eff_mass;
				if (dyn_a)
				{
					lin_velA -= impluse * inv_massA * axis;
					ang_velA -= impluse * invIrAXaxis;
				}
				if (dyn_b)
				{
					lin_velB += impluse * inv_massB * axis;
					ang_velB += impluse * invIrBXaxis;
				}



				///effective mass along Y axis 

				axis = eff_M.GetAxisY().Normalised();
				eff_mass = eff_M.GetAxisY().Length();

				rAXaxis = rA.Cross(axis);
				rBXaxis = rB.Cross(axis);

				invIrAXaxis = invIA_x_rAx.GetAxisY();
				invIrBXaxis = invIB_x_rBx.GetAxisY();

				jv;
				if (dyn_a && dyn_b) ///if constexpr (
					jv = (lin_velA - lin_velB).Dot(axis);
				else if (dyn_a)
					jv = lin_velA.Dot(axis);
				else if (dyn_b)
					jv = (-lin_velB).Dot(axis);
				else
				{
					VX_LOG_ERROR("Static vs static this should not be possible");
					jv = 0.0f;
				}

				if (dyn_a)
					jv += rAXaxis.Dot(ang_velA);
				if (dyn_b)
					jv -= rBXaxis.Dot(ang_velB);

				impluse = (jv - bias.Y()) * eff_mass;
				if (dyn_a)
				{
					lin_velA -= impluse * inv_massA * axis;
					ang_velA -= impluse * invIrAXaxis;
				}
				if (dyn_b)
				{
					lin_velB += impluse * inv_massB * axis;
					ang_velB += impluse * invIrBXaxis;
				}


				///effective mass along Z axis 

				axis = eff_M.GetAxisZ().Normalised();
				eff_mass = eff_M.GetAxisZ().Length();

				rAXaxis = rA.Cross(axis);
				rBXaxis = rB.Cross(axis);

				invIrAXaxis = invIA_x_rAx.GetAxisZ();
				invIrBXaxis = invIB_x_rBx.GetAxisZ();

				jv;
				if (dyn_a && dyn_b) ///if constexpr (
					jv = (lin_velA - lin_velB).Dot(axis);
				else if (dyn_a)
					jv = lin_velA.Dot(axis);
				else if (dyn_b)
					jv = (-lin_velB).Dot(axis);
				else
				{
					VX_LOG_ERROR("Static vs static this should not be possible");
					jv = 0.0f;
				}

				if (dyn_a)
					jv += rAXaxis.Dot(ang_velA);
				if (dyn_b)
					jv -= rBXaxis.Dot(ang_velB);

				impluse = (jv - bias.Z()) * eff_mass;
				if (dyn_a)
				{
					lin_velA -= impluse * inv_massA * axis;
					ang_velA -= impluse * invIrAXaxis;
				}
				if (dyn_b)
				{
					lin_velB += impluse * inv_massB * axis;
					ang_velB += impluse * invIrBXaxis;
				}

				
			}

			//write back to body 
			mBodyA->SetLinearVelocity(lin_velA);
			mBodyA->SetAngularVelocity(ang_velA);

			mBodyB->SetLinearVelocity(lin_velB);
			mBodyB->SetAngularVelocity(ang_velB);


			//return;
			///cache for position constrint props
			Mat44 invIA = mBodyA->ComputeInvInteriaWorld();
			Mat44 invIB = mBodyB->ComputeInvInteriaWorld();
			for (int i = 0; i < 2; ++i)
			{
				//Vec3 separation = (Vec3(mBodyB->GetPosition() - mBodyA->GetPosition()) + mLocalAnchorB - mLocalAnchorA);

				Vec3 rA = mBodyA->GetOrientation().Rotate(mLocalAnchorA);
				Vec3 rB = mBodyB->GetOrientation().Rotate(mLocalAnchorB);

				invIA_x_rAx = Mat44(0.0f);
				invIB_x_rBx = Mat44(0.0f);

				//recompute properties
				if (bodyA_nonstatic)
				{
					Mat44 rAx = Mat44::SkewSymmetric3x3(rA);
					invIA_x_rAx = invIA.Multiply3x3(rAx);
				}
				if (bodyB_nonstatic)
				{
					Mat44 rBx = Mat44::SkewSymmetric3x3(rB);
					invIB_x_rBx = invIB.Multiply3x3(rBx);
				}

				///recompute error 
				Vec3 pA = mBodyA->GetPosition() + rA;
				Vec3 pB = mBodyB->GetPosition() + rB;
				Vec3 separation = pB - pA;

				Vec3 axis = separation.Normalised();
				Vec3 new_err = separation - (axis * mErrorTreshold);
				if (Vec3::Greater(new_err.Abs(), Vec3(kEpsilon)))
					//if (!separation.IsZero())
					//if(Vec3::Greater(separation.Abs(), axis * mErrorTreshold))
				{
					separation = new_err;

					//// solve X axis
					Vec3 axis = eff_M.GetAxisX().Normalised();
					Vec3 invIrAXn = invIA_x_rAx.GetAxisX();
					Vec3 invIrBXn = invIB_x_rBx.GetAxisX();
					float effMass = eff_M.GetAxisX().Length();

					float lambda = -effMass * baumgarte * separation.X();

					if (dyn_a)
					{
						Vec3 x = lambda * inv_massA * axis;
						mBodyA->ApplyLinearDisplacement(-x);
						mBodyA->ApplyAngularDisplacement(-lambda * invIrAXn);
					}
					if (dyn_b)
					{
						Vec3 x = lambda * inv_massB * axis;
						mBodyB->ApplyLinearDisplacement(x);
						mBodyB->ApplyAngularDisplacement(lambda * invIrBXn);
					}




					//// solve Y axis
					axis = eff_M.GetAxisY().Normalised();
					invIrAXn = invIA_x_rAx.GetAxisY();
					invIrBXn = invIB_x_rBx.GetAxisY();
					effMass = eff_M.GetAxisY().Length();

					lambda = -effMass * baumgarte * separation.Y();

					if (dyn_a)
					{
						Vec3 x = lambda * inv_massA * axis;
						mBodyA->ApplyLinearDisplacement(-x);
						mBodyA->ApplyAngularDisplacement(-lambda * invIrAXn);
					}
					if (dyn_b)
					{
						Vec3 x = lambda * inv_massB * axis;
						mBodyB->ApplyLinearDisplacement(x);
						mBodyB->ApplyAngularDisplacement(lambda * invIrBXn);
					}

					//// solve Z axis
					axis = eff_M.GetAxisZ().Normalised();
					invIrAXn = invIA_x_rAx.GetAxisZ();
					invIrBXn = invIB_x_rBx.GetAxisZ();
					effMass = eff_M.GetAxisZ().Length();

					lambda = -effMass * baumgarte * separation.Z();

					if (dyn_a)
					{
						Vec3 x = lambda * inv_massA * axis;
						mBodyA->ApplyLinearDisplacement(-x);
						mBodyA->ApplyAngularDisplacement(-lambda * invIrAXn);
					}
					if (dyn_b)
					{
						Vec3 x = lambda * inv_massB * axis;
						mBodyB->ApplyLinearDisplacement(x);
						mBodyB->ApplyAngularDisplacement(lambda * invIrBXn);
					}

				}
			}
		}

	private:
		bool RequiresPositionCorrection()
		{
			return Contains(EConstraintFlags::SolvePosition, mFlags);
		}

		Body* mBodyA = nullptr;
		Body* mBodyB = nullptr;

		/// lets say this are points in body local frame
		Vec3 mLocalAnchorA = Vec3(0.0f);
		Vec3 mLocalAnchorB = Vec3(0.0f);

		//presisent state (warm starting)
		Vec3 mAccumulatedLambda = Vec3(0.0f);

	};
} //namespace vx