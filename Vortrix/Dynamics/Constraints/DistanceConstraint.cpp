#include "DistanceConstraint.h"
#include "PhysicsWorldSettings.h"
#include "Core/Profiler.h"

#include "Dynamics/ConstraintSolver.h"


namespace vx{

	DistanceConstraint::DistanceConstraint(Body* bodyA, Body* bodyB, const DistanceConstraintSettings& settings) :
		mBodyA(bodyA), mBodyB(bodyB), mLocalAnchorA(settings.localAnchorA),
		mLocalAnchorB(settings.localAnchorB), mMinDistance(settings.minDist),
		mMaxDistance(settings.maxDist),
		mSpring({ settings.frequency, settings.dampingRatio })
	{
		
		mFlags = EConstraintFlags::SolveVelocity;// | ~EConstraintFlags::SolvePosition;
		
		if(mSpring.mFrequency <= 0.0f)
			mFlags |= EConstraintFlags::SolvePosition;
	}
	bool DistanceConstraint::PrepSolver(ConstraintSolver* solver, const PhysicsStepContext& ctx)
	{
		VX_PROFILE_FUNCTION();
		mFlags |= EConstraintFlags::Active;

		bool active = (mBodyA->IsAwake() || mBodyB->IsAwake()) && (mBodyA->IsDynamic() || mBodyB->IsDynamic());
		if (!active)
		{
			mFlags &= ~EConstraintFlags::Active;
			return false;
		}

		Linear1DRow row = BuildDistanceJacobian(ctx.stepDeltaTime);

		if (row.effMass == 0.0f)
		{
			mFlags &= ~EConstraintFlags::Active;
			return false;
		}

		row.bodyAidx = solver->GetOrCreateSolverBody(mBodyA->GetID(), ctx);
		row.bodyBidx = solver->GetOrCreateSolverBody(mBodyB->GetID(), ctx);

		if(RequiresPositionCorrection())
			solver->AppendPositionCorrectionQueue(this);// Queue

		row.user = this;

		solver->AddLinearRow(row);
		return true;
	}


	Vec3 DistanceConstraint::ComputeConstraintPropertiesDisplacement(Vec3& o_rA, Vec3& o_rB)
	{
		//lets take into consideration that 
		// that the achor point is not COM
		o_rA = mBodyA->GetOrientation().Rotate(mLocalAnchorA);
		o_rB = mBodyB->GetOrientation().Rotate(mLocalAnchorB);

		mWorldAnchorA = o_rA + mBodyA->GetPosition();
		mWorldAnchorB = o_rB + mBodyB->GetPosition();


		Vec3 dispW = mWorldAnchorB - mWorldAnchorA;
		Vec3 nor = dispW.Normalised();
		mWorldAxis = nor;

		return dispW;
	}

	Linear1DRow DistanceConstraint::BuildDistanceJacobian(float dt)
	{
		Linear1DRow row;

		//lets take into consideration that 
		// that the achor point is not COM
		Vec3 rA, rB;
		Vec3 dispW = ComputeConstraintPropertiesDisplacement(rA, rB);
		Vec3 nor = dispW.Normalised();
		nor.Store(row.axis);

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

		if (!bodyA_nonstatic && !bodyB_nonstatic)
		{
			row.effMass = 0.0f;
			row.lambda = 0.0f;
			return row;
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
			row.lambda = 0.0f;
			return row;
		}


		mSpring.ComputeProperties(dt, inv_eff_mass, error, 0.0f, row.effMass, row.bias, row.gamma);

		///later when figure out, caching implmentation for warm start etc
		row.lambda = mAccumulatedLambda;
		return row;
	}


	Rigid1DConstraint DistanceConstraint::BuildRigidConstraint()
	{
		Rigid1DConstraint rigid_constraint;

		//lets take into consideration that 
		// that the achor point is not COM
		Vec3 rA, rB;
		Vec3 dispW = ComputeConstraintPropertiesDisplacement(rA, rB);
		Vec3 nor = dispW.Normalised();
		rigid_constraint.axis = nor;

		bool bodyA_nonstatic = !mBodyA->IsStatic();
		bool bodyB_nonstatic = !mBodyB->IsStatic();


		float inv_eff_mass = 0.0f;
		if (bodyA_nonstatic)
		{
			Vec3 rAXn = rA.Cross(nor);
			Vec3 invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);
			invIrAXn.Store(rigid_constraint.invIrAXn);

			inv_eff_mass += mBodyA->GetInverseMass() + invIrAXn.Dot(rAXn);
		}

		if (bodyB_nonstatic)
		{
			Vec3 rBXn = rB.Cross(nor);
			Vec3 invIrBXn = mBodyB->ComputeInvInteriaWorld().Multiply3x3(rBXn);

			invIrBXn.Store(rigid_constraint.invIrBXn);
			inv_eff_mass += mBodyB->GetInverseMass() + invIrBXn.Dot(rBXn);
		}

		if (!bodyA_nonstatic && !bodyB_nonstatic)
		{
			rigid_constraint.effMass = 0.0f;
			return rigid_constraint;
		}


		rigid_constraint.effMass = 1.0f / inv_eff_mass;
		return rigid_constraint;
	}


	void DistanceConstraint::SolvePositionConstraint(float dt, float baumgarte)
	{
		///check policy
		if (!mSpring.Active())
		{
			float distance = (mWorldAnchorB - mWorldAnchorA).Dot(mWorldAxis);


			float error = 0.0f;
			if (distance < mMinDistance)
				error = distance - mMinDistance;
			else if (distance > mMaxDistance)
				error = distance - mMaxDistance;

			if (error != 0)
			{
				Rigid1DConstraint constraint = BuildRigidConstraint();
				constraint.SolvePosition(*mBodyA, *mBodyB, error, baumgarte);
			}
		}
	}

	void DistanceConstraint::DrawConstraintBounds(DebugGizmosRenderer* debug_renderer, const Vec3& rAw, const Vec3& rBw, Colour col) const
	{
		//achorbound a
		AABB boundA = AABB(0.25);
		boundA.Translate(rAw);

		AABB boundB = AABB(0.25);
		boundB.Translate(rBw);

		boundA.Merge(boundB);
		debug_renderer->DrawAABB(boundA, col);
	}

	void DistanceConstraint::DebugGizmos(DebugGizmosRenderer* debug_renderer, const NonContactConstraintDrawSettings& draw_settings) const
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

			if (draw_settings.drawConstraintBounds)
				DrawConstraintBounds(debug_renderer, rAw, rBw, Colour::sDeepTeal);

			if (draw_settings.drawActiveBounds && Contains(EConstraintFlags::Active, mFlags))
				DrawConstraintBounds(debug_renderer, rAw, rBw, Colour::sOrange);
	
			if (draw_settings.drawVelocitySolveBounds && Contains(EConstraintFlags::SolveVelocity, mFlags))
				DrawConstraintBounds(debug_renderer, rAw, rBw, Colour::sTurquoise);
			if (draw_settings.drawPositionSolveBounds && Contains(EConstraintFlags::SolvePosition, mFlags))
				DrawConstraintBounds(debug_renderer, rAw, rBw, Colour::sCyan);


			//if(Contains(EConstraintFlags::Active, mFlags))
			////if(Contains(EConstraintFlags::SolvePosition, mFlags))
			//	DrawConstraintBounds(debug_renderer, rAw, rBw);

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

	void DistanceConstraint::QuickSolve(float dt)
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


		Vec3 axis = Vec3::LoadFloat3Raw(solver_row.axis);

		//jacobian 
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
			lin_velA -= impluse * inv_massA * axis;
			ang_velA -= impluse * invIrAXn;
		}
		if (dyn_b)
		{
			lin_velB += impluse * inv_massB * axis;
			ang_velB += impluse * invIrBXn;
		}

		//write back to body 
		bodyA.SetLinearVelocity(lin_velA);
		bodyA.SetAngularVelocity(ang_velA);

		bodyB.SetLinearVelocity(lin_velB);
		bodyB.SetAngularVelocity(ang_velB);
	}

} //namespace vx
