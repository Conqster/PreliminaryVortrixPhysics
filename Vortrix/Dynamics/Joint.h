#pragma once

#include "Body/Body.h"
#include "SampleFramework/Renderer/DebugGizmosRenderer.h"

namespace vx {

	enum class EConstraintFrame
	{
		Local,
		World
	};


	struct ConstraintRow
	{
		Float3 axis;

		float effectiveMass = 0.0f; //inv_mass + point_inv_mass(due rotation)
		float totalLamda = 0.0f;//lagrange multiplier / total accumulated impluse along axis

		Float3 r0XAxis{ 0.0f }; //relative point0 Cross axis 
		//angular factor
		Float3 invIr0XAxis{ 0.0f }; //body0 inv world inertia multiply r0XAxis

		Float3 r1XAxis{ 0.0f }; //relative point1 Cross axis 
		//angular factor
		Float3 invIr1XAxis{ 0.0f }; //body1 inv world inertia multiply r1XAxis
		float bias;
	};



	struct SolverRow
	{
		Vec3 linearA;	///-n
		Vec3 angularA;	///r0XAxis{ 0.0f }; //relative point0 Cross axis 

		Vec3 linearB;	///n
		Vec3 angularB;	///r1XAxis{ 0.0f }; //relative point1 Cross axis

		Vec3 invIangularA;	///invIr0XAxis{ 0.0f }; //body0 inv world inertia multiply r0XAxis
		Vec3 invIangularB;	///invIr1XAxis{ 0.0f }; //body1 inv world inertia multiply r1XAxis

		float effectiveMass;
		float bias;
		float lambda;
	};


	struct Linear1DRow
	{
		BodyID bodyA;	/// later change to SolverBody only caches required data 
		BodyID bodyB;	/// like position, velocities before write back, and constraint stores actual BodyID
		Vec3 axis;

		Float3 angularA;	/// rAXn = rA.Cross(nor);
		Float3 invIAngularA;	///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);

		Float3 angularB;	/// rAXn = rA.Cross(nor);
		Float3 invIAngularB;	///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);

		float effectiveMass;
		float bias;
		float lambda;

		float minLambda;
		float maxLambda;
	};


	//struct SolverRow
	//{
	//	virtual void SolveVelocity() = 0;;
	//};

	///for (const uint32* idx = idxBegin; idx < idxEnd; ++idx)
	//{
		//BaseRow* r = mRows[*idx];
		//r->SolveVelocityConstraint(inDeltaTime);
//	}

	enum class EConstraintType
	{
		Axis,
	};
	struct Stream
	{
		const void* data;
		//Func SolverFunc;
		EConstraintType type;;
	};

	class ConsrtaitSolver
	{
	public:
		template<EConstraintType Type>
		static void SolveVelocity()
		{
			if constexpr (Type == EConstraintType::Axis)
			{
				//Solve Linear Row
			}
			if constexpr (Type == EConstraintType::Point) / etc
		}
	};

	//void Solver()
	//{
	//	Stream* curr;
	//	//void* solver_row_data = &mRow[curr->data]
	//	//	ConsrtaitSolver::SolveVelocity<curr->type>(solver_row_data);


	//}


	class SolverBuilder
	{
	public:
		void AddLinearRow(Linear1DRow);
	};


	class Constraint
	{
	public:
		virtual bool PrepSolver(SolverBuilder*) = 0;
		/// essentailly used for commiting back accumulated lambda
		/// based on constraints policy
		virtual void CommitSolverState(const Linear1DRow& row) = 0;
	};

	//class DistanceConstraint : public Constraint
	//{
	//public: 
	//	bool PrepSolver(SolverBuilder* builder) override
	//	{
	//		builder->AddLinearRow(SetupJacobian());
	//		return true;//use for conditional, if ffinal builder full (preallocation avoid frame dyn alocation),
	//		//constaint  marked for removal etc. 
	//	}
	//private:
	//	Linear1DRow SetupJacobian();
	//};

	class PointConstraint : public Constraint
	{
	public:
		bool PrepSolver(SolverBuilder* builder) override;
	private:
		Linear1DRow* SetupJacobianAxisX();
		Linear1DRow* SetupJacobianAxisY();
		Linear1DRow* SetupJacobianAxisZ();

		//OR
		//Linear1DSolverRow* SetupJacobian();etc
	};

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

	class DistanceConstraint
	{
	public:
		Body* mBodyA = nullptr;
		Body* mBodyB = nullptr;


		float mRestLength;                         //[meters]
		float mStiffness;							//[constant]
		//[0.0 = none, 1.0 = critical damping]
		float mDampingRatio;							//[constant]    <-- prevent jitterness


		float minLambda = -kMaxf;// -1.0f;
		float maxLambda = kMaxf;// -1.0f;

		/// lets say this are points in body local frame
		Vec3 mLocalAnchorA = Vec3(0.0f);
		Vec3 mLocalAnchorB = Vec3(0.0f);

		//presisent state (warm starting)
		float mAccumulatedLambda = 0.0f;

		float mMinDistance;
		float mMaxDistance;

		SpringSettings mSpring;

		void Solve(float dt)
		{
			//Solve1(dt);

			//Solve2(dt);
			Solver3(dt);
		}


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


		Linear1DRow SetupDistanceJacobian(float dt)
		{
			//struct Linear1DRow
			//{
			//	BodyID bodyA;	/// later change to SolverBody only caches required data 
			//	BodyID bodyB;	/// like position, velocities before write back, and constraint stores actual BodyID
			//	Vec3 axis;
			//	float effectiveMass;
			//	float bias;
			//	float lambda;
			//};

			Linear1DRow row;

			row.bodyA = mBodyA->GetID();
			row.bodyB = mBodyB->GetID();

			//lets take into consideration that 
			// that the achor point is not COM
			Vec3 rA = mBodyA->GetOrientation().Rotate(mLocalAnchorA);
			Vec3 rB = mBodyB->GetOrientation().Rotate(mLocalAnchorB);

			Vec3 rAw = rA + mBodyA->GetPosition();
			Vec3 rBw = rB + mBodyB->GetPosition();


			Vec3 nor = (rBw - rAw).Normalised();

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

			//compute axis
			const Vec3 disp = rBw - rAw;

			const float diff = disp.Length();
			float error = diff - mRestLength; //error C

			//const Vec3 nor = disp / diff;
			row.axis = disp / diff;

			///effective mass
			float dtk = dt * mStiffness;
			float gamma = 1 / (dt * (mDampingRatio + dtk));
			row.effectiveMass = (1 / inv_eff_mass) + gamma;

			/// bias
			float beta = dtk / (mDampingRatio + dtk);
			row.bias = beta * error / dt;

			///later when figure out, caching implmentation for warm start etc
			row.lambda = 0.0f;

			row.minLambda = minLambda;
			row.maxLambda = maxLambda;

			return row;
		}


		void Solver3(float dt)
		{
			if (!mBodyA || !mBodyB)
				return;


			//Linear1DRow solver_row = SetupDistanceJacobian(dt);
			Linear1DRow solver_row = SetupDistanceJacobian2(dt);

			if (solver_row.effectiveMass <= 0.0f)
				return;

			Body& bodyA = *mBodyA; //context.BodyManager().GetBody(solver_row.bodyA)
			Body& bodyB = *mBodyB;

			bool dyn_a = bodyA.IsDynamic();
			bool dyn_b = bodyB.IsDynamic();

			//if (bodyA.IsStatic() && bodyB.IsStatic())
			//	return;


			///this is the work of solver body and not direct for Body
			Vec3 lin_velA = bodyA.GetLinearVelocity();
			Vec3 lin_velB = bodyB.GetLinearVelocity();
			Vec3 ang_velA = bodyA.GetAngularVelocity();
			Vec3 ang_velB = bodyB.GetAngularVelocity();
			float inv_massA = bodyA.GetInverseMass();
			float inv_massB = bodyB.GetInverseMass();

			///i.e 
			//struct SolverBody
			//{
			//	Vec3 linearVel;
			//	Vec3 angularVel;
			//	float invMass;
			//	EMotionType motionType = EMotionType::Dynamic;
			//};

			///// solver pair or just Linear1DRow etc have id/pointer to their pair
			///// could have a constraint with more than 2 bodies etc
			//struct SolverPair
			//{
			//	SolverBody bodyA;
			//	SolverBody bodyB;
			//};


			//load data
			Vec3 rAXn = Vec3::LoadFloat3Raw(solver_row.angularA);
			Vec3 invIrAXn = Vec3::LoadFloat3Raw(solver_row.invIAngularA);

			Vec3 rBXn = Vec3::LoadFloat3Raw(solver_row.angularB);
			Vec3 invIrBXn = Vec3::LoadFloat3Raw(solver_row.invIAngularB);


			//then later in constraint solver 
			for (int i = 0; i < 20; ++i)
			{
				//Vec3 lin_velA = bodyA.GetLinearVelocity();
				//Vec3 lin_velB = bodyB.GetLinearVelocity();

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

				///maybe later, has its almost the same code
				//if constexpr(ESolverRow::AngularPart)
				//{
				//	if (dyn_a)
				//		jv += r0XAxis.Dot(ang_vel0);
				//	if (dyn_b)
				//		jv -= r1XAxis.Dot(ang_vel1);
				//}

				/// -K^-1(Jv + b)
				/// -K^-1((1-e)Jv)
				/// nor_axis_contraint.effectiveMass = 1/inv effective mass
				//float lambda = (nor_axis_contraint.bias - jn) * nor_axis_contraint.effectiveMass;

				//float actual_bias = 
				float lambda = (jv - solver_row.bias) * solver_row.effectiveMass;

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

			}
			//write back to body 
			bodyA.SetLinearVelocity(lin_velA);
			bodyA.SetAngularVelocity(ang_velA);

			bodyB.SetLinearVelocity(lin_velB);
			bodyB.SetAngularVelocity(ang_velB);
		}


		void Solve2(float dt)
		{
			if (!mBodyA || !mBodyB)
				return;

			auto& bodyA = *mBodyA;
			auto& bodyB = *mBodyB;

			//opt out if both particle has infinte masses
			if (bodyA.IsStatic() && bodyB.IsStatic())
				return;

			const Vec3 disp = bodyB.GetPosition() - bodyA.GetPosition();

			const float diff = disp.Length();
			float error = diff - mRestLength; //error C

			const Vec3 nor = disp / diff;

			float dtk = dt * mStiffness;
			float beta = dtk / (mDampingRatio + dtk);

			float gamma = 1 / (dt * (mDampingRatio + dtk));

			float bias = beta * error / dt;








			///constraint normal
			Vec3 n = nor;


			bool dyn_a = bodyA.IsDynamic();
			bool dyn_b = bodyB.IsDynamic();


			Vec3 r0XAxis;
			Vec3 r1XAxis;

			//float bias = 1.0f;
			float totalLamda = 0.0f;

			float invMass0 = bodyA.GetInverseMass();
			float invMass1 = bodyB.GetInverseMass();

			float effectiveMass = invMass0 + invMass1 + gamma;

			Vec3 invIr0XAxis;
			Vec3 invIr1XAxis;


			//then later in constraint solver 
			for (int i = 0; i < 10; ++i)
			{
				Vec3 lin_vel0 = bodyA.GetLinearVelocity();
				Vec3 lin_vel1 = bodyB.GetLinearVelocity();

				Vec3 ang_vel0 = bodyA.GetAngularVelocity();
				Vec3 ang_vel1 = bodyB.GetAngularVelocity();


				//jacobian 
				float jn;
				if (dyn_a && dyn_b)
					jn = (lin_vel0 - lin_vel1).Dot(n);
				else if (dyn_a)
					jn = lin_vel0.Dot(n);
				else if (dyn_b)
					jn = (-lin_vel1).Dot(n);
				else
				{
					VX_LOG_ERROR("Static vs static this should not be possible");
					jn = 0.0f;
				}
				//simplify 
				//if (dyn_a)
				//	jn += r0XAxis.Dot(ang_vel0);
				//if (dyn_b)
				//	jn -= r1XAxis.Dot(ang_vel1);

				/// -K^-1(Jv + b)
				/// -K^-1((1-e)Jv)
				/// nor_axis_contraint.effectiveMass = 1/inv effective mass
				//float lambda = (nor_axis_contraint.bias - jn) * nor_axis_contraint.effectiveMass;

				//float actual_bias = 
				float lambda = (jn - bias) * effectiveMass;

				float old_lambda = totalLamda;
				//ensure non negative
				//bilateral constraint
				totalLamda += lambda;
				totalLamda = VxClamp(old_lambda + lambda, minLambda, maxLambda);
				//updated jn
				float impluse = totalLamda - old_lambda;

				//store changes
				if (dyn_a)
				{
					lin_vel0 -= impluse * invMass0 * n;
					//ang_vel0 -= impluse * invIr0XAxis;
				}
				if (dyn_b)
				{
					lin_vel1 += impluse * invMass1 * n;
					///ang_vel1 += impluse * invIr1XAxis;
				}

				bodyA.SetLinearVelocity(lin_vel0);
				bodyB.SetLinearVelocity(lin_vel1);
			}

		}


		void Solve1(float /*time_step*/)
		{
			if (!mBodyA || !mBodyB)
				return;

			auto& bodyA = *mBodyA;
			auto& bodyB = *mBodyB;

			//opt out if both particle has infinte masses
			if (bodyA.IsStatic() && bodyB.IsStatic())
				return;

			const Vec3 displacement = bodyB.GetPosition() - bodyA.GetPosition();
			const float length = displacement.Length();

			//opt out is distance/length is too small
			if (length < kEpsilon)
				return;

			const Vec3 dir = displacement / length;


			//Hookes Law F = -k * (X - X_rest)
			//where k = stiffness, X = curr position, X_rest = rest displacement
			// according:https://en.wikipedia.org/wiki/Hooke%27s_law
			//F & X are vector discribing a displacement in space
			//in this case f = -k * (x - x_rest)
			// f = Fmag => spring force magnitude, x = current length, x_rest = @rest length
			const float spring_mag = -mStiffness * (length - mRestLength);


			//effective mass along axis 
			// massA x massB / massA + mass B
			const float mA = 1.0f / bodyA.GetInverseMass();
			const float mB = 1.0f / bodyB.GetInverseMass();
			const float mass_eff = (mA * mB) / (mA + mB);
			//critical dampling 
			const float crit_damping = 4.0f * VxSqrt(mStiffness * mass_eff);
			const float damping_coeff = mDampingRatio * crit_damping;
			//damping dampingforce = -dampling constant * relative velocity
			const Vec3 relative_velocity = bodyB.GetLinearVelocity() - bodyA.GetLinearVelocity();
			const float damping_mag = -damping_coeff * relative_velocity.Dot(dir);


			//accomodating the displacement vector properties as both particle could be in motion
			//from F = -k * (X - X_rest)
			//displacement dir * total force i.e F = f(Fmag) * displacement
			const Vec3 force = dir * (spring_mag + damping_mag);

			if (bodyA.IsDynamic())
				bodyA.AddForce(-force);

			if (bodyB.IsDynamic())
				bodyB.AddForce(force);
		}

		void DebugGizmos2(DebugGizmosRenderer* debug_renderer) const
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

				Vec3 nor = (rBw - rAw).Normalised();
				//Vec3 nor = (mBodyB->GetPosition() - mBodyA->GetPosition()).Normalised();


				debug_renderer->DrawLine(rAw, rBw, Colour(0.3f, 0.3f, 0.3f));

				debug_renderer->DrawLine(rAw, (rAw + (nor * static_cast<float>(mRestLength * ratio))), Colour(1.0f, 1.0f, 0.0f));
				debug_renderer->DrawLine(rBw, (rBw + (-nor * static_cast<float>(mRestLength * (1 - ratio)))), Colour(0.0f, 1.0f, 0.0f));
			}
		}


		void DebugGizmos(DebugGizmosRenderer* debug_renderer) const
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

					if constexpr(k_draw_box)
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
	};

}
