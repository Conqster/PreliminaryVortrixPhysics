#pragma once

#include "Body/Body.h"
#include "SampleFramework/Renderer/DebugGizmosRenderer.h"

namespace vx {

	enum class EConstraintFrame
	{
		Local,
		Space
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


	//void ComputeJointFrame()
	//{

	//}

	enum class ProjectMode
	{
		None,
		ClampZero,
		ClampRange
	};

	class Joint
	{
	public:
		Body* mBodyA = nullptr;
		Body* mBodyB = nullptr;


		float mRestLength;                         //[meters]
		float mStiffness;							//[constant]
		//[0.0 = none, 1.0 = critical damping]
		float mDampingRatio;							//[constant]    <-- prevent jitterness


		float minLambda = -kInf;// -1.0f;
		float maxLambda = kInf;// -1.0f;

		void Solve(float dt)
		{
			//Solve1(dt);

			Solve2(dt);
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
			for(int i =0;i<10; ++i)
			{
				Vec3 lin_vel0 = bodyA.GetLinearVelocity();
				Vec3 lin_vel1 = bodyB.GetLinearVelocity();

				Vec3 ang_vel0 = bodyA.GetAngularVelocity();
				Vec3 ang_vel1 = bodyB.GetAngularVelocity();

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
				if (dyn_a)
					jn += r0XAxis.Dot(ang_vel0);
				if (dyn_b)
					jn -= r1XAxis.Dot(ang_vel1);

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
			const float mA = 1.0f/bodyA.GetInverseMass();
			const float mB = 1.0f/bodyB.GetInverseMass();
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

		void DebugGizmos(DebugGizmosRenderer* debug_renderer) const
		{
			if (debug_renderer && mBodyA && mBodyB)
			{
				debug_renderer->DrawLine(mBodyA->GetPosition(), mBodyB->GetPosition(), Colour(0.3f, 0.3f, 0.3f));

				const float mA = 1.0f / mBodyA->GetInverseMass();
				const float mB = 1.0f / mBodyB->GetInverseMass();
				//split rest length based on mass contribution 
				float total_mass = mA + mB;
				//distribute force if total mass is not too small else split
				float ratio = (total_mass > kEpsilon) ? (mB / total_mass) : 0.5;

				Vec3 dir = (mBodyB->GetPosition() - mBodyA->GetPosition()).Normalised();
				debug_renderer->DrawLine(mBodyA->GetPosition(), (mBodyA->GetPosition() + (dir * static_cast<float>(mRestLength * ratio))), Colour(1.0f, 1.0f, 0.0f));
				debug_renderer->DrawLine(mBodyB->GetPosition(), (mBodyB->GetPosition() + (-dir * static_cast<float>(mRestLength * (1 - ratio)))), Colour(0.0f, 1.0f, 0.0f));
			}
		}
	};

}
