#pragma once
#include "SolverBodyIndex.h"
#include "Vortrix/Maths/Vec3.h"
#include "Vortrix/Maths/Float3.h"

#include "Body/Body.h"

namespace vx {


	struct alignas(16) Linear1DRow
	{
		SolverBodyIndex bodyAidx;	/// later change to SolverBody only caches required data 
		SolverBodyIndex bodyBidx;	/// like position, velocities before write back, and constraint stores actual BodyID

		float effMass;
		float gamma;

		Float3 axis;
		float bias;

		/// rAXn = rA.Cross(nor);
		Float3 rAXn;
		float lambda;
		
		///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);
		Float3 invIrAXn;
		float minLambda;

		/// rAXn = rA.Cross(nor);
		Float3 rBXn;	/// rAXn = rA.Cross(nor);
		float maxLambda;
		
		///invIrBXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn
		Float3 invIrBXn;

		//for now has hack 
		class Constraint* user;

		//later have idx part as a single constraint have multiple rows 
		uint32 hackIdx;
	};
	static_assert(std::is_trivial_v<Linear1DRow>, "Linear1DRow Must be a trivial type!");

	struct Rigid1DConstraint
	{
		Vec3 axis;

		///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);
		Float3 invIrAXn;	

		///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn
		Float3 invIrBXn;


		float effMass;

		VX_INLINE void SolvePosition(Body& body0, Body& body1, float C, float baumgarte)
		{
			float lambda = -effMass * baumgarte * C;

			if (body0.IsDynamic())
			{
				Vec3 x = lambda * body0.GetInverseMass() * axis;
				body0.ApplyLinearDisplacement(-x);
				body0.ApplyAngularDisplacement(-lambda * Vec3::LoadFloat3Raw(invIrAXn));
			}
			if (body1.IsDynamic())
			{
				Vec3 x = lambda * body1.GetInverseMass() * axis;
				body1.ApplyLinearDisplacement(x);
				body1.ApplyAngularDisplacement(lambda * Vec3::LoadFloat3Raw(invIrBXn));
			}
		}
	};



} //namespace vx