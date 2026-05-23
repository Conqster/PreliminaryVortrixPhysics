#pragma once
#include "SolverBodyIndex.h"
#include "Maths/Vec3.h"
#include "Maths/Float3.h"

namespace vx {

	struct Linear1DRow
	{
		SolverBodyIndex bodyAidx;	/// later change to SolverBody only caches required data 
		SolverBodyIndex bodyBidx;	/// like position, velocities before write back, and constraint stores actual BodyID

		float effMass;
		float gamma;

		Vec3 axis;

		/// rAXn = rA.Cross(nor);
		Float3 rAXn;	/// rAXn = rA.Cross(nor);
		float bias;
		
		///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);
		Float3 invIrAXn;	///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);
		float lambda = 0.0f;

		/// rAXn = rA.Cross(nor);
		Float3 rBXn;	/// rAXn = rA.Cross(nor);
		float minLambda = 0.0f;
		
		///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn
		Float3 invIrBXn;	///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);


		float maxLambda = 0.0f;

		//for now has hack 
		class Constraint* user = nullptr;
	};


	struct Rigid1DConstraint
	{
		Vec3 axis;

		///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);
		Float3 invIrAXn;	

		///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn
		Float3 invIrBXn;	///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);


		float effMass;
	};

	static void SolveRigid1DPosition(Body& body0, Body& body1, const Rigid1DConstraint& rigid_constraint, float C, float baumgarte)
	{
		float lambda = -rigid_constraint.effMass * baumgarte * C;

		if (body0.IsDynamic())
		{
			Vec3 x = lambda * body0.GetInverseMass() * rigid_constraint.axis;
			body0.ApplyLinearDisplacement(-x);
			body0.ApplyAngularDisplacement(-lambda * Vec3::LoadFloat3Raw(rigid_constraint.invIrAXn));
		}
		if (body1.IsDynamic())
		{
			Vec3 x = lambda * body1.GetInverseMass() * rigid_constraint.axis;
			body1.ApplyLinearDisplacement(x);
			body1.ApplyAngularDisplacement(lambda * Vec3::LoadFloat3Raw(rigid_constraint.invIrBXn));
		}
	}

} //namespace vx