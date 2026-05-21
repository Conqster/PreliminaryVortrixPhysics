#pragma once
#include "Body/BodyID.h"
#include "Maths/Vec3.h"
#include "Maths/Float3.h"

namespace vx {

	struct Linear1DRow
	{
		BodyID bodyA;	/// later change to SolverBody only caches required data 
		BodyID bodyB;	/// like position, velocities before write back, and constraint stores actual BodyID
		Vec3 axis;

		/// rAXn = rA.Cross(nor);
		Float3 angularA;	/// rAXn = rA.Cross(nor);
		///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);
		Float3 invIAngularA;	///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);

		/// rAXn = rA.Cross(nor);
		Float3 angularB;	/// rAXn = rA.Cross(nor);
		///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn
		Float3 invIAngularB;	///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);

		float effectiveMass;
		float bias;
		float lambda;

		float minLambda;
		float maxLambda;

		//for now has hack 
		class Constraint* user = nullptr;
	};
} //namespace vx