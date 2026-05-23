#pragma once
#include "SolverBodyIndex.h"
#include "Maths/Vec3.h"
#include "Maths/Float3.h"

namespace vx {

	struct Linear1DRow
	{
		SolverBodyIndex bodyAidx;	/// later change to SolverBody only caches required data 
		SolverBodyIndex bodyBidx;	/// like position, velocities before write back, and constraint stores actual BodyID

		Vec3 axis;

		/// rAXn = rA.Cross(nor);
		Float3 rAXn;	/// rAXn = rA.Cross(nor);
		///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);
		Float3 invIrAXn;	///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);

		/// rAXn = rA.Cross(nor);
		Float3 rBXn;	/// rAXn = rA.Cross(nor);
		///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn
		Float3 invIrBXn;	///invIrAXn = mBodyA->ComputeInvInteriaWorld().Multiply3x3(rAXn);

		float effMass;
		float gamma;
		float bias;
		float lambda = 0.0f;

		float minLambda;
		float maxLambda;

		//for now has hack 
		class Constraint* user = nullptr;
	};
} //namespace vx