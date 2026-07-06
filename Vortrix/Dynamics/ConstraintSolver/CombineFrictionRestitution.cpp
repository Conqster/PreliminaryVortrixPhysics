#include "CombineFrictionRestitution.h"
#include "Vortrix/Dynamics/Body/Body.h"


namespace vx {


	float CombinedCoefficient::GetFriction(ECombineMode Mode, const Body& body0, const Body& body1)
	{
		const float f0 = body0.GetFriction();
		const float f1 = body1.GetFriction();

		VX_ASSERT(!VxIsNaN(f0) || !VxIsNaN(f1), "either bodies friction co-effiecent is nan");

		switch (Mode)
		{
		case vx::ECombineMode::Multiply: return f0 * f1;
		case vx::ECombineMode::SquareRoot: return VxSqrt(f0 * f1);
		case vx::ECombineMode::Average: return (f0 + f1) * 0.5f;
		case vx::ECombineMode::Minimum: return VxMin(f0, f1);
		case vx::ECombineMode::Maximum: return VxMax(f0, f1);

		default:
			VX_LOG_WARN("Custom Friction Combine coeffient not supported yet; defaulting to Average");
			return (f0 + f1) * 0.5f;
		}
	}

	float CombinedCoefficient::GetRestitution(ECombineMode Mode, const Body& body0, const Body& body1)
	{
		const float r0 = body0.GetRestitution();
		const float r1 = body1.GetRestitution();

		VX_ASSERT(!VxIsNaN(r0) && !VxIsNaN(r1), "either bodies friction co-effiecent is nan");

		switch (Mode)
		{
		case vx::ECombineMode::Multiply: return r0 * r1;
		case vx::ECombineMode::SquareRoot: return VxSqrt(r0 * r1);
		case vx::ECombineMode::Average: return (r0 + r1) * 0.5f;
		case vx::ECombineMode::Minimum: return VxMin(r0, r1);
		case vx::ECombineMode::Maximum: return VxMax(r0, r1);

		default:
			VX_LOG_WARN("Custom Restitution Combine coeffient not supported yet; defaulting to Average");
			return (r0 + r1) * 0.5f;
		}
	}
}