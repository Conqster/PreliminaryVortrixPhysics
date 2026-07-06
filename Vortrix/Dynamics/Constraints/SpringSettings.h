#pragma once
#include "Vortrix/Maths/MathCommons.h"

namespace vx {

	/// used for constraint to compute the softness of constraint (complinace)
	/// follows "Soft  Constraints Reinventing The Spring - Erin Catto GDC 2011"
	/// https://box2d.org/files/ErinCatto_SoftConstraints_GDC2011.pdf
	/// Drives the implicit effective mass constribution, bias factor, 
	/// and implicit game (softness) used in Lagrange multiplier
	struct SpringSettings
	{
		/// Oscillation frequency Hz
		/// 0.0f rigid, hard constraint
		float mFrequency = 0.0f;

		/// Damping ratio, energy disipation
		/// 1.0f: critical damping, >1.0f over-damping 
		/// and <1.0 is under damping, 0.0f is not infinite bouncing
		/// has system applies explicit velocity damping, "Velocity body intergration"
		/// [0, +inf)
		float mDampingRatio = 1.0f;

		VX_INLINE bool Active() const { return mFrequency != 0.0f; }


		/// <summary>
		/// Compute implicit soft constraint properties
		/// dt simulation time step 
		/// inv_eff_mass baseline inverse effective mass of bodies
		/// C Position constraint equation error
		/// vel_bias velocity bias term
		/// o_eff_mass regularied implicit effective mass, including constraint softness
		/// o_bias total bias term 
		/// o_gamma implicit constraint softness parameter, for constraint complinace
		void ComputeProperties(float dt, float inv_eff_mass, float C,
			float vel_bias,
			float& o_eff_mass, float& o_bias, float& o_gamma) const
		{

			/// DEfault hard constraint
			float gamma = 0.0f;
			float beta = 0.0f; //this is not used in position correction

			/// Soft constraint 
			/// gamma = 1.0f / (h(hk+c)
			/// beta = hk/(hk+c)
			/// h = dt, k = stiffness, and c = damping
			if (mFrequency > 0.0f && inv_eff_mass > 0.0f)
			{
				float eff_mass = 1.0f / inv_eff_mass;

				float omega = 2.0f * kVxPi * mFrequency;

				/// harmonic oscillator 
				float k = eff_mass * VxSqr(omega);
				float c = 2.0f * eff_mass * mDampingRatio * omega;


				gamma = 1.0f / (dt * (dt * k + c));
				beta = (dt * k) / (dt * k + c);
			}

			o_eff_mass = 1.0f / (inv_eff_mass + gamma);
			//o_bias = vel_bias + (beta * C) / dt;
			o_bias = vel_bias + beta * (C / dt);
			o_gamma = gamma;
		}
	};

}//namespace vx