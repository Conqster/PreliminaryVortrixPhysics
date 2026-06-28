#include "Body.h"

#include "Core/Profiler.h"
#include "Collision/Shapes/Shape.h"

#include "PhysicsWorld.h"

namespace vx
{
	Body::Body(Vec3 pos) : 
		mPosition(pos), 
		mOrientation(Quat::Identity()),
		mMotionState({}),
		mFriction(0.4f), mRestitution(0.4f)
	{}
	void Body::ApplyImpulse(const Vec3& impluse)
	{
		if (IsDynamic())
			mLinearVelocity += impluse * mInverseMass;
	}
	void Body::ApplyImpulse(const Vec3& impluse, const Vec3& pointA)
	{
		if (!IsDynamic()) return;

		const Vec3 r = pointA - mPosition;
		ApplyImpulseLocal(impluse, r);
	}

	void Body::ApplyImpulseLocal(const Vec3& impluse, const Vec3& pointA)
	{
		if (!IsDynamic()) return;

		mLinearVelocity += impluse * mInverseMass;

		//the torque vector (r x F) points along the axis 
		// which the body tends to rotate
		// r x F 
		// givens
		// magnitude -> torque proportional to the 
		// perpendicular distnce from the axis (body centr mass).
		// and direction -> axis of rotation.
		const Vec3 torque_world = pointA.Cross(impluse);

		//Vec3 test = ApplyInvInertiaTensorWorld(torque_world);
		//Vec3 test2 = GetInvInteriaWorld().Multiply3x3(torque_world);
		mAngularVelocity += ApplyInvInertiaTensorWorld(torque_world);
	}
	void Body::ApplyAngularImpulse(const Vec3& impluse, const Vec3& pointA)
	{
		const Vec3 r = pointA - mPosition;
		//the torque vector (r x F) points along the axis 
		// which the body tends to rotate
		// r x F 
		// givens
		// magnitude -> torque proportional to the 
		// perpendicular distnce from the axis (body centr mass).
		// and direction -> axis of rotation.
		const Vec3 torque_world = r.Cross(impluse);
		mAngularVelocity += ApplyInvInertiaTensorWorld(torque_world);
	}
	void Body::ApplyPositionCorrection(const Vec3& nudge, const Vec3& r)
	{
		if (!IsDynamic())
			return;
		//linear
		mPosition += nudge * mInverseMass;

		//return;
		///theta  = I^-1 (rxp)
		/// r lever arm 
		/// p = displacement implse
		//Vec3 ang_delta = mInverseInertiaTensor.Multiply3x3(lever_arm.Cross(displacement_impluse)); < old Mat44 method

		Vec3 rotation_axis_ws = r.Cross(nudge);
		//Vec3 torque = mRotation.Conjugated().Rotate(torque_world);
		Vec3 rotation_axis_ls = mOrientation.InverseRotate(rotation_axis_ws);


		Vec3 angular_disp_ls = rotation_axis_ls * mInvInertiaTensorDiagonal; //nudge
		Vec3 angular_disp_ws = mOrientation.Rotate(angular_disp_ls);

		ApplyAngularDisplacement(angular_disp_ws);
	}
	void Body::AddForce(const Vec3& force)
	{
		(Vec3::LoadFloat3Raw(mForceAccumulated) + force).Store(mForceAccumulated);
	}
	void Body::AddForce(const Vec3& force, const Vec3& pointA)
	{
		(Vec3::LoadFloat3Raw(mForceAccumulated) + force).Store(mForceAccumulated);
		//mForceAccumulated += force;
		Vec3 r = pointA - mPosition;
		//the torque vector (r x F) points along the axis 
		// which the body tends to rotate
		// r x F 
		// givens
		// magnitude -> torque proportional to the 
		// perpendicular distnce from the axis (body centr mass).
		// and direction -> axis of rotation.
		(Vec3::LoadFloat3Raw(mTorqueAccumulated) + r.Cross(force)).Store(mTorqueAccumulated);
	}


	void Body::SetMotionType(EMotionType type)
	{
		mMotionType = type;

		mLinearVelocity = Vec3::Zero();
		mAngularVelocity = Vec3::Zero();

		ClearAccumulatedForces();
	}

	void Body::ApplyGravity(const Vec3& gravity)
	{
		if (!IsDynamic()) return;

		(Vec3::LoadFloat3Raw(mForceAccumulated) + gravity * GetMass()).Store(mForceAccumulated);
		//mForceAccumulated += gravity * GetMass();
	}



	void Body::IntegrateAcceleration(float dt, const Vec3& gravity)
	{
		if (!IsDynamic())
			return;


		VX_ASSERT(!VxIsNaN(mInverseMass) || mInverseMass == 0.0f, "inverse mass is nan");
		VX_ASSERT(!mAngularVelocity.IsNaN(), "Linear velocituy is nan");
		VX_ASSERT(!Vec3::LoadFloat3Raw(mForceAccumulated).IsNaN(), "Accumulated force is nan");
		VX_ASSERT(!Vec3::LoadFloat3Raw(mTorqueAccumulated).IsNaN(), "Accumulated torque is nan");

		mLinearVelocity += (gravity + (Vec3::LoadFloat3Raw(mForceAccumulated) * mInverseMass)) * dt;
		mAngularVelocity += ApplyInvInertiaTensorWorld(Vec3::LoadFloat3Raw(mTorqueAccumulated)) * dt;

		DampVelocities(dt);
		ClampVelocities();
	}
	void Body::IntegrateVelocity(float dt)
	{
		ClampVelocities(); //hack if velocity constraint solve was too strong
		DampVelocities(dt);

   		VX_ASSERT(!mLinearVelocity.IsNaN(), "Linear velocituy is nan");
		VX_ASSERT(!mAngularVelocity.IsNaN(), "Angular velocituy is nan");

		mPosition += mLinearVelocity * dt;

		//sympletic
		//Quat dq = 0.5f * Quat(mAngularVelocity, 0.0f) * mRotation;
		//mRotation += dq * dt;
		//mRotation.Normalise();

		ApplyAngularDisplacement(mAngularVelocity * dt);

		ComputeWorldSpaceBoundsInternal();
	}
	inline void Body::DampVelocities(float dt)
	{
		//Newcastle university ncl damping lecture note
		//mLinearVelocity *= VxPow(1.0f - mLinearDamping, dt);
		//mAngularVelocity *= VxPow(1.0f - mAngularDamping, dt);

		//pow is expensive 
		//taylor expension 
		mLinearVelocity *= VxMax(0.0f, 1.0f - mLinearDamping * dt);
		mAngularVelocity *= VxMax(0.0f, 1.0f - mAngularDamping * dt);

	}
	inline void Body::ClampVelocities()
	{
		{
			float speed_sq = mLinearVelocity.LengthSq();
			if (speed_sq > mMaxLinearVelocity * mMaxLinearVelocity)
				mLinearVelocity *= (mMaxLinearVelocity / VxSqrt(speed_sq));
		}
		
		float ang_speed_sq = mAngularVelocity.LengthSq();
		if (ang_speed_sq > mMaxAngularVelocity * mMaxAngularVelocity)
			mAngularVelocity *= (mMaxAngularVelocity / VxSqrt(ang_speed_sq));
	}
	void Body::ApplyAngularDisplacement(const Vec3& disp)
	{
		float angle = disp.Length();
		if (angle < 1e-6f)
		{
			//taylor
			mOrientation = Quat(disp * 0.5f, 1.0f) * mOrientation;
			//mRotation = Quat(disp * 0.5f, 1.0f-(angle*angle*0.125f)) * mRotation;
		}
		else
		{
			Quat dq = Quat::FromAxisAngle(disp / angle, angle);
			mOrientation = dq * mOrientation;
		}
		mOrientation.Normalise();
	}
	bool Body::CanBodiesCollide(const Body& b0, const Body& b1)
	{
		VX_ASSERT(&b1 != &b0, "b0 & b1 are of the same object should not try to collide!!!.");


		const bool b0_can_move = b0.IsDynamic() && b0.IsAwake();
		const bool b1_can_move = b1.IsDynamic() && b1.IsAwake();

		if (!b0_can_move && !b1_can_move)
			return false;

		return true;


		//if (!b0.IsDynamic() && !b1.IsDynamic())
		//	return false;

		//if (!b0.IsDynamic() || (b0.IsDynamic() && b0.IsSleeping()))
		//	return false;


		///// bodies can't collide if below 
		///// 1. both bodies are dynamic but sleeping
		///// 2. b0 dynamic and sleeping but b1 is static 
		///// 3. b1 dynamic and sleeping but b0 is static
		//if (b0.IsDynamic() && b0.IsSleeping() && b1.IsDynamic() && b1.IsSleeping() ||
		//	(b0.IsDynamic() && b0.IsSleeping() && !b1.IsDynamic()) ||
		//	(b1.IsDynamic() && b1.IsSleeping() && !b0.IsDynamic()))
		//	return false;

		//return true;
	}


	void Body::UpdateSleepState(float dt, const SleepingSettings& settings)
	{
		auto reset_sleep = [this]() {
			mSleepTimer = 0.0f;
			mAwake = true;
		};

		//later remove this check 
		if (!IsDynamic()) return;
		//{
		//	reset_sleep();
		//	return;
		//}

		float linear_speed_sq = mLinearVelocity.LengthSq();
		float angular_speed_sq = mAngularVelocity.LengthSq();

		if (linear_speed_sq > settings.velocityThreshold * settings.velocityThreshold ||
			angular_speed_sq > settings.angularThreshold * settings.angularThreshold)
		{
			reset_sleep();
			return;
		}

		mSleepTimer += dt;
		if (mSleepTimer > settings.timeThreshold)
		{
			mAwake = false;
			mLinearVelocity = Vec3::Zero();
			mAngularVelocity = Vec3::Zero();
		}
		
	}
	void Body::SetWorldTransform(const Mat44& mat)
	{
		//extract rotation and position 
		//extract scale 
		Vec3 scale;
		Mat44 decompose = mat.Decompose(scale);
		mPosition = decompose.GetTranslation();
		mOrientation = decompose.GetRotationQuat();
		mOrientation.Normalise();
		ComputeWorldSpaceBoundsInternal();
	}
	void Body::SetShape(const RefConst<Shape>& shape, bool update_mass_inertia)
	{
		VX_ASSERT_WARN_VOID(shape != nullptr, "Trying to set body shape with null shape");
		mShape = shape;

		if (update_mass_inertia)
		{
			/// we are not checking if dynamic / static 
			/// manager/interface should ignore update 
			MassProperties mp = mShape->GetMassProperties();

			SetMass(mp.mass);
			SetInertiaTensor(Vec3::LoadFloat3Raw(mp.inertialTensorDiagonal));
		}

		ComputeWorldSpaceBoundsInternal();
	}
	Mat44 Body::TransformDiagonalInertiaTensor(const Vec3& inv_inertia_diagonal, const Quat& rot) const
	{
		//return Mat44::Identity();
		// I_world^-1 = R * I_body^-1 * R^T
		// I_world^-1: world inverse inertia, to transform angualr momentum
		// to angular velocity in world space
		// R: rotation matrix (orientation)
		// R^T: transposed rotation matrix (orientation)
		// I_body^-1: inverse inertia tensor in body space.

		//inverse inertia tensor in world space
		Mat44 R = Mat44::Rotation(rot);
		//return R * inv_inertia_body * R.Transposed3x3();
		//return R.MultiplyAffine(inv_inertia_body).MultiplyAffine(R.Transposed3x3());


		Mat44 I_world;
		//compute R * I_body_inv
		I_world = R.Multiply3x3(Mat44::Scale(inv_inertia_diagonal));

		//I_world = Mat44(
		//	R.GetColumn(0) * mInvInertiaTensorDiagonal.X(),
		//	R.GetColumn(1) * mInvInertiaTensorDiagonal.Y(),
		//	R.GetColumn(2) * mInvInertiaTensorDiagonal.Z());

		//compute (R* Ibody^-1) * R^T
		I_world = I_world.Multiply3x3RightTransposed(R);

		//to ensure 4 column is not used accidenatly with operations
		//I_world.SetTranslation(Vec3::Zero(), 0.0f);
		return I_world;




		//Mat44 R = Mat44::Rotation(mOrientation);
		//Mat44 I_world;
		//I_world = R.Multiply3x3(Mat44::Scale(inv_inertia_diagonal));
		//I_world = I_world.Multiply3x3RightTransposed(R);
		//v = I_world.Multiply3x3(ra.Cross(axis));



		//Vec3 torque = ra.Cross(axis);
		//Quat R_total = mOrientation;// *inertia_rot;
		////vector to q space
		//Vec3 local = R_total.InverseRotate(torque);
		//Vec3 delta_local = local * mInvInertiaTensorDiagonal;
		//v = R_total.Rotate(delta_local);

		//Mat44 R_body = Mat44::RotationMatrix(rot);
		//Mat44 R_I = Mat44::RotationMatrix(inv_inertia_rot);
		//Mat44 I_world;
		//I_world = R_body.Multiply3x3(R_I).Multiply3x3(Mat44::Scale(inv_inertia_diagonal));
		//I_world = I_world.Multiply3x3RightTransposed(R_I).Multiply3x3RightTransposed(R_body);
	}
	Mat44 Body::ComputeInvInertiaTensorWorld()
	{
		// I_world^-1 = R_total * I_diag^-1 * R_total^T
		//Quat inertia_rot;//<-- not implmented yet
		Quat R_total = mOrientation;// *inertia_rot;

		//R_total * I_diag
		Vec3 x = R_total.RotateScaledAxisX(mInvInertiaTensorDiagonal.X());
		Vec3 y = R_total.RotateScaledAxisY(mInvInertiaTensorDiagonal.Y());
		Vec3 z = R_total.RotateScaledAxisZ(mInvInertiaTensorDiagonal.Z());

		//*R_total^ T
		Mat44 I_world;
		I_world.SetColumn3(0, Vec3(x.Dot(x), x.Dot(y), x.Dot(z)));
		I_world.SetColumn3(1, Vec3(y.Dot(x), y.Dot(y), y.Dot(z)));
		I_world.SetColumn3(2, Vec3(z.Dot(x), z.Dot(y), z.Dot(z)));
		return I_world;
	}
	void Body::ComputeInvInertiaAxesWorld(Vec3& axis_x, Vec3& axis_y, Vec3& axis_z)
	{
		// I_world^-1 = R_total * I_diag^-1 * R_total^T
		//Quat inertia_rot; //<-- not implmented yet
		Quat R_total = mOrientation;// *inertia_rot;

		//R_total * I_diag^-1
		axis_x = R_total.RotateScaledAxisX(mInvInertiaTensorDiagonal.X());
		axis_y = R_total.RotateScaledAxisY(mInvInertiaTensorDiagonal.Y());
		axis_z = R_total.RotateScaledAxisZ(mInvInertiaTensorDiagonal.Z());
	}
	Vec3 Body::ApplyInvInertiaTensorWorld_Basis(const Vec3& vector)
	{
		/// v' = I_world^-1 * v
		/// I_world^-1 = R_total * I_diag^-1 * R_total^T
		/// 
		/// v' = R_total * I_diag^-1 * R_total^T * v
		Vec3 x, y, z;
		//R_total * I_diag^-1
		ComputeInvInertiaAxesWorld(x, y, z);
		//project vector onto axis & sum contribution 
		//*R_total^ Tranform vector
		return x * vector.Dot(x) +
			y * vector.Dot(y) +
			z * vector.Dot(z);
	}

	Vec3 Body::ApplyInvInertiaTensorWorld(const Vec3& vector)
	{
		/// v' = Q_total * (I_diag^-1 * (Q^T * v))
		/// 
		/// v' = I_world^-1 * v
		/// I_world^-1 = R_total * I_diag * R_total^T
		/// 
		/// v' = Q_total * I_diag^-1 * Q^T * v
		//Quat inertia_rot; //<-- not implmented yet
		Quat R_total = mOrientation;// *inertia_rot;

		//vector to q space
		Vec3 local = R_total.InverseRotate(vector);
		Vec3 delta_local = local * mInvInertiaTensorDiagonal;
		return R_total.Rotate(delta_local);
	}




	void Body::SetPosition(const Vec3& pos)
	{
		mPosition = pos;
		ComputeWorldSpaceBoundsInternal();
		mMotionState.lastUpdateStep = PhysicsWorld::GetCurrentSimStep();
	}

	void Body::SetLinearVelocity(const Vec3& velocity)
	{
		mLinearVelocity = velocity;
	}

	void Body::SetAngularVelocity(const Vec3& velocity)
	{
		mAngularVelocity = velocity;
	}


	void Body::SetOrientation(const Quat& quat)
	{
		mOrientation = quat;
		ComputeWorldSpaceBoundsInternal();
		mMotionState.lastUpdateStep = PhysicsWorld::GetCurrentSimStep();
	}

	Mat44 Body::ComputeInvInteriaWorld() const
	{
		if (IsDynamic())
			return TransformDiagonalInertiaTensor(mInvInertiaTensorDiagonal, mOrientation);
		else
			return Mat44(0.0f);
	}

	void Body::ComputeWorldSpaceBoundsInternal()
	{
		mMotionState.lastUpdateStep = PhysicsWorld::GetCurrentSimStep();
		mBounds = mShape->GetWorldBounds(Mat44::RotationTranslation(mOrientation, mPosition), Vec3::One());
	}

}