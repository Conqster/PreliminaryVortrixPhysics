#include "ContactConstraintSolver.h"

#include "Dynamics/Body/Body.h"
#include "Collision/Shapes/Shape.h"
#include "Renderer/DebugGizmosRenderer.h"

#include "SimulationContexts.h"

namespace vx {


	void ContactConstraintSolver::ContactConstraintPoint::SetupAxesVelocityConstraints(const Body& body0, const Body& body1, 
		const Vec3& world_pos0, const Vec3& world_pos1, const Vec3& normal, float e, float mu, const ContactConstraintAxesSetting& settings)
	{
		/// only accounting dynamic bodies 
		bool body0_nonstatic = !body0.IsStatic();
		bool body1_nonstatic = !body1.IsStatic();

		const float inv0 = (body0_nonstatic) ? body0.GetInverseMass() : 0.0f;
		const float inv1 = (body1_nonstatic) ? body1.GetInverseMass() : 0.0f;

		float total_inv_mass = inv0 + inv1;

		//contact average pt 
		Vec3 p = (world_pos0 + world_pos1) * 0.5f;
		/// point relative to bodies
		Vec3 r0 = p - body0.GetPosition();
		Vec3 r1 = p - body1.GetPosition();


		/// pos bias = beta / delta_time * max(0, penetration - slop)
		float penetration = VxAbs((world_pos0 - world_pos1).Dot(normal));


			Vec3 rel_vel = Vec3::Zero();
		/// normal axis properties
		{

					//			if (dyn_a && dyn_b)
					//	rel_vel = bodyB->GetPointVelocityRelCOM(rel_b) - bodyA->GetPointVelocityRelCOM(rel_a);
					//else if (dyn_a)
					//	rel_vel = -bodyA->GetPointVelocityRelCOM(rel_a);
					//else if (dyn_b)
					//	rel_vel = bodyB->GetPointVelocityRelCOM(rel_b);



			/// inverse effective mass: K = J M^-1 J^T
			float inv_effective_mass = total_inv_mass;

			if (body0_nonstatic)
			{
				rel_vel = -body0.GetPointVelocityRelCOM(r0);

				Vec3 r0_X_n = r0.Cross(normal);
				r0_X_n.Store(this->normal.r0XAxis);

				Vec3 inv_Ir0_n = body0.ComputeInvInteriaWorld().Multiply3x3(r0_X_n);
				inv_Ir0_n.Store(this->normal.invIr0XAxis);

				inv_effective_mass += inv_Ir0_n.Dot(r0_X_n);
			}


			if (body1_nonstatic)
			{
				rel_vel += body1.GetPointVelocityRelCOM(r1);

				Vec3 r1_X_n = r1.Cross(normal);
				r1_X_n.Store(this->normal.r1XAxis);

				Vec3 inv_Ir1_n = body1.ComputeInvInteriaWorld().Multiply3x3(r1_X_n);
				inv_Ir1_n.Store(this->normal.invIr1XAxis);

				inv_effective_mass += inv_Ir1_n.Dot(r1_X_n);
			}
			if (inv_effective_mass > 0)
				this->normal.effectiveMass = 1.0f / inv_effective_mass;
			this->normal.totalLamda = 0.0f;
			normal.Store(this->normal.axis);




			float restitution_bias = 0.0f;
			//relative velocity along normal
			float rel_velN = rel_vel.Dot(normal);
			if (rel_velN < -settings.restitutionThreshold)
				restitution_bias = e * rel_vel.Dot(normal);

			float pos_bias = -(settings.baumgarte / settings.timeStep) * VxMax(0.0f, penetration - settings.positionCorrectionSlop);
			pos_bias = VxClamp(pos_bias, -settings.maxSpeed, 0.0f);
			this->normal.bias = restitution_bias+pos_bias;
		}


		if (mu > 0)
		{
			Vec3 tangent_axis[2];
			GetTangentBasis(normal, &rel_vel, tangent_axis[0], tangent_axis[1]);

#if VX_DEBUG_DRAW
			if(settings.debug_renderer && settings.debugDrawAxes)
			{
				settings.debug_renderer->DrawArrowCone(p,
					p + normal,
					0.05f, 0.1, 0.05f, 3, Colour::sGreen);

				settings.debug_renderer->DrawArrowCone(p,
					p + tangent_axis[1],
					0.05f, 0.1, 0.05f, 3, Colour::sBlue);

				settings.debug_renderer->DrawArrowCone(p,
					p + tangent_axis[0],
					0.05f, 0.1, 0.05f, 3, Colour::sRed);
			}
#endif // VX_DEBUG_DRAW



			/// tangential axes constriat properties
			for (int i = 0; i < 2; ++i)
			{
				const Vec3& axis = tangent_axis[i];
				ConstraintAxis& constaint_axis = lateralTangent[i];
				axis.Store(constaint_axis.axis);

				/// inverse effective mass: K = J M^-1 J^T
				float inv_effective_mass = total_inv_mass;

				if (body0_nonstatic)
				{
					Vec3 r0_X_axis = r0.Cross(axis);
					r0_X_axis.Store(constaint_axis.r0XAxis);

					Vec3 inv_Ir0_axis = body0.ComputeInvInteriaWorld().Multiply3x3(r0_X_axis);
					inv_Ir0_axis.Store(constaint_axis.invIr0XAxis);

					inv_effective_mass += inv_Ir0_axis.Dot(r0_X_axis);
				}


				if (body1_nonstatic)
				{
					Vec3 r1_X_axis = r1.Cross(axis);
					r1_X_axis.Store(constaint_axis.r1XAxis);

					Vec3 inv_Ir1_axis = body1.ComputeInvInteriaWorld().Multiply3x3(r1_X_axis);
					inv_Ir1_axis.Store(constaint_axis.invIr1XAxis);

					inv_effective_mass += inv_Ir1_axis.Dot(r1_X_axis);
				}

				if(inv_effective_mass > 0)
					constaint_axis.effectiveMass = 1.0f / inv_effective_mass;
				constaint_axis.totalLamda = 0.0f;

				/// surface does not have velocity
				constaint_axis.bias = 0.0f;
			}
		}

	}


	void ContactConstraintSolver::Init(uint32 max_constraints)
	{
		mMaxConstraints = VxMin(max_constraints, kConstraintLimit);
		mConstraints = new ContactConstraint[max_constraints];
		mCachePoints = new CacheContactConstraint[max_constraints];
	}

	void ContactConstraintSolver::SetupContactConstraint(const ContactManifold& _manifold, const CollisionContext& ctx)
	{
		ContactManifold manifold = _manifold;

		/// for determintic simulation, enforce that 
		/// 1. if particpating bodies are a dynamic against a static
		/// 2. if both dynamic, for consitency id a < b
		/// 
		/// ensure that the A is the dynamic while B is the static 
		if(ctx.settings.consistentManifold)
		{
			int priority_a = static_cast<int>(manifold.a->GetMotionType());
			int priority_b = static_cast<int>(manifold.b->GetMotionType());

			/// 1. dynamic > static
			if (priority_a < priority_b)
				manifold.Swap();
			else if (priority_a == priority_b)
			{
				/// 2. dynamic - dynamic 
				if (manifold.a->GetID() < manifold.b->GetID())
					manifold.Swap();
			}
		}

		VX_ASSERT_WARN_VOID(manifold.a && manifold.b, "Either Bodies to not exists");
		//create constraint
		//allocate mem
		uint32 idx = mNumConstraints;
		VX_ASSERT_WARN_VOID(idx < mMaxConstraints, "Max frame contact constraint attianed returning");
		//mConstraints[idx] = {};
		ContactConstraint& constraint = mConstraints[idx];
		mNumConstraints++;
		mStats.numContactConstraints++;


		
		/// vaild tests 
		/// dyn - dyn 
		/// dyn - static 
		/// 
		/// 
		VX_ASSERT_WARN(manifold.a->IsDynamic(), "Contact Manifold body A is Static, while B is Dynamic");




		constraint.body0 = manifold.a;
		constraint.body1 = manifold.b;


		Body& body0 = *manifold.a;
		Body& body1 = *manifold.b;


		constraint.friction = CombinedCoefficient::GetFriction(mCombinedFrictionMode, body0, body1);
		constraint.restitution = CombinedCoefficient::GetRestitution(mCombinedRestitutionMode, body0, body1);


		constraint.invMass0 = manifold.a->GetInverseMass();
		constraint.invMass1 = manifold.b->GetInverseMass();

		manifold.normal.Normalise();
		manifold.normal.Store(constraint.normal);

		constraint.numContacts = 0;
		uint32 num_pts = VxMin(manifold.numManifoldPoints, kMaxPoints);

		///bodies inverse transforms 
		Mat44 transform0 = body0.ComputeWorldTransform();
		Mat44 transform1 = body1.ComputeWorldTransform();


		ContactConstraintAxesSetting constraint_axes_setting;
#if VX_DEBUG_DRAW
		constraint_axes_setting.debug_renderer = ctx.debugRenderer;
		constraint_axes_setting.debugDrawAxes = ctx.drawContactTBNs;
#endif // VX_DEBUG_DRAW
		constraint_axes_setting.timeStep = mPhysicsContext->stepDeltaTime;


		//create cache constrain
		//allocate mem
		uint32 _idx = mNumCachePoints;
		mNumCachePoints++;
		VX_ASSERT_WARN_VOID(_idx < mMaxConstraints, "Max frame contact constraint attianed returning");
		//mCachePoints[_idx] = {};
		CacheContactConstraint& cache_constraint = mCachePoints[_idx];
		//transfer points 
		for (int i = 0; i < num_pts; ++i)
		{
			const ManifoldPoint& mp = manifold.points[i];


			//add new constraint point
			ContactConstraintPoint& constraint_pt = constraint.contactPoints[constraint.numContacts++];


			Vec3 p0_ls = transform0.TransformInverse(mp.pointA);
			Vec3 p1_ls = transform1.TransformInverse(mp.pointB);
			//later use this to check the cache 

			//new cache 
			{
				constraint_pt.cacheLocalPoint = &cache_constraint.contactPoints[i];
				auto& clp = *constraint_pt.cacheLocalPoint;
				//clp = 

				p0_ls.Store(clp.localPoint0);
				p1_ls.Store(clp.localPoint1);

				clp.totalNormalLambda = 0.0f;
				clp.totalTangentLambda[0] = 0.0f;
				clp.totalTangentLambda[1] = 0.0f;
			}
			//Vec3 p = (mp.pointA + mp.pointB) * 0.5f;
			///// point relative to bodies
			//constraint.r0 = p - body0.GetPosition();
			//constraint.r1 = p - body1.GetPosition();

			Vec3 pt = mp.pointB;
			//hack to ensure right penetration for now
			float depth_sq = (mp.pointA - mp.pointB).LengthSq();
			if (depth_sq < mp.peneration)
				pt = mp.pointA + manifold.normal * mp.peneration;

			constraint_pt.SetupAxesVelocityConstraints(*manifold.a, *manifold.b, mp.pointA, mp.pointB, manifold.normal, constraint.restitution, constraint.friction, constraint_axes_setting);
		}

		VX_ASSERT_WARN(num_pts == constraint.numContacts, "Number of point stored needs to be equal compute attainable");
	}


	void ContactConstraintSolver::WarmStart()
	{
		VX_ASSERT(false, "Out of service!!!!"); //need house keep last frame cache frame etc

		//for a warm start up use info from last frame 
		for (uint32 contact_idx = 0; contact_idx < mNumConstraints; ++contact_idx)
		{
			auto& constraint_info = mConstraints[contact_idx];//constraint info
			if (!constraint_info.body0 || !constraint_info.body1 || constraint_info.numContacts < 0)
			{
				VX_ASSERT_WARN(constraint_info.body0, "Body 0 is invalid");
				VX_ASSERT_WARN(constraint_info.body1, "Body 1 is invalid");
				VX_ASSERT_WARN(constraint_info.numContacts > 0, "No contact points");
				continue;
			}

			Body* bodyA = constraint_info.body0;
			Body* bodyB = constraint_info.body1;

			bool dyn_a = (bodyA->IsDynamic());
			bool dyn_b = (bodyB->IsDynamic());

			Vec3 n = Vec3::LoadFloat3Raw(constraint_info.normal);
			n.Normalise();
			//contact basis


			Vec3 tangents[2];
			for (int i = 0; i < constraint_info.numContacts; ++i)
			{
				auto& pt = constraint_info.contactPoints[i];

				tangents[0] = Vec3::LoadFloat3Raw(pt.lateralTangent[0].axis);
				tangents[1] = Vec3::LoadFloat3Raw(pt.lateralTangent[1].axis);
				Vec3 impluse = (n * pt.normal.totalLamda) +
					(tangents[0] * pt.lateralTangent[0].totalLamda) +
					(tangents[1] * pt.lateralTangent[1].totalLamda);


				//handle this differenely make use of the arm for right displacement
				bodyA->ApplyImpulse(-impluse * constraint_info.invMass0, Vec3::LoadFloat3Raw(pt.cacheLocalPoint->localPoint0));
				if(dyn_b)
					bodyB->ApplyImpulse(impluse * constraint_info.invMass1, Vec3::LoadFloat3Raw(pt.cacheLocalPoint->localPoint1));
			}
		}
	}


	void ContactConstraintSolver::DebugDraw(DebugGizmosRenderer* debug_renderer, const DrawSettings& settings) const
	{

		/// Keep in mind for manifold debug
		/// the manifold points are store in world space 
		/// 
		/// making most point invalid after simulation step 
		/// as when velocity & positional contrainst are solved 
		/// bodies might have moved away for manifold points 
		/// 
		/// need to improve this later......
		auto draw_contact_manifold = [&](const ContactConstraint& constraint) {

			//Vec3 half_extent(mSettings.drawContactPointSize * 0.5f);

			//Draw bodies bound particapting 
			if (settings.drawCollidingPairRefAndInc)
			{
				AABB aabb = constraint.body0->ComputeAABBWorld();
				debug_renderer->DrawAABB(aabb.mMin, aabb.mMax, settings.collidingPairRefColour, false);
				aabb = constraint.body1->ComputeAABBWorld();
				debug_renderer->DrawAABB(aabb.mMin, aabb.mMax, settings.collidingPairIncColour, false);
			}

			if (!settings.drawContacts)return;

			Vec3 n = Vec3::LoadFloat3Raw(constraint.normal);

			Mat44 transform0 = constraint.body0->ComputeWorldTransform();
			Mat44 transform1 = constraint.body1->ComputeWorldTransform();
			//for (const auto& [pointA, peneration, pointB] : manifold.points)
			for (int i = 0; i < constraint.numContacts; ++i)
			{
				auto& contact_point = constraint.contactPoints[i];
				Vec3 pointA = transform0.Transform(Vec3::LoadFloat3Raw(contact_point.cacheLocalPoint->localPoint0));
				Vec3 pointB = transform1.Transform(Vec3::LoadFloat3Raw(contact_point.cacheLocalPoint->localPoint1));

				//const Vec3 p0 = transform0.Transform(Vec3::LoadFloat3Raw(contact_point.cacheLocalPoint->localPoint0));
				//const Vec3 p1 = transform1.Transform(Vec3::LoadFloat3Raw(contact_point.cacheLocalPoint->localPoint1));

				//hack penetration 
				float penetration = (pointA - pointB).Dot(n);

				//if draw as aabb
				//Vec3 min = point - half_extent;
				//Vec3 max = point + half_extent;

				//debug_renderer->DrawAABB(min.AsGLM(), max.AsGLM(), ColourToGLM(mSettings.drawContactPointColour));
				Colour col = settings.drawContactPointColour;
				//glm::vec3 col = ColourToGLM(mSettings.drawContactPointColour);

				/// point A
				debug_renderer->DrawLine(
					(pointA - Vec3(settings.drawContactPointSize, 0.0f, 0.0f)),
					(pointA + Vec3(settings.drawContactPointSize, 0.0f, 0.0f)),
					col);
				debug_renderer->DrawLine(
					(pointA - Vec3(0.0f, settings.drawContactPointSize, 0.0f)),
					(pointA + Vec3(0.0f, settings.drawContactPointSize, 0.0f)),
					col);

				col = Colour::sMagenta;
				/// point B
				debug_renderer->DrawLine(
					(pointB - Vec3(settings.drawContactPointSize, 0.0f, 0.0f)),
					(pointB + Vec3(settings.drawContactPointSize, 0.0f, 0.0f)),
					col);
				debug_renderer->DrawLine(
					(pointB - Vec3(0.0f, settings.drawContactPointSize, 0.0f)),
					(pointB + Vec3(0.0f, settings.drawContactPointSize, 0.0f)),
					col);

				if (settings.drawContactsNormals)
				{
					float depth = penetration;
					if (ManifoldPoint::kUseNewManifoldPt)
						//depth = (pointB - point).Length();
						depth = (pointA - pointB).Length();
					const float size = (settings.drawContactsNormalsWithPeneration) ? depth * settings.drawContactNormalSize :
						settings.drawContactNormalSize;

					//debug_renderer->DrawLine(point.AsGLM(), end.AsGLM(), ColourToGLM(mSettings.drawContactNormalsColour));
					//debug_renderer->DrawLine(point, end, mSettings.drawContactNormalsColour);

					//just for old manifold point 
					const Vec3 pt0 = pointA;
					const Vec3 pt1 = (ManifoldPoint::kUseNewManifoldPt) ? pointB : pointA - n * penetration;

					const Vec3 end = pt0 + n * size;

					debug_renderer->DrawArrowCone(pt0, end, 0.02f, 0.05f, 0.02f, 3, settings.drawContactNormalsColour);
					//This line need to match or almost match he above
					debug_renderer->DrawLine(pt1, pt0, vx::Colour::sTurquoise);
				}
			}
			};


		for (uint32 contact_idx = 0; contact_idx < mNumConstraints; ++contact_idx)
		{
			draw_contact_manifold(mConstraints[contact_idx]);
		}
	}

	void ContactConstraintSolver::PositionalCorrection(ContactConstraint& constraint, float baumgarte, float slop, float min_limit, float max_limit, float limit_scale)
	{
		Body* a = constraint.body0;
		Body* b = constraint.body1;

		VX_ASSERT_WARN_VOID(a && b, "either bodies needs to be valid");

		bool dyn_a = a->IsDynamic();
		bool dyn_b = b->IsDynamic();

		VX_ASSERT_WARN_VOID(dyn_a || dyn_b, "not possible one of the bodies need to be non static");

		Vec3 n = Vec3::LoadFloat3Raw(constraint.normal);

		float correction_limit = 0.01;
		{
			float limitA = dyn_a ?
				a->GetShape()->GetHalfExtents().MinComponent() * limit_scale: kMaxf;
			float limitB = dyn_b ?								
				b->GetShape()->GetHalfExtents().MinComponent() * limit_scale: kMaxf;

			correction_limit = VxMin(limitA, limitB);

			correction_limit = VxClamp(correction_limit, min_limit, max_limit);
		}

		Mat44 transform0 = a->ComputeWorldTransform();
		Mat44 transform1 = b->ComputeWorldTransform();

		//effective mass 
		float total_inv_mass = constraint.invMass0 + constraint.invMass1;

		for (int i = 0; i < constraint.numContacts; ++i)
		{
			auto& contact_point = constraint.contactPoints[i];

			///New contact point in world as bodies position might have been corrected
			const Vec3 p0 = transform0.Transform(Vec3::LoadFloat3Raw(contact_point.cacheLocalPoint->localPoint0));
			const Vec3 p1 = transform1.Transform(Vec3::LoadFloat3Raw(contact_point.cacheLocalPoint->localPoint1));

			float seperation = (p1 - p0).Dot(n);
			///seperation constant
			/// dist along normal + slop 
			/// to avoid jittering between bodies
			//float C = VxMax(seperation + slop, -kEpsilon);
			float C = seperation + slop;

			if (C < 0.0f)
			{
				float inv_effective_mass = total_inv_mass;
				Vec3 p = (p0 + p1) * 0.5f;
				/// point relative to bodies
				Vec3 r0 = p - a->GetPosition();
				Vec3 r1 = p - b->GetPosition();

				Vec3 inv_Ir0_Xn, inv_Ir1_Xn;
				if (dyn_a)
				{
					Vec3 r0_X_n = r0.Cross(n);
					inv_Ir0_Xn = a->ComputeInvInteriaWorld().Multiply3x3(r0_X_n);
					inv_effective_mass += r0_X_n.Dot(inv_Ir0_Xn);
				}
				if (dyn_b)
				{
					Vec3 r1_X_n = r1.Cross(n);
					inv_Ir1_Xn = b->ComputeInvInteriaWorld().Multiply3x3(r1_X_n);
					inv_effective_mass += r1_X_n.Dot(inv_Ir1_Xn);
				}

				if (inv_effective_mass < 1e-9f)
					continue;

				C = VxMax(C, -correction_limit);
				float lambda = -(baumgarte * C) / inv_effective_mass;
				Vec3 lambda_vector = lambda * n;

				if (dyn_a)
				{
					a->ApplyLinearDisplacement(-lambda_vector * a->GetInverseMass());
					a->ApplyAngularDisplacement(-lambda * inv_Ir0_Xn);
				}
				if (dyn_b)
				{
					b->ApplyLinearDisplacement(lambda_vector * b->GetInverseMass());
					b->ApplyAngularDisplacement(lambda * inv_Ir1_Xn);
				}
			}
		}
	}




	void ContactConstraintSolver::SolverContactManifold(const SolverSettings& phy_settings)
	{
		VX_PROFILE_FUNCTION();

		for (int i = 0; i < phy_settings.velocityIterations; ++i)
		{

			//curr_manifold_idx = -1;
			//for (auto& contact_info : manifolds)
			for (uint32 contact_idx = 0; contact_idx < mNumConstraints; ++contact_idx)
			{
				auto& constraint_info = mConstraints[contact_idx];//constraint info


				if (!constraint_info.body0 || !constraint_info.body1 || constraint_info.numContacts < 0)
				{
					VX_ASSERT_WARN(constraint_info.body0, "Body 0 is invalid");
					VX_ASSERT_WARN(constraint_info.body1, "Body 1 is invalid");
					VX_ASSERT_WARN(constraint_info.numContacts > 0, "No contact points");
					continue;
				}

				Body* bodyA = constraint_info.body0;
				Body* bodyB = constraint_info.body1;

				bool dyn_a = (bodyA->IsDynamic());
				bool dyn_b = (bodyB->IsDynamic());

				Vec3 n = Vec3::LoadFloat3Raw(constraint_info.normal);
				n.Normalise();
				//contact basis
				Vec3 tangents[2];
				//GetTangentBasis(n, nullptr, tangents[0], tangents[1]);
				//tangents[0] = Vec3::LoadFloat3Raw(constraint_info.contactPoints[0].tangent[0].axis);
				//tangents[1] = Vec3::LoadFloat3Raw(constraint_info.contactPoints[0].tangent[1].axis);


				float initial_total_energy = bodyA->GetKineticEnergy() + bodyB->GetKineticEnergy();

				//////Get velocities 
				Vec3 lin_vel0 = Vec3(0.0f);
				Vec3 ang_vel0 = Vec3(0.0f);

				if (dyn_a)
				{
					lin_vel0 = bodyA->GetLinearVelocity();
					ang_vel0 = bodyA->GetAngularVelocity();
				}

				Vec3 lin_vel1 = Vec3(0.0f);
				Vec3 ang_vel1 = Vec3(0.0f);

				if (dyn_b)
				{
					lin_vel1 = bodyB->GetLinearVelocity();
					ang_vel1 = bodyB->GetAngularVelocity();
				}




				//for (auto& pt : contact_info.points)
				for (int i = 0; i < constraint_info.numContacts; ++i)
				{
					auto& pt = constraint_info.contactPoints[i];

					tangents[0] = Vec3::LoadFloat3Raw(pt.lateralTangent[0].axis);
					tangents[1] = Vec3::LoadFloat3Raw(pt.lateralTangent[1].axis);


					///check closeness 
					//VX_ASSERT_WARN(n.IsApprox(Vec3::LoadFloat3Raw(pt.normal.axis)), "normal is bad");
					//VX_ASSERT_WARN(tangents[0].IsApprox(Vec3::LoadFloat3Raw(pt.tangent[0].axis)), "tangent0 is bad");
					//VX_ASSERT_WARN(tangents[1].IsApprox(Vec3::LoadFloat3Raw(pt.tangent[1].axis)), "tangent1 is bad");

					///this could be solve in simd parallel n, t0, t1
					///
					//////////////////////////
					//// Solve normal/penetration axis
					//////////////////////////
					ContactConstraintPoint::ConstraintAxis& nor_axis_contraint = pt.normal;
					// 
					{
						/// curreny relative velocity (J * v) 
						float jn;
						if (dyn_a && dyn_b)
							jn = (lin_vel0 - lin_vel1).Dot(n);
						else if (dyn_a)
							jn = lin_vel0.Dot(n);
						else if (dyn_b)
							jn = (-lin_vel1).Dot(n);
						else
						{
							VX_ERROR("Static vs static this should not be possible");
							jn = 0.0f;
						}
						//simplify 
						if (dyn_a)
							jn += Vec3::LoadFloat3Raw(nor_axis_contraint.r0XAxis).Dot(ang_vel0);
						if(dyn_b)
							jn -= Vec3::LoadFloat3Raw(nor_axis_contraint.r1XAxis).Dot(ang_vel1);

						/// -K^-1(Jv + b)
						/// -K^-1((1-e)Jv)
						/// nor_axis_contraint.effectiveMass = 1/inv effective mass
						//float lambda = (nor_axis_contraint.bias - jn) * nor_axis_contraint.effectiveMass;
						float lambda = (jn - nor_axis_contraint.bias) * nor_axis_contraint.effectiveMass;

						float old_lambda = nor_axis_contraint.totalLamda;
						//ensure non negative
						nor_axis_contraint.totalLamda = VxMax(old_lambda + lambda, 0.0f);
						//updated jn
						float impluse = nor_axis_contraint.totalLamda - old_lambda;
			
						//store changes
						if(dyn_a)
						{
							lin_vel0 -= impluse * constraint_info.invMass0 * n;
							ang_vel0 -= impluse * Vec3::LoadFloat3Raw(nor_axis_contraint.invIr0XAxis);
						}
						if (dyn_b)
						{
							lin_vel1 += impluse * constraint_info.invMass1 * n;
							ang_vel1 += impluse * Vec3::LoadFloat3Raw(nor_axis_contraint.invIr1XAxis);
						}

					}

				
					////////////////////////////
					////// Solve tangential axis
					////////////////////////////
					if(constraint_info.friction > 0.0f)
					{
						float max_friction = constraint_info.friction * nor_axis_contraint.totalLamda;

						for (int i = 0; i < 2; ++i)
						{
							ContactConstraintPoint::ConstraintAxis& axis_contraint = pt.lateralTangent[i];
							const Vec3& axis = Vec3::LoadFloat3Raw(axis_contraint.axis);// = tangents[i];

							float jv;
							if (dyn_a && dyn_b)
								jv = (lin_vel0 - lin_vel1).Dot(axis);
							else if (dyn_a)
								jv = lin_vel0.Dot(axis);
							else if (dyn_b)
								jv = (-lin_vel1).Dot(axis);
							else
							{
								VX_ERROR("Static vs static this should not be possible");
								jv = 0.0f;
							}


							//simplify 
							if (dyn_a)
								jv += Vec3::LoadFloat3Raw(axis_contraint.r0XAxis).Dot(ang_vel0);
							if (dyn_b)
								jv -= Vec3::LoadFloat3Raw(axis_contraint.r1XAxis).Dot(ang_vel1);

							//float lambda = contact_info.friction * axis_contraint.effectiveMass * jv;
							//float lambda = contact_info.friction * 0.5f * axis_contraint.effectiveMass * jv;

							//ignore surface relative velocity
							float lambda = jv * axis_contraint.effectiveMass;

							float old_lambda = axis_contraint.totalLamda;
							//ensure non negative
							axis_contraint.totalLamda = VxClamp(old_lambda + lambda, -max_friction, max_friction);
							//updated jn
							float impluse = axis_contraint.totalLamda - old_lambda;

							//store changes
							if (dyn_a)
							{
								lin_vel0 -= impluse * constraint_info.invMass0 * axis;
								ang_vel0 -= impluse * Vec3::LoadFloat3Raw(axis_contraint.invIr0XAxis);
							}
							if (dyn_b)
							{
								lin_vel1 += impluse * constraint_info.invMass1 * axis;
								ang_vel1 += impluse * Vec3::LoadFloat3Raw(axis_contraint.invIr1XAxis);
							}
						}
					}



					VX_ASSERT(!lin_vel0.IsNaN(), "lin_vel0 is nan");
					VX_ASSERT(!ang_vel0.IsNaN(), "ang_vel0 is nan");
					VX_ASSERT(!lin_vel1.IsNaN(), "lin_vel1 is nan");
					VX_ASSERT(!ang_vel0.IsNaN(), "ang_vel1 is nan");

					////set velocities; prevent multiple bodies value value changes
					/// and heavy torque level & world moment inetria internal to bodies ApplyImpluse
					if(dyn_a)
					{
						bodyA->SetLinearVelocity(lin_vel0);
						bodyA->SetAngularVelocity(ang_vel0);
					}

					if(dyn_b)
					{
						bodyB->SetLinearVelocity(lin_vel1);
						bodyB->SetAngularVelocity(ang_vel1);
					}

					float total_enery_after = bodyA->GetKineticEnergy() + bodyB->GetKineticEnergy();

					float solver_work = total_enery_after - initial_total_energy;
					mStats.totalKineticWork += solver_work;

					mStats.totalStepKineticWork += solver_work;

					//good / expected behaviour
					/// failed work 
					/// if after is greater than start -> solver system add energy to body
					/// if solver work if negative means solver add more energy 
					/// if solver work is positive means the solver is good job of removing energy
					if(solver_work < 0.0f)
						mStats.totalStepWorkLoss += VxAbs(solver_work);
					else 
						mStats.totalStepWorkGain += VxAbs(solver_work);

					//track peak instability 	
					mStats.maxAttainedKineticWork = VxMax(mStats.maxAttainedKineticWork, mStats.totalStepWorkGain);
				}
			}
		}
	}
}