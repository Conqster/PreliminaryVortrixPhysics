#pragma once

#include "Vortrix/Collision/BoxBoxContactDebug.h"

#include "Vortrix/Collision/Shapes/Shape.h"
#include "Vortrix/Collision/Shapes/SphereShape.h"
#include "Vortrix/Collision/Shapes/BoxShape.h"
#include "Vortrix/Collision/Shapes/PlaneShape.h"
#include "Vortrix/Collision/Shapes/CapsuleShape.h"

#include "Vortrix/Collision/ContactManifold.h"
#include "Vortrix/Geometry/GeometricAlgorithms.h"


#define VX_DEBUG_CONTACT_GENERATION 0


namespace vx::Narrowphase {


	/// 
	/// shape a (0) -> as reference shape
	/// shape b (1) -> as incident shape
	/// 
	/// contact normal: reference shape -> incident shape 
	/// 
	/// while orignal working mixed everything to togerther 
	/// 
	/// SphereVsSphere
	/// SphereVsPlane  {Plane Vs Sphere} (points & normal swap)
	/// CapsuleVsCapsule
	/// CapsuleVsPlane  {Plane Vs Capsule} (points & normal swap)
	/// 
	/// BoxVsSphere {SphereVsBox} (point & normal swap)
	/// BoxVsPlane {PlaneVsBox} (point & normal swap)
	/// Box-Capsule {CapsuleVsBox} (point & normal swap)
	/// 
	/// 
	/// 
	/// current issue Box collisions 
	/// multi bodies at close proxy bad is gravity at full scale 1.0
	/// but at 1/4th scale, collsion is fine 
	/// this would be interprentration bodies fine it hard to solve
	/// 
	/// debug case A
	/// a ground plane, 3 boxes 7.5 units on y away from ground and second box is above the first by 2.5 unit and third 2.5 above second
	/// at the proximity the causes the first and seconf to interpenetrate and jitters
	/// 
	/// boxes are of 0.5 unit (half extent on all axes)
	/// 
	/// another case B
	/// ground plane  3 boxes 7.5 units on y away from ground and second box is above the first by 5 unit 
	/// but this time first box is of 0.5 unit (half extent on all axes)
	/// and second is 3.0, 0.3. 0.3 unit (half extent) along x, y and z axes resp 
	/// 
	/// B1:-> contact point on BoxVsPlane & BoxVsPlane are sort 
	///		by pt0 -> deepest, pt1 -> max segement dist pt0-pt1
	///		pt3 -> max tri area, pt4 -> max tetrahedron area
	///		
	///		this causes the second box to ignore collision with first box.
	/// 
	/// B2 :-> same sorting but only BoxVsBox not BoxVsPlane 
	///		this is stable.
	///
	/// 
	

	bool SphereVsSphere(const Shape* in_shapeA, const Vec3& in_posA, const Quat& in_orientationA, 
						const Shape* in_shapeB, const Vec3& in_posB, const Quat& in_orientationB,
						ContactManifold& o_manifold);

	bool SphereVsPlane(const Shape* in_shapeA, const Vec3& in_posA, const Quat& in_orientationA, 
					   const Shape* in_shapeB, const Vec3& in_posB, const Quat& in_orientationB,
					   ContactManifold& o_manifold);

	bool CapsuleVsSphere(const Shape* in_shapeA, const Vec3& in_posA, const Quat& in_orientationA,
						 const Shape* in_shapeB, const Vec3& in_posB, const Quat& in_orientationB, 
						 ContactManifold& o_manifold);

	bool CapsuleVsCapsule(const Shape* in_shapeA, const Vec3& in_posA, const Quat& in_orientationA,
						  const Shape* in_shapeB, const Vec3& in_posB, const Quat& in_orientationB,
						  ContactManifold& o_manifold);

	bool CapsuleVsPlane(const Shape* in_shapeA, const Vec3& in_posA, const Quat& in_orientationA,
						const Shape* in_shapeB, const Vec3& in_posB, const Quat& in_orientationB,
						ContactManifold& o_manifold);

	/// lets work in the space of box
	bool BoxVsSphere(const Shape* in_shapeA, const Vec3& in_posA, const Quat& in_orientationA,
					 const Shape* in_shapeB, const Vec3& in_posB, const Quat& in_orientationB,
					 ContactManifold& o_manifold);

	bool BoxVsPlane(const Shape* in_shapeA, const Vec3& in_posA, const Quat& in_orientationA,
					const Shape* in_shapeB, const Vec3& in_posB, const Quat& in_orientationB,
					ContactManifold& o_manifold);

	bool BoxVsBox(const Shape* in_shapeA, const Vec3& in_posA, const Quat& in_orientationA,
				  const Shape* in_shapeB, const Vec3& in_posB, const Quat& in_orientationB,
				  ContactManifold& o_manifold);

	bool BoxVsCapsule(const Shape* in_shapeA, const Vec3& in_posA, const Quat& in_orientationA,
					  const Shape* in_shapeB, const Vec3& in_posB, const Quat& in_orientationB,
					  ContactManifold& o_manifold);
	

}/// namespace vx::Narrowphase