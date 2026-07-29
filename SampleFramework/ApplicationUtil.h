#pragma once
struct ApplicationSpecification
{
	std::string name = "Test Application";
	bool launchFullScreen = false;
	int windowSize[2] = { 1920, 1080 };
	int windowPos[2] = { 100, 100 };
	bool disableBindlessSupport = false;
	bool centralisedWindow = true;
};



struct CreatePhysicsObjectSettings
{
	//Spawnbody() : impluse(500)
	enum class EType : std::uint8_t {
		Body,
		Particle
	} type = EType::Body;

	
	vx::uint32 count = 1;

	bool spawnFromView = true;
	float offsetFromView = 4.1f;
	float impulse = 1000.0f;
	float mass = 20.0f;


	/// Physics body properties
	vx::EShapeType bodyShape = vx::EShapeType::Sphere;
	vx::Float3 halfExtents{ 0.5f };

	bool applyImpulse = true;
	float initialLinearVelocity = 20.0f;

	float damping = 0.1f;// 0.95f;
	float angularDamping = 0.25f;

	float density = 1000.0f;
	bool overrideMasses = false;
	Float3 inertia;
	/// Multiply Inertia Tensor with Mass, for final Inertia
	bool multiplyInertiaTensor_Mass = false;

	float friction = 0.4f;
	float restitution = 0.2f;

	bool isDynamic = true;
	

	/// Physics particle properies
	bool attachWithLast = true;
	bool newAsAnchor = false;

	bool openWindow = false;
	bool allowKeyHeld = false;
};




struct DebugAngularImpulse
{
	vx::Float3 pointA{ 0 };
	bool onlyAngularImpluse = true;
	vx::Float3 impluse{-3.0f, 0.0f, 0.0f};
	bool apply = false;

	bool draw = false;
};


struct SpawnObjectCanon
{
	vx::Vec3 spawn = vx::Vec3(-4.0f, 2.0f, 0.0f);
	vx::Vec3 back =  vx::Vec3(-5.0f, 1.0f, 0.0f);
};




#include "Vortrix/Dynamics/Body/BodyDebug.h"

struct ExternalEffectDynamicBodyInfo
{
	static const int kMaxBodies = 20;
	int N = 0;
	vx::BodyID selectedBodies[kMaxBodies];

	vx::Float3 force;
	vx::Float3 impluse;

	bool pending = false;
	void AddUnsafe(vx::BodyID v) { selectedBodies[N++] = v; }

	int GetInstanceCount() { return vx::VxMin(N, kMaxBodies); }

	void ClearSelect() { N = 0; }

	void Reset()
	{
		force = { 0.0f, 0.0f, 0.0f };
		impluse = { 0.0f, 0.0f, 0.0f };
		pending = false;
	}
};


/// Shift + F to reset to origin 
/// maybe F for something else 
struct AppCameraConfig
{
	vx::Float3 teleportLocation = vx::Float3(0.0f, 5.0f, -10.0f);
	vx::Float3 teleportLookAt = vx::Float3(0.0f);

	bool keepViewDir;
};