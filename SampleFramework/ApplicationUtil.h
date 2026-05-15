#pragma once
struct ApplicationSpecification
{
	std::string name = "Test Application";
	bool launchFullScreen = false;
	int windowSize[2] = { 1920, 1080 };
	int windowPos[2] = { 100, 100 };
	bool disableBindlessSupport = false;
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

	float friction = 0.4f;
	float restitution = 0.2f;

	bool isDynamic = true;
	

	/// Physics particle properies
	bool attachWithLast = true;
	bool newAsAnchor = false;

	bool openWindow = false;
	bool allowKeyHeld = false;
};



class ShapeArena
{
public:
	ShapeArena() = default;
	explicit ShapeArena(size_t cap)
	{
		Init(cap);
	}

	VX_INLINE void Init(size_t cap)
	{

		data = new vx::uint8[cap];
		capacity = cap;
	}

	~ShapeArena()
	{
		delete[] data;
	}
	void* Allocate(size_t size, size_t alignment)
	{
		VX_ASSERT((alignment & (alignment - 1)) == 0);
		uintptr_t curr = reinterpret_cast<uintptr_t>(data + offset);
		uintptr_t aligned = (curr + alignment - 1) & ~(alignment - 1);
		size_t new_off = (aligned - reinterpret_cast<uintptr_t>(data)) + size;

		if (new_off > capacity)
			return nullptr;
		offset = new_off;
		return reinterpret_cast<void*>(aligned);
	}

	
	template<typename T, typename... Args> 
	T* AllocateObject(Args&&... args)
	{
		void* mem = Allocate(sizeof(T), alignof(T));
		if (!mem) return nullptr;
		return new (mem)T(std::forward<Args>(args)...);
	}

	template<typename T>
	T* AllocateArray(size_t count)
	{
		void* mem = Allocate(sizeof(T) * count, alignof(T));
		if (!mem) return nullptr;
		return reinterpret_cast<void*>(mem);
	}

	void Reset()
	{
		offset = 0;
	}

private: 
	uint8_t* data = nullptr;
	size_t offset = 0;
	size_t capacity = 0;

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




#include "Dynamics/Body/BodyDebug.h"

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