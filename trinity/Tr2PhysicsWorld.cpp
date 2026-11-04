// Copyright © 2026 CCP ehf.

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/AllowedDOFs.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include "StdAfx.h"
#include "Tr2PhysicsWorld.h"

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <mutex>
#include <unordered_map>

namespace
{
constexpr JPH::uint MAX_BODIES = 4096;
constexpr JPH::uint MAX_BODY_PAIRS = 4096;
constexpr JPH::uint MAX_CONTACT_CONSTRAINTS = 4096;
constexpr size_t TEMP_MEMORY = 8 * 1024 * 1024;	// bytes the solver works in each step; past it, malloc
constexpr float MIN_SECONDS = 1e-6f;	// s; shorter steps are refused (Jolt divides by the step)
constexpr float MIN_MASS = 1e-3f;	// kg; a mass given outside these is refused (Jolt inverts mass and inertia)
constexpr float MAX_MASS = 1e6f;	// kg
constexpr float MAX_STEP = 1.f / 60.f;	// s; a step longer than this goes in collision steps no longer
constexpr float MAX_SECONDS = 1.f;	// s; Step refuses more, a hitch rather than a simulation to catch up
constexpr int MAX_COLLISION_STEPS = 60;	// at MAX_STEP each, MAX_SECONDS
constexpr float MIN_HALF_EXTENT = 1e-3f;	// m; thinner boxes, and thinner capsules, are refused
constexpr float MAX_HALF_EXTENT = 1000.f;	// m
constexpr float MAX_SPEED = 500.f;	// m/s a velocity set may have at most: Jolt's own cap
constexpr float MAX_TURN_RATE = 40.f;	// rad/s; Jolt caps turns at 15 turns a second (about 47 rad/s)
constexpr float PENETRATION_SLOP = 0.005f;	// m bodies may rest sunk into each other; Jolt's 2 cm shows at a crate's scale
constexpr float FRICTION = 0.6f;	// every body's: a crate on concrete stays put on a gentle slope

// Static bodies never meet each other; moving ones (kinematic and dynamic) meet everything
constexpr JPH::ObjectLayer LAYER_STATIC = 0;
constexpr JPH::ObjectLayer LAYER_MOVING = 1;
constexpr JPH::uint OBJECT_LAYERS = 2;
constexpr JPH::uint BROAD_PHASE_LAYERS = 2;

void JoltTrace( const char* format, ... )
{
	char message[1024];
	va_list args;
	va_start( args, format );
	vsnprintf( message, sizeof( message ), format, args );
	va_end( args );
	CCP_LOG( "Tr2PhysicsWorld (Jolt): %s", message );
}

#ifdef JPH_ENABLE_ASSERTS
bool JoltAssertFailed( const char* expression, const char* message, const char* file, JPH::uint line )
{
	CCP_LOGERR( "Tr2PhysicsWorld (Jolt): assert %s failed at %s:%u: %s", expression, file, line, message ? message : "" );
	return false;	// carry on, never break into a debugger from the game
}
#endif

// Jolt's process-wide setup, run the first time a world is made and never undone: worlds may be made and freed at
// any time, and the factory and types are shared by all of them.  False if this code was compiled with other Jolt
// settings than the library was, which RegisterTypes answers with std::abort: no world is made then.
bool SetUpJolt()
{
	static std::once_flag once;
	static bool ready = false;
	std::call_once( once, [] {
		JPH::Trace = JoltTrace;
		JPH_IF_ENABLE_ASSERTS( JPH::AssertFailed = JoltAssertFailed; )
		if( !JPH::VerifyJoltVersionID() )
		{
			CCP_LOGERR( "Tr2PhysicsWorld: Trinity was built with other Jolt settings than its Jolt library; physics is off" );
			return;
		}
		JPH::RegisterDefaultAllocator();
		JPH::Factory::sInstance = new JPH::Factory();
		JPH::RegisterTypes();
		ready = true;
	} );
	return ready;
}

bool AllFinite( const Vector3& v )
{
	return std::isfinite( v.x ) && std::isfinite( v.y ) && std::isfinite( v.z );
}

// Finite, and no longer than limit
bool Within( const Vector3& v, float limit )
{
	return AllFinite( v ) && v.x * v.x + v.y * v.y + v.z * v.z <= limit * limit;
}

JPH::Vec3 ToJolt( const Vector3& v )
{
	return JPH::Vec3( v.x, v.y, v.z );
}

JPH::RVec3 ToJoltPosition( const Vector3& v )
{
	return JPH::RVec3( v.x, v.y, v.z );
}

Vector3 FromJolt( JPH::Vec3Arg v )
{
	return Vector3( v.GetX(), v.GetY(), v.GetZ() );
}

// A unit quaternion from (x, y, z, w), or none if it is not finite or has no length
bool ToJoltRotation( const Quaternion& q, JPH::Quat& out )
{
	if( !std::isfinite( q.x ) || !std::isfinite( q.y ) || !std::isfinite( q.z ) || !std::isfinite( q.w ) )
	{
		return false;
	}
	const JPH::Quat raw( q.x, q.y, q.z, q.w );
	const float length = raw.Length();
	if( !( length > 1e-6f ) )
	{
		return false;
	}
	out = raw / length;
	return true;
}

// Which object layers meet, set up as it is made (Jolt's tables cannot be copied)
class LayerPairs final : public JPH::ObjectLayerPairFilterTable
{
public:
	LayerPairs()
		: ObjectLayerPairFilterTable( OBJECT_LAYERS )
	{
		EnableCollision( LAYER_MOVING, LAYER_STATIC );
		EnableCollision( LAYER_MOVING, LAYER_MOVING );
	}
};

// Each object layer in a broad-phase tree of its own
class BroadPhaseLayers final : public JPH::BroadPhaseLayerInterfaceTable
{
public:
	BroadPhaseLayers()
		: BroadPhaseLayerInterfaceTable( OBJECT_LAYERS, BROAD_PHASE_LAYERS )
	{
		MapObjectToBroadPhaseLayer( LAYER_STATIC, JPH::BroadPhaseLayer( 0 ) );
		MapObjectToBroadPhaseLayer( LAYER_MOVING, JPH::BroadPhaseLayer( 1 ) );
	}
};
}

struct Tr2PhysicsWorld::Body
{
	JPH::BodyID id;
	bool upright = false;	// made with MOTION_UPRIGHT: never turns, whatever moves it
};

struct Tr2PhysicsWorld::Simulation
{
	// layerVsBroadPhase reads the other two as it is made, so they come first
	Simulation()
		: layerVsBroadPhase( broadPhaseLayers, BROAD_PHASE_LAYERS, layerPairs, OBJECT_LAYERS ),
		  tempAllocator( TEMP_MEMORY ),
		  jobSystem( JPH::cMaxPhysicsJobs )
	{
		system.Init( MAX_BODIES, 0, MAX_BODY_PAIRS, MAX_CONTACT_CONSTRAINTS, broadPhaseLayers, layerVsBroadPhase, layerPairs );
		JPH::PhysicsSettings settings = system.GetPhysicsSettings();
		settings.mPenetrationSlop = PENETRATION_SLOP;
		system.SetPhysicsSettings( settings );
	}

	~Simulation()
	{
		RemoveAll();
	}

	const Body* Find( int id ) const
	{
		const auto it = bodies.find( id );
		return it == bodies.end() ? nullptr : &it->second;
	}

	void RemoveAll()
	{
		JPH::BodyInterface& bodyInterface = system.GetBodyInterface();
		for( const auto& body : bodies )
		{
			bodyInterface.RemoveBody( body.second.id );
			bodyInterface.DestroyBody( body.second.id );
		}
		bodies.clear();
	}

	// Declared in the order they are made: the filters outlive the system that holds references to them
	LayerPairs layerPairs;
	BroadPhaseLayers broadPhaseLayers;
	JPH::ObjectVsBroadPhaseLayerFilterTable layerVsBroadPhase;
	JPH::TempAllocatorImplWithMallocFallback tempAllocator;	// the plain one aborts when it runs out
	JPH::JobSystemSingleThreaded jobSystem;
	JPH::PhysicsSystem system;

	std::unordered_map<int, Body> bodies;	// by the id the caller holds
	int nextId = 1;
};

Tr2PhysicsWorld::Tr2PhysicsWorld( IRoot* lockobj )
{
	if( SetUpJolt() )
	{
		m_simulation = std::make_unique<Simulation>();
	}
}

Tr2PhysicsWorld::~Tr2PhysicsWorld() = default;

bool Tr2PhysicsWorld::IsAvailable() const
{
	return m_simulation != nullptr;
}

const Tr2PhysicsWorld::Body* Tr2PhysicsWorld::Find( int id ) const
{
	return m_simulation ? m_simulation->Find( id ) : nullptr;
}

const Tr2PhysicsWorld::Body* Tr2PhysicsWorld::FindMoving( int id ) const
{
	const Body* body = Find( id );
	if( !body || m_simulation->system.GetBodyInterface().GetMotionType( body->id ) == JPH::EMotionType::Static )
	{
		return nullptr;
	}
	return body;
}

Vector3 Tr2PhysicsWorld::GetGravity() const
{
	return m_simulation ? FromJolt( m_simulation->system.GetGravity() ) : Vector3( 0.f, 0.f, 0.f );
}

void Tr2PhysicsWorld::SetGravity( const Vector3& gravity )
{
	if( m_simulation && AllFinite( gravity ) )
	{
		m_simulation->system.SetGravity( ToJolt( gravity ) );
	}
}

int Tr2PhysicsWorld::GetBodyCount() const
{
	return m_simulation ? static_cast<int>( m_simulation->bodies.size() ) : 0;
}

int Tr2PhysicsWorld::AddBox( const Vector3& halfExtents, const Vector3& position, const Quaternion& rotation, int motion, float mass )
{
	TRINITY_STATS_ZONE( "Tr2PhysicsWorld::AddBox" );

	if( !m_simulation || !AllFinite( halfExtents ) )
	{
		return 0;
	}
	const float smallest = std::min( { halfExtents.x, halfExtents.y, halfExtents.z } );
	const float largest = std::max( { halfExtents.x, halfExtents.y, halfExtents.z } );
	if( smallest < MIN_HALF_EXTENT || largest > MAX_HALF_EXTENT )
	{
		return 0;
	}
	// The rounding of the box's edges, kept under half its thinnest side
	const float convexRadius = std::min( JPH::cDefaultConvexRadius, 0.5f * smallest );
	JPH::BoxShapeSettings shape( ToJolt( halfExtents ), convexRadius );
	return AddBody( shape, position, rotation, motion, mass );
}

int Tr2PhysicsWorld::AddCapsule( float halfHeight, float radius, const Vector3& position, const Quaternion& rotation, int motion, float mass )
{
	TRINITY_STATS_ZONE( "Tr2PhysicsWorld::AddCapsule" );

	if( !m_simulation || !std::isfinite( halfHeight ) || !std::isfinite( radius ) )
	{
		return 0;
	}
	if( radius < MIN_HALF_EXTENT || radius > MAX_HALF_EXTENT || halfHeight < 0.f || halfHeight > MAX_HALF_EXTENT )
	{
		return 0;
	}
	JPH::CapsuleShapeSettings shape( halfHeight, radius );
	return AddBody( shape, position, rotation, motion, mass );
}

int Tr2PhysicsWorld::AddBody( const JPH::ShapeSettings& shapeSettings, const Vector3& position, const Quaternion& rotation, int motion, float mass )
{
	JPH::Quat turn;
	if( !AllFinite( position ) || !ToJoltRotation( rotation, turn ) || !std::isfinite( mass ) )
	{
		return 0;
	}

	JPH::EMotionType motionType;
	switch( motion )
	{
	case MOTION_STATIC:
		motionType = JPH::EMotionType::Static;
		break;
	case MOTION_KINEMATIC:
		motionType = JPH::EMotionType::Kinematic;
		break;
	case MOTION_DYNAMIC:
	case MOTION_UPRIGHT:
		motionType = JPH::EMotionType::Dynamic;
		break;
	default:
		return 0;
	}
	const bool moving = motionType != JPH::EMotionType::Static;
	if( moving && mass > 0.f && ( mass < MIN_MASS || mass > MAX_MASS ) )
	{
		return 0;
	}

	JPH::ShapeSettings::ShapeResult shape = shapeSettings.Create();
	if( shape.HasError() )
	{
		CCP_LOGWARN( "Tr2PhysicsWorld: a shape could not be made: %s", shape.GetError().c_str() );
		return 0;
	}

	JPH::BodyCreationSettings settings( shape.Get(), ToJoltPosition( position ), turn, motionType, moving ? LAYER_MOVING : LAYER_STATIC );
	settings.mFriction = FRICTION;
	if( moving )
	{
		// Kinematic and dynamic bodies may be switched between the two (SetMotion), so each has a mass
		settings.mAllowDynamicOrKinematic = true;
		settings.mMotionQuality = JPH::EMotionQuality::LinearCast;	// a thrown thing does not pass through a thin wall
		if( mass > 0.f )
		{
			settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
			settings.mMassPropertiesOverride.mMass = mass;
		}
		if( motion == MOTION_UPRIGHT )
		{
			settings.mAllowedDOFs = JPH::EAllowedDOFs::TranslationX | JPH::EAllowedDOFs::TranslationY | JPH::EAllowedDOFs::TranslationZ;
		}
	}

	const JPH::BodyID id = m_simulation->system.GetBodyInterface().CreateAndAddBody( settings, moving ? JPH::EActivation::Activate : JPH::EActivation::DontActivate );
	if( id.IsInvalid() )
	{
		CCP_LOGWARN( "Tr2PhysicsWorld: no room for another body (%u at most)", MAX_BODIES );
		return 0;
	}

	const int handle = m_simulation->nextId++;
	Body body;
	body.id = id;
	body.upright = motion == MOTION_UPRIGHT;
	m_simulation->bodies.emplace( handle, body );
	return handle;
}

bool Tr2PhysicsWorld::RemoveBody( int id )
{
	if( !m_simulation )
	{
		return false;
	}
	const auto it = m_simulation->bodies.find( id );
	if( it == m_simulation->bodies.end() )
	{
		return false;
	}
	JPH::BodyInterface& bodyInterface = m_simulation->system.GetBodyInterface();
	bodyInterface.RemoveBody( it->second.id );
	bodyInterface.DestroyBody( it->second.id );
	m_simulation->bodies.erase( it );
	return true;
}

bool Tr2PhysicsWorld::HasBody( int id ) const
{
	return Find( id ) != nullptr;
}

void Tr2PhysicsWorld::Clear()
{
	if( m_simulation )
	{
		m_simulation->RemoveAll();
	}
}

int Tr2PhysicsWorld::GetMotion( int id ) const
{
	const Body* body = Find( id );
	if( !body )
	{
		return -1;
	}
	switch( m_simulation->system.GetBodyInterface().GetMotionType( body->id ) )
	{
	case JPH::EMotionType::Static:
		return MOTION_STATIC;
	case JPH::EMotionType::Kinematic:
		return MOTION_KINEMATIC;
	default:
		return body->upright ? MOTION_UPRIGHT : MOTION_DYNAMIC;
	}
}

bool Tr2PhysicsWorld::SetMotion( int id, int motion )
{
	const Body* body = FindMoving( id );
	if( !body )
	{
		return false;
	}
	JPH::EMotionType motionType;
	if( motion == MOTION_KINEMATIC )
	{
		motionType = JPH::EMotionType::Kinematic;
	}
	else if( motion == ( body->upright ? MOTION_UPRIGHT : MOTION_DYNAMIC ) )
	{
		motionType = JPH::EMotionType::Dynamic;
	}
	else
	{
		return false;	// static stays static, and a body's freedom to turn is set as it is made
	}
	m_simulation->system.GetBodyInterface().SetMotionType( body->id, motionType, JPH::EActivation::Activate );
	return true;
}

std::pair<Vector3, Quaternion> Tr2PhysicsWorld::GetTransform( int id ) const
{
	const Body* body = Find( id );
	if( !body )
	{
		return { Vector3( 0.f, 0.f, 0.f ), Quaternion( 0.f, 0.f, 0.f, 1.f ) };
	}
	JPH::RVec3 position;
	JPH::Quat rotation;
	m_simulation->system.GetBodyInterface().GetPositionAndRotation( body->id, position, rotation );
	return {
		Vector3( static_cast<float>( position.GetX() ), static_cast<float>( position.GetY() ), static_cast<float>( position.GetZ() ) ),
		Quaternion( rotation.GetX(), rotation.GetY(), rotation.GetZ(), rotation.GetW() ) };
}

bool Tr2PhysicsWorld::SetTransform( int id, const Vector3& position, const Quaternion& rotation )
{
	const Body* body = Find( id );
	JPH::Quat turn;
	if( !body || !AllFinite( position ) || !ToJoltRotation( rotation, turn ) )
	{
		return false;
	}
	JPH::BodyInterface& bodyInterface = m_simulation->system.GetBodyInterface();
	const bool moving = bodyInterface.GetMotionType( body->id ) != JPH::EMotionType::Static;
	bodyInterface.SetPositionAndRotation( body->id, ToJoltPosition( position ), turn, moving ? JPH::EActivation::Activate : JPH::EActivation::DontActivate );
	return true;
}

bool Tr2PhysicsWorld::MoveKinematic( int id, const Vector3& position, const Quaternion& rotation, float seconds )
{
	const Body* body = Find( id );
	JPH::Quat turn;
	if( !body || !AllFinite( position ) || !ToJoltRotation( rotation, turn ) || !std::isfinite( seconds ) || seconds < MIN_SECONDS || seconds > MAX_SECONDS )
	{
		return false;
	}
	JPH::BodyInterface& bodyInterface = m_simulation->system.GetBodyInterface();
	if( bodyInterface.GetMotionType( body->id ) != JPH::EMotionType::Kinematic )
	{
		return false;
	}
	bodyInterface.MoveKinematic( body->id, ToJoltPosition( position ), turn, seconds );
	return true;
}

Vector3 Tr2PhysicsWorld::GetLinearVelocity( int id ) const
{
	const Body* body = Find( id );
	return body ? FromJolt( m_simulation->system.GetBodyInterface().GetLinearVelocity( body->id ) ) : Vector3( 0.f, 0.f, 0.f );
}

bool Tr2PhysicsWorld::SetLinearVelocity( int id, const Vector3& velocity )
{
	const Body* body = FindMoving( id );
	if( !body || !Within( velocity, MAX_SPEED ) )
	{
		return false;
	}
	m_simulation->system.GetBodyInterface().SetLinearVelocity( body->id, ToJolt( velocity ) );	// wakes it
	return true;
}

Vector3 Tr2PhysicsWorld::GetAngularVelocity( int id ) const
{
	const Body* body = Find( id );
	return body ? FromJolt( m_simulation->system.GetBodyInterface().GetAngularVelocity( body->id ) ) : Vector3( 0.f, 0.f, 0.f );
}

bool Tr2PhysicsWorld::SetAngularVelocity( int id, const Vector3& velocity )
{
	const Body* body = FindMoving( id );
	if( !body || !Within( velocity, MAX_TURN_RATE ) )
	{
		return false;
	}
	JPH::BodyInterface& bodyInterface = m_simulation->system.GetBodyInterface();
	bodyInterface.SetAngularVelocity( body->id, ToJolt( velocity ) );	// an upright body's allowed freedoms drop it
	bodyInterface.ActivateBody( body->id );
	return true;
}

bool Tr2PhysicsWorld::IsSleeping( int id ) const
{
	const Body* body = Find( id );
	return body && !m_simulation->system.GetBodyInterface().IsActive( body->id );
}

bool Tr2PhysicsWorld::Step( float seconds )
{
	TRINITY_STATS_ZONE( "Tr2PhysicsWorld::Step" );

	if( !m_simulation || !std::isfinite( seconds ) || seconds < MIN_SECONDS || seconds > MAX_SECONDS )
	{
		return false;
	}
	const int collisionSteps = std::clamp( static_cast<int>( std::ceil( seconds / MAX_STEP - 1e-4f ) ), 1, MAX_COLLISION_STEPS );
	const JPH::EPhysicsUpdateError error =
		m_simulation->system.Update( seconds, collisionSteps, &m_simulation->tempAllocator, &m_simulation->jobSystem );
	if( error != JPH::EPhysicsUpdateError::None )
	{
		CCP_LOGWARN( "Tr2PhysicsWorld: the step ran out of room (error flags %u)", static_cast<unsigned>( error ) );
		return false;
	}
	return true;
}
