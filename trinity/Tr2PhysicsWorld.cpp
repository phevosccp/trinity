// Copyright © 2026 CCP ehf.

#include "StdAfx.h"
#include "Tr2PhysicsWorld.h"

#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

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
constexpr float MIN_HALF_EXTENT = 1e-3f;	// m; thinner boxes are refused
constexpr float MAX_HALF_EXTENT = 1000.f;	// m
constexpr float PENETRATION_SLOP = 0.005f;	// m bodies may rest sunk into each other; Jolt's 2 cm shows at a crate's scale

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

JPH::Vec3 ToJolt( const Vector3& v )
{
	return JPH::Vec3( v.x, v.y, v.z );
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

	const JPH::BodyID* Find( int id ) const
	{
		const auto it = bodies.find( id );
		return it == bodies.end() ? nullptr : &it->second;
	}

	void RemoveAll()
	{
		JPH::BodyInterface& bodyInterface = system.GetBodyInterface();
		for( const auto& body : bodies )
		{
			bodyInterface.RemoveBody( body.second );
			bodyInterface.DestroyBody( body.second );
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

	std::unordered_map<int, JPH::BodyID> bodies;	// by the id the caller holds
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

const JPH::BodyID* Tr2PhysicsWorld::Find( int id ) const
{
	return m_simulation ? m_simulation->Find( id ) : nullptr;
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

	if( !m_simulation )
	{
		return 0;
	}
	JPH::Quat turn;
	if( !AllFinite( halfExtents ) || !AllFinite( position ) || !ToJoltRotation( rotation, turn ) || !std::isfinite( mass ) )
	{
		return 0;
	}
	if( motion == MOTION_DYNAMIC && mass > 0.f && ( mass < MIN_MASS || mass > MAX_MASS ) )
	{
		return 0;
	}
	const float smallest = std::min( { halfExtents.x, halfExtents.y, halfExtents.z } );
	const float largest = std::max( { halfExtents.x, halfExtents.y, halfExtents.z } );
	if( smallest < MIN_HALF_EXTENT || largest > MAX_HALF_EXTENT )
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
		motionType = JPH::EMotionType::Dynamic;
		break;
	default:
		return 0;
	}

	// The rounding of the box's edges, kept under half its thinnest side
	const float convexRadius = std::min( JPH::cDefaultConvexRadius, 0.5f * smallest );
	JPH::BoxShapeSettings shapeSettings( ToJolt( halfExtents ), convexRadius );
	JPH::ShapeSettings::ShapeResult shape = shapeSettings.Create();
	if( shape.HasError() )
	{
		CCP_LOGWARN( "Tr2PhysicsWorld: a box could not be made: %s", shape.GetError().c_str() );
		return 0;
	}

	JPH::BodyCreationSettings settings(
		shape.Get(),
		JPH::RVec3( position.x, position.y, position.z ),
		turn,
		motionType,
		motionType == JPH::EMotionType::Static ? LAYER_STATIC : LAYER_MOVING );
	if( motionType == JPH::EMotionType::Dynamic && mass > 0.f )
	{
		settings.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
		settings.mMassPropertiesOverride.mMass = mass;
	}

	const JPH::BodyID body = m_simulation->system.GetBodyInterface().CreateAndAddBody(
		settings,
		motionType == JPH::EMotionType::Static ? JPH::EActivation::DontActivate : JPH::EActivation::Activate );
	if( body.IsInvalid() )
	{
		CCP_LOGWARN( "Tr2PhysicsWorld: no room for another body (%u at most)", MAX_BODIES );
		return 0;
	}

	const int id = m_simulation->nextId++;
	m_simulation->bodies.emplace( id, body );
	return id;
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
	bodyInterface.RemoveBody( it->second );
	bodyInterface.DestroyBody( it->second );
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

std::pair<Vector3, Quaternion> Tr2PhysicsWorld::GetTransform( int id ) const
{
	const JPH::BodyID* body = Find( id );
	if( !body )
	{
		return { Vector3( 0.f, 0.f, 0.f ), Quaternion( 0.f, 0.f, 0.f, 1.f ) };
	}
	JPH::RVec3 position;
	JPH::Quat rotation;
	m_simulation->system.GetBodyInterface().GetPositionAndRotation( *body, position, rotation );
	return {
		Vector3( static_cast<float>( position.GetX() ), static_cast<float>( position.GetY() ), static_cast<float>( position.GetZ() ) ),
		Quaternion( rotation.GetX(), rotation.GetY(), rotation.GetZ(), rotation.GetW() ) };
}

Vector3 Tr2PhysicsWorld::GetLinearVelocity( int id ) const
{
	const JPH::BodyID* body = Find( id );
	return body ? FromJolt( m_simulation->system.GetBodyInterface().GetLinearVelocity( *body ) ) : Vector3( 0.f, 0.f, 0.f );
}

bool Tr2PhysicsWorld::SetLinearVelocity( int id, const Vector3& velocity )
{
	const JPH::BodyID* body = Find( id );
	if( !body || !AllFinite( velocity ) )
	{
		return false;
	}
	JPH::BodyInterface& bodyInterface = m_simulation->system.GetBodyInterface();
	if( bodyInterface.GetMotionType( *body ) == JPH::EMotionType::Static )
	{
		return false;
	}
	bodyInterface.SetLinearVelocity( *body, ToJolt( velocity ) );	// wakes it
	return true;
}

bool Tr2PhysicsWorld::IsSleeping( int id ) const
{
	const JPH::BodyID* body = Find( id );
	return body && !m_simulation->system.GetBodyInterface().IsActive( *body );
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
