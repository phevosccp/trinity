// Copyright © 2026 CCP ehf.

// Jolt's inline functions as this file compiles them are the ones the linker keeps, its library's own calls to them
// included (this file's objects come before the library).  MSVC 14.52 (Visual Studio 18 Insiders) miscompiles some of
// them: bodies turned towards 90 degrees about x or z sink through what they rest on, as a Jolt built by it does.  So
// with that compiler they are compiled unoptimised, as Jolt's library is built with VS 2022's 14.44; this file's own
// code is optimised as ever, and so is the standard library's, included first.  It costs a little: a step of 50 boxes
// falling into a pile took 0.101 ms median, against 0.095 ms with everything optimised (2026-11-06).  Retest this with
// a released VS 18 compiler and narrow the condition if it builds Jolt right.
#include <algorithm>
#include <atomic>
#include <cfloat>
#include <climits>
#include <cmath>
#include <condition_variable>
#include <cstddef>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <functional>
#include <limits>
#include <memory>
#include <mutex>
#include <new>
#include <sstream>
#include <string>
#include <string_view>
#include <thread>
#include <type_traits>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>
#if defined( _MSC_VER ) && !defined( __clang__ ) && _MSC_VER >= 1950
#define TRINITY_JOLT_HEADERS_UNOPTIMISED
#pragma optimize( "", off )
#endif
#include <Jolt/Jolt.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemSingleThreaded.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/AllowedDOFs.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Physics/Character/CharacterVirtual.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/CollideShape.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/CapsuleShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/Constraints/SwingTwistConstraint.h>
#include <Jolt/Physics/PhysicsSettings.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Ragdoll/Ragdoll.h>
#include <Jolt/Physics/SoftBody/SoftBodyCreationSettings.h>
#include <Jolt/Physics/SoftBody/SoftBodyMotionProperties.h>
#include <Jolt/Physics/SoftBody/SoftBodySharedSettings.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Skeleton/Skeleton.h>
#ifdef TRINITY_JOLT_HEADERS_UNOPTIMISED
#pragma optimize( "", on )
#endif

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
constexpr size_t MIN_HULL_POINTS = 4;	// a hull needs a volume
constexpr size_t MAX_HULL_POINTS = 1024;	// given; Jolt keeps at most 256 of them on the hull it makes
constexpr float MIN_HULL_VOLUME = 1e-9f;	// m^3: a millimetre cube; flatter is a plane, which Jolt makes a hull of anyway
constexpr float MAX_DAMPING = 100.f;	// 1/s
constexpr float MAX_IMPULSE = 1e9f;	// N s; Jolt caps the velocity it gives (MAX_SPEED, MAX_TURN_RATE) whatever it is
constexpr float MAX_RAY = 100000.f;	// m, a cast's length at most
constexpr size_t MAX_MESH_VERTICES = 1000000;
constexpr size_t MAX_MESH_TRIANGLES = 1000000;
constexpr size_t MIN_ROPE_POINTS = 3;	// two rods, the fewest a bend joins
constexpr size_t MAX_ROPE_POINTS = 1024;
constexpr float MIN_ROPE_SPACING = 1e-3f;	// m between neighbouring points; Jolt divides by a rod's length
constexpr float MAX_ROPE_COMPLIANCE = 1e6f;	// 1/N: limp long before this
constexpr JPH::uint ROPE_ITERATIONS = 10;	// solver passes each collision step; Jolt's 5 leaves a long rope stretchy
constexpr float ROPE_DAMPING = 0.2f;	// 1/s: a swing dies away over some seconds
constexpr float ROPE_FRICTION = 0.4f;	// drags on what it lies on, and still slides off an edge
constexpr float ROPE_MAX_SPEED = 60.f;	// m/s a point may fall or whip at: wedged in a mesh's triangles, the solver flung one at 231
constexpr size_t MAX_STATIC_BOXES = 65536;	// in one AddStaticBoxes
// m the edges of AddStaticBoxes are rounded by.  Little enough for a walker to slide over the seam between two boxes (the
// 5 cm of AddBox catches it); and not none, as Jolt takes a square corner met by a capsule's foot for the box's top or
// its side by the axis it lies along, and only sees a step to walk up in its side
constexpr float STATIC_BOX_ROUNDING = 0.01f;
constexpr size_t MAX_STANCES = 8;
constexpr float MIN_CHARACTER_RADIUS = 0.01f;	// m
constexpr float MAX_CHARACTER_SIZE = 100.f;	// m
constexpr float INNER_BODY_SHARE = 0.9f;	// of a stance's size, so the character's own shape meets the world first, as Jolt's sample has it
constexpr float FIT_DEPTH = 0.01f;	// m a stance may sink into what is round it and still fit: past the padding Jolt keeps
// m stepped forward onto a step at least.  Jolt's 2 cm is for a walker of 6 m/s: at 1.6 m/s a capsule then lands on the
// step's edge on the steep part of its round foot, is stopped by it and rocks back and forth at the kerb
constexpr float MIN_STEP_FORWARD = 0.1f;
constexpr float STEP_FORWARD_TEST = 0.15f;	// m ahead a step's top is looked at when the near side of it is too steep (Jolt's)
constexpr float STEP_FORWARD_COS = 0.2588f;	// cos 75 degrees: a wall further round than this from where the feet go is looked past (Jolt's)
constexpr float MIN_GROUND_NORMAL_Y = 0.1f;	// flatter ground than this is not walked along
constexpr float PRESSING_DISTANCE = 0.02f;	// m: a character this near a body, moving towards it, is pressing on it
constexpr float MIN_SLOPE_DEGREES = 1.f;	// below about 0.8, Jolt takes a slope limit for none at all
constexpr float MAX_CHARACTER_GRAVITY = 1000.f;	// m/s^2
constexpr size_t MAX_RAGDOLL_PARTS = 64;
constexpr float RAGDOLL_SEPARATION = 0.02f;	// m: parts nearer than this as the ragdoll is built never meet, as the two thighs
constexpr float RAGDOLL_ANGULAR_DAMPING = 0.5f;	// 1/s: a limp limb swings a few times, not for a minute
constexpr float RAGDOLL_MAX_SPEED = 30.f;	// m/s a part goes at most: a joint pulled apart by a hard hit would fling it faster
constexpr float RAGDOLL_MAX_TURN_RATE = 30.f;	// rad/s
constexpr float RAGDOLL_FRICTION = 3.f;	// N m each joint resists turning by while limp, until set: a body settles
constexpr float MAX_RAGDOLL_FRICTION = 10000.f;	// N m
constexpr float MAX_AXIS_SKEW = 0.01f;	// how far from square a joint's twist and plane axes may be given (their dot)

// Static bodies never meet each other; moving ones (kinematic and dynamic) meet the static and moving ones of their own
// group.  A character's inner body meets moving bodies of every group.
constexpr int GROUPS = 4;
constexpr int ANY_GROUP = -1;	// to a query: where a character looks
constexpr JPH::ObjectLayer LAYER_CHARACTER = 2 * GROUPS;
constexpr JPH::uint OBJECT_LAYERS = 2 * GROUPS + 1;
constexpr JPH::uint BROAD_PHASE_LAYERS = 2;

constexpr JPH::ObjectLayer StaticLayer( int group )
{
	return static_cast<JPH::ObjectLayer>( 2 * group );
}

constexpr JPH::ObjectLayer MovingLayer( int group )
{
	return static_cast<JPH::ObjectLayer>( 2 * group + 1 );
}

// A body's user data: the id the caller holds, and above it the tags queries may look past
constexpr int TAG_SHIFT = 32;
constexpr JPH::uint32 TAG_CHARACTER = 0x80000000u;	// a character's inner body, which no query meets

constexpr JPH::uint64 UserData( int handle, JPH::uint32 tags )
{
	return static_cast<JPH::uint64>( static_cast<JPH::uint32>( handle ) ) | ( static_cast<JPH::uint64>( tags ) << TAG_SHIFT );
}

constexpr int HandleOf( JPH::uint64 userData )
{
	return static_cast<int>( userData & 0xffffffffu );
}

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

Vector3 FromJoltPosition( JPH::RVec3Arg v )
{
	return Vector3( static_cast<float>( v.GetX() ), static_cast<float>( v.GetY() ), static_cast<float>( v.GetZ() ) );
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
		for( int group = 0; group < GROUPS; ++group )
		{
			EnableCollision( MovingLayer( group ), StaticLayer( group ) );
			EnableCollision( MovingLayer( group ), MovingLayer( group ) );
			EnableCollision( MovingLayer( group ), LAYER_CHARACTER );
		}
	}
};

// Static bodies in one broad-phase tree, moving ones in another
class BroadPhaseLayers final : public JPH::BroadPhaseLayerInterfaceTable
{
public:
	BroadPhaseLayers()
		: BroadPhaseLayerInterfaceTable( OBJECT_LAYERS, BROAD_PHASE_LAYERS )
	{
		for( int group = 0; group < GROUPS; ++group )
		{
			MapObjectToBroadPhaseLayer( StaticLayer( group ), JPH::BroadPhaseLayer( 0 ) );
			MapObjectToBroadPhaseLayer( MovingLayer( group ), JPH::BroadPhaseLayer( 1 ) );
		}
		MapObjectToBroadPhaseLayer( LAYER_CHARACTER, JPH::BroadPhaseLayer( 1 ) );
	}
};

// The layers a query looks in: one group's, or where a character looks (group 0's static bodies, every moving one)
class QueryLayers final : public JPH::ObjectLayerFilter
{
public:
	explicit QueryLayers( int group )
		: m_group( group )
	{
	}

	bool ShouldCollide( JPH::ObjectLayer layer ) const override
	{
		if( m_group != ANY_GROUP )
		{
			return layer == StaticLayer( m_group ) || layer == MovingLayer( m_group );
		}
		return layer == StaticLayer( 0 ) || ( layer != LAYER_CHARACTER && layer % 2 == 1 );
	}

private:
	int m_group;
};

// The bodies a query looks past: those named, those with a tag given, and always the inner bodies of characters
class QueryBodies final : public JPH::BodyFilter
{
public:
	explicit QueryBodies( const std::vector<JPH::BodyID>& ignored, JPH::uint32 ignoredTags = 0 )
		: m_ignored( ignored ),
		  m_ignoredTags( ignoredTags | TAG_CHARACTER )
	{
	}

	bool ShouldCollide( const JPH::BodyID& id ) const override
	{
		return std::find( m_ignored.begin(), m_ignored.end(), id ) == m_ignored.end();
	}

	bool ShouldCollideLocked( const JPH::Body& body ) const override
	{
		return ( ( body.GetUserData() >> TAG_SHIFT ) & m_ignoredTags ) == 0;
	}

private:
	const std::vector<JPH::BodyID>& m_ignored;
	JPH::uint32 m_ignoredTags;
};

// The first thing a sweep meets.  Something it starts in counts only if the move goes further into it, so a body
// resting on a floor still sees the wall ahead; then the hit is the deepest such.  Two faces just touching have no way
// out for Jolt to find, and it gives the direction of the sweep instead, so they do not count either
class SweepHits final : public JPH::CastShapeCollector
{
public:
	explicit SweepHits( JPH::Vec3Arg along )
		: m_along( along )
	{
	}

	void AddHit( const JPH::ShapeCastResult& result ) override
	{
		const bool startsIn = result.mFraction <= 0.f;
		if( startsIn && result.mPenetrationDepth <= 1e-4f )
		{
			return;
		}
		if( m_along.Dot( result.mPenetrationAxis.NormalizedOr( JPH::Vec3::sZero() ) ) <= 1e-3f * m_along.Length() )
		{
			return;	// along it or out of it
		}
		const float earlyOut = result.GetEarlyOutFraction();
		if( earlyOut < GetEarlyOutFraction() )
		{
			UpdateEarlyOutFraction( earlyOut );
			hit = result;
			found = true;
		}
	}

	JPH::ShapeCastResult hit;
	bool found = false;

private:
	JPH::Vec3 m_along;
};

// Whether anything reaches into a shape further than a depth
class DeepContact final : public JPH::CollideShapeCollector
{
public:
	void AddHit( const JPH::CollideShapeResult& result ) override
	{
		if( result.mPenetrationDepth > FIT_DEPTH )
		{
			found = true;
			ForceEarlyOut();
		}
	}

	bool found = false;
};

// What a character's touching does to it.  Jolt lets anything moving into a character push it, whatever it weighs: a
// thrown cup would carry it along.  So a body lighter than the character only stops at it
class CharacterContacts final : public JPH::CharacterContactListener
{
public:
	// ragdollMass: kg of all of a ragdoll, by its id
	CharacterContacts( const JPH::BodyLockInterface& locks, std::function<float( int )> ragdollMass )
		: m_locks( locks ),
		  m_ragdollMass( std::move( ragdollMass ) )
	{
	}

	void OnContactAdded( const JPH::CharacterVirtual* character, const JPH::BodyID& body, const JPH::SubShapeID&, JPH::RVec3Arg, JPH::Vec3Arg normal, JPH::CharacterContactSettings& settings ) override
	{
		Decide( character, body, normal, settings );
	}

	void OnContactPersisted( const JPH::CharacterVirtual* character, const JPH::BodyID& body, const JPH::SubShapeID&, JPH::RVec3Arg, JPH::Vec3Arg normal, JPH::CharacterContactSettings& settings ) override
	{
		Decide( character, body, normal, settings );
	}

private:
	void Decide( const JPH::CharacterVirtual* character, const JPH::BodyID& id, JPH::Vec3Arg normal, JPH::CharacterContactSettings& settings ) const
	{
		JPH::BodyLockRead lock( m_locks, id );
		if( !lock.Succeeded() )
		{
			return;
		}
		const JPH::Body& body = lock.GetBody();
		// Only a ragdoll's parts are in a collision group, and it is the ragdoll's id
		const JPH::CollisionGroup::GroupID ragdoll = body.GetCollisionGroup().GetGroupID();
		const bool part = ragdoll != JPH::CollisionGroup::cInvalidGroup;
		if( !body.IsDynamic() )
		{
			if( part )
			{
				settings.mCanPushCharacter = false;	// a keyed limb swings through its clip: it carries no walker off
			}
			return;
		}
		// A limp ragdoll is one thing to a walker, as heavy as all of it: a forearm alone would count as a cup
		const bool lighter = part ? m_ragdollMass( static_cast<int>( ragdoll ) ) < character->GetMass() : body.GetMotionProperties()->GetInverseMass() * character->GetMass() > 1.f;
		// The normal Jolt gives here runs from the character to the body, so towards the character is below 0
		settings.mCanPushCharacter = !lighter || body.GetLinearVelocity().Dot( normal ) >= 0.f;
	}

	const JPH::BodyLockInterface& m_locks;
	std::function<float( int )> m_ragdollMass;
};

// What keeps a character's inner body from meeting what the character passes through (SetCharacterIgnored)
class InnerBodyContacts final : public JPH::ContactListener
{
public:
	explicit InnerBodyContacts( std::function<bool( int, const JPH::BodyID& )> ignores )
		: m_ignores( std::move( ignores ) )
	{
	}

	JPH::ValidateResult OnContactValidate( const JPH::Body& first, const JPH::Body& second, JPH::RVec3Arg, const JPH::CollideShapeResult& ) override
	{
		const bool firstIsInner = ( first.GetUserData() >> TAG_SHIFT & TAG_CHARACTER ) != 0;
		const bool secondIsInner = ( second.GetUserData() >> TAG_SHIFT & TAG_CHARACTER ) != 0;
		if( !firstIsInner && !secondIsInner )
		{
			return JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
		}
		const JPH::Body& inner = firstIsInner ? first : second;
		const JPH::Body& other = firstIsInner ? second : first;
		return m_ignores( HandleOf( inner.GetUserData() ), other.GetID() ) ? JPH::ValidateResult::RejectAllContactsForThisBodyPair : JPH::ValidateResult::AcceptAllContactsForThisBodyPair;
	}

private:
	std::function<bool( int, const JPH::BodyID& )> m_ignores;
};

// A stance's shape at share of its size, its feet at the origin (or, smaller, its middle where the full size's is)
JPH::RefConst<JPH::Shape> StanceShape( const Vector3& stance, float share )
{
	const float radius = stance.x * share;
	const float height = stance.y * share;
	JPH::ShapeSettings::ShapeResult shape;
	if( stance.z != 0.f )
	{
		const float convexRadius = std::min( JPH::cDefaultConvexRadius, 0.5f * std::min( radius, 0.5f * height ) );
		shape = JPH::CylinderShapeSettings( 0.5f * height, radius, convexRadius ).Create();
	}
	else
	{
		shape = JPH::CapsuleShapeSettings( 0.5f * height - radius, radius ).Create();
	}
	if( shape.HasError() )
	{
		return nullptr;
	}
	const JPH::ShapeSettings::ShapeResult placed = JPH::RotatedTranslatedShapeSettings( JPH::Vec3( 0.f, 0.5f * stance.y, 0.f ), JPH::Quat::sIdentity(), shape.Get() ).Create();
	return placed.HasError() ? nullptr : placed.Get();
}

// A prop set going at velocity along the ground, by no more than impulse at a time.  Jolt pushes from where the character
// touches it, which on a low prop is the round of the foot on its top edge: that only rocks it, or presses it to the floor
void Shove( JPH::PhysicsSystem& system, const JPH::BodyID& prop, JPH::Vec3Arg velocity, float impulse )
{
	const float speed = velocity.Length();
	if( !( speed > 0.f ) )
	{
		return;
	}
	const JPH::Vec3 along = velocity / speed;
	JPH::BodyInterface& bodyInterface = system.GetBodyInterface();
	const float missing = speed - bodyInterface.GetLinearVelocity( prop ).Dot( along );
	if( missing > 0.f )
	{
		float mass = 0.f;
		{
			JPH::BodyLockRead lock( system.GetBodyLockInterface(), prop );
			if( !lock.Succeeded() )
			{
				return;
			}
			mass = 1.f / lock.GetBody().GetMotionProperties()->GetInverseMass();
		}
		bodyInterface.AddImpulse( prop, along * std::min( mass * missing, impulse ) );
		bodyInterface.ActivateBody( prop );
	}
}

// The dynamic body a character walking at velocity is pressing on and strong enough to slide, if any: a prop to shove,
// not a step to walk up nor a wall to stop at
JPH::BodyID PropShoved( const JPH::CharacterVirtual& character, JPH::Vec3Arg velocity, const JPH::BodyLockInterface& locks, float gravity )
{
	const JPH::Vec3 flat = velocity - velocity.GetY() * JPH::Vec3::sAxisY();
	for( const JPH::CharacterVirtual::Contact& contact : character.GetActiveContacts() )
	{
		// Not only what it hit this move: going slowly it comes to rest just short of a prop, and hits it only now and then
		if( contact.mWasDiscarded || ( !contact.mHadCollision && contact.mDistance > PRESSING_DISTANCE ) || contact.mMotionTypeB != JPH::EMotionType::Dynamic || contact.mContactNormal.Dot( flat ) >= 0.f )
		{
			continue;
		}
		JPH::BodyLockRead lock( locks, contact.mBodyB );
		if( !lock.Succeeded() )
		{
			continue;
		}
		if( lock.GetBody().GetCollisionGroup().GetGroupID() != JPH::CollisionGroup::cInvalidGroup )
		{
			continue;	// a ragdoll's part: slid along by one limb, the rest would trail a walker counted as walking freely
		}
		const JPH::MotionProperties* motion = lock.GetBody().GetMotionProperties();
		const float weight = gravity * motion->GetGravityFactor() / motion->GetInverseMass();
		if( FRICTION * weight <= character.GetMaxStrength() )
		{
			return contact.mBodyB;
		}
	}
	return JPH::BodyID();
}
}

struct Tr2PhysicsWorld::Body
{
	JPH::BodyID id;
	bool upright = false;	// made with MOTION_UPRIGHT: never turns, whatever moves it
	bool rope = false;	// made by AddRope: a soft body, which the methods for bodies leave alone
	float pointInvMass = 0.f;	// a rope's: what a point goes back to when it is let go
	int group = 0;
	int ragdoll = 0;	// the ragdoll whose part it is, 0 for a body of its own
	int part = -1;	// which of the ragdoll's parts
};

struct Tr2PhysicsWorld::Character
{
	Character( const JPH::BodyLockInterface& locks, std::function<float( int )> ragdollMass )
		: contacts( locks, std::move( ragdollMass ) )
	{
	}

	CharacterContacts contacts;	// outlives the character, which holds it by pointer
	JPH::Ref<JPH::CharacterVirtual> character;
	std::vector<JPH::RefConst<JPH::Shape>> stances;
	std::vector<JPH::RefConst<JPH::Shape>> innerStances;
	std::vector<JPH::BodyID> ignored;
	JPH::uint32 passedTags = 0;	// what it walks through, its inner body knocking it aside
};

struct Tr2PhysicsWorld::Ragdoll
{
	struct Part
	{
		int parent = -1;
		JPH::RefConst<JPH::Shape> shape;
		JPH::RVec3 position = JPH::RVec3::sZero();
		JPH::Quat rotation = JPH::Quat::sIdentity();
		float mass = 0.f;
		JPH::Ref<JPH::SwingTwistConstraintSettings> joint;	// to its parent: none for the root, nor until SetRagdollJoint
		int handle = 0;	// its own body's id, once built
	};

	std::vector<Part> parts;
	JPH::Ref<JPH::Ragdoll> ragdoll;	// none until built
	int group = 0;
	int mode = RAGDOLL_KEYED;
	float friction = RAGDOLL_FRICTION;
	float mass = 0.f;	// kg of all its parts
};

struct Tr2PhysicsWorld::Simulation
{
	// layerVsBroadPhase reads the other two as it is made, so they come first
	Simulation()
		: layerVsBroadPhase( broadPhaseLayers, BROAD_PHASE_LAYERS, layerPairs, OBJECT_LAYERS ),
		  tempAllocator( TEMP_MEMORY ),
		  jobSystem( JPH::cMaxPhysicsJobs ),
		  innerBodyContacts( [this]( int handle, const JPH::BodyID& body ) { return Ignores( handle, body ); } )
	{
		system.Init( MAX_BODIES, 0, MAX_BODY_PAIRS, MAX_CONTACT_CONSTRAINTS, broadPhaseLayers, layerVsBroadPhase, layerPairs );
		system.SetContactListener( &innerBodyContacts );
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
		return it == bodies.end() || it->second.rope ? nullptr : &it->second;
	}

	bool Ignores( int character, const JPH::BodyID& body ) const
	{
		const auto it = characters.find( character );
		if( it == characters.end() )
		{
			return false;
		}
		const std::vector<JPH::BodyID>& ignored = it->second->ignored;
		return std::find( ignored.begin(), ignored.end(), body ) != ignored.end();
	}

	float RagdollMass( int ragdoll ) const
	{
		const auto it = ragdolls.find( ragdoll );
		return it != ragdolls.end() ? it->second->mass : 0.f;
	}

	void RemoveFromWorld( const JPH::BodyID& body )
	{
		JPH::BodyInterface& bodyInterface = system.GetBodyInterface();
		bodyInterface.RemoveBody( body );
		bodyInterface.DestroyBody( body );
		Forget( body );
	}

	// A body gone from the world: no character ignores it, or stands on it, any more
	void Forget( const JPH::BodyID& body )
	{
		const JPH::BroadPhaseLayerFilter broadPhase;
		const QueryLayers layers( ANY_GROUP );
		const JPH::ShapeFilter shapes;
		for( auto& entry : characters )
		{
			Character& character = *entry.second;
			character.ignored.erase( std::remove( character.ignored.begin(), character.ignored.end(), body ), character.ignored.end() );
			if( character.character->GetGroundBodyID() == body )
			{
				const QueryBodies bodies( character.ignored, character.passedTags );
				character.character->RefreshContacts( broadPhase, layers, bodies, shapes, tempAllocator );
			}
		}
	}

	// A ragdoll goes whole: its joints first, then its parts, which are its own to destroy
	void RemoveRagdoll( int handle )
	{
		const auto it = ragdolls.find( handle );
		if( it == ragdolls.end() )
		{
			return;
		}
		const std::unique_ptr<Ragdoll> gone = std::move( it->second );
		ragdolls.erase( it );
		if( !gone->ragdoll )
		{
			return;
		}
		gone->ragdoll->RemoveFromPhysicsSystem();
		const JPH::Array<JPH::BodyID> parts = gone->ragdoll->GetBodyIDs();
		for( const Ragdoll::Part& part : gone->parts )
		{
			bodies.erase( part.handle );
		}
		gone->ragdoll = nullptr;
		for( const JPH::BodyID& part : parts )
		{
			Forget( part );
		}
	}

	void RemoveAll()
	{
		characters.clear();	// each takes its inner body with it
		for( auto& ragdoll : ragdolls )
		{
			if( ragdoll.second->ragdoll )
			{
				ragdoll.second->ragdoll->RemoveFromPhysicsSystem();
			}
		}
		ragdolls.clear();	// each destroys its parts
		JPH::BodyInterface& bodyInterface = system.GetBodyInterface();
		for( const auto& body : bodies )
		{
			if( body.second.ragdoll == 0 )
			{
				bodyInterface.RemoveBody( body.second.id );
				bodyInterface.DestroyBody( body.second.id );
			}
		}
		bodies.clear();
	}

	// Declared in the order they are made: the filters outlive the system that holds references to them
	LayerPairs layerPairs;
	BroadPhaseLayers broadPhaseLayers;
	JPH::ObjectVsBroadPhaseLayerFilterTable layerVsBroadPhase;
	JPH::TempAllocatorImplWithMallocFallback tempAllocator;	// the plain one aborts when it runs out
	JPH::JobSystemSingleThreaded jobSystem;
	InnerBodyContacts innerBodyContacts;
	JPH::PhysicsSystem system;

	std::unordered_map<int, Body> bodies;	// by the id the caller holds
	std::unordered_map<int, std::unique_ptr<Character>> characters;	// the same ids
	std::unordered_map<int, std::unique_ptr<Ragdoll>> ragdolls;	// the same ids; each part's body is in bodies too
	int nextId = 1;
	int stepCount = 0;
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

const Tr2PhysicsWorld::Body* Tr2PhysicsWorld::FindLoose( int id ) const
{
	const Body* body = Find( id );
	return body && body->ragdoll == 0 ? body : nullptr;
}

Tr2PhysicsWorld::Ragdoll* Tr2PhysicsWorld::FindRagdoll( int id ) const
{
	if( !m_simulation )
	{
		return nullptr;
	}
	const auto it = m_simulation->ragdolls.find( id );
	return it != m_simulation->ragdolls.end() ? it->second.get() : nullptr;
}

Tr2PhysicsWorld::Ragdoll* Tr2PhysicsWorld::FindBuiltRagdoll( int id ) const
{
	Ragdoll* found = FindRagdoll( id );
	return found && found->ragdoll ? found : nullptr;
}

int Tr2PhysicsWorld::GetStepCount() const
{
	return m_simulation ? m_simulation->stepCount : 0;
}

const Tr2PhysicsWorld::Body* Tr2PhysicsWorld::FindRope( int id ) const
{
	if( !m_simulation )
	{
		return nullptr;
	}
	const auto it = m_simulation->bodies.find( id );
	return it != m_simulation->bodies.end() && it->second.rope ? &it->second : nullptr;
}

Tr2PhysicsWorld::Character* Tr2PhysicsWorld::FindCharacter( int id ) const
{
	if( !m_simulation )
	{
		return nullptr;
	}
	const auto it = m_simulation->characters.find( id );
	return it != m_simulation->characters.end() ? it->second.get() : nullptr;
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
	return m_simulation ? static_cast<int>( m_simulation->bodies.size() + m_simulation->characters.size() ) : 0;
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

int Tr2PhysicsWorld::AddConvexHull( const std::vector<Vector3>& points, const Vector3& position, const Quaternion& rotation, int motion, float mass )
{
	TRINITY_STATS_ZONE( "Tr2PhysicsWorld::AddConvexHull" );

	if( !m_simulation || points.size() < MIN_HULL_POINTS || points.size() > MAX_HULL_POINTS )
	{
		return 0;
	}
	JPH::Array<JPH::Vec3> corners;
	corners.reserve( points.size() );
	for( const Vector3& point : points )
	{
		if( !Within( point, MAX_HALF_EXTENT ) )
		{
			return 0;
		}
		corners.push_back( ToJolt( point ) );
	}
	// Jolt rounds the hull's edges by up to this, less on a hull too thin for it; the points stay its outside
	JPH::ConvexHullShapeSettings shape( corners, JPH::cDefaultConvexRadius );
	const JPH::ShapeSettings::ShapeResult made = shape.Create();	// kept by the settings, which AddBody makes it from
	if( made.HasError() || !( made.Get()->GetVolume() > MIN_HULL_VOLUME ) )
	{
		return 0;	// points all in one place, on a line or in a plane
	}
	return AddBody( shape, position, rotation, motion, mass );
}

int Tr2PhysicsWorld::AddMesh( const std::vector<Vector3>& vertices, const std::vector<int>& triangles, const Vector3& position, const Quaternion& rotation )
{
	TRINITY_STATS_ZONE( "Tr2PhysicsWorld::AddMesh" );

	if( !m_simulation || vertices.size() < 3 || vertices.size() > MAX_MESH_VERTICES )
	{
		return 0;
	}
	if( triangles.empty() || triangles.size() % 3 != 0 || triangles.size() / 3 > MAX_MESH_TRIANGLES )
	{
		return 0;
	}
	JPH::VertexList corners;
	corners.reserve( vertices.size() );
	for( const Vector3& vertex : vertices )
	{
		if( !AllFinite( vertex ) )
		{
			return 0;
		}
		corners.push_back( JPH::Float3( vertex.x, vertex.y, vertex.z ) );
	}
	JPH::IndexedTriangleList faces;
	faces.reserve( triangles.size() / 3 );
	for( size_t i = 0; i < triangles.size(); i += 3 )
	{
		for( size_t corner = 0; corner < 3; ++corner )
		{
			if( triangles[i + corner] < 0 || static_cast<size_t>( triangles[i + corner] ) >= vertices.size() )
			{
				return 0;
			}
		}
		faces.push_back( JPH::IndexedTriangle( triangles[i], triangles[i + 1], triangles[i + 2] ) );
	}
	JPH::MeshShapeSettings shape( std::move( corners ), std::move( faces ) );
	return AddBody( shape, position, rotation, MOTION_STATIC, 0.f );
}

int Tr2PhysicsWorld::AddRope( const std::vector<Vector3>& points, float radius, float mass, float bendCompliance )
{
	TRINITY_STATS_ZONE( "Tr2PhysicsWorld::AddRope" );

	if( !m_simulation || points.size() < MIN_ROPE_POINTS || points.size() > MAX_ROPE_POINTS )
	{
		return 0;
	}
	if( !std::isfinite( radius ) || radius < MIN_HALF_EXTENT || radius > MAX_HALF_EXTENT )
	{
		return 0;
	}
	if( !std::isfinite( mass ) || mass < MIN_MASS || mass > MAX_MASS )
	{
		return 0;
	}
	if( !std::isfinite( bendCompliance ) || bendCompliance < 0.f || bendCompliance > MAX_ROPE_COMPLIANCE )
	{
		return 0;
	}

	const float pointInvMass = static_cast<float>( points.size() ) / mass;
	JPH::Ref<JPH::SoftBodySharedSettings> shared = new JPH::SoftBodySharedSettings();
	shared->mVertices.reserve( points.size() );
	for( const Vector3& point : points )
	{
		if( !AllFinite( point ) )
		{
			return 0;
		}
		JPH::SoftBodySharedSettings::Vertex vertex;
		vertex.mPosition = JPH::Float3( point.x, point.y, point.z );
		vertex.mInvMass = pointInvMass;
		shared->mVertices.push_back( vertex );
	}
	float along = 0.f;
	for( size_t i = 0; i + 1 < points.size(); ++i )
	{
		const Vector3& a = points[i];
		const Vector3& b = points[i + 1];
		const float spacing = std::sqrt( ( b.x - a.x ) * ( b.x - a.x ) + ( b.y - a.y ) * ( b.y - a.y ) + ( b.z - a.z ) * ( b.z - a.z ) );
		if( spacing < MIN_ROPE_SPACING )
		{
			return 0;
		}
		along += spacing;
		const JPH::uint first = static_cast<JPH::uint>( i );
		shared->mRodStretchShearConstraints.push_back( JPH::SoftBodySharedSettings::RodStretchShear( first, first + 1 ) );
		if( i > 0 )
		{
			shared->mRodBendTwistConstraints.push_back( JPH::SoftBodySharedSettings::RodBendTwist( first - 1, first, bendCompliance ) );
		}
		// However far the solver lets the rods stretch, no point strays from the first by more than the rope to there
		shared->mLRAConstraints.push_back( JPH::SoftBodySharedSettings::LRA( 0, first + 1, along ) );
	}
	shared->CalculateRodProperties();
	for( JPH::SoftBodySharedSettings::RodBendTwist& bend : shared->mRodBendTwistConstraints )
	{
		bend.mOmega0 = JPH::Quat::sIdentity();	// at rest it lies straight, whatever shape it starts in
	}
	shared->Optimize();

	const int handle = m_simulation->nextId;
	JPH::SoftBodyCreationSettings settings( shared, JPH::RVec3::sZero(), JPH::Quat::sIdentity(), MovingLayer( 0 ) );
	settings.mUserData = UserData( handle, 0 );
	settings.mNumIterations = ROPE_ITERATIONS;
	settings.mLinearDamping = ROPE_DAMPING;
	settings.mMaxLinearVelocity = ROPE_MAX_SPEED;
	settings.mFriction = ROPE_FRICTION;
	settings.mVertexRadius = radius;
	settings.mUpdatePosition = false;	// the points move, the frame they are in doesn't: GetRopePoints stays in the caller's
	settings.mAllowSleeping = false;	// a pin the caller moves wakes nothing by itself

	const JPH::BodyID id = m_simulation->system.GetBodyInterface().CreateAndAddSoftBody( settings, JPH::EActivation::Activate );
	if( id.IsInvalid() )
	{
		CCP_LOGWARN( "Tr2PhysicsWorld: no room for another body (%u at most)", MAX_BODIES );
		return 0;
	}

	m_simulation->nextId++;
	Body body;
	body.id = id;
	body.rope = true;
	body.pointInvMass = pointInvMass;
	m_simulation->bodies.emplace( handle, body );
	return handle;
}

int Tr2PhysicsWorld::AddStaticBoxes( const std::vector<Vector3>& halfExtents, const std::vector<Vector3>& positions, const std::vector<Quaternion>& rotations )
{
	TRINITY_STATS_ZONE( "Tr2PhysicsWorld::AddStaticBoxes" );

	if( !m_simulation || halfExtents.empty() || halfExtents.size() > MAX_STATIC_BOXES || halfExtents.size() != positions.size() || halfExtents.size() != rotations.size() )
	{
		return 0;
	}
	JPH::StaticCompoundShapeSettings compound;
	for( size_t i = 0; i < halfExtents.size(); ++i )
	{
		const Vector3& half = halfExtents[i];
		JPH::Quat turn;
		if( !AllFinite( half ) || !AllFinite( positions[i] ) || !ToJoltRotation( rotations[i], turn ) )
		{
			return 0;
		}
		if( std::min( { half.x, half.y, half.z } ) < MIN_HALF_EXTENT || std::max( { half.x, half.y, half.z } ) > MAX_HALF_EXTENT )
		{
			return 0;
		}
		compound.AddShape( ToJolt( positions[i] ), turn, new JPH::BoxShape( ToJolt( half ), std::min( STATIC_BOX_ROUNDING, 0.5f * std::min( { half.x, half.y, half.z } ) ) ) );
	}
	return AddBody( compound, Vector3( 0.f, 0.f, 0.f ), Quaternion( 0.f, 0.f, 0.f, 1.f ), MOTION_STATIC, 0.f );
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

	const int handle = m_simulation->nextId;
	JPH::BodyCreationSettings settings( shape.Get(), ToJoltPosition( position ), turn, motionType, moving ? MovingLayer( 0 ) : StaticLayer( 0 ) );
	settings.mFriction = FRICTION;
	settings.mUserData = UserData( handle, 0 );	// CastRay names what it hits by it
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

	m_simulation->nextId++;
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
	if( m_simulation->characters.erase( id ) > 0 )
	{
		return true;
	}
	if( FindRagdoll( id ) )
	{
		m_simulation->RemoveRagdoll( id );
		return true;
	}
	const auto it = m_simulation->bodies.find( id );
	if( it == m_simulation->bodies.end() || it->second.ragdoll != 0 )
	{
		return false;	// a part leaves with its ragdoll
	}
	const JPH::BodyID body = it->second.id;
	m_simulation->bodies.erase( it );
	m_simulation->RemoveFromWorld( body );
	return true;
}

bool Tr2PhysicsWorld::HasBody( int id ) const
{
	return Find( id ) != nullptr || FindRope( id ) != nullptr || FindCharacter( id ) != nullptr || FindRagdoll( id ) != nullptr;
}

void Tr2PhysicsWorld::Clear()
{
	if( m_simulation )
	{
		m_simulation->RemoveAll();
	}
}

void Tr2PhysicsWorld::ClearGroup( int group )
{
	if( !m_simulation )
	{
		return;
	}
	std::vector<int> ragdolls;
	for( const auto& ragdoll : m_simulation->ragdolls )
	{
		if( ragdoll.second->group == group )
		{
			ragdolls.push_back( ragdoll.first );
		}
	}
	for( int ragdoll : ragdolls )
	{
		m_simulation->RemoveRagdoll( ragdoll );
	}
	auto& bodies = m_simulation->bodies;
	for( auto it = bodies.begin(); it != bodies.end(); )
	{
		if( it->second.group == group && it->second.ragdoll == 0 )
		{
			const JPH::BodyID body = it->second.id;
			it = bodies.erase( it );
			m_simulation->RemoveFromWorld( body );
		}
		else
		{
			++it;
		}
	}
}

void Tr2PhysicsWorld::OptimizeBroadPhase()
{
	if( m_simulation )
	{
		m_simulation->system.OptimizeBroadPhase();
	}
}

bool Tr2PhysicsWorld::SetBodyGroup( int id, int group )
{
	if( !m_simulation || group < 0 || group >= GROUPS )
	{
		return false;
	}
	JPH::BodyInterface& bodyInterface = m_simulation->system.GetBodyInterface();
	if( Ragdoll* ragdoll = FindRagdoll( id ) )
	{
		ragdoll->group = group;	// one not built yet is built there
		for( const Ragdoll::Part& part : ragdoll->parts )
		{
			const auto body = m_simulation->bodies.find( part.handle );
			if( body != m_simulation->bodies.end() )
			{
				bodyInterface.SetObjectLayer( body->second.id, MovingLayer( group ) );
				bodyInterface.ActivateBody( body->second.id );
				body->second.group = group;
			}
		}
		return true;
	}
	const auto it = m_simulation->bodies.find( id );
	if( it == m_simulation->bodies.end() || it->second.ragdoll != 0 )
	{
		return false;	// a part is in its ragdoll's group
	}
	const bool moving = bodyInterface.GetMotionType( it->second.id ) != JPH::EMotionType::Static;
	bodyInterface.SetObjectLayer( it->second.id, moving ? MovingLayer( group ) : StaticLayer( group ) );
	if( moving )
	{
		bodyInterface.ActivateBody( it->second.id );	// asleep, it would hang where it is whatever is under it now
	}
	it->second.group = group;
	return true;
}

bool Tr2PhysicsWorld::SetBodyTags( int id, int tags )
{
	const Body* body = Find( id );
	if( !body || tags < 0 )
	{
		return false;
	}
	m_simulation->system.GetBodyInterface().SetUserData( body->id, UserData( id, static_cast<JPH::uint32>( tags ) ) );
	return true;
}

bool Tr2PhysicsWorld::SetGravityFactor( int id, float factor )
{
	const Body* body = FindMoving( id );
	if( !body || !std::isfinite( factor ) )
	{
		return false;
	}
	JPH::BodyInterface& bodyInterface = m_simulation->system.GetBodyInterface();
	bodyInterface.SetGravityFactor( body->id, factor );
	bodyInterface.ActivateBody( body->id );
	return true;
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
	if( !body || body->ragdoll != 0 )
	{
		return false;	// a part's motion is its ragdoll's mode
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
	const Body* body = FindLoose( id );
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
	const Body* body = FindLoose( id );
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

bool Tr2PhysicsWorld::AddImpulse( int id, const Vector3& impulse, const Vector3& point )
{
	const Body* body = Find( id );
	if( !body || !Within( impulse, MAX_IMPULSE ) || !AllFinite( point ) )
	{
		return false;
	}
	JPH::BodyInterface& bodyInterface = m_simulation->system.GetBodyInterface();
	if( bodyInterface.GetMotionType( body->id ) != JPH::EMotionType::Dynamic )
	{
		return false;	// only what the simulation moves takes a push; Jolt asserts on the rest
	}
	// Off its centre of mass it turns it too, about what its allowed freedoms leave (an upright body only slides)
	bodyInterface.AddImpulse( body->id, ToJolt( impulse ), ToJoltPosition( point ) );
	bodyInterface.ActivateBody( body->id );
	return true;
}

float Tr2PhysicsWorld::GetRestitution( int id ) const
{
	const Body* body = Find( id );
	return body ? m_simulation->system.GetBodyInterface().GetRestitution( body->id ) : 0.f;
}

bool Tr2PhysicsWorld::SetRestitution( int id, float restitution )
{
	const Body* body = Find( id );
	if( !body || !std::isfinite( restitution ) || restitution < 0.f || restitution > 1.f )
	{
		return false;
	}
	// Two bodies meeting bounce by the larger of their two (Jolt's default), so a lively prop bounces off a dead wall
	m_simulation->system.GetBodyInterface().SetRestitution( body->id, restitution );
	return true;
}

std::pair<float, float> Tr2PhysicsWorld::GetDamping( int id ) const
{
	const Body* body = FindMoving( id );
	if( !body )
	{
		return { 0.f, 0.f };
	}
	JPH::BodyLockRead lock( m_simulation->system.GetBodyLockInterface(), body->id );
	if( !lock.Succeeded() )
	{
		return { 0.f, 0.f };
	}
	const JPH::MotionProperties* motion = lock.GetBody().GetMotionProperties();
	return { motion->GetLinearDamping(), motion->GetAngularDamping() };
}

bool Tr2PhysicsWorld::SetDamping( int id, float linear, float angular )
{
	const Body* body = FindMoving( id );
	if( !body || !std::isfinite( linear ) || !std::isfinite( angular ) || linear < 0.f || angular < 0.f || linear > MAX_DAMPING || angular > MAX_DAMPING )
	{
		return false;
	}
	JPH::BodyLockWrite lock( m_simulation->system.GetBodyLockInterface(), body->id );
	if( !lock.Succeeded() )
	{
		return false;
	}
	JPH::MotionProperties* motion = lock.GetBody().GetMotionProperties();
	motion->SetLinearDamping( linear );
	motion->SetAngularDamping( angular );
	return true;
}

bool Tr2PhysicsWorld::GetGyroscopic( int id ) const
{
	const Body* body = FindMoving( id );
	if( !body )
	{
		return false;
	}
	JPH::BodyLockRead lock( m_simulation->system.GetBodyLockInterface(), body->id );
	return lock.Succeeded() && lock.GetBody().GetApplyGyroscopicForce();
}

bool Tr2PhysicsWorld::SetGyroscopic( int id, bool on )
{
	const Body* body = FindMoving( id );
	if( !body )
	{
		return false;
	}
	JPH::BodyLockWrite lock( m_simulation->system.GetBodyLockInterface(), body->id );
	if( !lock.Succeeded() )
	{
		return false;
	}
	lock.GetBody().SetApplyGyroscopicForce( on );
	return true;
}

std::vector<Vector3> Tr2PhysicsWorld::GetRopePoints( int id ) const
{
	std::vector<Vector3> points;
	const Body* body = FindRope( id );
	if( !body )
	{
		return points;
	}
	JPH::BodyLockRead lock( m_simulation->system.GetBodyLockInterface(), body->id );
	if( !lock.Succeeded() )
	{
		return points;
	}
	const JPH::Body& rope = lock.GetBody();
	const JPH::RVec3 origin = rope.GetCenterOfMassPosition();
	const auto* motion = static_cast<const JPH::SoftBodyMotionProperties*>( rope.GetMotionProperties() );
	points.reserve( motion->GetVertices().size() );
	for( const JPH::SoftBodyVertex& vertex : motion->GetVertices() )
	{
		const JPH::RVec3 point = origin + vertex.mPosition;
		points.emplace_back( static_cast<float>( point.GetX() ), static_cast<float>( point.GetY() ), static_cast<float>( point.GetZ() ) );
	}
	return points;
}

bool Tr2PhysicsWorld::SetRopePin( int id, int index, bool pinned )
{
	const Body* body = FindRope( id );
	if( !body || index < 0 )
	{
		return false;
	}
	JPH::BodyLockWrite lock( m_simulation->system.GetBodyLockInterface(), body->id );
	if( !lock.Succeeded() )
	{
		return false;
	}
	auto* motion = static_cast<JPH::SoftBodyMotionProperties*>( lock.GetBody().GetMotionProperties() );
	if( static_cast<size_t>( index ) >= motion->GetVertices().size() )
	{
		return false;
	}
	JPH::SoftBodyVertex& vertex = motion->GetVertex( static_cast<JPH::uint>( index ) );
	vertex.mInvMass = pinned ? 0.f : body->pointInvMass;
	vertex.mVelocity = JPH::Vec3::sZero();
	return true;
}

bool Tr2PhysicsWorld::MoveRopePin( int id, int index, const Vector3& position, float seconds )
{
	const Body* body = FindRope( id );
	if( !body || index < 0 || !AllFinite( position ) || !std::isfinite( seconds ) || seconds < MIN_SECONDS || seconds > MAX_SECONDS )
	{
		return false;
	}
	JPH::BodyLockWrite lock( m_simulation->system.GetBodyLockInterface(), body->id );
	if( !lock.Succeeded() )
	{
		return false;
	}
	JPH::Body& rope = lock.GetBody();
	auto* motion = static_cast<JPH::SoftBodyMotionProperties*>( rope.GetMotionProperties() );
	if( static_cast<size_t>( index ) >= motion->GetVertices().size() )
	{
		return false;
	}
	JPH::SoftBodyVertex& vertex = motion->GetVertex( static_cast<JPH::uint>( index ) );
	if( vertex.mInvMass != 0.f )
	{
		return false;
	}
	// Jolt moves a pinned point by its velocity alone, so it sweeps there over the step and the rope feels it go
	const JPH::Vec3 target = JPH::Vec3( ToJoltPosition( position ) - rope.GetCenterOfMassPosition() );
	vertex.mVelocity = ( target - vertex.mPosition ) / seconds;
	return true;
}

int Tr2PhysicsWorld::AddCharacter( const Vector3& position, const std::vector<Vector3>& stances, float mass, float strength, float maxSlopeDegrees )
{
	TRINITY_STATS_ZONE( "Tr2PhysicsWorld::AddCharacter" );

	if( !m_simulation || !AllFinite( position ) || stances.empty() || stances.size() > MAX_STANCES )
	{
		return 0;
	}
	if( !std::isfinite( mass ) || mass < MIN_MASS || mass > MAX_MASS || !std::isfinite( strength ) || strength < 0.f )
	{
		return 0;
	}
	if( !std::isfinite( maxSlopeDegrees ) || maxSlopeDegrees < MIN_SLOPE_DEGREES || maxSlopeDegrees > 90.f )
	{
		return 0;
	}

	Simulation* simulation = m_simulation.get();
	auto made = std::make_unique<Character>( simulation->system.GetBodyLockInterface(), [simulation]( int ragdoll ) { return simulation->RagdollMass( ragdoll ); } );
	for( const Vector3& stance : stances )
	{
		if( !AllFinite( stance ) || stance.x < MIN_CHARACTER_RADIUS || stance.y > MAX_CHARACTER_SIZE )
		{
			return 0;
		}
		if( stance.y < ( stance.z != 0.f ? stance.x : 2.f * stance.x ) )
		{
			return 0;	// a capsule at least as tall as it is wide, a cylinder at least half as
		}
		made->stances.push_back( StanceShape( stance, 1.f ) );
		made->innerStances.push_back( StanceShape( stance, INNER_BODY_SHARE ) );
		if( !made->stances.back() || !made->innerStances.back() )
		{
			return 0;
		}
	}

	JPH::Ref<JPH::CharacterVirtualSettings> settings = new JPH::CharacterVirtualSettings();
	settings->mShape = made->stances[0];
	settings->mInnerBodyShape = made->innerStances[0];
	settings->mInnerBodyLayer = LAYER_CHARACTER;
	settings->mMass = mass;
	settings->mMaxStrength = strength;
	settings->mMaxSlopeAngle = JPH::DegreesToRadians( maxSlopeDegrees );
	settings->mSupportingVolume = JPH::Plane( JPH::Vec3::sAxisY(), -stances[0].x );	// only what is under the round of the feet holds it up
	settings->mEnhancedInternalEdgeRemoval = true;

	const int handle = m_simulation->nextId;
	made->character = new JPH::CharacterVirtual( settings, ToJoltPosition( position ), JPH::Quat::sIdentity(), UserData( handle, TAG_CHARACTER ), &m_simulation->system );
	if( made->character->GetInnerBodyID().IsInvalid() )
	{
		CCP_LOGWARN( "Tr2PhysicsWorld: no room for another body (%u at most)", MAX_BODIES );
		return 0;
	}
	made->character->SetListener( &made->contacts );

	m_simulation->nextId++;
	m_simulation->characters.emplace( handle, std::move( made ) );
	RefreshCharacter( handle, 0.f );
	return handle;
}

std::pair<Vector3, Vector3> Tr2PhysicsWorld::MoveCharacter( int id, const Vector3& velocity, float seconds, float gravity, float stepUp, float stickDown )
{
	TRINITY_STATS_ZONE( "Tr2PhysicsWorld::MoveCharacter" );

	const Vector3 still( 0.f, 0.f, 0.f );
	Character* found = FindCharacter( id );
	if( !found )
	{
		return { still, still };
	}
	JPH::CharacterVirtual& character = *found->character;
	if( !Within( velocity, MAX_SPEED ) || !std::isfinite( seconds ) || seconds < MIN_SECONDS || seconds > MAX_SECONDS
		|| !std::isfinite( gravity ) || gravity < 0.f || gravity > MAX_CHARACTER_GRAVITY
		|| !std::isfinite( stepUp ) || stepUp < 0.f || stepUp > MAX_CHARACTER_SIZE || !std::isfinite( stickDown ) || stickDown < 0.f || stickDown > MAX_CHARACTER_SIZE )
	{
		return { FromJoltPosition( character.GetPosition() ), still };
	}

	const JPH::BroadPhaseLayerFilter broadPhase;
	const QueryLayers layers( ANY_GROUP );
	const QueryBodies bodies( found->ignored, found->passedTags );
	const JPH::ShapeFilter shapes;
	JPH::TempAllocator& temp = m_simulation->tempAllocator;

	character.UpdateGroundVelocity();
	const bool supported = character.IsSupported();
	const JPH::Vec3 ground = supported ? character.GetGroundVelocity() : JPH::Vec3::sZero();

	// On the ground, along it, keeping the speed across the ground asked for, which is what the clips are matched to
	JPH::Vec3 wanted = ToJolt( velocity );
	const JPH::Vec3 normal = character.GetGroundNormal();
	if( character.GetGroundState() == JPH::CharacterBase::EGroundState::OnGround && wanted.GetY() <= 0.f && normal.GetY() > MIN_GROUND_NORMAL_Y )
	{
		wanted.SetY( -( wanted.GetX() * normal.GetX() + wanted.GetZ() * normal.GetZ() ) / normal.GetY() );
	}
	const JPH::Vec3 desired = wanted + ground;
	const JPH::RVec3 start = character.GetPosition();
	const float worldGravity = m_simulation->system.GetGravity().Length();
	const JPH::BodyLockInterface& locks = m_simulation->system.GetBodyLockInterface();

	// CharacterVirtual::ExtendedUpdate's stages, taken apart so a prop being shoved is not walked up as a step
	character.SetLinearVelocity( character.CancelVelocityTowardsSteepSlopes( desired ) );
	character.StartTrackingContactChanges();
	character.Update( seconds, JPH::Vec3( 0.f, -gravity, 0.f ), broadPhase, layers, bodies, shapes, temp );
	if( stickDown > 0.f && supported && !character.IsSupported() && JPH::Vec3( character.GetPosition() - start ).GetY() <= 1e-6f * seconds )
	{
		character.StickToFloor( JPH::Vec3( 0.f, -stickDown, 0.f ), broadPhase, layers, bodies, shapes, temp );
	}
	JPH::Vec3 flat = desired * seconds;
	flat.SetY( 0.f );
	const float wantedStep = flat.Length();
	if( stepUp > 0.f && wantedStep > 0.f )
	{
		const JPH::Vec3 forward = flat / wantedStep;
		JPH::Vec3 achieved( character.GetPosition() - start );
		achieved.SetY( 0.f );
		const float achievedStep = std::max( 0.f, achieved.Dot( forward ) );
		if( achievedStep + 1e-4f < wantedStep && character.CanWalkStairs( desired ) && PropShoved( character, desired, locks, worldGravity ).IsInvalid() )
		{
			const JPH::Vec3 stepForward = forward * std::max( MIN_STEP_FORWARD, wantedStep - achievedStep );
			JPH::Vec3 forwardTest = -character.GetGroundNormal();
			forwardTest.SetY( 0.f );
			forwardTest = forwardTest.NormalizedOr( forward );
			if( forwardTest.Dot( forward ) < STEP_FORWARD_COS )
			{
				forwardTest = forward;
			}
			character.WalkStairs( seconds, JPH::Vec3( 0.f, stepUp, 0.f ), stepForward, forwardTest * STEP_FORWARD_TEST, JPH::Vec3::sZero(), broadPhase, layers, bodies, shapes, temp );
		}
	}
	character.FinishTrackingContactChanges();

	// What it walked: being pushed out of something is not to come back as speed, nor is the ground's own move.  Shoving
	// a prop it counts as all it asked for: blocked, the speed the caller gives it back would fall to nothing, and with
	// it the shove
	const JPH::Vec3 moved = JPH::Vec3( character.GetPosition() - start ) - ground * seconds;
	JPH::Vec3 walked = moved;
	walked.SetY( 0.f );
	JPH::Vec3 flatWanted = wanted * seconds;
	flatWanted.SetY( 0.f );
	const float wantedLength = flatWanted.Length();
	const float walkedLength = walked.Length();
	const JPH::BodyID prop = PropShoved( character, desired, locks, worldGravity );
	if( !prop.IsInvalid() )
	{
		walked = flatWanted;
		Shove( m_simulation->system, prop, flatWanted / seconds, character.GetMaxStrength() * seconds );
	}
	else if( walkedLength > wantedLength )
	{
		walked = walkedLength > 0.f ? walked * ( wantedLength / walkedLength ) : JPH::Vec3::sZero();
	}
	walked.SetY( moved.GetY() );

	// Jolt teleports the inner body with the character, so to the simulation it stands still.  It goes back to where it
	// was and sweeps over to here in the next Step, knocking aside what it meets at the speed the character went
	JPH::BodyInterface& bodyInterface = m_simulation->system.GetBodyInterface();
	const JPH::BodyID inner = character.GetInnerBodyID();
	const JPH::RVec3 innerEnd = bodyInterface.GetPosition( inner );
	bodyInterface.SetPosition( inner, innerEnd - ( character.GetPosition() - start ), JPH::EActivation::DontActivate );
	bodyInterface.MoveKinematic( inner, innerEnd, JPH::Quat::sIdentity(), seconds );

	return { FromJoltPosition( character.GetPosition() ), FromJolt( walked ) };
}

Vector3 Tr2PhysicsWorld::GetCharacterPosition( int id ) const
{
	const Character* found = FindCharacter( id );
	return found ? FromJoltPosition( found->character->GetPosition() ) : Vector3( 0.f, 0.f, 0.f );
}

bool Tr2PhysicsWorld::SetCharacterPosition( int id, const Vector3& position )
{
	Character* found = FindCharacter( id );
	if( !found || !AllFinite( position ) )
	{
		return false;
	}
	found->character->SetPosition( ToJoltPosition( position ) );
	m_simulation->system.GetBodyInterface().SetLinearVelocity( found->character->GetInnerBodyID(), JPH::Vec3::sZero() );
	return true;
}

bool Tr2PhysicsWorld::RefreshCharacter( int id, float stickDown )
{
	Character* found = FindCharacter( id );
	if( !found || !std::isfinite( stickDown ) || stickDown < 0.f || stickDown > MAX_CHARACTER_SIZE )
	{
		return false;
	}
	const JPH::BroadPhaseLayerFilter broadPhase;
	const QueryLayers layers( ANY_GROUP );
	const QueryBodies bodies( found->ignored, found->passedTags );
	const JPH::ShapeFilter shapes;
	JPH::CharacterVirtual& character = *found->character;
	character.SetLinearVelocity( JPH::Vec3::sZero() );
	character.RefreshContacts( broadPhase, layers, bodies, shapes, m_simulation->tempAllocator );
	if( stickDown > 0.f && !character.IsSupported() )
	{
		character.StickToFloor( JPH::Vec3( 0.f, -stickDown, 0.f ), broadPhase, layers, bodies, shapes, m_simulation->tempAllocator );
	}
	return true;
}

std::pair<std::pair<int, int>, std::pair<Vector3, Vector3>> Tr2PhysicsWorld::GetCharacterGround( int id ) const
{
	const Vector3 none( 0.f, 0.f, 0.f );
	const Character* found = FindCharacter( id );
	if( !found )
	{
		return { { -1, 0 }, { none, none } };
	}
	const JPH::CharacterVirtual& character = *found->character;
	return {
		{ static_cast<int>( character.GetGroundState() ), HandleOf( character.GetGroundUserData() ) },
		{ FromJoltPosition( character.GetGroundPosition() ), FromJolt( character.GetGroundNormal() ) } };
}

Vector3 Tr2PhysicsWorld::GetCharacterGroundVelocity( int id ) const
{
	const Character* found = FindCharacter( id );
	return found ? FromJolt( found->character->GetGroundVelocity() ) : Vector3( 0.f, 0.f, 0.f );
}

std::vector<std::pair<int, Vector3>> Tr2PhysicsWorld::GetCharacterContacts( int id ) const
{
	std::vector<std::pair<int, Vector3>> contacts;
	const Character* found = FindCharacter( id );
	if( !found )
	{
		return contacts;
	}
	for( const JPH::CharacterVirtual::Contact& contact : found->character->GetActiveContacts() )
	{
		if( contact.mHadCollision && !contact.mWasDiscarded && !contact.mIsSensorB )
		{
			contacts.emplace_back( HandleOf( contact.mUserData ), FromJolt( contact.mSurfaceNormal ) );
		}
	}
	return contacts;
}

bool Tr2PhysicsWorld::FitsCharacter( int id, const Vector3& position, int stance ) const
{
	const Character* found = FindCharacter( id );
	if( !found || !AllFinite( position ) || stance < 0 || static_cast<size_t>( stance ) >= found->stances.size() )
	{
		return false;
	}
	const JPH::BroadPhaseLayerFilter broadPhase;
	const QueryLayers layers( ANY_GROUP );
	const QueryBodies bodies( found->ignored, found->passedTags );
	const JPH::ShapeFilter shapes;
	DeepContact contact;
	found->character->CheckCollision(
		ToJoltPosition( position ), JPH::Quat::sIdentity(), JPH::Vec3::sZero(), 0.f, found->stances[stance], JPH::RVec3::sZero(), contact, broadPhase, layers, bodies, shapes );
	return !contact.found;
}

bool Tr2PhysicsWorld::SetCharacterStance( int id, int stance, bool force )
{
	Character* found = FindCharacter( id );
	if( !found || stance < 0 || static_cast<size_t>( stance ) >= found->stances.size() )
	{
		return false;
	}
	const JPH::BroadPhaseLayerFilter broadPhase;
	const QueryLayers layers( ANY_GROUP );
	const QueryBodies bodies( found->ignored, found->passedTags );
	const JPH::ShapeFilter shapes;
	JPH::CharacterVirtual& character = *found->character;
	// SetShape's depths are measured from the padding, FitsCharacter's from the shape
	if( !character.SetShape( found->stances[stance], force ? FLT_MAX : FIT_DEPTH + character.GetCharacterPadding(), broadPhase, layers, bodies, shapes, m_simulation->tempAllocator ) )
	{
		return false;
	}
	character.SetInnerBodyShape( found->innerStances[stance] );
	return true;
}

bool Tr2PhysicsWorld::SetCharacterIgnored( int id, const std::vector<int>& ignored )
{
	Character* found = FindCharacter( id );
	if( !found )
	{
		return false;
	}
	found->ignored.clear();
	for( int handle : ignored )
	{
		const auto it = m_simulation->bodies.find( handle );
		if( it != m_simulation->bodies.end() )
		{
			found->ignored.push_back( it->second.id );
		}
	}
	return true;
}

bool Tr2PhysicsWorld::SetCharacterPassing( int id, int tags )
{
	Character* found = FindCharacter( id );
	if( !found || tags < 0 )
	{
		return false;
	}
	found->passedTags = static_cast<JPH::uint32>( tags );
	return true;
}

bool Tr2PhysicsWorld::SetCharacterForce( int id, float mass, float strength )
{
	Character* found = FindCharacter( id );
	if( !found || !std::isfinite( mass ) || mass < MIN_MASS || mass > MAX_MASS || !std::isfinite( strength ) || strength < 0.f )
	{
		return false;
	}
	found->character->SetMass( mass );
	found->character->SetMaxStrength( strength );
	return true;
}

int Tr2PhysicsWorld::AddRagdoll()
{
	if( !m_simulation )
	{
		return 0;
	}
	const int handle = m_simulation->nextId++;
	m_simulation->ragdolls.emplace( handle, std::make_unique<Ragdoll>() );
	return handle;
}

int Tr2PhysicsWorld::AddRagdollPart( int id, int parent, int kind, const Vector3& size, const Vector3& position, const Quaternion& rotation, float mass )
{
	Ragdoll* ragdoll = FindRagdoll( id );
	JPH::Quat turn;
	if( !ragdoll || ragdoll->ragdoll || ragdoll->parts.size() >= MAX_RAGDOLL_PARTS || !AllFinite( size ) || !AllFinite( position ) || !ToJoltRotation( rotation, turn ) )
	{
		return -1;
	}
	// Parents first: the root alone has none, and every other part's is made already
	const int index = static_cast<int>( ragdoll->parts.size() );
	if( index == 0 ? parent != -1 : ( parent < 0 || parent >= index ) )
	{
		return -1;
	}
	if( !std::isfinite( mass ) || mass < MIN_MASS || mass > MAX_MASS )
	{
		return -1;
	}

	JPH::ShapeSettings::ShapeResult shape;
	if( kind == RAGDOLL_CAPSULE )
	{
		if( size.x < MIN_HALF_EXTENT || size.x > MAX_HALF_EXTENT || size.y < 0.f || size.y > MAX_HALF_EXTENT )
		{
			return -1;
		}
		shape = JPH::CapsuleShapeSettings( size.y, size.x ).Create();
	}
	else if( kind == RAGDOLL_BOX )
	{
		const float smallest = std::min( { size.x, size.y, size.z } );
		if( smallest < MIN_HALF_EXTENT || std::max( { size.x, size.y, size.z } ) > MAX_HALF_EXTENT )
		{
			return -1;
		}
		shape = JPH::BoxShapeSettings( ToJolt( size ), std::min( JPH::cDefaultConvexRadius, 0.5f * smallest ) ).Create();
	}
	else
	{
		return -1;
	}
	if( shape.HasError() )
	{
		CCP_LOGWARN( "Tr2PhysicsWorld: a ragdoll part's shape could not be made: %s", shape.GetError().c_str() );
		return -1;
	}

	Ragdoll::Part part;
	part.parent = parent;
	part.shape = shape.Get();
	part.position = ToJoltPosition( position );
	part.rotation = turn;
	part.mass = mass;
	ragdoll->parts.push_back( part );
	return index;
}

bool Tr2PhysicsWorld::SetRagdollJoint(
	int id,
	int part,
	const Vector3& anchor,
	const Vector3& twistAxis,
	const Vector3& planeAxis,
	const Vector3& parentTwistAxis,
	const Vector3& parentPlaneAxis,
	const Vector3& swing,
	const Vector3& twist )
{
	Ragdoll* ragdoll = FindRagdoll( id );
	if( !ragdoll || ragdoll->ragdoll || part < 1 || static_cast<size_t>( part ) >= ragdoll->parts.size() || !AllFinite( anchor ) )
	{
		return false;	// the root has no parent to be joined to
	}
	const float swingNormal = swing.x;
	const float swingPlane = swing.y;
	const float twistMin = twist.x;
	const float twistMax = twist.y;
	// Each pair unit and square; what little skew is let through is taken out of the plane axis
	JPH::Vec3 axes[4];
	const Vector3* given[4] = { &twistAxis, &planeAxis, &parentTwistAxis, &parentPlaneAxis };
	for( int i = 0; i < 4; ++i )
	{
		if( !AllFinite( *given[i] ) )
		{
			return false;
		}
		const JPH::Vec3 axis = ToJolt( *given[i] );
		if( !( axis.Length() > 1e-6f ) )
		{
			return false;
		}
		axes[i] = axis.Normalized();
	}
	for( int i = 0; i < 4; i += 2 )
	{
		const float skew = axes[i].Dot( axes[i + 1] );
		if( std::abs( skew ) > MAX_AXIS_SKEW )
		{
			return false;
		}
		axes[i + 1] = ( axes[i + 1] - skew * axes[i] ).Normalized();
	}
	// The joint is made at rest, so its limits hold the pose it is made in
	if( !std::isfinite( swingNormal ) || !std::isfinite( swingPlane ) || swingNormal < 0.f || swingNormal > JPH::JPH_PI || swingPlane < 0.f || swingPlane > JPH::JPH_PI )
	{
		return false;
	}
	if( !std::isfinite( twistMin ) || !std::isfinite( twistMax ) || twistMin < -JPH::JPH_PI || twistMin > 0.f || twistMax < 0.f || twistMax > JPH::JPH_PI )
	{
		return false;
	}

	JPH::Ref<JPH::SwingTwistConstraintSettings> joint = new JPH::SwingTwistConstraintSettings();
	joint->mSpace = JPH::EConstraintSpace::WorldSpace;
	joint->mPosition1 = joint->mPosition2 = ToJoltPosition( anchor );
	joint->mTwistAxis1 = axes[2];	// body 1 is the parent
	joint->mPlaneAxis1 = axes[3];
	joint->mTwistAxis2 = axes[0];
	joint->mPlaneAxis2 = axes[1];
	joint->mNormalHalfConeAngle = swingNormal;
	joint->mPlaneHalfConeAngle = swingPlane;
	joint->mTwistMinAngle = twistMin;
	joint->mTwistMaxAngle = twistMax;
	ragdoll->parts[part].joint = joint;
	return true;
}

bool Tr2PhysicsWorld::BuildRagdoll( int id )
{
	TRINITY_STATS_ZONE( "Tr2PhysicsWorld::BuildRagdoll" );

	Ragdoll* ragdoll = FindRagdoll( id );
	if( !ragdoll || ragdoll->ragdoll || ragdoll->parts.empty() )
	{
		return false;
	}
	const size_t count = ragdoll->parts.size();
	for( size_t i = 1; i < count; ++i )
	{
		if( !ragdoll->parts[i].joint )
		{
			return false;	// a part hanging from nothing
		}
	}

	JPH::Ref<JPH::RagdollSettings> settings = new JPH::RagdollSettings();
	settings->mSkeleton = new JPH::Skeleton();
	settings->mParts.resize( count );
	JPH::Array<JPH::Mat44> bind( count );
	float mass = 0.f;
	for( size_t i = 0; i < count; ++i )
	{
		const Ragdoll::Part& given = ragdoll->parts[i];
		settings->mSkeleton->AddJoint( std::to_string( i ), given.parent );
		JPH::RagdollSettings::Part& part = settings->mParts[i];
		part.SetShape( given.shape );
		part.mPosition = given.position;
		part.mRotation = given.rotation;
		part.mMotionType = JPH::EMotionType::Kinematic;	// where it was given, still, until its mode says otherwise
		part.mAllowDynamicOrKinematic = true;
		part.mObjectLayer = MovingLayer( ragdoll->group );
		// Swept, a limb fights the joint that holds it; the root alone is, and keeps the body the right side of a floor
		part.mMotionQuality = i == 0 ? JPH::EMotionQuality::LinearCast : JPH::EMotionQuality::Discrete;
		part.mFriction = FRICTION;
		part.mAngularDamping = RAGDOLL_ANGULAR_DAMPING;
		part.mMaxLinearVelocity = RAGDOLL_MAX_SPEED;
		part.mMaxAngularVelocity = RAGDOLL_MAX_TURN_RATE;
		part.mOverrideMassProperties = JPH::EOverrideMassProperties::CalculateInertia;
		part.mMassPropertiesOverride.mMass = given.mass;
		if( given.joint )
		{
			given.joint->mMaxFrictionTorque = ragdoll->friction;
			part.mToParent = given.joint;
		}
		bind[i] = JPH::Mat44::sRotationTranslation( given.rotation, JPH::Vec3( given.position ) );
		mass += given.mass;
	}
	// Stabilize evens the parts' masses out (a heavy part hanging from a light one pulls its joint apart), keeping their sum
	if( !settings->Stabilize() )
	{
		CCP_LOGWARN( "Tr2PhysicsWorld: a ragdoll's parts have no inertia to balance" );
		return false;
	}
	settings->CalculateConstraintPriorities();
	settings->DisableParentChildCollisions( bind.data(), RAGDOLL_SEPARATION );
	settings->CalculateBodyIndexToConstraintIndex();
	settings->CalculateConstraintIndexToBodyIdxPair();

	// Its id is its collision group, which keeps one ragdoll's parts from meeting each other and no other's
	JPH::Ref<JPH::Ragdoll> made = settings->CreateRagdoll( static_cast<JPH::CollisionGroup::GroupID>( id ), 0, &m_simulation->system );
	if( !made )
	{
		CCP_LOGWARN( "Tr2PhysicsWorld: no room for another body (%u at most)", MAX_BODIES );
		return false;
	}
	// Jolt gives every part the one user data: each gets its own id there, which a ray names it by
	JPH::BodyInterface& bodyInterface = m_simulation->system.GetBodyInterface();
	for( size_t i = 0; i < count; ++i )
	{
		const int handle = m_simulation->nextId++;
		const JPH::BodyID part = made->GetBodyID( static_cast<int>( i ) );
		bodyInterface.SetUserData( part, UserData( handle, 0 ) );
		Body body;
		body.id = part;
		body.group = ragdoll->group;
		body.ragdoll = id;
		body.part = static_cast<int>( i );
		m_simulation->bodies.emplace( handle, body );
		ragdoll->parts[i].handle = handle;
	}
	made->AddToPhysicsSystem( JPH::EActivation::Activate );
	ragdoll->ragdoll = made;
	ragdoll->mode = RAGDOLL_KEYED;
	ragdoll->mass = mass;
	return true;
}

std::vector<int> Tr2PhysicsWorld::GetRagdollParts( int id ) const
{
	std::vector<int> parts;
	if( const Ragdoll* ragdoll = FindBuiltRagdoll( id ) )
	{
		parts.reserve( ragdoll->parts.size() );
		for( const Ragdoll::Part& part : ragdoll->parts )
		{
			parts.push_back( part.handle );
		}
	}
	return parts;
}

std::pair<int, int> Tr2PhysicsWorld::GetRagdollOf( int bodyId ) const
{
	const Body* body = Find( bodyId );
	return body && body->ragdoll != 0 ? std::make_pair( body->ragdoll, body->part ) : std::make_pair( 0, -1 );
}

int Tr2PhysicsWorld::GetRagdollMode( int id ) const
{
	const Ragdoll* ragdoll = FindBuiltRagdoll( id );
	return ragdoll ? ragdoll->mode : -1;
}

bool Tr2PhysicsWorld::SetRagdollMode( int id, int mode )
{
	Ragdoll* ragdoll = FindBuiltRagdoll( id );
	if( !ragdoll || ( mode != RAGDOLL_KEYED && mode != RAGDOLL_LIMP ) )
	{
		return false;
	}
	if( mode == ragdoll->mode )
	{
		return true;
	}
	JPH::BodyInterface& bodyInterface = m_simulation->system.GetBodyInterface();
	for( const JPH::BodyID& part : ragdoll->ragdoll->GetBodyIDs() )
	{
		if( mode == RAGDOLL_LIMP )
		{
			bodyInterface.SetMotionType( part, JPH::EMotionType::Dynamic, JPH::EActivation::Activate );	// keeping the speed it had
		}
		else
		{
			bodyInterface.SetMotionType( part, JPH::EMotionType::Kinematic, JPH::EActivation::Activate );
			bodyInterface.SetLinearAndAngularVelocity( part, JPH::Vec3::sZero(), JPH::Vec3::sZero() );
		}
	}
	ragdoll->ragdoll->ResetWarmStart();	// what the joints learnt holding the last mode's pose would kick this one
	ragdoll->mode = mode;
	return true;
}

bool Tr2PhysicsWorld::PlaceRagdoll( int id, const std::vector<Vector3>& positions, const std::vector<Quaternion>& rotations )
{
	Ragdoll* ragdoll = FindBuiltRagdoll( id );
	if( !ragdoll || positions.size() != ragdoll->parts.size() || rotations.size() != ragdoll->parts.size() )
	{
		return false;
	}
	std::vector<JPH::Quat> turns( rotations.size() );
	for( size_t i = 0; i < positions.size(); ++i )
	{
		if( !AllFinite( positions[i] ) || !ToJoltRotation( rotations[i], turns[i] ) )
		{
			return false;
		}
	}
	JPH::BodyInterface& bodyInterface = m_simulation->system.GetBodyInterface();
	for( size_t i = 0; i < positions.size(); ++i )
	{
		const JPH::BodyID part = ragdoll->ragdoll->GetBodyID( static_cast<int>( i ) );
		bodyInterface.SetPositionAndRotation( part, ToJoltPosition( positions[i] ), turns[i], JPH::EActivation::Activate );
		bodyInterface.SetLinearAndAngularVelocity( part, JPH::Vec3::sZero(), JPH::Vec3::sZero() );
	}
	ragdoll->ragdoll->ResetWarmStart();
	return true;
}

std::vector<std::pair<Vector3, Quaternion>> Tr2PhysicsWorld::GetRagdollTransforms( int id ) const
{
	std::vector<std::pair<Vector3, Quaternion>> transforms;
	const Ragdoll* ragdoll = FindBuiltRagdoll( id );
	if( !ragdoll )
	{
		return transforms;
	}
	const JPH::BodyInterface& bodyInterface = m_simulation->system.GetBodyInterface();
	transforms.reserve( ragdoll->parts.size() );
	for( const JPH::BodyID& part : ragdoll->ragdoll->GetBodyIDs() )
	{
		JPH::RVec3 position;
		JPH::Quat rotation;
		bodyInterface.GetPositionAndRotation( part, position, rotation );
		transforms.emplace_back( FromJoltPosition( position ), Quaternion( rotation.GetX(), rotation.GetY(), rotation.GetZ(), rotation.GetW() ) );
	}
	return transforms;
}

bool Tr2PhysicsWorld::SetRagdollFriction( int id, float torque )
{
	Ragdoll* ragdoll = FindRagdoll( id );
	if( !ragdoll || !std::isfinite( torque ) || torque < 0.f || torque > MAX_RAGDOLL_FRICTION )
	{
		return false;
	}
	ragdoll->friction = torque;	// one not built yet is built with it
	if( ragdoll->ragdoll )
	{
		for( size_t i = 0; i < ragdoll->ragdoll->GetConstraintCount(); ++i )
		{
			// Every joint is one of SetRagdollJoint's
			static_cast<JPH::SwingTwistConstraint*>( ragdoll->ragdoll->GetConstraint( static_cast<int>( i ) ) )->SetMaxFrictionTorque( torque );
		}
		ragdoll->ragdoll->Activate();
	}
	return true;
}

bool Tr2PhysicsWorld::IsRagdollActive( int id ) const
{
	const Ragdoll* ragdoll = FindBuiltRagdoll( id );
	return ragdoll && ragdoll->ragdoll->IsActive();
}

std::pair<std::pair<int, float>, std::pair<Vector3, Vector3>> Tr2PhysicsWorld::RayHit( const Vector3& from, const Vector3& to, int group ) const
{
	const std::pair<std::pair<int, float>, std::pair<Vector3, Vector3>> none = { { 0, 0.f }, { Vector3( 0.f, 0.f, 0.f ), Vector3( 0.f, 0.f, 0.f ) } };
	if( !m_simulation || !AllFinite( from ) || !AllFinite( to ) )
	{
		return none;
	}
	const Vector3 along( to.x - from.x, to.y - from.y, to.z - from.z );
	const float length = std::sqrt( along.x * along.x + along.y * along.y + along.z * along.z );
	if( !( length > 1e-6f ) || length > MAX_RAY )
	{
		return none;
	}
	const JPH::RRayCast ray( ToJoltPosition( from ), ToJolt( along ) );
	const std::vector<JPH::BodyID> nothing;
	const QueryBodies bodies( nothing );
	const JPH::BroadPhaseLayerFilter broadPhase;
	const JPH::ObjectLayerFilter everyLayer;
	const QueryLayers groupLayers( group );
	JPH::RayCastResult hit;
	if( !m_simulation->system.GetNarrowPhaseQuery().CastRay( ray, hit, broadPhase, group == ANY_GROUP ? everyLayer : static_cast<const JPH::ObjectLayerFilter&>( groupLayers ), bodies ) )
	{
		return none;
	}
	JPH::BodyLockRead lock( m_simulation->system.GetBodyLockInterface(), hit.mBodyID );
	if( !lock.Succeeded() )
	{
		return none;
	}
	const JPH::Body& body = lock.GetBody();
	const int handle = HandleOf( body.GetUserData() );
	if( !Find( handle ) )
	{
		return none;
	}
	const JPH::RVec3 point = ray.GetPointOnRay( hit.mFraction );
	const JPH::Vec3 normal = body.GetWorldSpaceSurfaceNormal( hit.mSubShapeID2, point );
	return { { handle, hit.mFraction * length }, { FromJoltPosition( point ), FromJolt( normal ) } };
}

std::pair<std::pair<int, float>, std::pair<Vector3, Vector3>> Tr2PhysicsWorld::CastRay( const Vector3& from, const Vector3& to ) const
{
	TRINITY_STATS_ZONE( "Tr2PhysicsWorld::CastRay" );

	return RayHit( from, to, ANY_GROUP );
}

std::pair<std::pair<int, float>, std::pair<Vector3, Vector3>> Tr2PhysicsWorld::CastRayInGroup( const Vector3& from, const Vector3& to, int group ) const
{
	TRINITY_STATS_ZONE( "Tr2PhysicsWorld::CastRayInGroup" );

	if( group < 0 || group >= GROUPS )
	{
		return { { 0, 0.f }, { Vector3( 0.f, 0.f, 0.f ), Vector3( 0.f, 0.f, 0.f ) } };
	}
	return RayHit( from, to, group );
}

std::pair<std::pair<int, float>, std::pair<Vector3, Vector3>> Tr2PhysicsWorld::Trace( const Vector3& from, const Vector3& to, float radius, const std::vector<int>& ignored, int ignoredTags, int group ) const
{
	TRINITY_STATS_ZONE( "Tr2PhysicsWorld::Trace" );

	const std::pair<std::pair<int, float>, std::pair<Vector3, Vector3>> none = { { 0, 0.f }, { Vector3( 0.f, 0.f, 0.f ), Vector3( 0.f, 0.f, 0.f ) } };
	if( !m_simulation || !AllFinite( from ) || !AllFinite( to ) || !std::isfinite( radius ) || radius > MAX_HALF_EXTENT || ignoredTags < 0 || group < ANY_GROUP || group >= GROUPS )
	{
		return none;
	}
	const Vector3 along( to.x - from.x, to.y - from.y, to.z - from.z );
	const float length = std::sqrt( along.x * along.x + along.y * along.y + along.z * along.z );
	if( !( length > 1e-6f ) || length > MAX_RAY )
	{
		return none;
	}

	std::vector<JPH::BodyID> ignoredBodies;
	ignoredBodies.reserve( ignored.size() );
	for( int handle : ignored )
	{
		const auto it = m_simulation->bodies.find( handle );
		if( it != m_simulation->bodies.end() )
		{
			ignoredBodies.push_back( it->second.id );
		}
	}
	const QueryBodies bodies( ignoredBodies, static_cast<JPH::uint32>( ignoredTags ) );
	const QueryLayers layers( group );
	const JPH::BroadPhaseLayerFilter broadPhase;
	const JPH::NarrowPhaseQuery& query = m_simulation->system.GetNarrowPhaseQuery();

	if( radius > 0.f )
	{
		JPH::SphereShape sphere( radius );
		sphere.SetEmbedded();
		const JPH::RShapeCast cast( &sphere, JPH::Vec3::sOne(), JPH::RMat44::sTranslation( ToJoltPosition( from ) ), ToJolt( along ) );
		JPH::ShapeCastSettings settings;
		settings.SetBackFaceMode( JPH::EBackFaceMode::CollideWithBackFaces );
		settings.mReturnDeepestPoint = true;
		SweepHits hits( ToJolt( along ) );
		query.CastShape( cast, settings, JPH::RVec3::sZero(), hits, broadPhase, layers, bodies );
		if( !hits.found )
		{
			return none;
		}
		const JPH::ShapeCastResult& hit = hits.hit;
		JPH::BodyLockRead lock( m_simulation->system.GetBodyLockInterface(), hit.mBodyID2 );
		if( !lock.Succeeded() )
		{
			return none;
		}
		const float distance = hit.mFraction > 0.f ? hit.mFraction * length : -std::max( hit.mPenetrationDepth, 1e-6f );
		const JPH::Vec3 normal = -hit.mPenetrationAxis.NormalizedOr( JPH::Vec3::sAxisY() );
		return { { HandleOf( lock.GetBody().GetUserData() ), distance }, { FromJolt( hit.mContactPointOn2 ), FromJolt( normal ) } };
	}

	// A ray starting inside a convex shape does not see it, and a mesh has two sides, as destiny's casts have it
	const JPH::RRayCast ray( ToJoltPosition( from ), ToJolt( along ) );
	JPH::RayCastSettings settings;
	settings.mBackFaceModeTriangles = JPH::EBackFaceMode::CollideWithBackFaces;
	settings.mBackFaceModeConvex = JPH::EBackFaceMode::IgnoreBackFaces;
	settings.mTreatConvexAsSolid = false;
	JPH::ClosestHitCollisionCollector<JPH::CastRayCollector> hits;
	query.CastRay( ray, settings, hits, broadPhase, layers, bodies );
	if( !hits.HadHit() )
	{
		return none;
	}
	JPH::BodyLockRead lock( m_simulation->system.GetBodyLockInterface(), hits.mHit.mBodyID );
	if( !lock.Succeeded() )
	{
		return none;
	}
	const JPH::Body& body = lock.GetBody();
	const JPH::RVec3 point = ray.GetPointOnRay( hits.mHit.mFraction );
	JPH::Vec3 normal = body.GetWorldSpaceSurfaceNormal( hits.mHit.mSubShapeID2, point );
	if( normal.Dot( ToJolt( along ) ) > 0.f )
	{
		normal = -normal;	// a mesh met from behind
	}
	return { { HandleOf( body.GetUserData() ), hits.mHit.mFraction * length }, { FromJoltPosition( point ), FromJolt( normal ) } };
}

std::pair<std::pair<int, float>, std::pair<Vector3, Vector3>> Tr2PhysicsWorld::TraceBody( int id, const Vector3& movement, const std::vector<int>& ignored ) const
{
	TRINITY_STATS_ZONE( "Tr2PhysicsWorld::TraceBody" );

	const std::pair<std::pair<int, float>, std::pair<Vector3, Vector3>> none = { { 0, 0.f }, { Vector3( 0.f, 0.f, 0.f ), Vector3( 0.f, 0.f, 0.f ) } };
	const Body* moved = Find( id );
	if( !moved || !Within( movement, MAX_RAY ) )
	{
		return none;
	}
	const float length = std::sqrt( movement.x * movement.x + movement.y * movement.y + movement.z * movement.z );
	if( !( length > 1e-6f ) )
	{
		return none;
	}

	std::vector<JPH::BodyID> ignoredBodies = { moved->id };
	for( int handle : ignored )
	{
		const auto it = m_simulation->bodies.find( handle );
		if( it != m_simulation->bodies.end() )
		{
			ignoredBodies.push_back( it->second.id );
		}
	}

	JPH::RefConst<JPH::Shape> shape;
	JPH::RMat44 start;
	JPH::ObjectLayer layer;
	{
		JPH::BodyLockRead lock( m_simulation->system.GetBodyLockInterface(), moved->id );
		if( !lock.Succeeded() )
		{
			return none;
		}
		const JPH::Body& body = lock.GetBody();
		shape = body.GetShape();
		start = body.GetCenterOfMassTransform();
		layer = body.GetObjectLayer();
	}

	const QueryBodies bodies( ignoredBodies );
	const JPH::DefaultObjectLayerFilter layers = m_simulation->system.GetDefaultLayerFilter( layer );
	const JPH::BroadPhaseLayerFilter broadPhase;
	const JPH::RShapeCast cast( shape, JPH::Vec3::sOne(), start, ToJolt( movement ) );
	JPH::ShapeCastSettings settings;
	settings.SetBackFaceMode( JPH::EBackFaceMode::CollideWithBackFaces );
	settings.mReturnDeepestPoint = true;
	SweepHits hits( ToJolt( movement ) );
	m_simulation->system.GetNarrowPhaseQuery().CastShape( cast, settings, JPH::RVec3::sZero(), hits, broadPhase, layers, bodies );
	if( !hits.found )
	{
		return none;
	}
	const JPH::ShapeCastResult& hit = hits.hit;
	JPH::BodyLockRead lock( m_simulation->system.GetBodyLockInterface(), hit.mBodyID2 );
	if( !lock.Succeeded() )
	{
		return none;
	}
	const float distance = hit.mFraction > 0.f ? hit.mFraction * length : -std::max( hit.mPenetrationDepth, 1e-6f );
	const JPH::Vec3 normal = -hit.mPenetrationAxis.NormalizedOr( JPH::Vec3::sAxisY() );
	return { { HandleOf( lock.GetBody().GetUserData() ), distance }, { FromJolt( hit.mContactPointOn2 ), FromJolt( normal ) } };
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
	m_simulation->stepCount++;
	return true;
}
