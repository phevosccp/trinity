// Copyright © 2026 CCP ehf.

#include "StdAfx.h"
#include "Tr2AnimationRetargeter.h"
#include "Tr2GrannyAnimation.h"
#include "Tr2PhysicsWorld.h"
#include "Resources/TriGrannyRes.h"

#include <TriMath.h>
#include <cmf/transforms.h>

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstring>

// Quaternions compose like the rest of the cmf code: XMQuaternionMultiply( a, b ) applies a, then b, and
// XMVector3Rotate applies a rotation to a vector.  A world rotation takes bone space to model space.

namespace
{
constexpr uint32_t GROUND_SAMPLES = 32;	// poses per clip looked at to find where the feet come down
constexpr uint32_t TRAVEL_SAMPLES = 32;	// poses per clip looked at to find the middle of the pelvis's sway
constexpr float FOOT_BAND = 0.05f;	// paired bones this close to the lowest in the target bind pose are feet
constexpr float MIN_PELVIS_HEIGHT = 1e-3f;	// below this height above the feet, the bob is left unscaled
constexpr size_t MAX_FIRED_EVENTS = 256;	// events kept unread; past this the oldest go

// Parents precede children, so a real parent always has the smaller index
bool HasParent( const cmf::Skeleton& skeleton, uint32_t bone )
{
	return skeleton.parents[bone] < bone;
}

bool AllFinite( const Vector3& v )
{
	return std::isfinite( v.x ) && std::isfinite( v.y ) && std::isfinite( v.z );
}

// The entry of that name, added at the end if there is none
template<typename T>
T& FindOrAdd( std::vector<T>& entries, const std::string& name )
{
	auto existing = std::find_if( entries.begin(), entries.end(), [&]( const T& entry ) { return entry.name == name; } );
	T& entry = existing != entries.end() ? *existing : entries.emplace_back();
	entry.name = name;
	return entry;
}

// Whether ancestor is bone or above it
bool IsAncestor( const cmf::Skeleton& skeleton, uint32_t ancestor, uint32_t bone )
{
	while( bone != ancestor && HasParent( skeleton, bone ) )
	{
		bone = skeleton.parents[bone];
	}
	return bone == ancestor;
}

// From bone from on: the bones before it are not below it, so stay as they were
void ComputeWorld( const cmf::Skeleton& skeleton, const cmf::Transform* locals, std::vector<cmf::Transform>& world, uint32_t from = 0 )
{
	world.resize( skeleton.bones.size() );
	for( uint32_t i = from; i < world.size(); ++i )
	{
		world[i] = HasParent( skeleton, i ) ? cmf::Multiply( locals[i], world[skeleton.parents[i]] ) : locals[i];
	}
}

const cmf::Data* GetData( const TriGrannyResPtr& res )
{
	if( !res || !res->IsGood() )
	{
		return nullptr;
	}
	const auto* data = res->GetCMFData();
	return data && !data->skeletons.empty() ? data : nullptr;
}

const cmf::Data* GetClipData( const TriGrannyResPtr& res )
{
	const auto* data = GetData( res );
	return data && !data->animations.empty() ? data : nullptr;
}

// NaN and infinity come out as 0: the clip sampling does no validation of the time it is given
float Wrap01( float x )
{
	if( !std::isfinite( x ) )
	{
		return 0.f;
	}
	x = fmodf( x, 1.f );
	return x < 0.f ? x + 1.f : x;
}

float Clamp01( float x )
{
	return x > 0.f ? ( x < 1.f ? x : 1.f ) : 0.f;	// NaN comes out as 0
}

// The helper drivers' maths is written as the Hamilton product a * b (b's turn, then a's), as their fit is
Quaternion Hamilton( const Quaternion& a, const Quaternion& b )
{
	return XMQuaternionMultiply( b, a );
}

// The turn's angle, radians, 0 to pi
float AngleOf( const Quaternion& q )
{
	const float sinHalf = sqrtf( q.x * q.x + q.y * q.y + q.z * q.z );
	return 2.f * atan2f( sinHalf, fabsf( q.w ) );
}

// The same turn taken share of the way about its axis; a negative share turns the other way
Quaternion Scaled( const Quaternion& q, float share )
{
	Quaternion r = XMQuaternionNormalize( q );
	if( r.w < 0.f )
	{
		r = -r;
	}
	const float sinHalf = sqrtf( r.x * r.x + r.y * r.y + r.z * r.z );
	if( sinHalf < 1e-7f )
	{
		return IdentityQuaternion();
	}
	const Vector3 axis( r.x / sinHalf, r.y / sinHalf, r.z / sinHalf );
	return XMQuaternionRotationNormal( axis, 2.f * atan2f( sinHalf, r.w ) * share );
}

// q split into a twist about axis (unit) and the swing left: q = swing * twist
void SwingTwist( const Quaternion& q, const Vector3& axis, Quaternion& swing, Quaternion& twist )
{
	const float along = q.x * axis.x + q.y * axis.y + q.z * axis.z;
	const float length = sqrtf( along * along + q.w * q.w );
	twist = length < 1e-6f ? IdentityQuaternion() : Quaternion( axis.x * along / length, axis.y * along / length, axis.z * along / length, q.w / length );
	swing = XMQuaternionNormalize( Hamilton( q, XMQuaternionConjugate( twist ) ) );
}

// The turns, in the model's space, that put a two-bone limb's end (an ankle, a wrist) on target, or as near as it
// reaches along the line to it: bend, at the middle joint, opens or closes the limb in the plane it bends in now (a
// straight limb in the plane that holds straightBend, the way its end goes as it bends), then swing, at the top, turns
// the whole limb to the target.  False if the limb has no length or no plane to bend in.
bool SolveTwoBone( const Vector3& top, const Vector3& middle, const Vector3& end, const Vector3& target, const Vector3& straightBend, Quaternion& bend, Quaternion& swing )
{
	const float upper = Length( middle - top );
	const float lower = Length( end - middle );
	if( upper < 1e-4f || lower < 1e-4f )
	{
		return false;
	}
	const float reach = std::max( fabsf( upper - lower ) + 1e-4f, std::min( upper + lower - 1e-4f, Length( target - top ) ) );

	// Bend the middle joint to the angle that puts the end that far from the top
	const Vector3 toEnd = end - middle;
	const Vector3 alongUpper = ( top - middle ) / upper;
	Vector3 across = toEnd - alongUpper * Dot( toEnd, alongUpper );
	if( Length( across ) < 1e-5f )
	{
		across = straightBend - alongUpper * Dot( straightBend, alongUpper );
	}
	if( Length( across ) < 1e-5f )
	{
		return false;
	}
	across = across / Length( across );
	const float cosMiddle = std::max( -1.f, std::min( 1.f, ( upper * upper + lower * lower - reach * reach ) / ( 2.f * upper * lower ) ) );
	const float middleAngle = acosf( cosMiddle );
	const Vector3 bentToEnd = ( alongUpper * cosf( middleAngle ) + across * sinf( middleAngle ) ) * lower;
	TriQuaternionRotationArc( &bend, &toEnd, &bentToEnd );

	// Then swing the limb about the top so the end lands on the target
	const Vector3 bentFromTop = middle + bentToEnd - top;
	const Vector3 targetFromTop = target - top;
	TriQuaternionRotationArc( &swing, &bentFromTop, &targetFromTop );
	return true;
}

// Writes a limb's new world rotations, its top's parent's given, into the pose as rotations relative to their parents:
// top, middle and end each the parent of the next
void SetLimb( cmf::SkeletonPose& pose, const Quaternion& parent, uint32_t top, uint32_t middle, uint32_t end, const Quaternion& topRotation, const Quaternion& middleRotation, const Quaternion& endRotation )
{
	pose.boneTransforms[top].rotation = XMQuaternionNormalize( XMQuaternionMultiply( topRotation, XMQuaternionConjugate( parent ) ) );
	pose.boneTransforms[middle].rotation = XMQuaternionNormalize( XMQuaternionMultiply( middleRotation, XMQuaternionConjugate( topRotation ) ) );
	pose.boneTransforms[end].rotation = XMQuaternionNormalize( XMQuaternionMultiply( endRotation, XMQuaternionConjugate( middleRotation ) ) );
}
}

Tr2AnimationRetargeter::Tr2AnimationRetargeter( IRoot* lockobj )
{
}

Tr2AnimationRetargeter::~Tr2AnimationRetargeter()
{
	SetAnimation( nullptr );

	// The players point into the clips' data, so they go before the files
	Unbind();

	for( Clip& clip : m_clips )
	{
		ReleaseRes( clip.res );
	}
	for( Rig& rig : m_rigs )
	{
		ReleaseRes( rig.bindPoseRes );
	}
	ReleaseRes( m_targetBindPoseRes );
}

uint32_t Tr2AnimationRetargeter::FindBone( const cmf::Skeleton& skeleton, const std::string& name )
{
	for( uint32_t i = 0; i < skeleton.bones.size(); ++i )
	{
		if( cmf::ToStdStringView( skeleton.bones[i] ) == name )
		{
			return i;
		}
	}
	return NO_BONE;
}

Tr2GrannyAnimation* Tr2AnimationRetargeter::GetAnimation() const
{
	return m_animation;
}

void Tr2AnimationRetargeter::SetAnimation( Tr2GrannyAnimation* animation )
{
	if( m_animation )
	{
		m_animation->RemoveNotifyTarget( this );
		if( m_animation->GetPoseModifier() == this )
		{
			m_animation->SetPoseModifier( nullptr );
		}
	}

	m_animation = animation;
	m_lastPoseCurrent = false;

	if( m_animation )
	{
		if( m_animation->GetPoseModifier() && m_animation->GetPoseModifier() != this )
		{
			CCP_LOGWARN( "Tr2AnimationRetargeter: replacing the animation's existing pose modifier" );
		}
		m_animation->SetPoseModifier( this );

		// Hear when the animation's skeleton is released or rebuilt, as Tr2AnimationMeshBinding does
		m_animation->AddNotifyTarget( this );
	}

	m_needsBind = true;
}

const std::string& Tr2AnimationRetargeter::GetTargetBindPosePath() const
{
	return m_targetBindPosePath;
}

void Tr2AnimationRetargeter::SetTargetBindPosePath( const std::string& path )
{
	m_targetBindPosePath = path;
	LoadRes( m_targetBindPosePath, m_targetBindPoseRes );
}

bool Tr2AnimationRetargeter::GetPelvisSway() const
{
	return m_pelvisSway;
}

void Tr2AnimationRetargeter::SetPelvisSway( bool on )
{
	m_pelvisSway = on;
	m_lastPoseCurrent = false;
}

bool Tr2AnimationRetargeter::GetChainsOn() const
{
	return m_chainsOn;
}

void Tr2AnimationRetargeter::SetChainsOn( bool on )
{
	m_chainsOn = on;
	m_lastPoseCurrent = false;
}

bool Tr2AnimationRetargeter::GetHelperDriversOn() const
{
	return m_helperDriversOn;
}

void Tr2AnimationRetargeter::SetHelperDriversOn( bool on )
{
	m_helperDriversOn = on;
	m_lastPoseCurrent = false;
}

bool Tr2AnimationRetargeter::GetFootIkOn() const
{
	return m_footIkOn;
}

void Tr2AnimationRetargeter::SetFootIkOn( bool on )
{
	m_footIkOn = on;
	m_lastPoseCurrent = false;
}

float Tr2AnimationRetargeter::GetIkMaxPelvisDrop() const
{
	return m_ikMaxPelvisDrop;
}

void Tr2AnimationRetargeter::SetIkMaxPelvisDrop( float metres )
{
	m_ikMaxPelvisDrop = std::max( 0.f, metres );
	m_lastPoseCurrent = false;
}

float Tr2AnimationRetargeter::GetIkMaxFootTilt() const
{
	return m_ikMaxFootTilt;
}

void Tr2AnimationRetargeter::SetIkMaxFootTilt( float degrees )
{
	m_ikMaxFootTilt = std::max( 0.f, std::min( 90.f, degrees ) );
	m_lastPoseCurrent = false;
}

bool Tr2AnimationRetargeter::GetHandIkOn() const
{
	return m_handIkOn;
}

void Tr2AnimationRetargeter::SetHandIkOn( bool on )
{
	m_handIkOn = on;
	m_lastPoseCurrent = false;
}

bool Tr2AnimationRetargeter::GetLookAtOn() const
{
	return m_lookAtOn;
}

void Tr2AnimationRetargeter::SetLookAtOn( bool on )
{
	m_lookAtOn = on;
	m_lastPoseCurrent = false;
}

bool Tr2AnimationRetargeter::AddChain( const std::string& rig, const std::string& firstTarget, const std::string& lastTarget )
{
	Rig* found = FindRig( rig );
	if( !found )
	{
		return false;
	}
	found->chains.emplace_back( firstTarget, lastTarget );
	m_needsBind = true;
	return true;
}

void Tr2AnimationRetargeter::AddHelperDriver( const std::string& helper, const std::string& driver, const Vector3& driverAxis, const Vector3& helperAxis, float twist, float swing )
{
	auto unit = []( const Vector3& axis ) {
		const float length = Length( axis );
		return length > 1e-6f && std::isfinite( length ) ? axis / length : Vector3( 1.f, 0.f, 0.f );
	};
	HelperDriver entry;
	entry.helper = helper;
	entry.driver = driver;
	entry.driverAxis = unit( driverAxis );
	entry.helperAxis = unit( helperAxis );
	entry.twist = twist;
	entry.swing = swing;
	m_helperDrivers.push_back( entry );
	m_needsBind = true;
}

void Tr2AnimationRetargeter::ClearHelperDrivers()
{
	m_helperDrivers.clear();
	m_needsBind = true;
}

void Tr2AnimationRetargeter::AddLeg( const std::string& name, const std::string& hip, const std::string& knee, const std::string& ankle )
{
	Leg& leg = FindOrAdd( m_legs, name );
	leg.hip = hip;
	leg.knee = knee;
	leg.ankle = ankle;
	m_needsBind = true;
}

void Tr2AnimationRetargeter::ClearLegs()
{
	m_legs.clear();
	m_needsBind = true;
}

bool Tr2AnimationRetargeter::SetLegGround( const std::string& name, float height, const Vector3& normal, float weight )
{
	for( Leg& leg : m_legs )
	{
		if( leg.name == name )
		{
			leg.ground = std::isfinite( height ) ? height : 0.f;
			const float length = Length( normal );
			leg.normal = length > 1e-6f && std::isfinite( length ) ? normal / length : Vector3( 0.f, 1.f, 0.f );
			leg.weight = Clamp01( weight );
			m_lastPoseCurrent = false;
			return true;
		}
	}
	return false;
}

bool Tr2AnimationRetargeter::SetClipContacts( const std::string& name, const std::string& leg, const std::vector<std::pair<float, float>>& contacts )
{
	Clip* clip = FindClip( name );
	if( !clip )
	{
		return false;
	}
	auto setting = std::find_if( clip->contactSettings.begin(), clip->contactSettings.end(), [&]( const auto& s ) { return s.first == leg; } );
	if( setting != clip->contactSettings.end() )
	{
		setting->second = contacts;
	}
	else
	{
		clip->contactSettings.emplace_back( leg, contacts );
	}
	// Legs are looked up by name when binding
	m_needsBind = true;
	return true;
}

void Tr2AnimationRetargeter::AddArm( const std::string& name, const std::string& shoulder, const std::string& elbow, const std::string& wrist )
{
	Arm& arm = FindOrAdd( m_arms, name );
	arm.shoulder = shoulder;
	arm.elbow = elbow;
	arm.wrist = wrist;
	m_needsBind = true;
}

void Tr2AnimationRetargeter::ClearArms()
{
	m_arms.clear();
	m_needsBind = true;
}

bool Tr2AnimationRetargeter::SetHandTarget( const std::string& name, const Vector3& position, float weight )
{
	for( Arm& arm : m_arms )
	{
		if( arm.name == name )
		{
			const bool finite = AllFinite( position );
			arm.target = finite ? position : Vector3( 0.f, 0.f, 0.f );
			arm.weight = finite ? Clamp01( weight ) : 0.f;
			m_lastPoseCurrent = false;
			return true;
		}
	}
	return false;
}

bool Tr2AnimationRetargeter::SetHandPole( const std::string& name, const Vector3& pole, bool on )
{
	for( Arm& arm : m_arms )
	{
		if( arm.name == name )
		{
			const bool finite = AllFinite( pole );
			arm.pole = finite ? pole : Vector3( 0.f, 0.f, 0.f );
			arm.poleOn = on && finite;
			m_lastPoseCurrent = false;
			return true;
		}
	}
	return false;
}

bool Tr2AnimationRetargeter::SetHandRotation( const std::string& name, const Quaternion& rotation, bool on )
{
	for( Arm& arm : m_arms )
	{
		if( arm.name == name )
		{
			// A turn has a length: none, or one not finite, turns nothing
			const float length = XMVectorGetX( XMQuaternionLength( rotation ) );
			const bool usable = std::isfinite( length ) && length > 1e-6f;
			arm.rotation = usable ? Quaternion( XMQuaternionNormalize( rotation ) ) : IdentityQuaternion();
			arm.rotationOn = on && usable;
			m_lastPoseCurrent = false;
			return true;
		}
	}
	return false;
}

void Tr2AnimationRetargeter::AddLookBone( const std::string& bone, float share )
{
	LookBone& look = FindOrAdd( m_lookBones, bone );
	look.share = std::isfinite( share ) ? std::max( 0.f, share ) : 0.f;
	m_needsBind = true;
}

void Tr2AnimationRetargeter::ClearLookBones()
{
	m_lookBones.clear();
	m_needsBind = true;
}

bool Tr2AnimationRetargeter::SetLookTarget( const Vector3& position, float weight, float maxDegrees )
{
	const bool finite = AllFinite( position );
	m_lookTarget = finite ? position : Vector3( 0.f, 0.f, 0.f );
	m_lookWeight = finite ? Clamp01( weight ) : 0.f;
	m_lookMaxAngle = XMConvertToRadians( std::isfinite( maxDegrees ) ? std::max( 0.f, std::min( 180.f, maxDegrees ) ) : 0.f );
	m_lastPoseCurrent = false;
	return !m_lookBones.empty();
}

bool Tr2AnimationRetargeter::SetClipEvents( const std::string& name, const std::vector<std::pair<float, std::string>>& events )
{
	Clip* clip = FindClip( name );
	if( !clip )
	{
		return false;
	}
	clip->events.clear();
	for( const auto& event : events )
	{
		// A looping clip's end is its start
		clip->events.emplace_back( clip->looping ? Wrap01( event.first ) : Clamp01( event.first ), event.second );
	}
	std::stable_sort( clip->events.begin(), clip->events.end(), []( const auto& a, const auto& b ) { return a.first < b.first; } );
	return true;
}

std::vector<std::pair<std::string, std::string>> Tr2AnimationRetargeter::GetFiredEvents()
{
	std::vector<std::pair<std::string, std::string>> fired;
	fired.swap( m_firedEvents );
	return fired;
}

bool Tr2AnimationRetargeter::SetRagdoll( Tr2PhysicsWorld* world, int id )
{
#ifdef TRINITY_HAS_PHYSICS_WORLD
	if( ( world == nullptr ) != ( id == 0 ) || id < 0 )
	{
		return false;	// a world and a ragdoll in it, or neither
	}
	m_ragdollWorld = world;
	m_ragdoll = id;
	m_lastRagdollVersion = -1;
	m_ragdollWarned = false;
	m_lastPoseCurrent = false;
	return true;
#else
	return false;	// this build has no physics world
#endif
}

bool Tr2AnimationRetargeter::AddRagdollBone( int part, const std::string& bone, const Vector3& offsetPosition, const Quaternion& offsetRotation )
{
	const float length = XMVectorGetX( XMQuaternionLength( offsetRotation ) );
	if( part < 0 || bone.empty() || !AllFinite( offsetPosition ) || !std::isfinite( length ) || length < 1e-6f )
	{
		return false;
	}
	auto existing = std::find_if( m_ragdollBones.begin(), m_ragdollBones.end(), [&]( const RagdollBone& entry ) { return entry.part == part; } );
	RagdollBone& entry = existing != m_ragdollBones.end() ? *existing : m_ragdollBones.emplace_back();
	entry.part = part;
	entry.name = bone;
	entry.offsetPosition = offsetPosition;
	entry.offsetRotation = XMQuaternionNormalize( offsetRotation );
	entry.bone = NO_BONE;
	m_needsBind = true;
	return true;
}

bool Tr2AnimationRetargeter::AddRagdollChain( const std::string& first, const std::string& last )
{
	if( first.empty() || last.empty() || first == last )
	{
		return false;
	}
	m_ragdollChains.emplace_back( first, last );
	m_needsBind = true;
	return true;
}

void Tr2AnimationRetargeter::ClearRagdollBones()
{
	m_ragdollBones.clear();
	m_ragdollChains.clear();
	m_needsBind = true;
}

bool Tr2AnimationRetargeter::SetRagdollFrame( const Vector3& position, const Quaternion& rotation, float scale )
{
	const float length = XMVectorGetX( XMQuaternionLength( rotation ) );
	if( !AllFinite( position ) || !std::isfinite( length ) || length < 1e-6f || !std::isfinite( scale ) || scale < 1e-6f )
	{
		return false;
	}
	m_ragdollFramePosition = position;
	m_ragdollFrameRotation = XMQuaternionNormalize( rotation );
	m_ragdollFrameScale = scale;
	m_lastPoseCurrent = false;
	return true;
}

float Tr2AnimationRetargeter::GetRagdollWeight() const
{
	return m_ragdollWeight;
}

void Tr2AnimationRetargeter::SetRagdollWeight( float weight )
{
	m_ragdollWeight = Clamp01( weight );
	m_lastPoseCurrent = false;
}

bool Tr2AnimationRetargeter::GetRagdollOn() const
{
	return m_ragdollOn;
}

void Tr2AnimationRetargeter::SetRagdollOn( bool on )
{
	m_ragdollOn = on;
	m_lastPoseCurrent = false;
}

void Tr2AnimationRetargeter::AddRig( const std::string& name, const std::string& bindPosePath, const std::string& rootBone )
{
	Rig* rig = FindRig( name );
	if( !rig )
	{
		m_rigs.emplace_back();
		rig = &m_rigs.back();
		rig->name = name;
	}
	rig->rootBone = rootBone;
	rig->boneMap.clear();
	rig->sameBones = false;
	rig->chains.clear();
	rig->turns.clear();
	rig->bindPosePath = bindPosePath;
	LoadRes( rig->bindPosePath, rig->bindPoseRes );
}

bool Tr2AnimationRetargeter::MapBone( const std::string& rig, const std::string& sourceBone, const std::string& targetBone )
{
	Rig* found = FindRig( rig );
	if( !found )
	{
		return false;
	}
	found->boneMap.push_back( { sourceBone, targetBone, false } );
	m_needsBind = true;
	return true;
}

bool Tr2AnimationRetargeter::MapBoneFromBind( const std::string& rig, const std::string& sourceBone, const std::string& targetBone )
{
	Rig* found = FindRig( rig );
	if( !found )
	{
		return false;
	}
	found->boneMap.push_back( { sourceBone, targetBone, true } );
	m_needsBind = true;
	return true;
}

bool Tr2AnimationRetargeter::MapSameBones( const std::string& rig )
{
	Rig* found = FindRig( rig );
	if( !found )
	{
		return false;
	}
	found->sameBones = true;
	m_needsBind = true;
	return true;
}

bool Tr2AnimationRetargeter::SetRigBoneTurn( const std::string& rig, const std::string& bone, const Vector3& axis, float degrees )
{
	Rig* found = FindRig( rig );
	const float length = Length( axis );
	if( !found || !std::isfinite( degrees ) || !AllFinite( axis ) || length < 1e-6f )
	{
		return false;
	}
	auto it = std::find_if( found->turns.begin(), found->turns.end(), [&]( const BoneTurn& turn ) { return turn.bone == bone; } );
	if( it == found->turns.end() )
	{
		// A bone not turned before is found in the skeleton at the next bind
		found->turns.push_back( BoneTurn{ bone } );
		it = found->turns.end() - 1;
		m_needsBind = true;
	}
	it->axis = axis / length;
	it->radians = XMConvertToRadians( degrees );
	m_lastPoseCurrent = false;
	return true;
}

void Tr2AnimationRetargeter::ClearRigs()
{
	Unbind();
	m_needsBind = true;

	for( Rig& rig : m_rigs )
	{
		ReleaseRes( rig.bindPoseRes );
	}
	m_rigs.clear();
}

void Tr2AnimationRetargeter::AddClip( const std::string& name, const std::string& path, bool looping, const std::string& rig, uint32_t layer )
{
	Clip* clip = FindClip( name );
	if( !clip )
	{
		m_clips.emplace_back();
		clip = &m_clips.back();
		clip->name = name;
	}
	clip->path = path;
	clip->looping = looping;
	clip->rig = rig;
	clip->layer = layer;
	clip->verticalMotion = true;
	clip->boneWeightSettings.clear();
	clip->contactSettings.clear();
	clip->events.clear();
	clip->eventsShowing = false;
	LoadRes( clip->path, clip->res );
}

void Tr2AnimationRetargeter::ClearClips()
{
	// The players point into the clips' data, so they go before the files
	Unbind();
	m_needsBind = true;

	for( Clip& clip : m_clips )
	{
		ReleaseRes( clip.res );
	}
	m_clips.clear();
}

bool Tr2AnimationRetargeter::SetClipWeight( const std::string& name, float weight )
{
	Clip* clip = FindClip( name );
	if( !clip )
	{
		return false;
	}
	clip->weight = weight;
	m_lastPoseCurrent = false;
	return true;
}

bool Tr2AnimationRetargeter::SetClipPhase( const std::string& name, float phase )
{
	Clip* clip = FindClip( name );
	if( !clip )
	{
		return false;
	}
	clip->phase = phase;
	m_lastPoseCurrent = false;
	return true;
}

bool Tr2AnimationRetargeter::SetClipBoneWeight( const std::string& name, const std::string& bone, float weight )
{
	Clip* clip = FindClip( name );
	if( !clip )
	{
		return false;
	}
	auto setting = std::find_if( clip->boneWeightSettings.begin(), clip->boneWeightSettings.end(), [&]( const auto& s ) { return s.first == bone; } );
	if( setting != clip->boneWeightSettings.end() )
	{
		setting->second = weight;
	}
	else
	{
		clip->boneWeightSettings.emplace_back( bone, weight );
	}
	// Bone names are looked up in the animation's skeleton when binding
	m_needsBind = true;
	return true;
}

bool Tr2AnimationRetargeter::SetClipVerticalMotion( const std::string& name, bool on )
{
	Clip* clip = FindClip( name );
	if( !clip )
	{
		return false;
	}
	if( clip->verticalMotion != on )
	{
		clip->verticalMotion = on;
		// Where the feet come down changes with the bob
		m_groundCalibrated = false;
		m_lastPoseCurrent = false;
	}
	return true;
}

Tr2AnimationRetargeter::Rig* Tr2AnimationRetargeter::FindRig( const std::string& name )
{
	const int32_t index = FindRigIndex( name );
	return index >= 0 ? &m_rigs[index] : nullptr;
}

int32_t Tr2AnimationRetargeter::FindRigIndex( const std::string& name ) const
{
	for( size_t i = 0; i < m_rigs.size(); ++i )
	{
		if( m_rigs[i].name == name )
		{
			return (int32_t)i;
		}
	}
	return -1;
}

Tr2AnimationRetargeter::Clip* Tr2AnimationRetargeter::FindClip( const std::string& name )
{
	for( Clip& clip : m_clips )
	{
		if( clip.name == name )
		{
			return &clip;
		}
	}
	return nullptr;
}

bool Tr2AnimationRetargeter::IsReady() const
{
	if( m_clips.empty() || !GetData( m_targetBindPoseRes ) )
	{
		return false;
	}
	for( const Rig& rig : m_rigs )
	{
		if( !rig.bindPosePath.empty() && !GetData( rig.bindPoseRes ) )
		{
			return false;
		}
	}
	for( const Clip& clip : m_clips )
	{
		if( !GetClipData( clip.res ) )
		{
			return false;
		}
	}
	return true;
}

void Tr2AnimationRetargeter::LoadRes( const std::string& path, TriGrannyResPtr& res )
{
	// Drop anything pointing into the old data first
	Unbind();
	m_needsBind = true;

	ReleaseRes( res );

	if( !path.empty() )
	{
		BeResMan->GetResource( path.c_str(), "raw", BlueInterfaceIID<TriGrannyRes>(), (void**)&res );
	}

	// A file can sit in more than one slot, a clip and a rig's bind pose say; it is watched once
	if( res && CountSlotsHolding( res ) == 1 )
	{
		res->AddNotifyTarget( this );
	}
}

void Tr2AnimationRetargeter::ReleaseRes( TriGrannyResPtr& res )
{
	if( !res )
	{
		return;
	}
	// Stop watching the file once no other slot holds it
	if( CountSlotsHolding( res ) == 1 )
	{
		res->RemoveNotifyTarget( this );
	}
	res.Unlock();
}

uint32_t Tr2AnimationRetargeter::CountSlotsHolding( const TriGrannyRes* res ) const
{
	auto holds = [res]( const TriGrannyResPtr& slot ) -> uint32_t {
		const TriGrannyRes* held = slot;
		return held == res ? 1 : 0;
	};
	uint32_t count = holds( m_targetBindPoseRes );
	for( const Rig& rig : m_rigs )
	{
		count += holds( rig.bindPoseRes );
	}
	for( const Clip& clip : m_clips )
	{
		count += holds( clip.res );
	}
	return count;
}

void Tr2AnimationRetargeter::ReleaseCachedData( BlueAsyncRes* )
{
	Unbind();
	m_needsBind = true;
}

void Tr2AnimationRetargeter::RebuildCachedData( BlueAsyncRes* )
{
	m_needsBind = true;
}

void Tr2AnimationRetargeter::Unbind()
{
	for( Clip& clip : m_clips )
	{
		clip.rigIndex = -1;
		clip.player.reset();
		clip.skeleton = nullptr;
		clip.duration = 0.f;
		clip.sourceBones.clear();
		clip.boneWeights.clear();
		clip.groundOffset = 0.f;
		clip.travelStart = clip.travel = clip.swayCentre = Vector3( 0.f, 0.f, 0.f );
		clip.legContacts.clear();
	}
	for( Rig& rig : m_rigs )
	{
		rig.bound = false;
		rig.linkSources.clear();
		rig.offsets.clear();
		rig.linkForTargetBone.clear();
		rig.linkTargets.clear();
		for( BoneTurn& turn : rig.turns )
		{
			turn.targetBone = NO_BONE;
		}
		rig.feet.clear();
		rig.pelvisTarget = NO_BONE;
		rig.chainJoints.clear();
		rig.chainJointForTargetBone.clear();
	}
	for( HelperDriver& driver : m_helperDrivers )
	{
		driver.helperBone = driver.driverBone = NO_BONE;
	}
	for( Leg& leg : m_legs )
	{
		leg.pelvisBone = leg.hipBone = leg.kneeBone = leg.ankleBone = NO_BONE;
	}
	m_legsBound = false;
	for( Arm& arm : m_arms )
	{
		arm.shoulderBone = arm.elbowBone = arm.wristBone = NO_BONE;
	}
	for( LookBone& look : m_lookBones )
	{
		look.bone = NO_BONE;
	}
	m_lookBound = false;
	for( RagdollBone& part : m_ragdollBones )
	{
		part.bone = NO_BONE;
	}
	m_ragdollShares.clear();
	m_ragdollShareForBone.clear();
	m_ragdollBound = false;
	m_bound = false;
	m_boundTargetBoneCount = 0;
	m_groundCalibrated = false;

	// The skeleton the last pose was made on may be the data being released
	m_lastSkeleton = nullptr;
	m_lastPoseCurrent = false;
}

bool Tr2AnimationRetargeter::Bind( const cmf::Skeleton& target )
{
	Unbind();
	m_boundTargetBoneCount = target.bones.size();

	if( !IsReady() || m_rigs.empty() )
	{
		// Try again next frame: a file is still loading, or no clip or rig is set up yet
		m_needsBind = true;
		return false;
	}
	// From here a failure is not retried until a file, a rig, a clip or the animation's skeleton changes
	m_needsBind = false;

	const cmf::Skeleton& targetBind = GetData( m_targetBindPoseRes )->skeletons[0];
	std::vector<cmf::Transform> targetBindWorld;
	ComputeWorld( targetBind, targetBind.restTransforms.data(), targetBindWorld );

	for( Clip& clip : m_clips )
	{
		clip.rigIndex = FindRigIndex( clip.rig );
		if( clip.rigIndex < 0 )
		{
			CCP_LOGWARN( "Tr2AnimationRetargeter: clip %s names rig %s, but there is no such rig", clip.name.c_str(), clip.rig.c_str() );
		}

		// A bone takes its own setting, or else its parent's; parents come first
		std::vector<float> settings( target.bones.size(), -1.f );
		for( const auto& setting : clip.boneWeightSettings )
		{
			const uint32_t bone = FindBone( target, setting.first );
			if( bone == NO_BONE )
			{
				CCP_LOGWARN( "Tr2AnimationRetargeter: clip %s weights bone %s, which is not in the animation's skeleton", clip.name.c_str(), setting.first.c_str() );
				continue;
			}
			settings[bone] = Clamp01( setting.second );
		}
		clip.boneWeights.resize( target.bones.size() );
		for( uint32_t i = 0; i < target.bones.size(); ++i )
		{
			clip.boneWeights[i] = settings[i] >= 0.f ? settings[i] : ( HasParent( target, i ) ? clip.boneWeights[target.parents[i]] : 1.f );
		}
	}

	BindHelpersAndLegs( target, targetBind, targetBindWorld );
	BindReach( target, targetBind, targetBindWorld );
	BindRagdoll( target, targetBind, targetBindWorld );

	for( uint32_t r = 0; r < m_rigs.size(); ++r )
	{
		m_bound = BindRig( r, target, targetBind, targetBindWorld ) || m_bound;
	}
	return m_bound;
}

bool Tr2AnimationRetargeter::BindRig( uint32_t rigIndex, const cmf::Skeleton& target, const cmf::Skeleton& targetBind, const std::vector<cmf::Transform>& targetBindWorld )
{
	Rig& rig = m_rigs[rigIndex];

	std::vector<Clip*> clips;
	std::vector<const cmf::Data*> clipData;
	for( Clip& clip : m_clips )
	{
		if( clip.rigIndex == (int32_t)rigIndex )
		{
			clips.push_back( &clip );
			clipData.push_back( GetClipData( clip.res ) );
		}
	}
	if( clips.empty() )
	{
		// Nothing plays on it
		return false;
	}
	const size_t clipCount = clips.size();

	const cmf::Data* sourceBindData = rig.bindPosePath.empty() ? clipData[0] : GetData( rig.bindPoseRes );
	const cmf::Skeleton& sourceBind = sourceBindData->skeletons[0];
	std::vector<cmf::Transform> sourceBindWorld;
	ComputeWorld( sourceBind, sourceBind.restTransforms.data(), sourceBindWorld );

	// The pairs given one by one come first, so they win over the same-named ones
	std::vector<BoneMapping> pairs = rig.boneMap;
	const size_t givenCount = pairs.size();
	if( rig.sameBones )
	{
		for( uint32_t i = 0; i < target.bones.size(); ++i )
		{
			const std::string name = cmf::ToStdString( target.bones[i] );
			pairs.push_back( { name, name, true } );
		}
	}

	// Resolve every pair in each skeleton; bone order differs between files
	struct Resolved
	{
		uint32_t target, sourceBind, targetBind;
		std::vector<uint32_t> source;	// per clip
		bool fromBind;
		const std::string* sourceName;
	};
	std::vector<Resolved> resolved;
	std::vector<int32_t> resolvedForTarget( target.bones.size(), -1 );
	uint32_t skipped = 0;
	for( size_t p = 0; p < pairs.size(); ++p )
	{
		const BoneMapping& mapping = pairs[p];
		const bool given = p < givenCount;
		Resolved r{ FindBone( target, mapping.target ), FindBone( sourceBind, mapping.source ), FindBone( targetBind, mapping.target ), std::vector<uint32_t>( clipCount, NO_BONE ), mapping.fromBind, &mapping.source };
		bool missing = r.target == NO_BONE || r.sourceBind == NO_BONE || r.targetBind == NO_BONE;
		for( size_t c = 0; c < clipCount && !missing; ++c )
		{
			r.source[c] = FindBone( clipData[c]->skeletons[0], mapping.source );
			missing = r.source[c] == NO_BONE;
		}
		if( missing )
		{
			// A bone of the animation's skeleton the rig does not have is no surprise when pairing by name
			if( given )
			{
				CCP_LOGWARN( "Tr2AnimationRetargeter: rig %s skips %s -> %s, a bone is missing from one of the skeletons", rig.name.c_str(), mapping.source.c_str(), mapping.target.c_str() );
			}
			++skipped;
			continue;
		}
		if( resolvedForTarget[r.target] >= 0 )
		{
			if( given )
			{
				CCP_LOGWARN( "Tr2AnimationRetargeter: rig %s pairs %s more than once, keeping its first pair", rig.name.c_str(), mapping.target.c_str() );
			}
			++skipped;
			continue;
		}
		resolvedForTarget[r.target] = (int32_t)resolved.size();
		resolved.push_back( std::move( r ) );
	}

	// The pelvis: the pair of the rig's root bone
	uint32_t pelvis = NO_BONE;
	for( uint32_t i = 0; i < resolved.size(); ++i )
	{
		if( *resolved[i].sourceName == rig.rootBone )
		{
			pelvis = i;
			break;
		}
	}
	if( pelvis == NO_BONE )
	{
		CCP_LOGWARN( "Tr2AnimationRetargeter: rig %s has no pair for its root bone %s, so its clips do not play", rig.name.c_str(), rig.rootBone.c_str() );
		return false;
	}

	// The lowest paired bone in each bind pose: the feet are found and the pelvis heights measured from it
	float sourceLowest = FLT_MAX;
	float targetLowest = FLT_MAX;
	for( const Resolved& r : resolved )
	{
		sourceLowest = std::min( sourceLowest, sourceBindWorld[r.sourceBind].position.y );
		targetLowest = std::min( targetLowest, targetBindWorld[r.targetBind].position.y );
	}

	rig.pelvisTarget = resolved[pelvis].target;
	rig.pelvisLink = pelvis;
	rig.pelvisSourceBindHeight = sourceBindWorld[resolved[pelvis].sourceBind].position.y;
	rig.pelvisTargetBindPosition = targetBindWorld[resolved[pelvis].targetBind].position;

	// Heights above the feet: a rig's origin can sit anywhere (Mixamo's is at the hips)
	const float sourceHeight = rig.pelvisSourceBindHeight - sourceLowest;
	const float targetHeight = rig.pelvisTargetBindPosition.y - targetLowest;
	rig.pelvisHeightScale = sourceHeight > MIN_PELVIS_HEIGHT && targetHeight > MIN_PELVIS_HEIGHT ? targetHeight / sourceHeight : 1.f;

	// Nearest paired ancestor of each target bone, from the live skeleton
	auto pairedAncestor = [&]( uint32_t bone ) -> int32_t {
		while( HasParent( target, bone ) )
		{
			bone = target.parents[bone];
			if( resolvedForTarget[bone] >= 0 )
			{
				return resolvedForTarget[bone];
			}
		}
		return -1;
	};

	// Per paired bone, a correction that turns the target's bind direction (towards its first paired child, in pair
	// order) onto the source's.  Bones without a paired child keep their parent's correction, the topmost paired bones
	// and bones paired from the bind pose none.  Visiting in target order does the parent first.
	std::vector<Quaternion> correction( resolved.size(), IdentityQuaternion() );
	std::vector<uint32_t> order( resolved.size() );
	for( uint32_t i = 0; i < order.size(); ++i )
	{
		order[i] = i;
	}
	std::sort( order.begin(), order.end(), [&]( uint32_t a, uint32_t b ) { return resolved[a].target < resolved[b].target; } );

	for( uint32_t i : order )
	{
		const int32_t ancestor = pairedAncestor( resolved[i].target );
		if( ancestor < 0 )
		{
			continue;
		}
		correction[i] = correction[ancestor];
		if( resolved[i].fromBind )
		{
			correction[i] = IdentityQuaternion();
			continue;
		}
		for( uint32_t j = 0; j < resolved.size(); ++j )
		{
			if( pairedAncestor( resolved[j].target ) != (int32_t)i )
			{
				continue;
			}
			const Vector3 targetDirection = targetBindWorld[resolved[j].targetBind].position - targetBindWorld[resolved[i].targetBind].position;
			const Vector3 sourceDirection = sourceBindWorld[resolved[j].sourceBind].position - sourceBindWorld[resolved[i].sourceBind].position;
			if( LengthSq( targetDirection ) > 1e-10f && LengthSq( sourceDirection ) > 1e-10f )
			{
				TriQuaternionRotationArc( &correction[i], &targetDirection, &sourceDirection );
			}
			break;
		}
	}

	// Target bind rotation, then the correction, then back out of the source bind rotation
	rig.linkForTargetBone.assign( target.bones.size(), -1 );
	for( uint32_t i = 0; i < resolved.size(); ++i )
	{
		const Quaternion targetBindRotation = targetBindWorld[resolved[i].targetBind].rotation;
		const Quaternion sourceBindRotation = sourceBindWorld[resolved[i].sourceBind].rotation;
		const Quaternion offset = XMQuaternionNormalize( XMQuaternionMultiply( XMQuaternionMultiply( targetBindRotation, correction[i] ), XMQuaternionConjugate( sourceBindRotation ) ) );
		rig.linkForTargetBone[resolved[i].target] = (int32_t)rig.offsets.size();
		rig.linkTargets.push_back( resolved[i].target );
		rig.offsets.push_back( offset );
		rig.linkSources.push_back( *resolved[i].sourceName );
	}

	for( BoneTurn& turn : rig.turns )
	{
		turn.targetBone = FindBone( target, turn.bone );
	}

	BindChains( rig, target, targetBind, targetBindWorld );

	// The feet: the paired bones lowest in the target bind pose
	for( const Resolved& r : resolved )
	{
		if( targetBindWorld[r.targetBind].position.y < targetLowest + FOOT_BAND )
		{
			rig.feet.push_back( r.target );
		}
	}

	for( size_t c = 0; c < clipCount; ++c )
	{
		Clip& clip = *clips[c];
		clip.skeleton = &clipData[c]->skeletons[0];
		clip.player = std::make_unique<cmf::AnimationPlayer>( *clip.skeleton, clipData[c]->animations[0] );
		clip.duration = clipData[c]->animations[0].duration;
		clip.sourceBones.reserve( resolved.size() );
		for( const Resolved& r : resolved )
		{
			clip.sourceBones.push_back( r.source[c] );
		}
		if( clip.looping )
		{
			MeasureTravel( clip, rig );
		}
	}
	rig.bound = true;

	CCP_LOG(
		"Tr2AnimationRetargeter: rig %s bound %u bone pairs (%u skipped) for %u clips; pelvis %s, %u foot bones",
		rig.name.c_str(),
		(uint32_t)resolved.size(),
		skipped,
		(uint32_t)clipCount,
		cmf::ToStdString( target.bones[rig.pelvisTarget] ).c_str(),
		(uint32_t)rig.feet.size() );
	return true;
}

void Tr2AnimationRetargeter::BindChains( Rig& rig, const cmf::Skeleton& target, const cmf::Skeleton& targetBind, const std::vector<cmf::Transform>& targetBindWorld )
{
	rig.chainJoints.clear();
	rig.chainJointForTargetBone.assign( target.bones.size(), -1 );
	auto bindIndex = [&]( uint32_t bone ) { return FindBone( targetBind, cmf::ToStdString( target.bones[bone] ) ); };

	for( const auto& chain : rig.chains )
	{
		const uint32_t first = FindBone( target, chain.first );
		const uint32_t last = FindBone( target, chain.second );
		if( first == NO_BONE || last == NO_BONE || rig.linkForTargetBone[first] < 0 || rig.linkForTargetBone[last] < 0 )
		{
			CCP_LOGWARN( "Tr2AnimationRetargeter: rig %s's chain %s to %s needs both ends in the skeleton and paired", rig.name.c_str(), chain.first.c_str(), chain.second.c_str() );
			continue;
		}
		// The bones from the first down to the last
		std::vector<uint32_t> path{ last };
		while( path.back() != first && HasParent( target, path.back() ) )
		{
			path.push_back( target.parents[path.back()] );
		}
		if( path.back() != first )
		{
			CCP_LOGWARN( "Tr2AnimationRetargeter: rig %s's chain %s to %s: %s is not above %s", rig.name.c_str(), chain.first.c_str(), chain.second.c_str(), chain.first.c_str(), chain.second.c_str() );
			continue;
		}
		std::reverse( path.begin(), path.end() );

		// How far down the chain each bone sits in the target bind pose
		std::vector<uint32_t> bind( path.size() );
		std::vector<float> along( path.size(), 0.f );
		bool complete = true;
		for( size_t k = 0; k < path.size(); ++k )
		{
			bind[k] = bindIndex( path[k] );
			complete = complete && bind[k] != NO_BONE;
			if( complete && k > 0 )
			{
				along[k] = along[k - 1] + Length( targetBindWorld[bind[k]].position - targetBindWorld[bind[k - 1]].position );
			}
		}
		if( !complete )
		{
			CCP_LOGWARN( "Tr2AnimationRetargeter: rig %s's chain %s to %s has a bone the target bind pose lacks", rig.name.c_str(), chain.first.c_str(), chain.second.c_str() );
			continue;
		}

		// Between each two paired bones, the unpaired ones take a share of their turns
		size_t from = 0;
		for( size_t k = 1; k < path.size(); ++k )
		{
			if( rig.linkForTargetBone[path[k]] < 0 )
			{
				continue;
			}
			const float span = along[k] - along[from];
			for( size_t j = from + 1; j < k; ++j )
			{
				if( rig.chainJointForTargetBone[path[j]] >= 0 )
				{
					continue;	// an earlier chain has it
				}
				ChainJoint joint;
				joint.fromLink = rig.linkForTargetBone[path[from]];
				joint.toLink = rig.linkForTargetBone[path[k]];
				joint.share = span > 1e-6f ? ( along[j] - along[from] ) / span : 0.5f;
				joint.bindWorld = targetBindWorld[bind[j]].rotation;
				joint.fromBindWorld = targetBindWorld[bind[from]].rotation;
				joint.toBindWorld = targetBindWorld[bind[k]].rotation;
				rig.chainJointForTargetBone[path[j]] = (int32_t)rig.chainJoints.size();
				rig.chainJoints.push_back( joint );
			}
			from = k;
		}
	}
}

void Tr2AnimationRetargeter::BindHelpersAndLegs( const cmf::Skeleton& target, const cmf::Skeleton& targetBind, const std::vector<cmf::Transform>& targetBindWorld )
{
	for( HelperDriver& driver : m_helperDrivers )
	{
		driver.helperBone = FindBone( target, driver.helper );
		driver.driverBone = FindBone( target, driver.driver );
		const uint32_t helperBind = FindBone( targetBind, driver.helper );
		const uint32_t driverBind = FindBone( targetBind, driver.driver );
		if( driver.helperBone == NO_BONE || driver.driverBone == NO_BONE || helperBind == NO_BONE || driverBind == NO_BONE )
		{
			CCP_LOGWARN( "Tr2AnimationRetargeter: helper %s, driven by %s: a bone is missing from the skeleton or the target bind pose", driver.helper.c_str(), driver.driver.c_str() );
			driver.helperBone = driver.driverBone = NO_BONE;
			continue;
		}
		driver.helperRest = targetBind.restTransforms[helperBind].rotation;
		driver.driverRest = targetBind.restTransforms[driverBind].rotation;
		// A turn in the driver's bind frame, seen from the helper's
		driver.toHelper = XMQuaternionNormalize( Hamilton( XMQuaternionConjugate( targetBindWorld[helperBind].rotation ), targetBindWorld[driverBind].rotation ) );
	}

	m_legsBound = false;
	for( Leg& leg : m_legs )
	{
		leg.hipBone = FindBone( target, leg.hip );
		leg.kneeBone = FindBone( target, leg.knee );
		leg.ankleBone = FindBone( target, leg.ankle );
		const bool chain = leg.hipBone != NO_BONE && leg.kneeBone != NO_BONE && leg.ankleBone != NO_BONE && HasParent( target, leg.hipBone ) &&
			target.parents[leg.kneeBone] == leg.hipBone && target.parents[leg.ankleBone] == leg.kneeBone;
		if( !chain )
		{
			CCP_LOGWARN( "Tr2AnimationRetargeter: leg %s needs %s, %s and %s in the skeleton, each the parent of the next", leg.name.c_str(), leg.hip.c_str(), leg.knee.c_str(), leg.ankle.c_str() );
			leg.pelvisBone = leg.hipBone = leg.kneeBone = leg.ankleBone = NO_BONE;
			continue;
		}
		leg.pelvisBone = target.parents[leg.hipBone];
		m_legsBound = true;
	}

	for( Clip& clip : m_clips )
	{
		clip.legContacts.assign( m_legs.size(), {} );
		for( const auto& setting : clip.contactSettings )
		{
			auto leg = std::find_if( m_legs.begin(), m_legs.end(), [&]( const Leg& l ) { return l.name == setting.first; } );
			if( leg == m_legs.end() )
			{
				CCP_LOGWARN( "Tr2AnimationRetargeter: clip %s has contacts for leg %s, but there is no such leg", clip.name.c_str(), setting.first.c_str() );
				continue;
			}
			clip.legContacts[leg - m_legs.begin()] = setting.second;
		}
	}
}

void Tr2AnimationRetargeter::BindReach( const cmf::Skeleton& target, const cmf::Skeleton& targetBind, const std::vector<cmf::Transform>& targetBindWorld )
{
	for( Arm& arm : m_arms )
	{
		arm.shoulderBone = FindBone( target, arm.shoulder );
		arm.elbowBone = FindBone( target, arm.elbow );
		arm.wristBone = FindBone( target, arm.wrist );
		const bool chain = arm.shoulderBone != NO_BONE && arm.elbowBone != NO_BONE && arm.wristBone != NO_BONE &&
			target.parents[arm.elbowBone] == arm.shoulderBone && target.parents[arm.wristBone] == arm.elbowBone;
		if( !chain )
		{
			CCP_LOGWARN( "Tr2AnimationRetargeter: arm %s needs %s, %s and %s in the skeleton, each the parent of the next", arm.name.c_str(), arm.shoulder.c_str(), arm.elbow.c_str(), arm.wrist.c_str() );
			arm.shoulderBone = arm.elbowBone = arm.wristBone = NO_BONE;
		}
	}

	m_lookBound = false;
	if( m_lookBones.empty() )
	{
		return;
	}
	bool chain = true;
	for( size_t k = 0; k < m_lookBones.size() && chain; ++k )
	{
		LookBone& look = m_lookBones[k];
		look.bone = FindBone( target, look.name );
		chain = look.bone != NO_BONE && ( k == 0 || IsAncestor( target, m_lookBones[k - 1].bone, look.bone ) );
	}
	const uint32_t headBind = FindBone( targetBind, m_lookBones.back().name );
	if( !chain || headBind == NO_BONE )
	{
		CCP_LOGWARN( "Tr2AnimationRetargeter: the look bones need to be in the skeleton, each above the next, and the last in the target bind pose" );
		return;
	}
	// The head's forward: where the model's +Z points in the target bind pose, in the head's frame
	m_lookForward = XMVector3InverseRotate( Vector3( 0.f, 0.f, 1.f ), targetBindWorld[headBind].rotation );
	m_lookBound = true;
}

bool Tr2AnimationRetargeter::IsPlayable( const Clip& clip ) const
{
	return clip.rigIndex >= 0 && m_rigs[clip.rigIndex].bound && clip.player;
}

void Tr2AnimationRetargeter::SampleSource( Clip& clip, float time )
{
	cmf::RestPose( clip.sourcePose, *clip.skeleton );
	clip.player->SampleAtLocalTime( clip.sourcePose, time );
	ComputeWorld( *clip.skeleton, clip.sourcePose.boneTransforms.data(), clip.sourceWorld );
}

void Tr2AnimationRetargeter::MeasureTravel( Clip& clip, const Rig& rig )
{
	// The line the source pelvis travels along, from where it starts to where it ends, and the middle of its sway about
	// that line, which a clip made in place need not start at
	const uint32_t pelvis = clip.sourceBones[rig.pelvisLink];
	SampleSource( clip, 0.f );
	clip.travelStart = clip.sourceWorld[pelvis].position;
	SampleSource( clip, clip.duration );
	clip.travel = clip.sourceWorld[pelvis].position - clip.travelStart;

	Vector3 centre( 0.f, 0.f, 0.f );
	for( uint32_t k = 0; k < TRAVEL_SAMPLES; ++k )
	{
		const float phase = (float)k / TRAVEL_SAMPLES;
		SampleSource( clip, phase * clip.duration );
		centre += clip.sourceWorld[pelvis].position - ( clip.travelStart + clip.travel * phase );
	}
	clip.swayCentre = centre / (float)TRAVEL_SAMPLES;
}

void Tr2AnimationRetargeter::Retarget( Clip& clip, float phase, const cmf::Skeleton& skeleton, const cmf::SkeletonPose& base, cmf::SkeletonPose& out )
{
	const Rig& rig = m_rigs[clip.rigIndex];

	// Source pose and its world transforms; a clip that does not loop stops at its end
	const float at = clip.looping ? Wrap01( phase ) : Clamp01( phase );
	SampleSource( clip, at * clip.duration );

	// Retargeted pose: unpaired bones keep the pose below
	out.skeleton = base.skeleton;
	out.boneTransforms = base.boneTransforms;
	m_targetWorld.resize( skeleton.bones.size() );

	// The world rotation each paired bone takes, which does not depend on the target's parents: a chain's bones
	// between two paired ones need the turns of both before either is reached
	m_linkWorld.resize( rig.offsets.size() );
	for( size_t link = 0; link < rig.offsets.size(); ++link )
	{
		m_linkWorld[link] = XMQuaternionNormalize( XMQuaternionMultiply( rig.offsets[link], clip.sourceWorld[clip.sourceBones[link]].rotation ) );
	}
	if( !rig.turns.empty() )
	{
		// Each bone's own turn, then those of the turned bones above it, so a limb turns about its top joint as one
		m_turnForBone.assign( skeleton.bones.size(), IdentityQuaternion() );
		for( uint32_t i = 0; i < skeleton.bones.size(); ++i )
		{
			Quaternion own = IdentityQuaternion();
			for( const BoneTurn& turn : rig.turns )
			{
				if( turn.targetBone == i && turn.radians != 0.f )
				{
					own = XMQuaternionMultiply( own, XMQuaternionRotationNormal( turn.axis, turn.radians ) );
				}
			}
			m_turnForBone[i] = HasParent( skeleton, i ) ? Quaternion( XMQuaternionMultiply( own, m_turnForBone[skeleton.parents[i]] ) ) : own;
		}
		for( size_t link = 0; link < rig.linkTargets.size(); ++link )
		{
			m_linkWorld[link] = XMQuaternionNormalize( XMQuaternionMultiply( m_linkWorld[link], m_turnForBone[rig.linkTargets[link]] ) );
		}
	}
	const bool chains = m_chainsOn && !rig.chainJoints.empty();

	const cmf::Transform identity{ Vector3( 0.f, 0.f, 0.f ), IdentityQuaternion(), Vector3( 1.f, 1.f, 1.f ) };
	for( uint32_t i = 0; i < skeleton.bones.size(); ++i )
	{
		const cmf::Transform& parentWorld = HasParent( skeleton, i ) ? m_targetWorld[skeleton.parents[i]] : identity;
		cmf::Transform& local = out.boneTransforms[i];

		const int32_t link = rig.linkForTargetBone[i];
		if( chains && link < 0 && rig.chainJointForTargetBone[i] >= 0 )
		{
			// The paired ends' turns from their bind poses, in the model's space, shared out by where it sits between them
			const ChainJoint& joint = rig.chainJoints[rig.chainJointForTargetBone[i]];
			const Quaternion from = XMQuaternionMultiply( XMQuaternionConjugate( joint.fromBindWorld ), m_linkWorld[joint.fromLink] );
			const Quaternion to = XMQuaternionMultiply( XMQuaternionConjugate( joint.toBindWorld ), m_linkWorld[joint.toLink] );
			const Quaternion world = XMQuaternionNormalize( XMQuaternionMultiply( joint.bindWorld, XMQuaternionSlerp( from, to, joint.share ) ) );
			local.rotation = XMQuaternionNormalize( XMQuaternionMultiply( world, XMQuaternionConjugate( parentWorld.rotation ) ) );
		}
		if( link >= 0 )
		{
			const cmf::Transform& source = clip.sourceWorld[clip.sourceBones[link]];
			const Quaternion& world = m_linkWorld[link];
			local.rotation = XMQuaternionNormalize( XMQuaternionMultiply( world, XMQuaternionConjugate( parentWorld.rotation ) ) );

			if( i == rig.pelvisTarget )
			{
				// The bob, if any, and a looping clip's sway about its line of travel: the clip plays in place and the
				// caller moves the character
				Vector3 wanted = rig.pelvisTargetBindPosition;
				if( clip.verticalMotion )
				{
					wanted.y += ( source.position.y - rig.pelvisSourceBindHeight ) * rig.pelvisHeightScale;
				}
				wanted.y += clip.groundOffset;
				if( clip.looping && m_pelvisSway )
				{
					const Vector3 line = clip.travelStart + clip.travel * at + clip.swayCentre;
					wanted.x += ( source.position.x - line.x ) * rig.pelvisHeightScale;
					wanted.z += ( source.position.z - line.z ) * rig.pelvisHeightScale;
				}
				local.position = cmf::TransformPoint( wanted, cmf::Inverse( parentWorld ) );
			}
		}

		m_targetWorld[i] = HasParent( skeleton, i ) ? cmf::Multiply( local, parentWorld ) : local;
	}

	if( m_helperDriversOn )
	{
		DriveHelpers( rig, out );
	}
}

void Tr2AnimationRetargeter::DriveHelpers( const Rig& rig, cmf::SkeletonPose& out ) const
{
	for( const HelperDriver& driver : m_helperDrivers )
	{
		// A helper the rig pairs plays from the clip
		if( driver.helperBone == NO_BONE || rig.linkForTargetBone[driver.helperBone] >= 0 )
		{
			continue;
		}
		DriveHelper( driver, out );
	}
}

void Tr2AnimationRetargeter::DriveHelper( const HelperDriver& driver, cmf::SkeletonPose& out )
{
	// The driver's turn from its bind pose, split in its own frame into a twist about its axis and a swing
	const Quaternion turn = Hamilton( XMQuaternionConjugate( driver.driverRest ), out.boneTransforms[driver.driverBone].rotation );
	Quaternion swing, twist;
	SwingTwist( turn, driver.driverAxis, swing, twist );
	float angle = 2.f * atan2f( Dot( Vector3( twist.x, twist.y, twist.z ), driver.driverAxis ), twist.w );
	angle = angle > XM_PI ? angle - XM_2PI : ( angle < -XM_PI ? angle + XM_2PI : angle );
	// The helper twists about its own axis by its share, and swings by its share of the swing seen from its frame
	const Quaternion seenSwing = Hamilton( Hamilton( driver.toHelper, swing ), XMQuaternionConjugate( driver.toHelper ) );
	const Quaternion helperTurn = Hamilton( Scaled( seenSwing, driver.swing ), XMQuaternionRotationNormal( driver.helperAxis, angle * driver.twist ) );
	out.boneTransforms[driver.helperBone].rotation = XMQuaternionNormalize( Hamilton( driver.helperRest, helperTurn ) );
}

float Tr2AnimationRetargeter::Planted( uint32_t leg ) const
{
	// Each clip showing says, by its contacts at its phase, whether the foot is down; standing in the pose below, it is
	float down = 0.f;
	float total = 0.f;
	for( const Clip& clip : m_clips )
	{
		const float weight = Clamp01( clip.weight );
		if( weight <= 0.f || !IsPlayable( clip ) || clip.legContacts.size() <= leg || clip.legContacts[leg].empty() )
		{
			continue;
		}
		const float phase = clip.looping ? Wrap01( clip.phase ) : Clamp01( clip.phase );
		bool inContact = false;
		for( const auto& contact : clip.legContacts[leg] )
		{
			// A contact over the seam runs past 1
			inContact = inContact || ( phase >= contact.first && phase < contact.second ) || ( contact.second > 1.f && phase < contact.second - 1.f );
		}
		down += inContact ? weight : 0.f;
		total += weight;
	}
	return total > 0.f ? down / total : 1.f;
}

bool Tr2AnimationRetargeter::HasGroundToStandOn() const
{
	if( !m_footIkOn )
	{
		return false;
	}
	for( const Leg& leg : m_legs )
	{
		if( leg.weight > 0.f )
		{
			return true;
		}
	}
	return false;
}

void Tr2AnimationRetargeter::StandOnGround( const cmf::Skeleton& skeleton, cmf::SkeletonPose& pose )
{
	if( !m_legsBound || !HasGroundToStandOn() )
	{
		return;
	}
	ComputeWorld( skeleton, pose.boneTransforms.data(), m_ikWorld );
	const cmf::Transform identity{ Vector3( 0.f, 0.f, 0.f ), IdentityQuaternion(), Vector3( 1.f, 1.f, 1.f ) };

	// The pelvis drops by the deepest the ground goes under a foot, as far as allowed, for that foot to reach
	float drop = 0.f;
	uint32_t pelvis = NO_BONE;
	for( const Leg& leg : m_legs )
	{
		if( leg.hipBone != NO_BONE )
		{
			drop = std::min( drop, leg.weight * leg.ground );
			pelvis = leg.pelvisBone;
		}
	}
	drop = std::max( drop, -m_ikMaxPelvisDrop );
	if( drop < 0.f && pelvis != NO_BONE )
	{
		Vector3 wanted = m_ikWorld[pelvis].position;
		wanted.y += drop;
		const cmf::Transform& parentWorld = HasParent( skeleton, pelvis ) ? m_ikWorld[skeleton.parents[pelvis]] : identity;
		pose.boneTransforms[pelvis].position = cmf::TransformPoint( wanted, cmf::Inverse( parentWorld ) );
		ComputeWorld( skeleton, pose.boneTransforms.data(), m_ikWorld );
	}

	const Vector3 up( 0.f, 1.f, 0.f );
	const float maxTilt = XMConvertToRadians( m_ikMaxFootTilt );
	for( uint32_t l = 0; l < m_legs.size(); ++l )
	{
		const Leg& leg = m_legs[l];
		if( leg.hipBone == NO_BONE )
		{
			continue;
		}
		// How much higher the foot goes than it is now, and how far it tilts to the ground's slope while planted. A leg
		// not following the ground (weight 0) still takes back the pelvis's drop, keeping its foot where the clips hold it
		const float raise = leg.weight * leg.ground - drop;
		Quaternion tilt = IdentityQuaternion();
		const float planted = Planted( l ) * leg.weight;
		if( planted > 0.f && Dot( leg.normal, up ) < 0.99999f )
		{
			TriQuaternionRotationArc( &tilt, &up, &leg.normal );
			const float angle = AngleOf( tilt );
			const float limit = angle > maxTilt && angle > 0.f ? maxTilt / angle : 1.f;
			tilt = Scaled( tilt, planted * limit );
		}
		if( fabsf( raise ) < 1e-4f && AngleOf( tilt ) < 1e-4f )
		{
			continue;
		}

		const cmf::Transform& hipWorld = m_ikWorld[leg.hipBone];
		const cmf::Transform& kneeWorld = m_ikWorld[leg.kneeBone];
		const cmf::Transform& ankleWorld = m_ikWorld[leg.ankleBone];
		const Vector3 target = ankleWorld.position + Vector3( 0.f, raise, 0.f );
		// A straight leg bends the way knees do, its foot going back
		Quaternion bend, swing;
		if( !SolveTwoBone( hipWorld.position, kneeWorld.position, ankleWorld.position, target, Vector3( 0.f, 0.f, -1.f ), bend, swing ) )
		{
			continue;
		}

		// World rotations: the knee bends, the leg swings, the foot keeps how it was turned, tilted to the ground
		const Quaternion hipRotation = XMQuaternionNormalize( XMQuaternionMultiply( hipWorld.rotation, swing ) );
		const Quaternion kneeRotation = XMQuaternionNormalize( XMQuaternionMultiply( XMQuaternionMultiply( kneeWorld.rotation, bend ), swing ) );
		const Quaternion ankleRotation = XMQuaternionNormalize( XMQuaternionMultiply( ankleWorld.rotation, tilt ) );
		SetLimb( pose, m_ikWorld[leg.pelvisBone].rotation, leg.hipBone, leg.kneeBone, leg.ankleBone, hipRotation, kneeRotation, ankleRotation );
		m_ikMoved.insert( m_ikMoved.end(), { leg.hipBone, leg.kneeBone, leg.ankleBone } );
	}
}

bool Tr2AnimationRetargeter::HasTargets() const
{
	if( m_lookAtOn && m_lookWeight > 0.f && !m_lookBones.empty() )
	{
		return true;
	}
	if( m_handIkOn )
	{
		for( const Arm& arm : m_arms )
		{
			if( arm.weight > 0.f )
			{
				return true;
			}
		}
	}
	return false;
}

void Tr2AnimationRetargeter::LookAtTarget( const cmf::Skeleton& skeleton, cmf::SkeletonPose& pose )
{
	if( !m_lookAtOn || !m_lookBound || m_lookWeight <= 0.f )
	{
		return;
	}
	ComputeWorld( skeleton, pose.boneTransforms.data(), m_ikWorld );
	const uint32_t head = m_lookBones.back().bone;
	// Where the clips have the head look: the limit is measured from there
	const Vector3 was = XMVector3Rotate( m_lookForward, m_ikWorld[head].rotation );
	float remaining = 0.f;
	for( const LookBone& look : m_lookBones )
	{
		remaining += look.share;
	}

	// Each bone in turn, from the top, takes its share of the turn still to go, aimed from where the head has got to,
	// so the last turns the head exactly onto it
	for( size_t k = 0; k < m_lookBones.size(); ++k )
	{
		const LookBone& look = m_lookBones[k];
		const Vector3 toTarget = m_lookTarget - m_ikWorld[head].position;
		if( Length( toTarget ) < 1e-4f )
		{
			return;
		}
		// At the target, or as far towards it as the limit allows, the weight of the way there
		Quaternion toward;
		TriQuaternionRotationArc( &toward, &was, &toTarget );
		const float angle = AngleOf( toward );
		const float limit = angle > m_lookMaxAngle ? m_lookMaxAngle / angle : 1.f;
		const Vector3 wanted = XMVector3Rotate( was, Scaled( toward, limit * m_lookWeight ) );

		const Vector3 looking = XMVector3Rotate( m_lookForward, m_ikWorld[head].rotation );
		Quaternion turn;
		TriQuaternionRotationArc( &turn, &looking, &wanted );
		const float share = k + 1 == m_lookBones.size() || remaining <= 0.f ? 1.f : std::min( 1.f, look.share / remaining );
		remaining -= look.share;

		const Quaternion rotation = XMQuaternionMultiply( m_ikWorld[look.bone].rotation, Scaled( turn, share ) );
		const Quaternion parent = HasParent( skeleton, look.bone ) ? m_ikWorld[skeleton.parents[look.bone]].rotation : IdentityQuaternion();
		pose.boneTransforms[look.bone].rotation = XMQuaternionNormalize( XMQuaternionMultiply( rotation, XMQuaternionConjugate( parent ) ) );
		m_ikMoved.push_back( look.bone );
		ComputeWorld( skeleton, pose.boneTransforms.data(), m_ikWorld, look.bone );
	}
}

void Tr2AnimationRetargeter::ReachWithHands( const cmf::Skeleton& skeleton, cmf::SkeletonPose& pose )
{
	if( !m_handIkOn )
	{
		return;
	}
	bool worldCurrent = false;
	for( const Arm& arm : m_arms )
	{
		if( arm.shoulderBone == NO_BONE || arm.weight <= 0.f )
		{
			continue;
		}
		if( !worldCurrent )
		{
			ComputeWorld( skeleton, pose.boneTransforms.data(), m_ikWorld );
			worldCurrent = true;
		}
		const cmf::Transform& shoulderWorld = m_ikWorld[arm.shoulderBone];
		const cmf::Transform& elbowWorld = m_ikWorld[arm.elbowBone];
		const cmf::Transform& wristWorld = m_ikWorld[arm.wristBone];
		const Vector3 shoulder = shoulderWorld.position;
		// The wrist goes the weight of the way to the target from where the clips hold it; a straight arm bends the way
		// elbows do, its hand going forward
		const Vector3 target = wristWorld.position + ( arm.target - wristWorld.position ) * arm.weight;
		Quaternion bend, swing;
		if( !SolveTwoBone( shoulder, elbowWorld.position, wristWorld.position, target, Vector3( 0.f, 0.f, 1.f ), bend, swing ) )
		{
			continue;
		}

		// Then roll the arm about the line from the shoulder to the wrist, which leaves the wrist where it is, to bring
		// the elbow round towards the pole
		Quaternion roll = IdentityQuaternion();
		const Vector3 line = target - shoulder;
		if( arm.poleOn && Length( line ) > 1e-5f )
		{
			const Vector3 axis = line / Length( line );
			const Vector3 toElbow = XMVector3Rotate( elbowWorld.position - shoulder, swing );
			const Vector3 toPole = arm.pole - shoulder;
			const Vector3 elbowOff = toElbow - axis * Dot( toElbow, axis );
			const Vector3 poleOff = toPole - axis * Dot( toPole, axis );
			if( Length( elbowOff ) > 1e-5f && Length( poleOff ) > 1e-5f )
			{
				const float angle = atan2f( Dot( Cross( elbowOff, poleOff ), axis ), Dot( elbowOff, poleOff ) );
				roll = XMQuaternionRotationNormal( axis, angle * arm.weight );
			}
		}

		// World rotations: the elbow bends, the arm swings and rolls, the hand keeps how it was turned, or turns the
		// weight of the way to the turn it was given
		const Quaternion turn = XMQuaternionMultiply( swing, roll );
		const Quaternion shoulderRotation = XMQuaternionNormalize( XMQuaternionMultiply( shoulderWorld.rotation, turn ) );
		const Quaternion elbowRotation = XMQuaternionNormalize( XMQuaternionMultiply( XMQuaternionMultiply( elbowWorld.rotation, bend ), turn ) );
		const Quaternion handRotation = arm.rotationOn ? Quaternion( XMQuaternionNormalize( XMQuaternionSlerp( wristWorld.rotation, arm.rotation, arm.weight ) ) ) : wristWorld.rotation;
		const Quaternion parent = HasParent( skeleton, arm.shoulderBone ) ? m_ikWorld[skeleton.parents[arm.shoulderBone]].rotation : IdentityQuaternion();
		SetLimb( pose, parent, arm.shoulderBone, arm.elbowBone, arm.wristBone, shoulderRotation, elbowRotation, handRotation );
		m_ikMoved.insert( m_ikMoved.end(), { arm.shoulderBone, arm.elbowBone, arm.wristBone } );
		// The next arm may hang below this one
		ComputeWorld( skeleton, pose.boneTransforms.data(), m_ikWorld, arm.shoulderBone );
	}
}

void Tr2AnimationRetargeter::BindRagdoll( const cmf::Skeleton& target, const cmf::Skeleton& targetBind, const std::vector<cmf::Transform>& targetBindWorld )
{
	m_ragdollBound = false;
	m_ragdollShares.clear();
	m_ragdollShareForBone.assign( target.bones.size(), -1 );
	if( m_ragdollBones.empty() )
	{
		return;
	}
	// A bone for each part, from part 0 up with none left out: the target is given part by part
	std::sort( m_ragdollBones.begin(), m_ragdollBones.end(), []( const RagdollBone& a, const RagdollBone& b ) { return a.part < b.part; } );
	std::vector<char> partOfBone( target.bones.size(), 0 );
	for( size_t k = 0; k < m_ragdollBones.size(); ++k )
	{
		RagdollBone& part = m_ragdollBones[k];
		part.bone = FindBone( target, part.name );
		if( part.part != (int)k || part.bone == NO_BONE || partOfBone[part.bone] )
		{
			CCP_LOGWARN( "Tr2AnimationRetargeter: the ragdoll's parts need a bone each, in the skeleton and no two the same, with no part left out: part %d (%s)", part.part, part.name.c_str() );
			return;
		}
		partOfBone[part.bone] = 1;
	}

	auto bindIndex = [&]( uint32_t bone ) { return FindBone( targetBind, cmf::ToStdString( target.bones[bone] ) ); };
	for( const auto& chain : m_ragdollChains )
	{
		const uint32_t first = FindBone( target, chain.first );
		const uint32_t last = FindBone( target, chain.second );
		if( first == NO_BONE || last == NO_BONE || !partOfBone[first] || !partOfBone[last] )
		{
			CCP_LOGWARN( "Tr2AnimationRetargeter: the ragdoll's chain %s to %s needs both ends in the skeleton, each a part's bone", chain.first.c_str(), chain.second.c_str() );
			continue;
		}
		// The bones from the first down to the last
		std::vector<uint32_t> path{ last };
		while( path.back() != first && HasParent( target, path.back() ) )
		{
			path.push_back( target.parents[path.back()] );
		}
		if( path.back() != first )
		{
			CCP_LOGWARN( "Tr2AnimationRetargeter: the ragdoll's chain %s to %s: %s is not above %s", chain.first.c_str(), chain.second.c_str(), chain.first.c_str(), chain.second.c_str() );
			continue;
		}
		std::reverse( path.begin(), path.end() );

		// How far down the chain each bone sits in the target bind pose
		std::vector<uint32_t> bind( path.size() );
		std::vector<float> along( path.size(), 0.f );
		bool complete = true;
		for( size_t k = 0; k < path.size(); ++k )
		{
			bind[k] = bindIndex( path[k] );
			complete = complete && bind[k] != NO_BONE;
			if( complete && k > 0 )
			{
				along[k] = along[k - 1] + Length( targetBindWorld[bind[k]].position - targetBindWorld[bind[k - 1]].position );
			}
		}
		if( !complete )
		{
			CCP_LOGWARN( "Tr2AnimationRetargeter: the ragdoll's chain %s to %s has a bone the target bind pose lacks", chain.first.c_str(), chain.second.c_str() );
			continue;
		}

		// Between each two parts' bones, the bones that are no part's take a share of their turns
		size_t from = 0;
		for( size_t k = 1; k < path.size(); ++k )
		{
			if( !partOfBone[path[k]] )
			{
				continue;
			}
			const float span = along[k] - along[from];
			for( size_t j = from + 1; j < k; ++j )
			{
				if( m_ragdollShareForBone[path[j]] >= 0 )
				{
					continue;	// an earlier chain has it
				}
				RagdollShare share;
				share.from = path[from];
				share.to = path[k];
				share.share = span > 1e-6f ? ( along[j] - along[from] ) / span : 0.5f;
				share.bindWorld = targetBindWorld[bind[j]].rotation;
				share.fromBindWorld = targetBindWorld[bind[from]].rotation;
				share.toBindWorld = targetBindWorld[bind[k]].rotation;
				m_ragdollShareForBone[path[j]] = (int32_t)m_ragdollShares.size();
				m_ragdollShares.push_back( share );
			}
			from = k;
		}
	}
	m_ragdollBound = true;
}

bool Tr2AnimationRetargeter::HasRagdoll() const
{
#ifdef TRINITY_HAS_PHYSICS_WORLD
	// One not built, or removed since, has no mode
	return m_ragdollOn && m_ragdollWorld && m_ragdoll != 0 && !m_ragdollBones.empty() && m_ragdollWorld->GetRagdollMode( m_ragdoll ) >= 0;
#else
	return false;
#endif
}

void Tr2AnimationRetargeter::ApplyRagdoll( const cmf::Skeleton& skeleton, cmf::SkeletonPose& pose )
{
#ifdef TRINITY_HAS_PHYSICS_WORLD
	if( !m_ragdollBound || !HasRagdoll() )
	{
		return;
	}
	const size_t partCount = m_ragdollBones.size();
	const size_t boneCount = skeleton.bones.size();

	// The pose the clips and the feet, head and hands have made is where the ragdoll's parts are to be: keyed parts
	// are moved there by the world's next step, so they push what the body pushes and carry its speed when let go
	ComputeWorld( skeleton, pose.boneTransforms.data(), m_ikWorld );
	m_ragdollPositions.resize( partCount );
	m_ragdollRotations.resize( partCount );
	for( const RagdollBone& part : m_ragdollBones )
	{
		const cmf::Transform& bone = m_ikWorld[part.bone];
		const Vector3 offset = XMVector3Rotate( part.offsetPosition, bone.rotation );
		const Vector3 position = ( bone.position + offset ) * m_ragdollFrameScale;
		const Vector3 placed = XMVector3Rotate( position, m_ragdollFrameRotation );
		const Quaternion rotation = XMQuaternionMultiply( part.offsetRotation, bone.rotation );
		m_ragdollPositions[part.part] = m_ragdollFramePosition + placed;
		m_ragdollRotations[part.part] = XMQuaternionNormalize( XMQuaternionMultiply( rotation, m_ragdollFrameRotation ) );
	}
	if( !m_ragdollWorld->SetRagdollTarget( m_ragdoll, m_ragdollPositions, m_ragdollRotations ) )
	{
		if( !m_ragdollWarned )
		{
			CCP_LOGWARN( "Tr2AnimationRetargeter: ragdoll %d has another number of parts than the %u bones it was given; it stays on the clips", m_ragdoll, (unsigned)partCount );
			m_ragdollWarned = true;
		}
		return;
	}

	const float weight = Clamp01( m_ragdollWeight );
	if( weight <= 0.f )
	{
		return;
	}
	const Vector3 rootTargetPosition = m_ragdollPositions[0];
	const Quaternion rootTargetRotation = m_ragdollRotations[0];
	if( !m_ragdollWorld->GetRagdollPose( m_ragdoll, m_ragdollPositions, m_ragdollRotations, m_ragdollDynamic ) || m_ragdollPositions.size() != partCount )
	{
		return;	// the ragdoll in the world is not the one the bones were given for
	}
	if( !m_ragdollDynamic[0] )
	{
		// Powered: the parts the simulation moves hang from a root that is keyed, and so a step behind the pose being
		// made.  They are taken as they hang from it, onto where the pose has the root now, or the body would trail
		// its own legs by a frame
		const Vector3 rootPosition = m_ragdollPositions[0];
		const Quaternion onward = XMQuaternionMultiply( XMQuaternionConjugate( m_ragdollRotations[0] ), rootTargetRotation );
		for( size_t part = 1; part < partCount; ++part )
		{
			if( m_ragdollDynamic[part] )
			{
				const Vector3 from = XMVector3Rotate( m_ragdollPositions[part] - rootPosition, onward );
				m_ragdollPositions[part] = rootTargetPosition + from;
				m_ragdollRotations[part] = XMQuaternionNormalize( XMQuaternionMultiply( m_ragdollRotations[part], onward ) );
			}
		}
	}

	// The turn each part the simulation moves gives its bone, in the model's space; a keyed part is where the clips
	// had it a step ago, and its bone stays on the clips
	const Quaternion frameBack = XMQuaternionConjugate( m_ragdollFrameRotation );
	m_ragdollBoneRotation.resize( boneCount );
	m_ragdollTaken.assign( boneCount, 0 );
	uint32_t rootBone = NO_BONE;
	Vector3 rootPosition( 0.f, 0.f, 0.f );
	for( const RagdollBone& part : m_ragdollBones )
	{
		if( !m_ragdollDynamic[part.part] )
		{
			continue;
		}
		const Quaternion partRotation = XMQuaternionMultiply( m_ragdollRotations[part.part], frameBack );
		const Quaternion rotation = XMQuaternionNormalize( XMQuaternionMultiply( XMQuaternionConjugate( part.offsetRotation ), partRotation ) );
		m_ragdollBoneRotation[part.bone] = rotation;
		m_ragdollTaken[part.bone] = 1;
		if( part.part == 0 )
		{
			// The root part alone says where the body is: every other bone hangs from its parent at its own length
			const Vector3 partPosition = XMVector3Rotate( m_ragdollPositions[0] - m_ragdollFramePosition, frameBack );
			const Vector3 offset = XMVector3Rotate( part.offsetPosition, rotation );
			rootBone = part.bone;
			rootPosition = partPosition / m_ragdollFrameScale - offset;
		}
	}

	// The ragdoll's whole pose, parents first, each bone blended in by the weight as it is made
	const cmf::Transform identity{ Vector3( 0.f, 0.f, 0.f ), IdentityQuaternion(), Vector3( 1.f, 1.f, 1.f ) };
	m_ragdollPose.resize( boneCount );
	for( uint32_t i = 0; i < boneCount; ++i )
	{
		const cmf::Transform& parentWorld = HasParent( skeleton, i ) ? m_ragdollPose[skeleton.parents[i]] : identity;
		cmf::Transform local = pose.boneTransforms[i];
		bool moved = false;
		if( m_ragdollTaken[i] )
		{
			local.rotation = XMQuaternionNormalize( XMQuaternionMultiply( m_ragdollBoneRotation[i], XMQuaternionConjugate( parentWorld.rotation ) ) );
			if( i == rootBone )
			{
				local.position = cmf::TransformPoint( rootPosition, cmf::Inverse( parentWorld ) );
			}
			moved = true;
		}
		else if( m_ragdollShareForBone[i] >= 0 )
		{
			const RagdollShare& share = m_ragdollShares[m_ragdollShareForBone[i]];
			if( m_ragdollTaken[share.from] && m_ragdollTaken[share.to] )
			{
				// The two parts' turns from their bind poses, in the model's space, shared out by where it sits between them
				const Quaternion from = XMQuaternionMultiply( XMQuaternionConjugate( share.fromBindWorld ), m_ragdollBoneRotation[share.from] );
				const Quaternion to = XMQuaternionMultiply( XMQuaternionConjugate( share.toBindWorld ), m_ragdollBoneRotation[share.to] );
				const Quaternion world = XMQuaternionNormalize( XMQuaternionMultiply( share.bindWorld, XMQuaternionSlerp( from, to, share.share ) ) );
				local.rotation = XMQuaternionNormalize( XMQuaternionMultiply( world, XMQuaternionConjugate( parentWorld.rotation ) ) );
				moved = true;
			}
		}
		m_ragdollPose[i] = HasParent( skeleton, i ) ? cmf::Multiply( local, parentWorld ) : local;
		if( moved )
		{
			cmf::Transform& made = pose.boneTransforms[i];
			made.rotation = XMQuaternionNormalize( XMQuaternionSlerp( made.rotation, local.rotation, weight ) );
			made.position = made.position + ( local.position - made.position ) * weight;
			m_ikMoved.push_back( i );
		}
	}
#endif
}

void Tr2AnimationRetargeter::RedriveHelpers( const cmf::Skeleton& skeleton, cmf::SkeletonPose& pose ) const
{
	if( !m_helperDriversOn || m_ikMoved.empty() )
	{
		return;
	}
	for( const HelperDriver& driver : m_helperDrivers )
	{
		if( driver.helperBone == NO_BONE || std::find( m_ikMoved.begin(), m_ikMoved.end(), driver.driverBone ) == m_ikMoved.end() )
		{
			continue;
		}
		// A helper with a moved bone below it would carry that bone away from where it was put
		const bool above = std::any_of( m_ikMoved.begin(), m_ikMoved.end(), [&]( uint32_t moved ) { return IsAncestor( skeleton, driver.helperBone, moved ); } );
		if( !above )
		{
			DriveHelper( driver, pose );
		}
	}
}

void Tr2AnimationRetargeter::FireEvents()
{
	for( Clip& clip : m_clips )
	{
		const bool showing = Clamp01( clip.weight ) > 0.f && IsPlayable( clip );
		// A looping clip's phase as given, unwrapped, so a caller counting phases up past 1 says which way it went
		const float phase = clip.looping ? ( std::isfinite( clip.phase ) ? clip.phase : 0.f ) : Clamp01( clip.phase );
		// A clip that has just come into the pose starts from where it is
		if( showing && clip.eventsShowing && !clip.events.empty() )
		{
			// How far it has played on since the last pose: up to a whole loop forward, or for a looping clip whose
			// phase went down, the short way round, so a wrapped phase coming round the seam goes forward and a step
			// back is not most of a loop forward
			const float step = phase - clip.eventPhase;
			float played = step;
			if( clip.looping )
			{
				played = step > 0.f ? std::min( 1.f, step ) : ( Wrap01( step ) <= 0.5f ? Wrap01( step ) : 0.f );
			}
			if( played > 0.f )
			{
				// In the order it plays past them: from the first after where it was, round over the seam
				const float was = clip.looping ? Wrap01( clip.eventPhase ) : clip.eventPhase;
				const auto next = std::upper_bound( clip.events.begin(), clip.events.end(), was, []( float p, const auto& e ) { return p < e.first; } );
				const size_t first = next - clip.events.begin();
				for( size_t k = 0; k < clip.events.size(); ++k )
				{
					const auto& event = clip.events[( first + k ) % clip.events.size()];
					// One where the clip was comes round again only after a whole loop
					float ahead = event.first - was;
					if( clip.looping )
					{
						ahead = Wrap01( ahead ) > 0.f ? Wrap01( ahead ) : 1.f;
					}
					if( ahead > 0.f && ahead <= played )
					{
						if( m_firedEvents.size() >= MAX_FIRED_EVENTS )
						{
							m_firedEvents.erase( m_firedEvents.begin() );
						}
						m_firedEvents.emplace_back( clip.name, event.second );
					}
				}
			}
		}
		clip.eventsShowing = showing;
		clip.eventPhase = phase;
	}
}

void Tr2AnimationRetargeter::CalibrateGround( const cmf::Skeleton& skeleton, const cmf::SkeletonPose& base )
{
	// Where the sampled pose (an idle) holds the feet is the ground; over each clip the feet come down to it
	std::vector<cmf::Transform> world;
	ComputeWorld( skeleton, base.boneTransforms.data(), world );

	for( Clip& clip : m_clips )
	{
		clip.groundOffset = 0.f;
		if( !IsPlayable( clip ) )
		{
			continue;
		}
		const Rig& rig = m_rigs[clip.rigIndex];
		if( rig.feet.empty() )
		{
			continue;
		}
		float ground = FLT_MAX;
		for( uint32_t foot : rig.feet )
		{
			ground = std::min( ground, world[foot].position.y );
		}
		float lowest = FLT_MAX;
		for( uint32_t k = 0; k < GROUND_SAMPLES; ++k )
		{
			Retarget( clip, (float)k / GROUND_SAMPLES, skeleton, base, m_clipPose );
			for( uint32_t foot : rig.feet )
			{
				lowest = std::min( lowest, m_targetWorld[foot].position.y );
			}
		}
		clip.groundOffset = ground - lowest;
	}
	m_groundCalibrated = true;
}

void Tr2AnimationRetargeter::ModifyPose( const cmf::Skeleton& skeleton, cmf::SkeletonPose& pose )
{
	TRINITY_STATS_ZONE( __FUNCTION__ );

	// The animation asks once per mesh that shares it; the pose made for the first serves the rest
	if( IsLastPose( skeleton, pose ) )
	{
		pose.boneTransforms = m_lastPose.boneTransforms;
		return;
	}
	m_lastInput = pose.boneTransforms;

	Apply( skeleton, pose );

	m_lastSkeleton = &skeleton;
	m_lastPose = pose;
	m_lastPoseCurrent = true;
#ifdef TRINITY_HAS_PHYSICS_WORLD
	m_lastHadRagdoll = HasRagdoll();
	m_lastRagdollVersion = m_ragdollWorld ? m_ragdollWorld->GetRagdollVersion() : -1;
#endif
}

bool Tr2AnimationRetargeter::IsLastPose( const cmf::Skeleton& skeleton, const cmf::SkeletonPose& input ) const
{
	if( !m_lastPoseCurrent || m_needsBind || m_lastSkeleton != &skeleton || m_lastInput.size() != input.boneTransforms.size() )
	{
		return false;
	}
#ifdef TRINITY_HAS_PHYSICS_WORLD
	// A ragdoll comes and goes, and its parts move with the world's steps, whatever the clips do.  Even at weight 0 the
	// pose is made again after a step, as a ragdoll placed or keyed again waits for the target only a pose gives it
	const bool hasRagdoll = HasRagdoll();
	if( hasRagdoll != m_lastHadRagdoll || ( hasRagdoll && m_ragdollWorld->GetRagdollVersion() != m_lastRagdollVersion ) )
	{
		return false;
	}
#endif
	for( size_t i = 0; i < m_lastInput.size(); ++i )
	{
		const cmf::Transform& a = m_lastInput[i];
		const cmf::Transform& b = input.boneTransforms[i];
		if( memcmp( &a.position, &b.position, sizeof( a.position ) ) != 0 || memcmp( &a.rotation, &b.rotation, sizeof( a.rotation ) ) != 0 || memcmp( &a.scale, &b.scale, sizeof( a.scale ) ) != 0 )
		{
			return false;
		}
	}
	return true;
}

bool Tr2AnimationRetargeter::Evaluate()
{
	if( m_animation )
	{
		CCP_LOGWARN( "Tr2AnimationRetargeter: Evaluate is for a retargeter without an animation; its last pose can be read back instead" );
		return false;
	}
	const cmf::Data* bind = GetData( m_targetBindPoseRes );
	if( !bind || !IsReady() )
	{
		return false;
	}
	const cmf::Skeleton& skeleton = bind->skeletons[0];
	if( m_needsBind || m_boundTargetBoneCount != skeleton.bones.size() )
	{
		Bind( skeleton );
	}
	cmf::SkeletonPose pose;
	cmf::RestPose( pose, skeleton );
	ModifyPose( skeleton, pose );
	return m_bound;
}

std::vector<std::string> Tr2AnimationRetargeter::GetBoneNames() const
{
	std::vector<std::string> names;
	if( m_lastSkeleton )
	{
		names.reserve( m_lastSkeleton->bones.size() );
		for( const auto& bone : m_lastSkeleton->bones )
		{
			names.push_back( cmf::ToStdString( bone ) );
		}
	}
	return names;
}

std::vector<std::pair<Vector3, Quaternion>> Tr2AnimationRetargeter::GetLocalTransforms() const
{
	std::vector<std::pair<Vector3, Quaternion>> transforms;
	if( m_lastSkeleton )
	{
		transforms.reserve( m_lastPose.boneTransforms.size() );
		for( const cmf::Transform& t : m_lastPose.boneTransforms )
		{
			transforms.emplace_back( t.position, t.rotation );
		}
	}
	return transforms;
}

std::vector<std::pair<Vector3, Quaternion>> Tr2AnimationRetargeter::GetModelTransforms() const
{
	std::vector<std::pair<Vector3, Quaternion>> transforms;
	if( m_lastSkeleton && m_lastPose.boneTransforms.size() == m_lastSkeleton->bones.size() )
	{
		std::vector<cmf::Transform> world;
		ComputeWorld( *m_lastSkeleton, m_lastPose.boneTransforms.data(), world );
		transforms.reserve( world.size() );
		for( const cmf::Transform& t : world )
		{
			transforms.emplace_back( t.position, t.rotation );
		}
	}
	return transforms;
}

void Tr2AnimationRetargeter::Apply( const cmf::Skeleton& skeleton, cmf::SkeletonPose& pose )
{
	float total = 0.f;
	for( const Clip& clip : m_clips )
	{
		total += Clamp01( clip.weight );
	}
	// Standing in the pose below with no clip showing, the feet may still need standing on the ground, and the head and
	// hands turning to their targets
	const bool posing = total > 0.f || HasGroundToStandOn() || HasTargets() || HasRagdoll();
	if( posing && ( m_needsBind || m_boundTargetBoneCount != skeleton.bones.size() ) )
	{
		Bind( skeleton );
	}
	// Every pose made moves the events on, a clip not showing to start afresh when it next shows
	FireEvents();
	if( !posing || !m_bound || pose.boneTransforms.size() != skeleton.bones.size() )
	{
		return;
	}

	if( !m_groundCalibrated )
	{
		CalibrateGround( skeleton, pose );
	}

	// The layers with clips showing, lowest first
	m_layers.clear();
	for( const Clip& clip : m_clips )
	{
		if( Clamp01( clip.weight ) > 0.f && IsPlayable( clip ) )
		{
			m_layers.push_back( clip.layer );
		}
	}
	std::sort( m_layers.begin(), m_layers.end() );
	m_layers.erase( std::unique( m_layers.begin(), m_layers.end() ), m_layers.end() );

	const size_t boneCount = skeleton.bones.size();
	m_blendWeights.skeleton = &skeleton;
	m_blendWeights.boneWeights.resize( boneCount );
	for( uint32_t layer : m_layers )
	{
		// Per bone, each clip blends in by its share of the weight so far; the clips retarget over the pose below
		m_layerPose = pose;
		m_layerTotals.assign( boneCount, 0.f );
		for( Clip& clip : m_clips )
		{
			const float weight = Clamp01( clip.weight );
			if( clip.layer != layer || weight <= 0.f || !IsPlayable( clip ) )
			{
				continue;
			}
			Retarget( clip, clip.phase, skeleton, pose, m_clipPose );
			for( size_t b = 0; b < boneCount; ++b )
			{
				const float boneWeight = weight * clip.boneWeights[b];
				m_layerTotals[b] += boneWeight;
				m_blendWeights.boneWeights[b] = boneWeight > 0.f ? boneWeight / m_layerTotals[b] : 0.f;
			}
			cmf::BlendPoses( m_layerPose, m_layerPose, m_clipPose, m_blendWeights, 1.f );
		}

		// The layer replaces the pose below by its total weight, up to 1
		for( size_t b = 0; b < boneCount; ++b )
		{
			m_blendWeights.boneWeights[b] = Clamp01( m_layerTotals[b] );
		}
		cmf::BlendPoses( pose, pose, m_layerPose, m_blendWeights, 1.f );
	}

	// The head turns before the hands reach, as turning the spine moves the shoulders they reach from; the ragdoll is
	// given the pose those leave, and takes over as much of it as its weight says; then the helpers of the bones moved
	// follow them
	m_ikMoved.clear();
	StandOnGround( skeleton, pose );
	LookAtTarget( skeleton, pose );
	ReachWithHands( skeleton, pose );
	ApplyRagdoll( skeleton, pose );
	RedriveHelpers( skeleton, pose );
}
