// Copyright © 2026 CCP ehf.

#include "StdAfx.h"
#include "Tr2AnimationRetargeter.h"
#include "Tr2GrannyAnimation.h"
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

// Parents precede children, so a real parent always has the smaller index
bool HasParent( const cmf::Skeleton& skeleton, uint32_t bone )
{
	return skeleton.parents[bone] < bone;
}

void ComputeWorld( const cmf::Skeleton& skeleton, const cmf::Transform* locals, std::vector<cmf::Transform>& world )
{
	world.resize( skeleton.bones.size() );
	for( uint32_t i = 0; i < world.size(); ++i )
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
	auto existing = std::find_if( m_legs.begin(), m_legs.end(), [&]( const Leg& leg ) { return leg.name == name; } );
	Leg& leg = existing != m_legs.end() ? *existing : m_legs.emplace_back();
	leg.name = name;
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
		rig.offsets.push_back( offset );
		rig.linkSources.push_back( *resolved[i].sourceName );
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
		const Vector3 hip = hipWorld.position;
		const Vector3 knee = kneeWorld.position;
		const Vector3 ankle = ankleWorld.position;
		const Vector3 target = ankle + Vector3( 0.f, raise, 0.f );
		const float thigh = Length( knee - hip );
		const float shin = Length( ankle - knee );
		if( thigh < 1e-4f || shin < 1e-4f )
		{
			continue;
		}
		const float reach = std::max( fabsf( thigh - shin ) + 1e-4f, std::min( thigh + shin - 1e-4f, Length( target - hip ) ) );

		// Bend the knee to the angle that puts the ankle that far from the hip, in the plane the leg bends in now (a
		// straight leg bends the way knees do, its foot going back)
		const Vector3 toAnkle = ankle - knee;
		const Vector3 alongThigh = ( hip - knee ) / thigh;
		Vector3 across = toAnkle - alongThigh * Dot( toAnkle, alongThigh );
		if( Length( across ) < 1e-5f )
		{
			const Vector3 back( 0.f, 0.f, -1.f );
			across = back - alongThigh * Dot( back, alongThigh );
		}
		if( Length( across ) < 1e-5f )
		{
			continue;
		}
		across = across / Length( across );
		const float cosKnee = std::max( -1.f, std::min( 1.f, ( thigh * thigh + shin * shin - reach * reach ) / ( 2.f * thigh * shin ) ) );
		const float kneeAngle = acosf( cosKnee );
		const Vector3 bentToAnkle = ( alongThigh * cosf( kneeAngle ) + across * sinf( kneeAngle ) ) * shin;
		Quaternion bend;
		TriQuaternionRotationArc( &bend, &toAnkle, &bentToAnkle );

		// Then swing the leg about the hip so the ankle lands on the target
		const Vector3 bentFromHip = knee + bentToAnkle - hip;
		const Vector3 targetFromHip = target - hip;
		Quaternion swing;
		TriQuaternionRotationArc( &swing, &bentFromHip, &targetFromHip );

		// World rotations: the knee bends, the leg swings, the foot keeps how it was turned, tilted to the ground
		const Quaternion hipRotation = XMQuaternionNormalize( XMQuaternionMultiply( hipWorld.rotation, swing ) );
		const Quaternion kneeRotation = XMQuaternionNormalize( XMQuaternionMultiply( XMQuaternionMultiply( kneeWorld.rotation, bend ), swing ) );
		const Quaternion ankleRotation = XMQuaternionNormalize( XMQuaternionMultiply( ankleWorld.rotation, tilt ) );
		const Quaternion& pelvisRotation = m_ikWorld[leg.pelvisBone].rotation;
		pose.boneTransforms[leg.hipBone].rotation = XMQuaternionNormalize( XMQuaternionMultiply( hipRotation, XMQuaternionConjugate( pelvisRotation ) ) );
		pose.boneTransforms[leg.kneeBone].rotation = XMQuaternionNormalize( XMQuaternionMultiply( kneeRotation, XMQuaternionConjugate( hipRotation ) ) );
		pose.boneTransforms[leg.ankleBone].rotation = XMQuaternionNormalize( XMQuaternionMultiply( ankleRotation, XMQuaternionConjugate( kneeRotation ) ) );
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
}

bool Tr2AnimationRetargeter::IsLastPose( const cmf::Skeleton& skeleton, const cmf::SkeletonPose& input ) const
{
	if( !m_lastPoseCurrent || m_needsBind || m_lastSkeleton != &skeleton || m_lastInput.size() != input.boneTransforms.size() )
	{
		return false;
	}
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
	// Standing in the pose below with no clip showing, the feet may still need standing on the ground
	if( total <= 0.f && !HasGroundToStandOn() )
	{
		return;
	}
	if( m_needsBind || m_boundTargetBoneCount != skeleton.bones.size() )
	{
		Bind( skeleton );
	}
	if( !m_bound || pose.boneTransforms.size() != skeleton.bones.size() )
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

	StandOnGround( skeleton, pose );
}
