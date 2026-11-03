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
	}
	for( Rig& rig : m_rigs )
	{
		rig.bound = false;
		rig.linkSources.clear();
		rig.offsets.clear();
		rig.linkForTargetBone.clear();
		rig.feet.clear();
		rig.pelvisTarget = NO_BONE;
	}
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

bool Tr2AnimationRetargeter::IsPlayable( const Clip& clip ) const
{
	return clip.rigIndex >= 0 && m_rigs[clip.rigIndex].bound && clip.player;
}

void Tr2AnimationRetargeter::Retarget( Clip& clip, float phase, const cmf::Skeleton& skeleton, const cmf::SkeletonPose& base, cmf::SkeletonPose& out )
{
	const Rig& rig = m_rigs[clip.rigIndex];

	// Source pose and its world transforms; a clip that does not loop stops at its end
	cmf::RestPose( clip.sourcePose, *clip.skeleton );
	clip.player->SampleAtLocalTime( clip.sourcePose, ( clip.looping ? Wrap01( phase ) : Clamp01( phase ) ) * clip.duration );
	ComputeWorld( *clip.skeleton, clip.sourcePose.boneTransforms.data(), clip.sourceWorld );

	// Retargeted pose: unpaired bones keep the pose below
	out.skeleton = base.skeleton;
	out.boneTransforms = base.boneTransforms;
	m_targetWorld.resize( skeleton.bones.size() );

	const cmf::Transform identity{ Vector3( 0.f, 0.f, 0.f ), IdentityQuaternion(), Vector3( 1.f, 1.f, 1.f ) };
	for( uint32_t i = 0; i < skeleton.bones.size(); ++i )
	{
		const cmf::Transform& parentWorld = HasParent( skeleton, i ) ? m_targetWorld[skeleton.parents[i]] : identity;
		cmf::Transform& local = out.boneTransforms[i];

		const int32_t link = rig.linkForTargetBone[i];
		if( link >= 0 )
		{
			const cmf::Transform& source = clip.sourceWorld[clip.sourceBones[link]];
			const Quaternion world = XMQuaternionNormalize( XMQuaternionMultiply( rig.offsets[link], source.rotation ) );
			local.rotation = XMQuaternionNormalize( XMQuaternionMultiply( world, XMQuaternionConjugate( parentWorld.rotation ) ) );

			if( i == rig.pelvisTarget )
			{
				// Only the bob, if any: the clip plays in place and the caller moves the character
				Vector3 wanted = rig.pelvisTargetBindPosition;
				if( clip.verticalMotion )
				{
					wanted.y += ( source.position.y - rig.pelvisSourceBindHeight ) * rig.pelvisHeightScale;
				}
				wanted.y += clip.groundOffset;
				local.position = cmf::TransformPoint( wanted, cmf::Inverse( parentWorld ) );
			}
		}

		m_targetWorld[i] = HasParent( skeleton, i ) ? cmf::Multiply( local, parentWorld ) : local;
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
	if( total <= 0.f )
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
}
