// Copyright © 2026 CCP ehf.

#pragma once
#ifndef Tr2AnimationRetargeter_h
#define Tr2AnimationRetargeter_h

#include "Include/ITr2PoseModifier.h"

BLUE_DECLARE( TriGrannyRes );
BLUE_DECLARE( Tr2GrannyAnimation );
BLUE_DECLARE( Tr2AnimationRetargeter );

/**
 * @class Tr2AnimationRetargeter
 * @brief Plays clips authored for other skeletons on top of whatever a Tr2GrannyAnimation already plays.
 *
 * Each skeleton clips were made for is a rig, added by name with AddRig, with its own bind pose, its pelvis (the root
 * bone) and its bone pairs.  Bones are paired by name with MapBone.  Each paired target bone takes the world-space
 * rotation its source bone has moved through since the rig's bind pose, after a fixed correction that lines up the
 * bone directions of the two bind poses.  Bones paired with MapBoneFromBind skip that correction: both skeletons hold
 * them the same way in their bind poses (feet standing flat), so each keeps its own shape.  MapSameBones pairs every
 * bone of the same name that way, for clips made on the animation's own skeleton.  The rig's pelvis also takes the
 * source's up and down bob, scaled by the ratio of the two pelvis heights above the feet (the paired bones lowest in
 * each bind pose), then raised or lowered so that over each clip the target's feet come down exactly to where the
 * sampled pose holds them; the clips play in place.
 *
 * Clips are added by name with AddClip, each for a rig and on a layer, and driven from outside: each has a weight and
 * a phase, 0 to 1 through the clip, set every frame.  Per bone, the clips of a layer blend by their weights times
 * their bone weights (SetClipBoneWeight), and each layer replaces the pose below it, the sampled pose under layer 0,
 * by its total weight, up to 1.
 *
 * The target bind pose comes from its own file because the animation's skeleton may hold a frame of animation rather
 * than the pose the meshes were bound in.  A rig's bind pose may come from its own file too; left empty, it is the
 * skeleton of the rig's first clip, for clips whose rest pose is the bind pose.
 */
BLUE_CLASS( Tr2AnimationRetargeter ) :
	public IRoot,
	public ITr2PoseModifier,
	public IBlueAsyncResNotifyTarget
{
public:
	EXPOSE_TO_BLUE();

	Tr2AnimationRetargeter( IRoot* lockobj = NULL );
	~Tr2AnimationRetargeter();

	Tr2GrannyAnimation* GetAnimation() const;
	void SetAnimation( Tr2GrannyAnimation* animation );

	const std::string& GetTargetBindPosePath() const;
	void SetTargetBindPosePath( const std::string& path );

	void AddRig( const std::string& name, const std::string& bindPosePath, const std::string& rootBone );
	bool MapBone( const std::string& rig, const std::string& sourceBone, const std::string& targetBone );
	bool MapBoneFromBind( const std::string& rig, const std::string& sourceBone, const std::string& targetBone );
	bool MapSameBones( const std::string& rig );
	void ClearRigs();

	void AddClip( const std::string& name, const std::string& path, bool looping, const std::string& rig, uint32_t layer );
	void ClearClips();
	bool SetClipWeight( const std::string& name, float weight );
	bool SetClipPhase( const std::string& name, float phase );
	bool SetClipBoneWeight( const std::string& name, const std::string& bone, float weight );
	bool SetClipVerticalMotion( const std::string& name, bool on );

	bool IsReady() const;

	// ITr2PoseModifier
	void ModifyPose( const cmf::Skeleton& skeleton, cmf::SkeletonPose& pose ) override;

	// IBlueAsyncResNotifyTarget
	void ReleaseCachedData( BlueAsyncRes* res ) override;
	void RebuildCachedData( BlueAsyncRes* res ) override;

private:
	static constexpr uint32_t NO_BONE = 0xFFFFFFFF;	// a bone index that names no bone

	struct BoneMapping
	{
		std::string source;
		std::string target;
		bool fromBind;
	};

	// A skeleton clips were made for: its bind pose, pelvis and bone pairs, and what is built from them while bound
	struct Rig
	{
		std::string name;
		std::string bindPosePath;
		std::string rootBone;	// the pelvis, in the rig's skeleton
		TriGrannyResPtr bindPoseRes;
		std::vector<BoneMapping> boneMap;
		bool sameBones = false;	// also pair every bone of the animation's skeleton with the rig's of the same name

		bool bound = false;
		std::vector<std::string> linkSources;	// per link, the source bone, found in each clip's skeleton
		std::vector<Quaternion> offsets;	// per link: takes the source bone's animated world rotation to the target bone's
		std::vector<int32_t> linkForTargetBone;	// index into the links, -1 for bones this rig leaves alone
		std::vector<uint32_t> feet;	// target bones that touch the ground
		uint32_t pelvisTarget = NO_BONE;
		uint32_t pelvisLink = 0;
		float pelvisSourceBindHeight = 0.f;
		Vector3 pelvisTargetBindPosition = Vector3( 0.f, 0.f, 0.f );
		float pelvisHeightScale = 1.f;
	};

	// A clip of a rig: its file and how it plays, and what is built from it while bound
	struct Clip
	{
		std::string name;
		std::string path;
		std::string rig;
		uint32_t layer = 0;
		TriGrannyResPtr res;
		bool looping = true;	// the phase wraps round; otherwise it stops at the end
		bool verticalMotion = true;	// the pelvis follows the clip up and down
		float weight = 0.f;
		float phase = 0.f;	// 0 to 1 through the clip
		std::vector<std::pair<std::string, float>> boneWeightSettings;	// target bone, and how much it and everything below take

		int32_t rigIndex = -1;
		const cmf::Skeleton* skeleton = nullptr;
		std::unique_ptr<cmf::AnimationPlayer> player;
		float duration = 0.f;
		std::vector<uint32_t> sourceBones;	// per link of the rig, the bone in this clip's skeleton
		std::vector<float> boneWeights;	// per target bone
		float groundOffset = 0.f;	// pelvis height change that puts the feet on the ground
		cmf::SkeletonPose sourcePose;
		std::vector<cmf::Transform> sourceWorld;
	};

	static uint32_t FindBone( const cmf::Skeleton& skeleton, const std::string& name );

	Rig* FindRig( const std::string& name );
	int32_t FindRigIndex( const std::string& name ) const;
	Clip* FindClip( const std::string& name );
	void LoadRes( const std::string& path, TriGrannyResPtr& res );
	void ReleaseRes( TriGrannyResPtr& res );
	uint32_t CountSlotsHolding( const TriGrannyRes* res ) const;
	bool Bind( const cmf::Skeleton& target );
	bool BindRig( uint32_t rigIndex, const cmf::Skeleton& target, const cmf::Skeleton& targetBind, const std::vector<cmf::Transform>& targetBindWorld );
	void Unbind();
	bool IsPlayable( const Clip& clip ) const;
	void Retarget( Clip& clip, float phase, const cmf::Skeleton& skeleton, const cmf::SkeletonPose& base, cmf::SkeletonPose& out );
	void CalibrateGround( const cmf::Skeleton& skeleton, const cmf::SkeletonPose& base );

	Tr2GrannyAnimationPtr m_animation;

	std::vector<Rig> m_rigs;
	std::vector<Clip> m_clips;
	std::string m_targetBindPosePath;
	TriGrannyResPtr m_targetBindPoseRes;

	// Bound state, rebuilt when a file, a rig, a clip or the animation's skeleton changes
	bool m_needsBind = true;
	bool m_groundCalibrated = false;
	bool m_bound = false;
	size_t m_boundTargetBoneCount = 0;	// bone count of the skeleton last bound to, whether or not binding succeeded

	std::vector<uint32_t> m_layers;	// the layers with clips showing this frame, lowest first
	std::vector<float> m_layerTotals;	// per bone, the weight of the layer being blended
	cmf::BoneWeights m_blendWeights;
	cmf::SkeletonPose m_clipPose;
	cmf::SkeletonPose m_layerPose;
	std::vector<cmf::Transform> m_targetWorld;
};

TYPEDEF_BLUECLASS( Tr2AnimationRetargeter );

#endif
