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
 * sampled pose holds them.  The clips play in place: the caller moves the character.  A looping clip's pelvis keeps
 * its sway, side to side and back and forth, about the line it travels along, scaled the same way; without it the
 * body above swings further and the planted feet slide.
 *
 * Clips are added by name with AddClip, each for a rig and on a layer, and driven from outside: each has a weight and
 * a phase, 0 to 1 through the clip, set every frame.  Per bone, the clips of a layer blend by their weights times
 * their bone weights (SetClipBoneWeight), and each layer replaces the pose below it, the sampled pose under layer 0,
 * by its total weight, up to 1.
 *
 * The target bind pose comes from its own file because the animation's skeleton may hold a frame of animation rather
 * than the pose the meshes were bound in.  A rig's bind pose may come from its own file too; left empty, it is the
 * skeleton of the rig's first clip, for clips whose rest pose is the bind pose.
 *
 * A rig's chains (AddChain) share the turn of their paired ends out over the bones between them that the rig does not
 * pair, by length: a three-bone source spine bends all five bones of the target's.
 *
 * Helper drivers (AddHelperDriver) turn helper bones, the twist and corrective joints that skin the body, with the
 * bones that drive them on the character's own rig: a twist by a share of the driver's twist about an axis, and a
 * swing by a share of the rest of its turn.  They act on each clip whose rig does not pair the helper, so clips made on the character's own skeleton
 * keep their helpers' own keys.
 *
 * Legs (AddLeg) stand the feet on the ground under them.  Every frame the caller gives each leg the height of the
 * ground under its foot, in the model's space, relative to the floor the character stands on (SetLegGround); the
 * pelvis drops to let the lower foot reach, each leg bends to put its foot that much higher or lower than the clips
 * hold it, and a planted foot tilts to the ground's slope, planted as the clips' contacts say (SetClipContacts).
 *
 * Each of these stages, and the pelvis's sway, can be turned off to compare with and without.
 *
 * The last pose made can be read back, per bone, relative to its parent or in the model's space.  Evaluate makes one
 * without an animation, over the target bind pose, so tools and tests see exactly what the game would.  The animation
 * asks once per mesh that shares it; the same pose asked for again with nothing changed is not made twice.
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

	bool GetPelvisSway() const;
	void SetPelvisSway( bool on );
	bool GetChainsOn() const;
	void SetChainsOn( bool on );
	bool GetHelperDriversOn() const;
	void SetHelperDriversOn( bool on );
	bool GetFootIkOn() const;
	void SetFootIkOn( bool on );
	float GetIkMaxPelvisDrop() const;
	void SetIkMaxPelvisDrop( float metres );
	float GetIkMaxFootTilt() const;
	void SetIkMaxFootTilt( float degrees );

	bool AddChain( const std::string& rig, const std::string& firstTarget, const std::string& lastTarget );
	void AddHelperDriver( const std::string& helper, const std::string& driver, const Vector3& driverAxis, const Vector3& helperAxis, float twist, float swing );
	void ClearHelperDrivers();
	void AddLeg( const std::string& name, const std::string& hip, const std::string& knee, const std::string& ankle );
	void ClearLegs();
	bool SetLegGround( const std::string& name, float height, const Vector3& normal, float weight );
	bool SetClipContacts( const std::string& clip, const std::string& leg, const std::vector<std::pair<float, float>>& contacts );

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

	bool Evaluate();
	std::vector<std::string> GetBoneNames() const;
	std::vector<std::pair<Vector3, Quaternion>> GetLocalTransforms() const;
	std::vector<std::pair<Vector3, Quaternion>> GetModelTransforms() const;

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

	// A target bone between two paired bones of a chain, which takes a share of their turns
	struct ChainJoint
	{
		int32_t fromLink;	// the paired end above it
		int32_t toLink;	// the paired end below it
		float share;	// how far down from the one to the other it sits, 0 to 1, by length
		Quaternion bindWorld;	// its rotation in the target bind pose, and the ends'
		Quaternion fromBindWorld;
		Quaternion toBindWorld;
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
		std::vector<std::pair<std::string, std::string>> chains;	// first and last target bone of each chain

		bool bound = false;
		std::vector<ChainJoint> chainJoints;
		std::vector<int32_t> chainJointForTargetBone;	// index into chainJoints, -1 for bones no chain shares out to
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
		std::vector<std::pair<std::string, std::vector<std::pair<float, float>>>> contactSettings;	// leg, and when its foot is down
		std::vector<std::vector<std::pair<float, float>>> legContacts;	// per leg, when its foot is down, as phases

		int32_t rigIndex = -1;
		const cmf::Skeleton* skeleton = nullptr;
		std::unique_ptr<cmf::AnimationPlayer> player;
		float duration = 0.f;
		std::vector<uint32_t> sourceBones;	// per link of the rig, the bone in this clip's skeleton
		std::vector<float> boneWeights;	// per target bone
		float groundOffset = 0.f;	// pelvis height change that puts the feet on the ground
		Vector3 travelStart = Vector3( 0.f, 0.f, 0.f );	// where the source pelvis starts, in the source's model space
		Vector3 travel = Vector3( 0.f, 0.f, 0.f );	// how far it goes over the clip
		Vector3 swayCentre = Vector3( 0.f, 0.f, 0.f );	// the middle of its sway about that line, from it
		cmf::SkeletonPose sourcePose;
		std::vector<cmf::Transform> sourceWorld;
	};

	// A helper bone turned with a driving bone, each from its bind pose: it twists about helperAxis by a share of the
	// driver's twist about driverAxis, and swings by a share of the rest of the driver's turn
	struct HelperDriver
	{
		std::string helper;
		std::string driver;
		Vector3 driverAxis;	// in the driver's frame
		Vector3 helperAxis;	// in the helper's frame
		float twist;
		float swing;

		uint32_t helperBone = NO_BONE;
		uint32_t driverBone = NO_BONE;
		Quaternion helperRest;	// local rotations in the target bind pose
		Quaternion driverRest;
		Quaternion toHelper;	// takes a turn in the driver's bind frame into the helper's
	};

	// A leg the feet are stood on the ground with
	struct Leg
	{
		std::string name;
		std::string hip;
		std::string knee;
		std::string ankle;
		float ground = 0.f;	// m the ground under the foot is above the character's floor, in the model's space
		Vector3 normal = Vector3( 0.f, 1.f, 0.f );	// the ground's up there, in the model's space
		float weight = 0.f;	// how much the leg follows the ground, 0 to 1

		uint32_t pelvisBone = NO_BONE;
		uint32_t hipBone = NO_BONE;
		uint32_t kneeBone = NO_BONE;
		uint32_t ankleBone = NO_BONE;
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
	void BindChains( Rig& rig, const cmf::Skeleton& target, const cmf::Skeleton& targetBind, const std::vector<cmf::Transform>& targetBindWorld );
	void BindHelpersAndLegs( const cmf::Skeleton& target, const cmf::Skeleton& targetBind, const std::vector<cmf::Transform>& targetBindWorld );
	void DriveHelpers( const Rig& rig, cmf::SkeletonPose& out ) const;
	void StandOnGround( const cmf::Skeleton& skeleton, cmf::SkeletonPose& pose );
	float Planted( uint32_t leg ) const;
	bool HasGroundToStandOn() const;
	void Unbind();
	bool IsPlayable( const Clip& clip ) const;
	void SampleSource( Clip& clip, float time );
	void MeasureTravel( Clip& clip, const Rig& rig );
	void Retarget( Clip& clip, float phase, const cmf::Skeleton& skeleton, const cmf::SkeletonPose& base, cmf::SkeletonPose& out );
	void CalibrateGround( const cmf::Skeleton& skeleton, const cmf::SkeletonPose& base );
	void Apply( const cmf::Skeleton& skeleton, cmf::SkeletonPose& pose );
	bool IsLastPose( const cmf::Skeleton& skeleton, const cmf::SkeletonPose& input ) const;

	Tr2GrannyAnimationPtr m_animation;

	std::vector<Rig> m_rigs;
	std::vector<Clip> m_clips;
	std::string m_targetBindPosePath;
	TriGrannyResPtr m_targetBindPoseRes;
	bool m_pelvisSway = true;	// looping clips' pelvises sway about their line of travel
	bool m_chainsOn = true;
	bool m_helperDriversOn = true;
	bool m_footIkOn = true;
	float m_ikMaxPelvisDrop = 0.3f;	// m
	float m_ikMaxFootTilt = 30.f;	// degrees
	std::vector<HelperDriver> m_helperDrivers;
	std::vector<Leg> m_legs;
	bool m_legsBound = false;
	std::vector<Quaternion> m_linkWorld;	// per link of the rig being retargeted, the world rotation its target bone takes
	std::vector<cmf::Transform> m_ikWorld;

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

	// The last pose made, what it was made from, and whether a setting has changed since
	const cmf::Skeleton* m_lastSkeleton = nullptr;
	std::vector<cmf::Transform> m_lastInput;
	cmf::SkeletonPose m_lastPose;
	bool m_lastPoseCurrent = false;
};

TYPEDEF_BLUECLASS( Tr2AnimationRetargeter );

#endif
