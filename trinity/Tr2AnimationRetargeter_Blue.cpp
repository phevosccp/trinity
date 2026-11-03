// Copyright © 2026 CCP ehf.

#include "StdAfx.h"
#include "Tr2AnimationRetargeter.h"
#include "Tr2GrannyAnimation.h"

BLUE_DEFINE( Tr2AnimationRetargeter );

const Be::ClassInfo* Tr2AnimationRetargeter::ExposeToBlue()
{
	EXPOSURE_BEGIN( Tr2AnimationRetargeter, "Plays clips authored for other skeletons on top of what a Tr2GrannyAnimation plays, pairing bones by name" )
		MAP_INTERFACE( Tr2AnimationRetargeter )

		MAP_PROPERTY(
			"animation",
			GetAnimation,
			SetAnimation,
			"The Tr2GrannyAnimation whose pose is modified.  Setting it takes the animation's single pose modifier slot.\n"
			"The animation does not keep the retargeter alive, so hold a reference to it for as long as it should play." )

		MAP_PROPERTY( "targetBindPosePath", GetTargetBindPosePath, SetTargetBindPosePath, "CMF file whose skeleton holds the target rig's bind pose" )

		MAP_PROPERTY_READONLY( "isReady", IsReady, "True once every clip and the bind pose files have loaded" )

		MAP_METHOD_AND_WRAP(
			"AddRig",
			AddRig,
			"AddRig( name, bindPosePath, rootBone )\n\n"
			"Adds a skeleton clips were made for, or replaces the one with that name.\n"
			":param name: name for clips and bone pairs to refer to it by\n"
			":param bindPosePath: CMF file whose skeleton holds this rig's bind pose; empty uses the skeleton of its first clip\n"
			":param rootBone: this rig's pelvis, which takes the clips' up and down bob" )

		MAP_METHOD_AND_WRAP(
			"MapBone",
			MapBone,
			"MapBone( rig, sourceBone, targetBone )\n\n"
			"Pairs a bone of a rig with a bone of the target skeleton; the target bone takes the source's world direction.\n"
			"Each paired bone's first paired child, in the order bones are paired, sets the direction used to line up the\n"
			"bind poses.  Returns False if there is no such rig.\n"
			":param rig: rig name\n"
			":param sourceBone: bone name in the rig's skeleton\n"
			":param targetBone: bone name in the animation's skeleton" )

		MAP_METHOD_AND_WRAP(
			"MapBoneFromBind",
			MapBoneFromBind,
			"MapBoneFromBind( rig, sourceBone, targetBone )\n\n"
			"Pairs two bones that both skeletons hold the same way in their bind poses (feet standing flat): the target bone\n"
			"turns from its own bind pose as the source turns from its, so each keeps its own shape.  Returns False if there\n"
			"is no such rig.\n"
			":param rig: rig name\n"
			":param sourceBone: bone name in the rig's skeleton\n"
			":param targetBone: bone name in the animation's skeleton" )

		MAP_METHOD_AND_WRAP(
			"MapSameBones",
			MapSameBones,
			"MapSameBones( rig )\n\n"
			"Pairs every bone of the animation's skeleton with the rig's bone of the same name, from the bind pose: for clips\n"
			"made on the animation's own skeleton.  Returns False if there is no such rig.\n"
			":param rig: rig name" )

		MAP_METHOD_AND_WRAP( "ClearRigs", ClearRigs, "Removes all rigs and their bone pairs" )

		MAP_METHOD_AND_WRAP(
			"AddClip",
			AddClip,
			"AddClip( name, path, looping, rig, layer )\n\n"
			"Adds a clip, or replaces the one with that name.  It plays once given a weight.\n"
			":param name: name to drive the clip by\n"
			":param path: CMF file holding the clip; its first animation is played\n"
			":param looping: the phase wraps round; otherwise it stops at the end of the clip\n"
			":param rig: name of the rig the clip was made for\n"
			":param layer: clips on a layer blend together; each layer lies over the ones below it, layer 0 over the sampled pose" )

		MAP_METHOD_AND_WRAP( "ClearClips", ClearClips, "Removes all clips" )

		MAP_METHOD_AND_WRAP(
			"SetClipWeight",
			SetClipWeight,
			"SetClipWeight( name, weight )\n\n"
			"Sets how much a clip counts, 0 to 1.  Returns False if there is no such clip.\n"
			":param name: clip name\n"
			":param weight: clip weight" )

		MAP_METHOD_AND_WRAP(
			"SetClipPhase",
			SetClipPhase,
			"SetClipPhase( name, phase )\n\n"
			"Sets where a clip is, 0 at its start and 1 at its end; set it every frame to drive the clip.  Returns False if\n"
			"there is no such clip.\n"
			":param name: clip name\n"
			":param phase: position in the clip" )

		MAP_METHOD_AND_WRAP(
			"SetClipBoneWeight",
			SetClipBoneWeight,
			"SetClipBoneWeight( name, bone, weight )\n\n"
			"Sets how much of a clip a bone and everything below it take, 0 to 1 (1 until set): a bone set deeper down\n"
			"keeps its own setting.  Returns False if there is no such clip.\n"
			":param name: clip name\n"
			":param bone: bone name in the animation's skeleton\n"
			":param weight: bone weight" )

		MAP_METHOD_AND_WRAP(
			"SetClipVerticalMotion",
			SetClipVerticalMotion,
			"SetClipVerticalMotion( name, on )\n\n"
			"Sets whether the pelvis follows the clip up and down (on until set); off for a clip like a jump whose height\n"
			"comes from movement.  Returns False if there is no such clip.\n"
			":param name: clip name\n"
			":param on: the pelvis follows the clip's vertical motion" )

		MAP_METHOD_AND_WRAP(
			"Evaluate",
			Evaluate,
			"Evaluate()\n\n"
			"Makes a pose without an animation: the clips at their weights and phases over the target bind pose, as the\n"
			"game would make it over the animation's pose.  Read it back with GetLocalTransforms or GetModelTransforms.\n"
			"Returns False if the retargeter has an animation, a file is still loading, or no rig could be bound." )

		MAP_METHOD_AND_WRAP( "GetBoneNames", GetBoneNames, "The bone names of the last pose made, in the order its transforms come in; empty before one is made" )

		MAP_METHOD_AND_WRAP(
			"GetLocalTransforms",
			GetLocalTransforms,
			"The last pose made, per bone a (position, rotation) relative to its parent; empty before one is made.  With an\n"
			"animation, this is the pose this retargeter handed it, before the animation's own bone offsets." )

		MAP_METHOD_AND_WRAP(
			"GetModelTransforms",
			GetModelTransforms,
			"The last pose made, per bone a (position, rotation) in the model's space; empty before one is made.  With an\n"
			"animation, this is the pose this retargeter handed it, before the animation's own bone offsets." )

	EXPOSURE_END()
}
