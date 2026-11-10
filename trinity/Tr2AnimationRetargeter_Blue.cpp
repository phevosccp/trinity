// Copyright © 2026 CCP ehf.

#include "StdAfx.h"
#include "Tr2AnimationRetargeter.h"
#include "Tr2GrannyAnimation.h"
#include "Tr2PhysicsWorld.h"

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

		MAP_PROPERTY(
			"pelvisSway",
			GetPelvisSway,
			SetPelvisSway,
			"Whether a looping clip's pelvis keeps its sway, side to side and back and forth, about the line it travels\n"
			"along (on until set).  Off pins it over the feet, as the retargeter did before." )

		MAP_PROPERTY( "chainsOn", GetChainsOn, SetChainsOn, "Whether chains share their ends' turns out over the bones between them (on until set)" )
		MAP_PROPERTY( "helperDriversOn", GetHelperDriversOn, SetHelperDriversOn, "Whether helper drivers turn their helper bones (on until set)" )
		MAP_PROPERTY( "footIkOn", GetFootIkOn, SetFootIkOn, "Whether legs stand the feet on the ground the caller gives (on until set)" )
		MAP_PROPERTY( "ikMaxPelvisDrop", GetIkMaxPelvisDrop, SetIkMaxPelvisDrop, "m the pelvis may drop for a foot to reach lower ground (0.3 until set)" )
		MAP_PROPERTY( "ikMaxFootTilt", GetIkMaxFootTilt, SetIkMaxFootTilt, "Degrees a planted foot may tilt to the ground's slope (30 until set)" )
		MAP_PROPERTY( "handIkOn", GetHandIkOn, SetHandIkOn, "Whether arms put the hands on their targets (on until set)" )
		MAP_PROPERTY( "lookAtOn", GetLookAtOn, SetLookAtOn, "Whether the look chain turns the head to its target (on until set)" )

		MAP_METHOD_AND_WRAP(
			"AddChain",
			AddChain,
			"AddChain( rig, firstTarget, lastTarget )\n\n"
			"Shares the turns of a chain's paired bones out over the bones between them the rig does not pair, by how far\n"
			"down the chain each sits: a three-bone source spine bends all of a five-bone target's.  Both ends must be paired\n"
			"and the first above the last.  Returns False if there is no such rig.\n"
			":param rig: rig name\n"
			":param firstTarget: the chain's top bone, in the animation's skeleton\n"
			":param lastTarget: its bottom bone" )

		MAP_METHOD_AND_WRAP(
			"AddHelperDriver",
			AddHelperDriver,
			"AddHelperDriver( helper, driver, driverAxis, helperAxis, twist, swing )\n\n"
			"Turns a helper bone (a twist or corrective joint that skins the body) with a driving bone, in each clip whose rig\n"
			"does not pair the helper, each from its bind pose: it twists about helperAxis by twist times the driver's twist\n"
			"about driverAxis, and swings by swing times the rest of the driver's turn, seen from the helper's bind frame.\n"
			":param helper: helper bone name\n"
			":param driver: driving bone name\n"
			":param driverAxis: (x, y, z) in the driver's frame, which its twist is taken about\n"
			":param helperAxis: (x, y, z) in the helper's frame, which it twists about; (1, 0, 0) along most rigs' bones\n"
			":param twist: share of the driver's twist, negative to turn against it\n"
			":param swing: share of the rest of the driver's turn" )

		MAP_METHOD_AND_WRAP( "ClearHelperDrivers", ClearHelperDrivers, "Removes all helper drivers" )

		MAP_METHOD_AND_WRAP(
			"AddLeg",
			AddLeg,
			"AddLeg( name, hip, knee, ankle )\n\n"
			"Adds a leg the feet are stood on the ground with, or replaces the one with that name: each bone the parent of the\n"
			"next, the hip's parent the pelvis.\n"
			":param name: name to give its ground and clips' contacts by\n"
			":param hip: thigh bone name\n"
			":param knee: shin bone name\n"
			":param ankle: foot bone name" )

		MAP_METHOD_AND_WRAP( "ClearLegs", ClearLegs, "Removes all legs" )

		MAP_METHOD_AND_WRAP(
			"SetLegGround",
			SetLegGround,
			"SetLegGround( name, height, normal, weight )\n\n"
			"Gives the ground under a leg's foot, every frame: the pelvis drops for the lower foot and each leg bends to put\n"
			"its foot height higher (lower if negative) than the clips hold it; while planted the foot tilts to normal.\n"
			"Returns False if there is no such leg.\n"
			":param name: leg name\n"
			":param height: m the ground under the foot is above the character's floor, in the model's space\n"
			":param normal: (x, y, z), the ground's up there, in the model's space\n"
			":param weight: how much the leg follows the ground, 0 to 1: 0 in the air" )

		MAP_METHOD_AND_WRAP(
			"SetClipContacts",
			SetClipContacts,
			"SetClipContacts( name, leg, contacts )\n\n"
			"Says when in a clip a leg's foot is down, as [(start, end), ...] phases of the whole clip; an end past 1 runs over\n"
			"the loop's seam.  A planted foot tilts to the ground.  Returns False if there is no such clip.\n"
			":param name: clip name\n"
			":param leg: leg name\n"
			":param contacts: list of (start, end) phases" )

		MAP_METHOD_AND_WRAP(
			"AddArm",
			AddArm,
			"AddArm( name, shoulder, elbow, wrist )\n\n"
			"Adds an arm whose hand can be put on a point, or replaces the one with that name: each bone the parent of the\n"
			"next.\n"
			":param name: name to give its target by\n"
			":param shoulder: upper arm bone name\n"
			":param elbow: forearm bone name\n"
			":param wrist: hand bone name" )

		MAP_METHOD_AND_WRAP( "ClearArms", ClearArms, "Removes all arms" )

		MAP_METHOD_AND_WRAP(
			"SetHandTarget",
			SetHandTarget,
			"SetHandTarget( name, position, weight )\n\n"
			"Gives an arm's hand a point to reach, every frame: the elbow bends and the shoulder turns for the wrist to go\n"
			"weight of the way there from where the clips hold it, or as far along the line to it as the arm reaches.  The\n"
			"hand keeps how the clips turn it, and helper drivers turn the arm's helpers with it.  Hands reach after the\n"
			"feet stand and the head turns.  Returns False if there is no such arm.\n"
			":param name: arm name\n"
			":param position: (x, y, z) for the wrist, in the model's space\n"
			":param weight: how far it goes, 0 to 1: 0 leaves the arm as the clips hold it" )

		MAP_METHOD_AND_WRAP(
			"SetHandPole",
			SetHandPole,
			"SetHandPole( name, pole, on )\n\n"
			"Gives an arm a point its elbow bends towards: the arm rolls about the line from the shoulder to the wrist to\n"
			"bring the elbow round to the pole's side.  Off, the elbow bends in the plane the clips bend it in.  Returns False\n"
			"if there is no such arm.\n"
			":param name: arm name\n"
			":param pole: (x, y, z), in the model's space\n"
			":param on: whether the elbow bends towards it" )

		MAP_METHOD_AND_WRAP(
			"SetHandRotation",
			SetHandRotation,
			"SetHandRotation( name, rotation, on )\n\n"
			"Gives an arm's hand a turn, every frame, in the model's space: the hand turns the arm's weight (SetHandTarget)\n"
			"of the way there from how the clips turn it, as the wrist goes to its target, so a palm lies on what it holds.\n"
			"Off, the hand keeps the clips' turn.  A turn of no length turns nothing.  Returns False if there is no such arm.\n"
			":param name: arm name\n"
			":param rotation: (x, y, z, w), the hand bone's turn in the model's space\n"
			":param on: whether the hand takes it" )

		MAP_METHOD_AND_WRAP(
			"AddLookBone",
			AddLookBone,
			"AddLookBone( bone, share )\n\n"
			"Adds a bone to the bottom of the chain that turns the head to look, or sets the share of one in it already.\n"
			"Each bone must be above the next; the last is the head, whose forward is where the model's +Z points in the\n"
			"target bind pose.  The bones turn in order from the top, each by its share of the turn still to go, so shares\n"
			"that sum to 1 are each bone's part of the whole; the head takes whatever is left.\n"
			":param bone: bone name\n"
			":param share: its part of the turn, 0 or more" )

		MAP_METHOD_AND_WRAP( "ClearLookBones", ClearLookBones, "Removes all look bones" )

		MAP_METHOD_AND_WRAP(
			"SetLookTarget",
			SetLookTarget,
			"SetLookTarget( position, weight, maxDegrees )\n\n"
			"Gives the head a point to look at, every frame: the look bones turn the head's forward towards it, up to\n"
			"maxDegrees from where the clips have it look, weight of the way.  Returns False if there are no look bones.\n"
			":param position: (x, y, z), in the model's space\n"
			":param weight: how far it turns, 0 to 1: 0 leaves the head as the clips hold it\n"
			":param maxDegrees: the most the head turns, 0 to 180" )

		MAP_METHOD_AND_WRAP(
			"SetClipEvents",
			SetClipEvents,
			"SetClipEvents( name, events )\n\n"
			"Sets the events a clip raises as it plays past them, read back with GetFiredEvents.  Each pose made raises\n"
			"those its showing clips have passed since the last, in the order passed.  A phase that goes up has gone\n"
			"forward, up to a whole loop; a looping clip's phase that goes down has come round over the seam if that is\n"
			"the short way round, and otherwise gone back, which raises none.  So a looping clip stepping more than half\n"
			"of itself a pose needs its phase counted up past 1 rather than wrapped.  A clip that has just come into the\n"
			"pose raises none until it moves on, so an event at the very start of a clip that does not loop never fires.\n"
			"Returns False if there is no such clip.\n"
			":param name: clip name\n"
			":param events: list of (phase, name), phase 0 to 1 through the clip" )

		MAP_METHOD_AND_WRAP(
			"GetFiredEvents",
			GetFiredEvents,
			"GetFiredEvents()\n\n"
			"The events raised since this was last called, as (clip, event) in the order raised; the oldest are dropped past\n"
			"256 unread." )

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

		MAP_METHOD_AND_WRAP(
			"SetRigBoneTurn",
			SetRigBoneTurn,
			"SetRigBoneTurn( rig, bone, axis, degrees )\n\n"
			"Turns a target bone and everything below it, in every clip of the rig, after retargeting: a clip made on a\n"
			"slimmer body holds the arms and legs closer in than the character is built for.  The turn is about an axis in\n"
			"the model's space, at the bone's joint; a turn of a bone below one already turned comes first, so the thigh can\n"
			"turn out and the ankle back, keeping the foot flat.  0 degrees takes it away.  Returns False if there is no\n"
			"such rig, or the axis has no length.\n"
			":param rig: rig name\n"
			":param bone: target bone name\n"
			":param axis: (x, y, z) to turn about, in the model's space\n"
			":param degrees: how far, as geo2.QuaternionRotationAxis turns by that angle about the axis" )

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

#ifdef TRINITY_HAS_PHYSICS_WORLD
		MAP_METHOD_AND_WRAP(
			"SetRagdoll",
			SetRagdoll,
			"SetRagdoll( world, id )\n\n"
			"The ragdoll whose parts stand for this character's bones: a built ragdoll's id in a Tr2PhysicsWorld, which is\n"
			"kept alive.  ( None, 0 ) lets it go.  Every pose made hands the world the pose as the ragdoll's target\n"
			"(SetRagdollTarget), and takes the ragdoll's pose back by ragdollWeight.  Like every stage, it runs only once a\n"
			"rig is bound, which takes a clip, at any weight.  Returns False for a world without an id or an id without a\n"
			"world." )

		MAP_METHOD_AND_WRAP(
			"AddRagdollBone",
			AddRagdollBone,
			"AddRagdollBone( part, bone, offsetPosition, offsetRotation )\n\n"
			"The bone a ragdoll part stands for, replacing the part's last.  Every part needs one, from part 0, the root,\n"
			"up, and no two the same.  Returns False for anything out of range.\n"
			":param part: the part's index in its ragdoll\n"
			":param bone: bone name in the animation's skeleton\n"
			":param offsetPosition: (x, y, z) of the part's middle in the bone's frame, in the model's units\n"
			":param offsetRotation: (x, y, z, w) of the part's frame in the bone's" )

		MAP_METHOD_AND_WRAP(
			"AddRagdollChain",
			AddRagdollChain,
			"AddRagdollChain( first, last )\n\n"
			"The bones between two parts' bones, each no part's own, share the two parts' turns by where they sit between\n"
			"them, as a rig's chain does: a spine of five bones bends evenly between three parts.\n"
			":param first: the chain's top bone, a part's\n"
			":param last: a part's bone below it" )

		MAP_METHOD_AND_WRAP( "ClearRagdollBones", ClearRagdollBones, "Forgets every ragdoll bone and chain" )

		MAP_METHOD_AND_WRAP(
			"SetRagdollFrame",
			SetRagdollFrame,
			"SetRagdollFrame( position, rotation, scale )\n\n"
			"Where the model is in the ragdoll's world's frame, to be given every frame the model moves: the origin at\n"
			"the world's until set.  Returns False for anything out of range.\n"
			":param position: (x, y, z) of the model's origin\n"
			":param rotation: (x, y, z, w) the model is turned by\n"
			":param scale: the model's size, above 0" )

		MAP_PROPERTY(
			"ragdollWeight",
			GetRagdollWeight,
			SetRagdollWeight,
			"How much of the pose is the ragdoll's, 0 to 1 (0 until set), for the parts the simulation moves" )

		MAP_PROPERTY( "ragdollOn", GetRagdollOn, SetRagdollOn, "Whether the ragdoll stage runs (on until set)" )

		MAP_PROPERTY(
			"ragdollFrozen",
			GetRagdollFrozen,
			SetRagdollFrozen,
			"True: the stage takes the ragdoll's parts as they are now, in the world, and keeps to that whatever they do\n"
			"after, as if every part were the simulation's; and the ragdoll's target is the pose drawn, blended, not the\n"
			"clips'.  For blending out of a body lying where it fell: put the model under it (SetRagdollFrame), key the\n"
			"ragdoll, and ease ragdollWeight down; the parts go back onto the clips with what is drawn.  Stays False\n"
			"without a built ragdoll" )
#endif

	EXPOSURE_END()
}
