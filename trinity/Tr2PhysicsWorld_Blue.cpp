// Copyright © 2026 CCP ehf.

#include "StdAfx.h"
#include "Tr2PhysicsWorld.h"

BLUE_DEFINE( Tr2PhysicsWorld );

const Be::ClassInfo* Tr2PhysicsWorld::ExposeToBlue()
{
	EXPOSURE_BEGIN(
		Tr2PhysicsWorld,
		"A rigid-body simulation at a character's scale (Jolt Physics), in one frame of reference the caller chooses, in\n"
		"metres.  Nothing runs until one is made; the caller steps it and reads its bodies back.  Rotations are (x, y, z, w).\n"
		"Motions: 0 static, never moves; 1 kinematic, moved by the caller (MoveKinematic), pushing dynamic bodies it meets;\n"
		"2 dynamic, moved by the simulation; 3 upright, dynamic but never turning.  Every body has a friction of 0.6,\n"
		"starts with no bounce (SetRestitution) and Jolt's damping (SetDamping), and rests sunk into what it rests on by\n"
		"up to 5 mm." )
		MAP_INTERFACE( Tr2PhysicsWorld )

		MAP_PROPERTY_READONLY(
			"isAvailable",
			IsAvailable,
			"False if Trinity was built with other Jolt settings than its Jolt library: then nothing is simulated, the Adds\n"
			"give 0 and Step returns False" )

		MAP_PROPERTY( "gravity", GetGravity, SetGravity, "(x, y, z) m/s^2 every dynamic body falls by; (0, -9.81, 0) until set" )

		MAP_PROPERTY_READONLY( "bodyCount", GetBodyCount, "How many bodies there are" )

		MAP_METHOD_AND_WRAP(
			"AddBox",
			AddBox,
			"AddBox( halfExtents, position, rotation, motion, mass )\n\n"
			"Adds a box and returns its id, or 0 if any argument is out of range or there is no room (4096 bodies).\n"
			":param halfExtents: (x, y, z) m from its middle to its faces, each 0.001 to 1000\n"
			":param position: (x, y, z) of its middle\n"
			":param rotation: (x, y, z, w), made unit length\n"
			":param motion: 0 static, 1 kinematic, 2 dynamic, 3 upright\n"
			":param mass: kg of a moving body, 0.001 to 1000000 (a kinematic one's counts once it is made dynamic); 0 or\n"
			"    less takes it from its size at 1000 kg/m^3.  Ignored for a static one" )

		MAP_METHOD_AND_WRAP(
			"AddCapsule",
			AddCapsule,
			"AddCapsule( halfHeight, radius, position, rotation, motion, mass )\n\n"
			"Adds a capsule along its own y, as AddBox adds a box.\n"
			":param halfHeight: m from its middle to the middle of each round end, 0 (a ball) to 1000\n"
			":param radius: m, 0.001 to 1000\n"
			":param position: (x, y, z) of its middle\n"
			":param rotation: (x, y, z, w)\n"
			":param motion: 0 static, 1 kinematic, 2 dynamic, 3 upright\n"
			":param mass: as AddBox's" )

		MAP_METHOD_AND_WRAP(
			"AddConvexHull",
			AddConvexHull,
			"AddConvexHull( points, position, rotation, motion, mass )\n\n"
			"Adds the convex hull round points, as AddBox adds a box, and returns its id; 0 also if the points are flat or\n"
			"all in one place.  Its mass is spread through the hull, so an odd shape turns as its shape says.\n"
			":param points: 4 to 1024 (x, y, z), in its own frame, each within 1000 m of its origin; the hull keeps at most\n"
			"    256 of them, its edges rounded by up to 5 cm inside them\n"
			":param position: (x, y, z) of its own frame's origin, which GetTransform gives back\n"
			":param rotation: (x, y, z, w)\n"
			":param motion: 0 static, 1 kinematic, 2 dynamic, 3 upright\n"
			":param mass: as AddBox's" )

		MAP_METHOD_AND_WRAP(
			"AddMesh",
			AddMesh,
			"AddMesh( vertices, triangles, position, rotation )\n\n"
			"Adds a static body of triangles and returns its id, 0 for anything out of range.  A rope's points pass through\n"
			"a triangle from behind, so a closed hull's must all face out.\n"
			":param vertices: (x, y, z) each, in its own frame\n"
			":param triangles: three indices into vertices for each triangle, one flat list, anticlockwise from in front\n"
			":param position: (x, y, z) of its own frame's origin\n"
			":param rotation: (x, y, z, w)" )

		MAP_METHOD_AND_WRAP(
			"AddRope",
			AddRope,
			"AddRope( points, radius, mass, bendCompliance )\n\n"
			"Adds a rope through points and returns its id, 0 for anything out of range.  It bends freely and keeps its\n"
			"length; left alone it straightens by bendCompliance, so it may start in a coil.  Each point meets the world's\n"
			"bodies as a ball of radius; the rope meets no other rope nor itself.  Pin the first point (SetRopePin): no\n"
			"point, pinned or not, strays from it by more than the rope's length to there, measured straight.\n"
			":param points: 3 to 1024 (x, y, z), each at least 1 mm from the last: the spacing is the rope's for good\n"
			":param radius: m, 0.001 to 1000\n"
			":param mass: kg of the whole rope, 0.001 to 1e6, shared evenly between its points\n"
			":param bendCompliance: 1/N, 0 to 1e6: 0 is stiff as a rod, and the larger the limper" )

		MAP_METHOD_AND_WRAP(
			"AddStaticBoxes",
			AddStaticBoxes,
			"AddStaticBoxes( halfExtents, positions, rotations )\n\n"
			"Adds one static body of many boxes with square edges, a walker sliding over the seams between them, and returns\n"
			"its id, 0 for anything out of range.  Its own frame is the world's.\n"
			":param halfExtents: (x, y, z) of each box, each 0.001 to 1000 m\n"
			":param positions: (x, y, z) of each box's middle\n"
			":param rotations: (x, y, z, w) of each box" )

		MAP_METHOD_AND_WRAP( "RemoveBody", RemoveBody, "RemoveBody( id )\n\nRemoves a body, rope or character.  Returns False if there is none with that id" )

		MAP_METHOD_AND_WRAP( "HasBody", HasBody, "HasBody( id )\n\nWhether there is a body, rope or character with that id" )

		MAP_METHOD_AND_WRAP( "Clear", Clear, "Removes every body, rope and character" )

		MAP_METHOD_AND_WRAP( "ClearGroup", ClearGroup, "ClearGroup( group )\n\nRemoves every body and rope of a group" )

		MAP_METHOD_AND_WRAP(
			"OptimizeBroadPhase",
			OptimizeBroadPhase,
			"Rebuilds the world's search trees: worth doing once after adding a level's worth of static bodies" )

		MAP_METHOD_AND_WRAP(
			"SetBodyGroup",
			SetBodyGroup,
			"SetBodyGroup( id, group )\n\n"
			"Puts a body in a group, 0 to 3 (0 at first).  It meets only the bodies of its group; characters meet group 0's\n"
			"static bodies and every group's moving ones.  Returns False if it is not there or the group is out of range" )

		MAP_METHOD_AND_WRAP(
			"SetBodyTags",
			SetBodyTags,
			"SetBodyTags( id, tags )\n\n"
			"Bits a Trace can be told to look past (its ignoredTags).  Returns False if it is not there or they are negative" )

		MAP_METHOD_AND_WRAP(
			"SetGravityFactor",
			SetGravityFactor,
			"SetGravityFactor( id, factor )\n\n"
			"How much of the world's gravity a moving body feels, 1 at first; 0 floats.  Returns False for a static body or none" )

		MAP_METHOD_AND_WRAP( "GetMotion", GetMotion, "GetMotion( id )\n\nA body's motion, 0 to 3; -1 if there is no body with that id" )

		MAP_METHOD_AND_WRAP(
			"SetMotion",
			SetMotion,
			"SetMotion( id, motion )\n\n"
			"Switches a moving body between kinematic (1) and what it was made as, dynamic (2) or upright (3), waking it;\n"
			"it keeps its velocity.  Returns False for a static body, one not there, or any other switch" )

		MAP_METHOD_AND_WRAP(
			"GetTransform",
			GetTransform,
			"GetTransform( id )\n\n"
			"Where a body is: ((x, y, z), (x, y, z, w)), its middle and its turn.  ((0, 0, 0), (0, 0, 0, 1)) if there is\n"
			"no body with that id" )

		MAP_METHOD_AND_WRAP(
			"SetTransform",
			SetTransform,
			"SetTransform( id, position, rotation )\n\n"
			"Puts a body there at once, keeping its velocity and waking it if it moves; it does not push what it lands in.\n"
			"An upright body takes the turn too.  Returns False if it is not there or an argument is not finite" )

		MAP_METHOD_AND_WRAP(
			"MoveKinematic",
			MoveKinematic,
			"MoveKinematic( id, position, rotation, seconds )\n\n"
			"Sets a kinematic body moving to reach position and rotation in seconds (0.000001 to 1), pushing what it meets\n"
			"on the way; give the time to the next Step.  An upright body moves but keeps its turn (SetTransform turns it).\n"
			"Returns False for a body that is not kinematic, or bad arguments" )

		MAP_METHOD_AND_WRAP(
			"GetLinearVelocity",
			GetLinearVelocity,
			"GetLinearVelocity( id )\n\nA body's velocity, (x, y, z) m/s; (0, 0, 0) for a static one or if there is none" )

		MAP_METHOD_AND_WRAP(
			"SetLinearVelocity",
			SetLinearVelocity,
			"SetLinearVelocity( id, velocity )\n\n"
			"Sets a kinematic or dynamic body moving at (x, y, z) m/s, up to 500, waking it.  Returns False for a static\n"
			"body, one that is not there, or a velocity out of range" )

		MAP_METHOD_AND_WRAP(
			"GetAngularVelocity",
			GetAngularVelocity,
			"GetAngularVelocity( id )\n\nHow a body turns, (x, y, z) rad/s about those axes; (0, 0, 0) if it does not or is not there" )

		MAP_METHOD_AND_WRAP(
			"SetAngularVelocity",
			SetAngularVelocity,
			"SetAngularVelocity( id, velocity )\n\n"
			"Sets a kinematic or dynamic body turning at (x, y, z) rad/s, up to 40, waking it; an upright body ignores it.\n"
			"Returns False for a static body, one that is not there, or a velocity out of range" )

		MAP_METHOD_AND_WRAP(
			"IsSleeping",
			IsSleeping,
			"IsSleeping( id )\n\n"
			"Whether a body is at rest and no longer simulated until something wakes it; static bodies always are.  False\n"
			"if there is no body with that id" )

		MAP_METHOD_AND_WRAP(
			"AddImpulse",
			AddImpulse,
			"AddImpulse( id, impulse, point )\n\n"
			"Pushes a dynamic or upright body by impulse, (x, y, z) N s, at point (x, y, z), waking it: its velocity changes\n"
			"by impulse over its mass, and a push off its centre of mass turns it too (an upright body only slides).\n"
			"Returns False for a body that is not dynamic or upright (kinematic now included), one not there, or bad\n"
			"arguments" )

		MAP_METHOD_AND_WRAP( "GetRestitution", GetRestitution, "GetRestitution( id )\n\nHow lively a body's bounce is, 0 to 1; 0 if it is not there" )

		MAP_METHOD_AND_WRAP(
			"SetRestitution",
			SetRestitution,
			"SetRestitution( id, restitution )\n\n"
			"How lively a body bounces, 0 (dead, every body's at first) to 1 (keeping its speed).  Two bodies meeting bounce\n"
			"by the larger of the two.  Returns False if it is not there or restitution is out of range" )

		MAP_METHOD_AND_WRAP(
			"GetDamping",
			GetDamping,
			"GetDamping( id )\n\n(linear, angular) 1/s a moving body's speed and turn die away at; (0, 0) for a static one or none" )

		MAP_METHOD_AND_WRAP(
			"SetDamping",
			SetDamping,
			"SetDamping( id, linear, angular )\n\n"
			"How fast a moving body's speed and turn die away, each 0 to 100 per second (0.05 at first); 0 keeps them for\n"
			"good, as in space.  Returns False for a static body, one not there, or values out of range" )

		MAP_METHOD_AND_WRAP(
			"GetGyroscopic",
			GetGyroscopic,
			"GetGyroscopic( id )\n\nWhether a moving body feels the gyroscopic force; False for a static one or none" )

		MAP_METHOD_AND_WRAP(
			"SetGyroscopic",
			SetGyroscopic,
			"SetGyroscopic( id, on )\n\n"
			"Whether a moving body feels the gyroscopic force (off at first): on, one with three different axes spun about\n"
			"its middle one flips over and back, as a T-handle does in orbit.  Returns False for a static body or none" )

		MAP_METHOD_AND_WRAP(
			"GetRopePoints",
			GetRopePoints,
			"GetRopePoints( id )\n\n"
			"Where a rope's points are, each (x, y, z), first to last; none if there is no rope with that id" )

		MAP_METHOD_AND_WRAP(
			"SetRopePin",
			SetRopePin,
			"SetRopePin( id, index, pinned )\n\n"
			"Pins a rope's point where it is, for the caller to move (MoveRopePin), or lets it go, stopped.  Returns False\n"
			"if there is no such rope or point" )

		MAP_METHOD_AND_WRAP(
			"MoveRopePin",
			MoveRopePin,
			"MoveRopePin( id, index, position, seconds )\n\n"
			"Moves a pinned point to position over the next seconds of Step, sweeping the rope after it, as far as the rope\n"
			"reaches.  It goes on at that speed until told otherwise, so call this every step.  Returns False if there is\n"
			"no such rope or point, the\n"
			"point is not pinned, or seconds is outside 0.000001 to 1" )

		MAP_METHOD_AND_WRAP(
			"CastRay",
			CastRay,
			"CastRay( start, end )\n\n"
			"The nearest body the segment from start to end (each (x, y, z), at most 100 km apart) meets:\n"
			"((id, distance), (point, normal)), distance in m from start, normal the body's surface's there.  A body the\n"
			"segment starts inside is met at start.  ((0, 0), ((0, 0, 0), (0, 0, 0))) if it meets none" )

		MAP_METHOD_AND_WRAP(
			"CastRayInGroup",
			CastRayInGroup,
			"CastRayInGroup( start, end, group )\n\nAs CastRay, meeting only the bodies of a group" )

		MAP_METHOD_AND_WRAP(
			"Trace",
			Trace,
			"Trace( start, end, radius, ignored, ignoredTags, group )\n\n"
			"The nearest body a ray, or a sphere swept along it, meets, with the rules a walker wants; the same result as\n"
			"CastRay.  A ray does not see a convex body it starts in, and sees a mesh from both sides.  A sphere overlapping\n"
			"something at the start meets it only if the move goes further in: then the distance is minus how deep it is\n"
			"and the normal the way out, so one resting on a floor still sees the wall ahead.\n"
			":param radius: m; 0 for a ray\n"
			":param ignored: ids of bodies to look past\n"
			":param ignoredTags: to look past bodies with any of these bits (SetBodyTags)\n"
			":param group: a group, or -1 for what a character meets" )

		MAP_METHOD_AND_WRAP(
			"TraceBody",
			TraceBody,
			"TraceBody( id, movement, ignored )\n\n"
			"The first thing a body would meet moved by movement (x, y, z) from where it is, as a Trace; a body sliding along\n"
			"what it rests on does not meet it" )

		MAP_METHOD_AND_WRAP(
			"AddCharacter",
			AddCharacter,
			"AddCharacter( position, stances, mass, strength, maxSlopeDegrees )\n\n"
			"Adds a walker with its feet at position and returns its id, 0 for anything out of range.  It moves only by\n"
			"MoveCharacter, called before Step.\n"
			":param position: (x, y, z) of its feet\n"
			":param stances: up to 8 (radius, height, flat), the first the one it starts in: a capsule for flat 0, a cylinder\n"
			"    otherwise; a capsule at least 2 radii tall, a cylinder at least 1\n"
			":param mass: kg, what it weighs on what it stands on; only something at least as heavy pushes it\n"
			":param strength: N it pushes with at most\n"
			":param maxSlopeDegrees: steeper ground than this does not hold it up" )

		MAP_METHOD_AND_WRAP(
			"MoveCharacter",
			MoveCharacter,
			"MoveCharacter( id, velocity, seconds, gravity, stepUp, stickDown )\n\n"
			"Moves a character for seconds (0.000001 to 1) and returns (feet, walked): where its feet are now, and the move\n"
			"it walked, x and z without the ground's own move or being pushed out of anything, and no longer than asked; y\n"
			"how far it climbed or fell.  On the ground it is carried by what it stands on, and walks along the ground at the\n"
			"speed across it that velocity gives.\n"
			":param velocity: (x, y, z) m/s it wants to go, not counting the ground's move; y above 0 leaves the ground\n"
			":param gravity: m/s^2 down that it weighs on what it stands on\n"
			":param stepUp: m it steps up a kerb; 0 for none\n"
			":param stickDown: m it follows the ground down a step, and a slope; 0 for none" )

		MAP_METHOD_AND_WRAP(
			"GetCharacterPosition",
			GetCharacterPosition,
			"GetCharacterPosition( id )\n\nWhere a character's feet are, (x, y, z); (0, 0, 0) if there is no such character" )

		MAP_METHOD_AND_WRAP(
			"SetCharacterPosition",
			SetCharacterPosition,
			"SetCharacterPosition( id, position )\n\n"
			"Puts a character's feet there, with the body inside it, meeting nothing on the way, for moves the caller scripts;\n"
			"what it stands on goes on as it was until RefreshCharacter or MoveCharacter.  False if there is none or it is\n"
			"not finite" )

		MAP_METHOD_AND_WRAP(
			"RefreshCharacter",
			RefreshCharacter,
			"RefreshCharacter( id, stickDown )\n\n"
			"Stops a character and looks at what is round it, after moving it or anything near it: and if nothing holds it up\n"
			"sets its feet on the ground up to stickDown m below (0 for none).  False if there is none" )

		MAP_METHOD_AND_WRAP(
			"GetCharacterGround",
			GetCharacterGround,
			"GetCharacterGround( id )\n\n"
			"What a character stands on: ((state, body id), (point, normal)).  State 0 on the ground, 1 on ground too steep to\n"
			"stand on, 2 touching something that does not hold it up, 3 in the air; -1 if there is no such character.  The\n"
			"body is 0 for none" )

		MAP_METHOD_AND_WRAP(
			"GetCharacterGroundVelocity",
			GetCharacterGroundVelocity,
			"GetCharacterGroundVelocity( id )\n\nHow fast what a character stands on moves, (x, y, z) m/s, at its feet" )

		MAP_METHOD_AND_WRAP(
			"GetCharacterContacts",
			GetCharacterContacts,
			"GetCharacterContacts( id )\n\n"
			"What a character touched in its last move, each (body id, (x, y, z) surface normal); none if there is none" )

		MAP_METHOD_AND_WRAP(
			"FitsCharacter",
			FitsCharacter,
			"FitsCharacter( id, position, stance )\n\n"
			"Whether a character could take a stance (an index into the stances it was made with) with its feet at position" )

		MAP_METHOD_AND_WRAP(
			"SetCharacterStance",
			SetCharacterStance,
			"SetCharacterStance( id, stance, force )\n\n"
			"Takes a stance; if it does not fit, False and it stays as it is, unless forced" )

		MAP_METHOD_AND_WRAP(
			"SetCharacterIgnored",
			SetCharacterIgnored,
			"SetCharacterIgnored( id, ignored )\n\nThe ids of the bodies a character passes through, in place of those before" )

		MAP_METHOD_AND_WRAP(
			"SetCharacterPassing",
			SetCharacterPassing,
			"SetCharacterPassing( id, tags )\n\n"
			"Bodies with any of these tags (SetBodyTags) don't stop a character: it walks through them, and the body inside it\n"
			"knocks them aside at its speed, however heavy they are or whatever is on them" )

		MAP_METHOD_AND_WRAP(
			"SetCharacterForce",
			SetCharacterForce,
			"SetCharacterForce( id, mass, strength )\n\nA character's mass in kg and the N it pushes with, as AddCharacter's" )

		MAP_METHOD_AND_WRAP(
			"Step",
			Step,
			"Step( seconds )\n\n"
			"Moves the simulation on by seconds (0.000001 to 1), in collision steps of at most 1/60 s.  Returns False,\n"
			"moving nothing, for a time out of range; False also if the step ran out of room, which is logged" )
	EXPOSURE_END()
}
