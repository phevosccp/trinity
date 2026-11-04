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
		"2 dynamic, moved by the simulation; 3 upright, dynamic but never turning.  Every body has a friction of 0.6 and\n"
		"no bounce, and rests sunk into what it rests on by up to 5 mm." )
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

		MAP_METHOD_AND_WRAP( "RemoveBody", RemoveBody, "RemoveBody( id )\n\nRemoves a body.  Returns False if there is none with that id" )

		MAP_METHOD_AND_WRAP( "HasBody", HasBody, "HasBody( id )\n\nWhether there is a body with that id" )

		MAP_METHOD_AND_WRAP( "Clear", Clear, "Removes every body" )

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
			"Step",
			Step,
			"Step( seconds )\n\n"
			"Moves the simulation on by seconds (0.000001 to 1), in collision steps of at most 1/60 s.  Returns False,\n"
			"moving nothing, for a time out of range; False also if the step ran out of room, which is logged" )
	EXPOSURE_END()
}
