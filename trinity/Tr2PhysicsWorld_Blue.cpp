// Copyright © 2026 CCP ehf.

#include "StdAfx.h"
#include "Tr2PhysicsWorld.h"

BLUE_DEFINE( Tr2PhysicsWorld );

const Be::ClassInfo* Tr2PhysicsWorld::ExposeToBlue()
{
	EXPOSURE_BEGIN(
		Tr2PhysicsWorld,
		"A rigid-body simulation at a character's scale (Jolt Physics), in one frame of reference the caller chooses, in\n"
		"metres.  Nothing runs until one is made; the caller steps it and reads its bodies back.  Rotations are (x, y, z, w)." )
		MAP_INTERFACE( Tr2PhysicsWorld )

		MAP_PROPERTY_READONLY(
			"isAvailable",
			IsAvailable,
			"False if Trinity was built with other Jolt settings than its Jolt library: then nothing is simulated, AddBox\n"
			"gives 0 and Step returns False" )

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
			":param motion: 0 static, never moves; 1 kinematic, moved by the caller and pushing dynamic bodies it meets;\n"
			"    2 dynamic, moved by the simulation\n"
			":param mass: kg of a dynamic box, 0.001 to 1000000; 0 or less takes it from its size at 1000 kg/m^3.  Ignored\n"
			"    for the others" )

		MAP_METHOD_AND_WRAP( "RemoveBody", RemoveBody, "RemoveBody( id )\n\nRemoves a body.  Returns False if there is none with that id" )

		MAP_METHOD_AND_WRAP( "HasBody", HasBody, "HasBody( id )\n\nWhether there is a body with that id" )

		MAP_METHOD_AND_WRAP( "Clear", Clear, "Removes every body" )

		MAP_METHOD_AND_WRAP(
			"GetTransform",
			GetTransform,
			"GetTransform( id )\n\n"
			"Where a body is: ((x, y, z), (x, y, z, w)), its middle and its turn.  ((0, 0, 0), (0, 0, 0, 1)) if there is\n"
			"no body with that id" )

		MAP_METHOD_AND_WRAP(
			"GetLinearVelocity",
			GetLinearVelocity,
			"GetLinearVelocity( id )\n\nA body's velocity, (x, y, z) m/s; (0, 0, 0) for a static one or if there is none" )

		MAP_METHOD_AND_WRAP(
			"SetLinearVelocity",
			SetLinearVelocity,
			"SetLinearVelocity( id, velocity )\n\n"
			"Sets a kinematic or dynamic body moving at (x, y, z) m/s, waking it.  Returns False for a static body, one\n"
			"that is not there, or a velocity that is not finite" )

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
