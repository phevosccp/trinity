// Copyright © 2026 CCP ehf.

#pragma once
#ifndef Tr2PhysicsWorld_h
#define Tr2PhysicsWorld_h

#include <memory>
#include <utility>

namespace JPH
{
class BodyID;
}

BLUE_DECLARE( Tr2PhysicsWorld );

/*
 * A rigid-body simulation at a character's scale (Jolt Physics): crates that fall, stack and get pushed, thrown
 * things, and later ragdolls.  Nothing in Trinity makes one; the caller does, steps it and reads its bodies back, so a
 * client that never makes one never runs any of it.
 *
 * Everything is in one frame of reference the caller chooses (a ball's, say), in metres, with +Y up as gravity's
 * default says.  Positions are single precision, so keep them within a few kilometres of the frame's origin.
 *
 * Bodies resting on each other may sink in by up to 5 mm (Jolt's penetration slop, its default of 2 cm tightened).
 *
 * Bodies are boxes, each static (never moves), kinematic (moved by the caller, pushes what it meets) or dynamic
 * (moved by the simulation), named by the id AddBox gives, never 0.  Rotations are (x, y, z, w).
 *
 * If Trinity was built with other Jolt settings than its Jolt library (which Jolt would answer with std::abort), the
 * world is not available: it logs an error once, AddBox gives 0 and Step does nothing.
 */
BLUE_CLASS( Tr2PhysicsWorld ) :
	public IRoot
{
public:
	EXPOSE_TO_BLUE();

	enum Motion
	{
		MOTION_STATIC = 0,
		MOTION_KINEMATIC = 1,
		MOTION_DYNAMIC = 2,
	};

	Tr2PhysicsWorld( IRoot* lockobj = nullptr );
	~Tr2PhysicsWorld();

	bool IsAvailable() const;
	Vector3 GetGravity() const;
	void SetGravity( const Vector3& gravity );
	int GetBodyCount() const;

	int AddBox( const Vector3& halfExtents, const Vector3& position, const Quaternion& rotation, int motion, float mass );
	bool RemoveBody( int id );
	bool HasBody( int id ) const;
	void Clear();

	std::pair<Vector3, Quaternion> GetTransform( int id ) const;
	Vector3 GetLinearVelocity( int id ) const;
	bool SetLinearVelocity( int id, const Vector3& velocity );
	bool IsSleeping( int id ) const;

	bool Step( float seconds );

private:
	const JPH::BodyID* Find( int id ) const;	// none without a simulation, or for an id it has no body for

	struct Simulation;	// none if Jolt could not be set up
	std::unique_ptr<Simulation> m_simulation;
};

TYPEDEF_BLUECLASS( Tr2PhysicsWorld );

#endif
