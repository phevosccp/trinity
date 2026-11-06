// Copyright © 2026 CCP ehf.

#pragma once
#ifndef Tr2PhysicsWorld_h
#define Tr2PhysicsWorld_h

#include <memory>
#include <utility>
#include <vector>

namespace JPH
{
class ShapeSettings;
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
 * Bodies are boxes, capsules and convex hulls, named by the id their Add gives, never 0.  Each is:
 * - static: never moves;
 * - kinematic: moved by the caller (MoveKinematic), pushing dynamic bodies it meets and pushed by nothing;
 * - dynamic: moved by the simulation;
 * - upright: dynamic, but never turns: it slides, falls and stacks keeping the turn it was made with.
 * A kinematic body and a dynamic (or upright) one may be switched between the two (SetMotion).  Rotations are
 * (x, y, z, w).  Bodies resting on each other may sink in by up to 5 mm (Jolt's penetration slop, its default of 2 cm
 * tightened); every body has a friction of 0.6, and starts with no bounce and Jolt's damping of 0.05/s, which a moving
 * body may change (SetRestitution, SetDamping).  A moving body may also feel the gyroscopic force (SetGyroscopic), off
 * until set: then a body turning about the middle one of its three axes flips over and back, as a T-handle does in
 * orbit (the Dzhanibekov effect).
 *
 * CastRay finds the nearest body a segment meets.
 *
 * If Trinity was built with other Jolt settings than its Jolt library (which Jolt would answer with std::abort), the
 * world is not available: it logs an error once, the Adds give 0 and Step does nothing.
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
		MOTION_UPRIGHT = 3,
	};

	Tr2PhysicsWorld( IRoot* lockobj = nullptr );
	~Tr2PhysicsWorld();

	bool IsAvailable() const;
	Vector3 GetGravity() const;
	void SetGravity( const Vector3& gravity );
	int GetBodyCount() const;

	int AddBox( const Vector3& halfExtents, const Vector3& position, const Quaternion& rotation, int motion, float mass );
	int AddCapsule( float halfHeight, float radius, const Vector3& position, const Quaternion& rotation, int motion, float mass );
	int AddConvexHull( const std::vector<Vector3>& points, const Vector3& position, const Quaternion& rotation, int motion, float mass );
	bool RemoveBody( int id );
	bool HasBody( int id ) const;
	void Clear();

	int GetMotion( int id ) const;
	bool SetMotion( int id, int motion );

	std::pair<Vector3, Quaternion> GetTransform( int id ) const;
	bool SetTransform( int id, const Vector3& position, const Quaternion& rotation );
	bool MoveKinematic( int id, const Vector3& position, const Quaternion& rotation, float seconds );
	Vector3 GetLinearVelocity( int id ) const;
	bool SetLinearVelocity( int id, const Vector3& velocity );
	Vector3 GetAngularVelocity( int id ) const;
	bool SetAngularVelocity( int id, const Vector3& velocity );
	bool IsSleeping( int id ) const;
	bool AddImpulse( int id, const Vector3& impulse, const Vector3& point );

	float GetRestitution( int id ) const;
	bool SetRestitution( int id, float restitution );
	std::pair<float, float> GetDamping( int id ) const;
	bool SetDamping( int id, float linear, float angular );
	bool GetGyroscopic( int id ) const;
	bool SetGyroscopic( int id, bool on );

	// ((id, distance), (point, normal)) of the nearest body the segment meets; id 0 for none
	std::pair<std::pair<int, float>, std::pair<Vector3, Vector3>> CastRay( const Vector3& from, const Vector3& to ) const;

	bool Step( float seconds );

private:
	struct Body;
	struct Simulation;	// none if Jolt could not be set up

	int AddBody( const JPH::ShapeSettings& shape, const Vector3& position, const Quaternion& rotation, int motion, float mass );
	const Body* Find( int id ) const;	// none without a simulation, or for an id it has no body for
	const Body* FindMoving( int id ) const;	// as Find, and none for a static body

	std::unique_ptr<Simulation> m_simulation;
};

TYPEDEF_BLUECLASS( Tr2PhysicsWorld );

#endif
