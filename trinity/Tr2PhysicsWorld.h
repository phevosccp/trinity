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
 * A mesh (AddMesh) is a static body of triangles, a hull too odd for a convex one.  Each faces the way its corners run
 * anticlockwise, and a rope's points pass through from behind, so a closed hull's must all face out.
 *
 * A rope (AddRope) is a line of points that bends freely and keeps its length, a Cosserat rod in Jolt's soft bodies:
 * a cable, a tether.  Each point meets the world's bodies as a ball of the rope's radius; a rope meets no other rope
 * nor itself.  The points' spacing as given is the rope's for good, whatever shape it starts in: left alone it
 * straightens by its bend compliance, so it may start in a coil.  Any point may be pinned, held by the caller
 * (SetRopePin, MoveRopePin); keep the first pinned, as no point, pinned or not, strays further from it than the rope's
 * length to there: a pin led past that stays at the end of the rope.  That length is measured straight, so a rope bent
 * round something can be led further and stretches.  A rope is not a body to the methods for bodies: only RemoveBody,
 * HasBody, Clear, SetBodyGroup, ClearGroup and bodyCount know it.
 *
 * A character (AddCharacter) is someone walking in the world, on Jolt's CharacterVirtual: it slides along walls, stands
 * on what is under it, steps up and down kerbs, stands on and shoves moving bodies, and is carried by them.  It moves
 * only when told to (MoveCharacter), not by Step; call that first, then Step.  Its position is where its feet are.  It
 * has a stance for each shape it can take (a capsule or a cylinder standing on its feet), and a body inside it a little
 * smaller, kinematic, so what is thrown at it bounces off.  Only something at least as heavy as it pushes it; a body
 * lighter than it moving into it stops at it.  What it ignores (SetCharacterIgnored) its inner body does too.  What it passes
 * through by tag (SetCharacterPassing) its inner body knocks aside, at the speed it walks and however heavy: small things
 * do not stop a walker, and a stack of them comes down.  A character is not a body to the methods for bodies: only RemoveBody,
 * HasBody, Clear and bodyCount know it.  Nothing that casts meets it.
 *
 * A body is in a group, 0 until told otherwise (SetBodyGroup).  Bodies meet only those of their own group, as if each
 * group had a world of its own, with its own gravity by the gravity factor (SetGravityFactor), and yet one character
 * meets group 0's static bodies and the moving bodies of every group.  Trace looks where the character does, unless
 * given a group.
 *
 * Static boxes by the hundred are best made as one body (AddStaticBoxes), their edges hardly rounded, as a walker slides
 * past the seams between them where it would catch on the corners of separate boxes.
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
	int AddMesh( const std::vector<Vector3>& vertices, const std::vector<int>& triangles, const Vector3& position, const Quaternion& rotation );
	int AddRope( const std::vector<Vector3>& points, float radius, float mass, float bendCompliance );
	int AddStaticBoxes( const std::vector<Vector3>& halfExtents, const std::vector<Vector3>& positions, const std::vector<Quaternion>& rotations );
	bool RemoveBody( int id );
	bool HasBody( int id ) const;
	void Clear();
	void ClearGroup( int group );
	void OptimizeBroadPhase();

	bool SetBodyGroup( int id, int group );
	bool SetBodyTags( int id, int tags );
	bool SetGravityFactor( int id, float factor );

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

	std::vector<Vector3> GetRopePoints( int id ) const;
	bool SetRopePin( int id, int index, bool pinned );
	bool MoveRopePin( int id, int index, const Vector3& position, float seconds );

	// Each stance is (radius, height, flat): flat 0 for a capsule, anything else for a cylinder
	int AddCharacter( const Vector3& position, const std::vector<Vector3>& stances, float mass, float strength, float maxSlopeDegrees );
	// (where the feet end up, the move the character walked: x and z without the ground's move or being pushed out of
	// anything, and no longer than it asked for; y as it climbed or fell)
	std::pair<Vector3, Vector3> MoveCharacter( int id, const Vector3& velocity, float seconds, float gravity, float stepUp, float stickDown );
	Vector3 GetCharacterPosition( int id ) const;
	bool SetCharacterPosition( int id, const Vector3& position );
	bool RefreshCharacter( int id, float stickDown );
	// ((state, body id), (point, normal)): state 0 on the ground, 1 on ground too steep, 2 touching but not held up, 3
	// in the air, -1 if there is no such character; the body 0 for none
	std::pair<std::pair<int, int>, std::pair<Vector3, Vector3>> GetCharacterGround( int id ) const;
	Vector3 GetCharacterGroundVelocity( int id ) const;
	std::vector<std::pair<int, Vector3>> GetCharacterContacts( int id ) const;
	bool FitsCharacter( int id, const Vector3& position, int stance ) const;
	bool SetCharacterStance( int id, int stance, bool force );
	bool SetCharacterIgnored( int id, const std::vector<int>& ignored );
	bool SetCharacterPassing( int id, int tags );
	bool SetCharacterForce( int id, float mass, float strength );

	// ((id, distance), (point, normal)) of the nearest body the segment meets; id 0 for none
	std::pair<std::pair<int, float>, std::pair<Vector3, Vector3>> CastRay( const Vector3& from, const Vector3& to ) const;
	std::pair<std::pair<int, float>, std::pair<Vector3, Vector3>> CastRayInGroup( const Vector3& from, const Vector3& to, int group ) const;
	// As CastRay, or a sphere swept along it for a radius above 0; a negative distance is how deep it started in what it
	// is going further into.  Group -1 looks where a character does
	std::pair<std::pair<int, float>, std::pair<Vector3, Vector3>> Trace( const Vector3& from, const Vector3& to, float radius, const std::vector<int>& ignored, int ignoredTags, int group ) const;
	// As Trace, for a body's own shape moved from where it is, meeting what it would
	std::pair<std::pair<int, float>, std::pair<Vector3, Vector3>> TraceBody( int id, const Vector3& movement, const std::vector<int>& ignored ) const;

	bool Step( float seconds );

private:
	struct Body;
	struct Character;
	struct Simulation;	// none if Jolt could not be set up

	int AddBody( const JPH::ShapeSettings& shape, const Vector3& position, const Quaternion& rotation, int motion, float mass );
	const Body* Find( int id ) const;	// none without a simulation, or for an id it has no body for; a rope is none
	const Body* FindMoving( int id ) const;	// as Find, and none for a static body
	const Body* FindRope( int id ) const;	// none without a simulation, or for an id it has no rope for
	Character* FindCharacter( int id ) const;	// none without a simulation, or for an id it has no character for
	std::pair<std::pair<int, float>, std::pair<Vector3, Vector3>> RayHit( const Vector3& from, const Vector3& to, int group ) const;

	std::unique_ptr<Simulation> m_simulation;
};

TYPEDEF_BLUECLASS( Tr2PhysicsWorld );

#endif
