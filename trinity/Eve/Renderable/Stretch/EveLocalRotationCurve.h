// Copyright © 2026 CCP ehf.

#ifndef _EVELOCALROTATIONCURVE_H_
#define _EVELOCALROTATIONCURVE_H_

#include <ITriFunction.h>

// A turn local to a parent's frame, as an ITriQuaternionFunction: rotation, then the parent's turn (a ship's ball,
// say), read from the parent at the blue time asked for. The turning partner of EveLocalPositionCurve's offsetPosition,
// so a model standing on a moving, turning ship (a character on its hull) is posed with the ship as the ship is drawn,
// never a frame behind it. As EveLocalPositionCurve, it is made at blue times; asked at a time in seconds it gives the
// turn last made. Python reads it through the interface's GetQuaternionAt( blueTime ).
BLUE_CLASS( EveLocalRotationCurve ) :
	public ITriQuaternionFunction
{
public:
	EXPOSE_TO_BLUE();

	EveLocalRotationCurve( IRoot* lockobj = nullptr );

	void UpdateValue( double time ) override;
	Quaternion* Update( Quaternion* in, Be::Time time ) override;
	Quaternion* Update( Quaternion* in, double time ) override;
	Quaternion* GetValueAt( Quaternion* in, Be::Time time ) override;
	Quaternion* GetValueAt( Quaternion* in, double time ) override;
	Quaternion* GetValueDotAt( Quaternion* in, Be::Time time ) override;
	Quaternion* GetValueDotAt( Quaternion* in, double time ) override;
	Quaternion* GetValueDoubleDotAt( Quaternion* in, Be::Time time ) override;
	Quaternion* GetValueDoubleDotAt( Quaternion* in, double time ) override;

private:
	Quaternion* Compose( Quaternion* in, Be::Time time );

	ITriQuaternionFunctionPtr m_parentRotationCurve;
	Quaternion m_rotation;
	Quaternion m_value;
};

TYPEDEF_BLUECLASS( EveLocalRotationCurve );

#endif
