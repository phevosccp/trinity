// Copyright © 2026 CCP ehf.

#include "StdAfx.h"
#include "EveLocalRotationCurve.h"

BLUE_DEFINE( EveLocalRotationCurve );

const Be::ClassInfo* EveLocalRotationCurve::ExposeToBlue()
{
	EXPOSURE_BEGIN( EveLocalRotationCurve, "A turn local to a parent's frame: the parent's turn followed by rotation" )
		MAP_INTERFACE( ITriQuaternionFunction )

		MAP_ATTRIBUTE(
			"parentRotationCurve",
			m_parentRotationCurve,
			"The parent's turn (a ball's, say), read at the time asked for; none is no turn",
			Be::READWRITE )

		MAP_ATTRIBUTE( "rotation", m_rotation, "The turn in the parent's frame, (x, y, z, w)", Be::READWRITE )

		MAP_ATTRIBUTE( "value", m_value, "The turn last made", Be::READ )
	EXPOSURE_END()
}
