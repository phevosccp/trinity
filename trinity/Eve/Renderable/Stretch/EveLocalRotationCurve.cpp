// Copyright © 2026 CCP ehf.

#include "StdAfx.h"
#include "EveLocalRotationCurve.h"

EveLocalRotationCurve::EveLocalRotationCurve( IRoot* ) :
	m_rotation( 0.f, 0.f, 0.f, 1.f ),
	m_value( 0.f, 0.f, 0.f, 1.f )
{
}

Quaternion* EveLocalRotationCurve::Compose( Quaternion* in, Be::Time time )
{
	Quaternion parent( 0.f, 0.f, 0.f, 1.f );
	if( m_parentRotationCurve )
	{
		m_parentRotationCurve->GetValueAt( &parent, time );
	}
	// The local turn first, then the parent's: XMQuaternionMultiply( a, b ) turns by a, then by b
	m_value = XMQuaternionNormalize( XMQuaternionMultiply( m_rotation, parent ) );
	*in = m_value;
	return in;
}

void EveLocalRotationCurve::UpdateValue( double )
{
}

Quaternion* EveLocalRotationCurve::Update( Quaternion* in, Be::Time time )
{
	return Compose( in, time );
}

// A parent such as a ball turns only at blue times, so a time in seconds gives the turn last made, as
// EveLocalPositionCurve does
Quaternion* EveLocalRotationCurve::Update( Quaternion* in, double )
{
	*in = m_value;
	return in;
}

Quaternion* EveLocalRotationCurve::GetValueAt( Quaternion* in, Be::Time time )
{
	return Compose( in, time );
}

Quaternion* EveLocalRotationCurve::GetValueAt( Quaternion* in, double )
{
	*in = m_value;
	return in;
}

Quaternion* EveLocalRotationCurve::GetValueDotAt( Quaternion* in, Be::Time )
{
	*in = Quaternion( 0.f, 0.f, 0.f, 0.f );
	return in;
}

Quaternion* EveLocalRotationCurve::GetValueDotAt( Quaternion* in, double )
{
	*in = Quaternion( 0.f, 0.f, 0.f, 0.f );
	return in;
}

Quaternion* EveLocalRotationCurve::GetValueDoubleDotAt( Quaternion* in, Be::Time )
{
	*in = Quaternion( 0.f, 0.f, 0.f, 0.f );
	return in;
}

Quaternion* EveLocalRotationCurve::GetValueDoubleDotAt( Quaternion* in, double )
{
	*in = Quaternion( 0.f, 0.f, 0.f, 0.f );
	return in;
}
