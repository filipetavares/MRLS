#include "stdafx.h"
#include "eMarker.h"


eMarker::eMarker()
{
}

eMarker::eMarker(int Id, float X, float Y, float ang)
{
	id = Id;
	x = X;
	y = Y;
	angle = ang;
}