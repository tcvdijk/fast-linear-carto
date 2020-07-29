#include "RoadNetwork.h"
#include "Segment.h"
#include "Point.h"
#include "Util.h"

#include <iostream>
#include <cassert>
#include <math.h>
using namespace std;

Point *Segment::oppositeOf( Point * p ) {
	if( a==p ) return b;
	assert( b==p );
	return a;
}

Real Segment::targetZoom() {
	return requestedLength / length;
}

Real Segment::signedLineDist( Real x, Real y ) {
	return x*nx + y*ny;
}

Real oneWayDistortion( Point *from, Point *to ) {
	Real expectX = from->displaced.x + from->displaced.scale * (to->original.x-from->original.x);
	Real dx = to->displaced.x - expectX;
	Real expectY = from->displaced.y + from->displaced.scale * (to->original.y-from->original.y);
	Real dy = to->displaced.y - expectY;
	return ( dx*dx + dy*dy );
}

Real Segment::distortion() {
	Real factor = 1.0 / length;
	return factor*(oneWayDistortion(a,b) + oneWayDistortion(b,a));
}

Real Segment::distToDisplacedSegment(Point *p, Real t) {
	Real x = p->original.x;
	Real y = p->original.y;
	Real ax = lerp(a->original.x, a->displaced.x, t);
	Real ay = lerp(a->original.y, a->displaced.y, t);
	Real bx = lerp(b->original.x, b->displaced.x, t);
	Real by = lerp(b->original.y, b->displaced.y, t);
	Real dx = bx - ax;
	Real dy = by - ay;
	Real delta = ((x - ax)*dx + (y - ay)*dy) / (dx*dx + dy*dy);
	Real Fx, Fy;
	if(delta < 0) {
		Fx = a->displaced.x;
		Fy = a->displaced.y;
	}
	else if(delta > 1) {
		Fx = b->displaced.x;
		Fy = b->displaced.y;
	}
	else {
		Fx = a->displaced.x + delta*dx;
		Fy = a->displaced.y + delta*dy;
	}
	dx = Fx - x;
	dy = Fy - y;
	return sqrt(dx*dx + dy*dy);
}

Real Segment::arcOriginalDisplaced(Real t) {
	const Real odx = a->original.x - b->original.x;
	const Real ody = a->original.y - b->original.y;
	const Real ax = lerp(a->original.x, a->displaced.x, t);
	const Real ay = lerp(a->original.y, a->displaced.y, t);
	const Real bx = lerp(b->original.x, b->displaced.x, t);
	const Real by = lerp(b->original.y, b->displaced.y, t);
	const Real dx = ax - bx;
	const Real dy = ay - by;
	Real cosAngle = (odx*dx + ody*dy) / (sqrt(odx*odx + ody*ody) * \
		sqrt(dx*dx + dy*dy));
	if(cosAngle > 1)
		cosAngle = 1.0;
	else if(cosAngle < -1.0)
		cosAngle = -1.0;
	const Real angle = acos(cosAngle);
	return angle;
}

Segment::Segment( Point *a, Point *b ) : a(a), b(b), length( Point::dist(a,b) ), isIntersecting(false), idx(-1), isFocus(false) {
	Real dx = b->original.x - a->original.x;
	Real dy = b->original.y - a->original.y;
	Real len = sqrt(dx*dx + dy*dy);
	nx = -(dy/len);
	ny = dx/len;
	requestedLength = length;
}
