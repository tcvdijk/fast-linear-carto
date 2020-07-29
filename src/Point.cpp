#include "Point.h"
#include "RoadNetwork.h"

#include <iostream>
#include <math.h>
using namespace std;


Real Point::dist( Point *a, Point *b ) {
	Real dx = b->original.x - a->original.x;
	Real dy = b->original.y - a->original.y;
	return sqrt( dx*dx + dy*dy );
}

void Point::considerEvent( const RoadNetwork::Event &e ) {
	if( !hasEvent || e.t < event.t ) setEvent( e );
}
void Point::setEvent( const RoadNetwork::Event &e ) {
	hasEvent = true;
	event.p = e.p;
	event.s = e.s;
	event.t = e.t;
	event.param = e.param;
}

Real Point::distortion() {
	Real stress = 0;
	for( auto si=segments.begin(); si!=segments.end(); ++si ) {
		Point *other = (*si)->oppositeOf(this);
		Real expectX = displaced.x + displaced.scale * (other->original.x-original.x);
		Real dx = other->displaced.x - expectX;
		Real expectY = displaced.y + displaced.scale * (other->original.y-original.y);
		Real dy = other->displaced.y - expectY;
		Real factor = 1.0 / (*si)->length;
		stress += factor * ( dx*dx + dy*dy );
	}
	return stress;
}

Real Point::optimalScale() {

	if( segments.empty() ) return 1;

	Real sumA2 = 0;
	Real sumAB = 0;
	for( auto si=segments.begin(); si!=segments.end(); ++si ) {
		Segment *s = *si;
		Point *other = s->oppositeOf(this);

		Real displacedDX = other->displaced.x - displaced.x;
		Real displacedDY = other->displaced.y - displaced.y;
		Real originalDX = other->original.x - original.x;
		Real originalDY = other->original.y - original.y;

		sumA2 += originalDX * originalDX;
		sumAB += displacedDX * originalDX;

		sumA2 += originalDY * originalDY;
		sumAB += displacedDY * originalDY;
	}

	return sumAB / sumA2;

}

bool operator==( const Point &p, const Point &q ) {
	return p.original.x==q.original.x && p.original.y==q.original.y;
}
bool operator<( const Point &p, const Point &q ) {
	// lexicographic order
	return p.original.x<q.original.x || (p.original.x==q.original.x && p.original.y<q.original.y);
}

ostream &operator<<( ostream &s, const Point &p ) {
	return s << "(" << p.original.x << "," << p.original.y << ")";
}
