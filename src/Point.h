#ifndef INCLUDED_POINT
#define INCLUDED_POINT

#include <vector>

#include "Util.h"
#include "Segment.h"

#include "pugixml/pugixml.hpp"

class Point {
public:
	struct {
		Real x, y;
	} original;
	struct {
		Real x, y;
		Real scale;
	} displaced;

	struct {
		Real x, y;
		Real scale;
	} current;

	struct {
		Real x, y;
	} cache;

	pugi::xml_node xmlnodeX, xmlnodeY;

	double targetZoom;

	int idx;
	bool intersectionPoint;

	bool visited;

	bool isFocus;

	bool hasEvent;
	RoadNetwork::Event event;
	void considerEvent( const RoadNetwork::Event &e );
	void setEvent( const RoadNetwork::Event &e );

	typedef std::vector<Segment*> Segments;
	Segments segments;

	Real distortion();
	Real optimalScale();

	static Real dist( Point *a, Point *b );

	Point( Real x, Real y ) : isFocus(false), hasEvent(false), visited(false),
		targetZoom(-1), intersectionPoint(false) {
		original.x = current.x = displaced.x = x;
		original.y = current.y = displaced.y = y;
		current.scale = displaced.scale = 1;
	}
};

bool operator==( const Point &p, const Point &q );

// lexicographic order
bool operator<( const Point &p, const Point &q );

std::ostream &operator<<( std::ostream &s, const Point &p );

#endif //ndef INCLUDED_POINT
