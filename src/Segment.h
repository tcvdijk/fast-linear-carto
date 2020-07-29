#ifndef INCLUDED_SEGMENT
#define INCLUDED_SEGMENT

#include <vector>
#include <set>
#include <string>

#include "RoadNetwork.h"
#include "Util.h"

class Point;

class Segment {
public:

	Point *a, *b;
	Real length;
	Real nx, ny; // normal

	std::vector<Segment*> allowedIntersections;

	std::vector<Segment*> proximitySegments;

	std::set<std::string> metroLines;

	Real testx1, testy1, testx2, testy2; // cached interpolated position

	bool isIntersecting;

	int strokeID;

	int idx;

	bool isFocus;
	double requestedLength;

	Real optimalAngle;
	Real octilinearAngle;

	Point *oppositeOf( Point *p );

	Real targetZoom();

	Real distortion();

	Real signedLineDist( Real x, Real y );

	Real distToDisplacedSegment(Point *p, Real t);
	Real arcOriginalDisplaced(Real t);

	Segment( Point *a, Point *b );

};

#endif //ndef INCLUDED_SEGMENT
