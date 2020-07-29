#ifndef INCLUDED_ROADNETWORK
#define INCLUDED_ROADNETWORK

#include <vector>
#include <set>
#include <string>
#include <map>
#include <utility>

#include "pugixml/pugixml.hpp"

#include "Util.h"

class Point;
class Segment;

class RoadNetwork {
public:

	typedef std::vector<Point*> Points;
	Points points;

	typedef std::map<Point,Point*> PointMap;
	PointMap pointMap;

	typedef std::vector<Segment*> Segments;
	Segments segments;

	std::vector<Real> segmentTargetZooms;

	std::set<std::string> metroLines;

	//void sync( Real t );

	// === IO ========================================================
	void loadShp( const std::string &fname );
	void writeShp( const std::string &original_fname );

	void loadGML( const std::string &fname );
	void writeGML();
	pugi::xml_document doc;
	std::string gmlFname;

	void loadIpe(const std::string &fname);
	void writeIpe(const std::string &original_fname, bool markFocusEdges = false);
	// === IO ========================================================

	Point *closestPoint( Real x, Real y );
	Segment *closestSegment(Point *p);

	Point* makePoint(Real x, Real y, bool intersectionPoint = false);
	Segment *makeSegment( Point* a, Point *b, int strokeID );

	void sortSegments(Point *p);
	void calculateOptimalAngles();
	void calculateOctilinearAngles();
	bool octilinearResolution = false;

	void markIntersections( Real t );
	void replaceIntersections();
	void removeSegment(Segment *s);

	void rasterMarker( Real t );
	void handleBin( const std::vector<Segment*> &segs );
	void markSegSeg( Segment *a, Segment *b );
	void drawPixel( int x, int y, Segment *s );
	void drawLine( Real x0, Real y0, Real x1, Real y1, Segment *s );
	void drawLineInt( int x0, int y0, int x1, int y1, Segment *s );
	static const int W = 34;
	std::vector<Segment*> buffer[W*W];

	Real focusX, focusY;

	Real borderWidth;
	void calculateDisplacement(Real cParFactor, Real cPerpFactor,
		bool optimalAngles);
	void calculateDisplacement_nlls(bool resolveAngles);
	void calculateDisplacement_netDistance();
	std::string netDistanceDisplacementWeighting = "cubed reciprocal";
	int xid(Point*);
	int yid(Point*);
	int sid(Point*);
	Real displacementTime;

	Real determineDistortion();
	Real determineError(Real t, bool resolveAngles = false);
	Real segmentError(Segment *s, Real t, bool resolveAngles = false);
	Real error, maxArcError, maxRadError, avgArcError, avgRadError, corr;
	bool oldIntersectionConstraints = false;
	bool netDistanceDisplacement = false;
	std::map<Point*, std::map<Point*, int>> netDistances;
	std::vector<Segment*> proximitySegments;

	int numberOfEventsInModel;

	//Points consNonneg;

	struct Intersection {
		Segment *a, *b;
		Intersection( Segment *a, Segment *b ) : a(a), b(b) {}
	};
	typedef std::vector<Intersection> Intersections;
	Intersections intersections;

	struct Event {
		Segment *s;
		Point *p;
		Real t;
		Real param;
		Event() {}
		Event( Segment *s, Point *p, Real t, Real param ) : s(s), p(p), t(t), param(param) {}
	};
	typedef std::vector<Event> Events;
	Events events, newEvents;
	void clearEvents();
	void clearProximityEvents();

	void resetMap(bool clearEvents);

	bool working = false;
	Real eventAcceptFactor = 2;
	int solve(Real cParFactor = 1.00390, Real cPerpFactor = 0.413051,
		bool resolveAngles = false, bool nllsDisplacement = false);
	int step(Real cParFactor, Real cPerpFactor, bool resolveAngles,
		bool nllsDisplacement);
	void intersectionTime(Segment *a, Segment *b);
	void intersectionPointSegment(Segment *s, Point *p);
	void intersectionSegmentOrigin(Segment *s, Point *p,
		const Real a1x, const Real a1y, const Real a2x, const Real a2y,
		const Real b1x, const Real b1y, const Real b2x,	const Real b2y);
	inline void insertEventIfValid(Segment *s, Point *p,
		const Real a1x, const Real a1y, const Real a2x, const Real a2y,
		const Real b1x,	const Real b1y,	const Real b2x,	const Real b2y,
		const Real t);
	bool valid(Real a1, Real a2, Real b1, Real b2, Real t);

	bool useIntersectionConstraints = true;
	bool useProximityConstraints = false;
	void getProximityDataAndCreateEvents(Real t);
	void createProximityEvents(const std::set<Segment*> &segs, Real t);
	Real pointToSegDistance(Point *p, Segment *s, Real t, Real &param);

	std::pair<Real,Real> preferredPlacement(Point*);

	Real getMinX();
	Real getMaxX();
	Real getMinY();
	Real getMaxY();
	Real screenToWorldX( Real x );
	Real screenToWorldY( Real y );
	Real worldToScreenX( Real x );
	Real worldToScreenY( Real y );
	Real screenToWorldScale( Real x );
	Real worldToScreenScale( Real x );
	Real midX;
	Real midY;
	Real width;
	Real height;
	Real halfsize;
	Real scaleFactor;
	Real minDistance;

	struct Weights {
		Real edge;
		Real focusEdge;
		Real border;
		Real scale;
		Real nonneg;
		Real events;
		Real placement;
		Real movement;
		Weights() :
			edge(		5		),
			focusEdge(	10		),
			border(		0.52	),
			scale(		1		),
			nonneg(		1		),
			events(     10		),
			placement(	0		),
			movement(	0.0		) 
			{}
	} weights;
	struct Settings {
		Real borderFraction;
		Real minScaleTrigger;
		Real minScaleFixValue;
		Real focusScaleTarget;
		Settings() :
			borderFraction(		0.3		),
			minScaleTrigger(	0.1		),
			minScaleFixValue(	0.2		),
			focusScaleTarget(	2.5	)
			{}
	} settings;

	Real minX, maxX, minY, maxY;
};

extern RoadNetwork net;

#endif //ndef INCLUDED_ROADNETWORK
