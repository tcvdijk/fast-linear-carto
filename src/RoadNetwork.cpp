#include "shapelib/shapefil.h"

#include <algorithm>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>
#include <map>
#include <set>
#include <queue>
using namespace std;

#include "RoadNetwork.h"
#include "Point.h"
#include "Segment.h"
#include "Timer.h"
#include "Util.h"
// #include "sweepline.h"

#include "Eigen/Sparse"
using namespace Eigen;

#include "pugixml/pugixml.hpp"
using namespace pugi;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

RoadNetwork net;

void dfs( Point *p ) {
	// DFS from p; marks p->visited in connected component
	if( p->visited ) return;
	p->visited = true;
	for( int i=0; i<p->segments.size(); ++i ) {
		dfs( p->segments[i]->oppositeOf(p) );
	}
}

void bfs(Point *s, std::map<Point*, int> &distance) {
    for(Point *p : net.points)
        p->visited = false;
    std::queue<Point*> q;
    q.push(s);
    s->visited = true;
    distance.insert(std::make_pair(s, 0));
    while(q.size() > 0) {
        Point *p = q.front();
        q.pop();
        for(Segment *s : p->segments) {
            Point *p2 = s->oppositeOf(p);
            if(!p2->visited) {
                p2->visited = true;
                q.push(p2);
                distance.insert(std::make_pair(p2, distance[p]+1));
            }
        }
    }
}

namespace minCostMaxFlow {
    // Source: Stanford University ACM Team Notebook (2013-14)
    // http://stanford.edu/~liszt90/acm/notebook.html#file2
    // August 23, 2014

    // Implementation of min cost max flow algorithm using adjacency
    // matrix (Edmonds and Karp 1972).  This implementation keeps track of
    // forward and reverse edges separately (so you can set cap[i][j] !=
    // cap[j][i]).  For a regular max flow, set all edge costs to 0.
    //
    // Running time, O(|V|^2) cost per augmentation
    //     max flow:           O(|V|^3) augmentations
    //     min cost max flow:  O(|V|^4 * MAX_EDGE_COST) augmentations
    //
    // INPUT:
    //     - graph, constructed using AddEdge()
    //     - source
    //     - sink
    //
    // OUTPUT:
    //     - (maximum flow value, minimum cost value)
    //     - To obtain the actual flow, look at positive values only.
    typedef vector<int> VI;
    typedef vector<VI> VVI;
    typedef long long L;
    typedef vector<L> VL;
    typedef vector<VL> VVL;
    typedef pair<int, int> PII;
    typedef vector<PII> VPII;

    const L INF = numeric_limits<L>::max() / 4;

    struct MinCostMaxFlow {
      int N;
      VVL cap, flow, cost;
      VI found;
      VL dist, pi, width;
      VPII dad;

      MinCostMaxFlow(int N) :
        N(N), cap(N, VL(N)), flow(N, VL(N)), cost(N, VL(N)),
        found(N), dist(N), pi(N), width(N), dad(N) {}

      void AddEdge(int from, int to, L cap, L cost) {
        this->cap[from][to] = cap;
        this->cost[from][to] = cost;
      }

      void Relax(int s, int k, L cap, L cost, int dir) {
        L val = dist[s] + pi[s] - pi[k] + cost;
        if (cap && val < dist[k]) {
          dist[k] = val;
          dad[k] = make_pair(s, dir);
          width[k] = min(cap, width[s]);
        }
      }

      L Dijkstra(int s, int t) {
        fill(found.begin(), found.end(), false);
        fill(dist.begin(), dist.end(), INF);
        fill(width.begin(), width.end(), 0);
        dist[s] = 0;
        width[s] = INF;

        while (s != -1) {
          int best = -1;
          found[s] = true;
          for (int k = 0; k < N; k++) {
            if (found[k]) continue;
            Relax(s, k, cap[s][k] - flow[s][k], cost[s][k], 1);
            Relax(s, k, flow[k][s], -cost[k][s], -1);
            if (best == -1 || dist[k] < dist[best]) best = k;
          }
          s = best;
        }

        for (int k = 0; k < N; k++)
          pi[k] = min(pi[k] + dist[k], INF);
        return width[t];
      }

      pair<L, L> GetMaxFlow(int s, int t) {
        L totflow = 0, totcost = 0;
        while (L amt = Dijkstra(s, t)) {
          totflow += amt;
          for (int x = t; x != s; x = dad[x].first) {
            if (dad[x].second == 1) {
              flow[dad[x].first][x] += amt;
              totcost += amt * cost[dad[x].first][x];
            } else {
              flow[x][dad[x].first] -= amt;
              totcost -= amt * cost[x][dad[x].first];
            }
          }
        }
        return make_pair(totflow, totcost);
      }
    };
}

void RoadNetwork::loadShp( const std::string &fname ) {

	int numNodes = 0, numEdges = 0;
	minX = numeric_limits<Real>::max();
	maxX = numeric_limits<Real>::lowest();
	minY = numeric_limits<Real>::max();
	maxY = numeric_limits<Real>::lowest();

	SHPHandle hSHP = SHPOpen( fname.c_str(), "rb" );

	int numEntities, shapeType;
	double 	minBound[4], maxBound[4];
	SHPGetInfo( hSHP, &numEntities, &shapeType, minBound, maxBound );

	int numHits = 0;
	int numEntries = 0;
	int numZeroLengthSegmentsSkipped = 0;

	cout << "numEntities: " << numEntities << endl;
	for( int i=0; i<numEntities; ++i ) {
		SHPObject *s = SHPReadObject( hSHP, i );

		Point *previousPointInThisStroke = 0;
		for( int j=0; j<s->nVertices; ++j ) {
			++numEntries;
			double x = s->padfX[j];
			double y = s->padfY[j];
			Point p( x, y );
			PointMap::iterator existingPoint = pointMap.find( p );
			Point *currentPoint = 0;
			if( existingPoint==pointMap.end() ) {
				// point didn't exist yet
				currentPoint = new Point(x,y);
				if( x<minX ) minX = x;
				if( x>maxX ) maxX = x;
				if( y<minY ) minY = y;
				if( y>maxY ) maxY = y;
				currentPoint->idx = points.size();
				points.push_back( currentPoint );
				pointMap.insert( make_pair(*currentPoint,currentPoint) );
				++numNodes;
			} else {
				// existing point
				currentPoint = existingPoint->second;
				++numHits;
			}

			if( previousPointInThisStroke==currentPoint ) {
				++numZeroLengthSegmentsSkipped;
				continue;
			}

			if( previousPointInThisStroke ) {
				makeSegment( currentPoint, previousPointInThisStroke, i );
				++numEdges;
			}
			previousPointInThisStroke = currentPoint;

		}

		SHPDestroyObject( s );

	}

	cout << "Number of segments: " << segments.size() << endl;

	cout << "Original X range: " << minX << " to " << maxX << endl;
	cout << "Original Y range: " << minY << " to " << maxY << endl;

	//*
	minX = numeric_limits<Real>::max();
	maxX = numeric_limits<Real>::lowest();
	minY = numeric_limits<Real>::max();
	maxY = numeric_limits<Real>::lowest();
	Point *p = points[0];
	dfs(p);
	//*
	int conn = 0, disconn = 0;
	for( int i=0; i<points.size(); ++i ) {
		if( !points[i]->visited ) {
			cout << "Disconnected point: " << points[i]->original.x << ", " << points[i]->original.y << endl;
			++disconn;
		} else {
			++conn;
			if( points[i]->original.x<minX ) minX = points[i]->original.x;
			if( points[i]->original.x>maxX ) maxX = points[i]->original.x;
			if( points[i]->original.y<minY ) minY = points[i]->original.y;
			if( points[i]->original.y>maxY ) maxY = points[i]->original.y;
		}
	}
	cout << "Connected nodes: " << conn << endl;

	SHPClose( hSHP );

	midX = 0.5 * ( minX + maxX );
	midY = 0.5 * ( minY + maxY );
	width = maxX - minX;
	height = maxY - minY;
	halfsize = 0.5 * max(width, height);

	markIntersections(0);
	cout << intersections.size() << " intersections detected" << endl;
	replaceIntersections();

	// Sort segments in counter-clockwise order and calculate optimal angles
	for(Point *p : points) {
		sortSegments(p);
	}
    calculateOptimalAngles();

	// Calculate points distances
	for(Point *p : points) {
		bfs(p, netDistances[p]);
	}

	cout << "Number of points: " << points.size() << endl;
	cout << "Number of entries: " << numEntries << endl;
	cout << "Number of hits: " << numHits << endl;
	cout << "Number of zero-length segments skipped: " << numZeroLengthSegmentsSkipped << endl;

}

void RoadNetwork::loadGML( const std::string &fname ) {

	int numNodes = 0, numEdges = 0;
	minX = numeric_limits<Real>::max();
	maxX = numeric_limits<Real>::lowest();
	minY = numeric_limits<Real>::max();
	maxY = numeric_limits<Real>::lowest();

	cout << "Reading GML: " << fname << endl;

	auto result = doc.load_file(fname.c_str());
	cout << "Load result         : " << result.description() << endl;
	if(result.status == pugi::status_file_not_found)
		exit(EXIT_FAILURE);
	if( !result ) {
		return;
	}

	gmlFname = fname;

	struct Filenode {
		double x, y;
		string name;
		Filenode( double x, double y, const string &name ) : x(x), y(y), name(name) {}
	};
	typedef map<string,Point*> NodeDic;
	NodeDic dic;

	auto graph = doc.child("graphml").child("graph");
	for( auto node=graph.child("node"); node; node=node.next_sibling("node") ) {
		string id = node.attribute("id").as_string();
		double x, y;
		xml_node xmlnodeX, xmlnodeY;
		for( auto data=node.child("data"); data; data=data.next_sibling() ) {
			string key = data.attribute("key").as_string();
			if( key=="x" ) {
				x = data.text().as_double();
				xmlnodeX = data;
			}
			if( key=="y" ) {
				y = data.text().as_double();
				xmlnodeY = data;
			}
			if( key=="label" ) data.text().set( "No label for you!" );
		}
		if( x<minX ) minX = x;
		if( x>maxX ) maxX = x;
		if( y<minY ) minY = y;
		if( y>maxY ) maxY = y;
		Point *p = new Point(x,y);
		cout << "Making point " << x << " " << y << endl;
		p->xmlnodeX = xmlnodeX;
		p->xmlnodeY = xmlnodeY;
		p->idx = points.size();
		points.push_back( p );
		dic.insert( make_pair( id, p ) );
	}
	for( auto edge=graph.child("edge"); edge; edge=edge.next_sibling("edge") ) {
		string source = edge.attribute("source").as_string();
		string target = edge.attribute("target").as_string();
		Point *s = dic.find(source)->second;
		Point *t = dic.find(target)->second;
		Segment *seg = makeSegment( s, t, -1 );
		for(auto data = edge.child("data"); data; data = data.next_sibling("data")) {
			string key = data.attribute("key").as_string();
			seg->metroLines.insert(key);
			metroLines.insert(key);
		}
	}

	midX = 0.5 * ( minX + maxX );
	midY = 0.5 * ( minY + maxY );
	width = maxX - minX;
	height = maxY - minY;
	halfsize = 0.5 * max(width, height);

	markIntersections(0);
	cout << intersections.size() << " intersections detected" << endl;
	replaceIntersections();

	// Sort segments in counter-clockwise order and calculate optimal angles
	for(Point *p : points) {
		sortSegments(p);
	}
    calculateOptimalAngles();

	// Calculate points distances
	for(Point *p : points) {
		bfs(p, netDistances[p]);
	}

	cout << minX << " " << maxX << "   " << minY << " " << maxY << endl;
}

void RoadNetwork::writeGML() {
	// write GML. only works if network was loaded from GML
	cout << "Saving to " << ("new_"+gmlFname) << endl;
	for( int i=0; i<points.size(); ++i ) {
		Point *p = points[i];
		p->xmlnodeX.text().set( p->displaced.x );
		p->xmlnodeY.text().set( p->displaced.y );
	}
	doc.save_file( (gmlFname+"new.graphml").c_str() );
}

void RoadNetwork::loadIpe( const std::string &fname ) {

	int numNodes = 0, numEdges = 0;
	minX = numeric_limits<Real>::max();
	maxX = numeric_limits<Real>::lowest();
	minY = numeric_limits<Real>::max();
	maxY = numeric_limits<Real>::lowest();

	cout << "Reading Ipe: " << fname << endl;

	auto result = doc.load_file(fname.c_str());
	cout << "Load result         : " << result.description() << endl;
	if(result.status == pugi::status_file_not_found)
		exit(EXIT_FAILURE);
	if( !result ) {
		return;
	}

	auto page = doc.child("ipe").child("page");
	while(page) {
		auto path = page.child("path");
		while(path) {
			stringstream in(path.text().get());
			Real x0, y0, x1, y1;
			string s0, s1;
			in >> x0 >> y0 >> s0;
			in >> x1 >> y1 >> s1;
			Point *a = makePoint(x0, y0);
			Point *b = makePoint(x1, y1);
			makeSegment(a, b, -1);
			segmentTargetZooms.push_back(path.attribute("pen").as_double());
			path = path.next_sibling();
		}
		page = page.next_sibling();
	}

	midX = 0.5 * ( minX + maxX );
	midY = 0.5 * ( minY + maxY );
	width = maxX - minX;
	height = maxY - minY;
	halfsize = 0.5 * max(width, height);

	markIntersections(0);
	cout << intersections.size() << " intersections detected" << endl;
	replaceIntersections();

	// Sort segments in counter-clockwise order and calculate optimal angles
	for(Point *p : points) {
		sortSegments(p);
	}
    calculateOptimalAngles();

	// Calculate points distances
	for(Point *p : points) {
		bfs(p, netDistances[p]);
	}

	cout << minX << " " << maxX << "   " << minY << " " << maxY << endl;
}

Point* RoadNetwork::makePoint(Real x, Real y, bool intersectionPoint) {
	// Check if point already exists
	for(auto i = points.rbegin(); i != points.rend(); i++) {
		Point *p = *i;
		if(p->original.x == x && p->original.y == y)
			return p;
	}
	// Update min/max
	if( x<minX ) minX = x;
	if( x>maxX ) maxX = x;
	if( y<minY ) minY = y;
	if( y>maxY ) maxY = y;
	// Make point
	Point *p = new Point(x,y);
	cout << "Making point " << x << " " << y << endl;
	p->idx = points.size();
	p->intersectionPoint = intersectionPoint;
	points.push_back( p );
	// Return point
	return p;
}

Segment *RoadNetwork::makeSegment( Point* a, Point *b, int strokeID ) {
	Segment *s = new Segment( a, b );
	s->strokeID = strokeID;
	s->idx = segments.size();
	segments.push_back( s );
	a->segments.push_back( s );
	b->segments.push_back( s );
	return s;
}

void RoadNetwork::sortSegments(Point *p) {
	sort(p->segments.begin(), p->segments.end(), [p](Segment *s0, Segment *s1) {
		Point *s0b;
		Point *s1b;
		if(s0->a != p) s0b = s0->a;
		else s0b = s0->b;
		if(s1->a != p) s1b = s1->a;
		else s1b = s1->b;
		const Real dx0 = s0b->original.x - p->original.x;
		const Real dy0 = s0b->original.y - p->original.y;
		const Real dx1 = s1b->original.x - p->original.x;
		const Real dy1 = s1b->original.y - p->original.y;

		int q0, q1;
		if(dx0 >= 0) q0 = (dy0 >= 0) ? 0 : 3;
		else q0 = (dy0 >= 0) ? 1 : 2;
		if(dx1 >= 0) q1 = (dy1 >= 0) ? 0 : 3;
		else q1 = (dy1 >= 0) ? 1 : 2;

		if(q0 == q1) {
			if(dx0 == 0) return (dy0 > 0) ? false : true;
			if(dx1 == 0) return (dy1 > 0) ? true : false;

			const Real slope0 = dy0 / dx0;
			const Real slope1 = dy1 / dx1;
			return slope0 < slope1;
		}

		return q0 < q1;
	});
}

void RoadNetwork::calculateOptimalAngles() {
    std::map<Segment*, std::map<Point*, Real>> segmentAngleDict;
    for(Point *p : points) {
    	Real angleSum = 0;
    	const int n = p->segments.size();
    	for(int i = 0; i < n; i++) {
    		Segment *s = p->segments[i];
    		Point *b;
    		if(s->a != p) b = s->a;
    		else b = s->b;
            Real angle = fmod(atan2(b->original.y - p->original.y,
                b->original.x - p->original.x) + 2*M_PI, 2*M_PI);
    		angleSum += angle - i*(2*M_PI)/n;
    	}
    	Real optimalAngle = angleSum / n;
        if(optimalAngle < 0)
            optimalAngle = fmod(optimalAngle, 2*M_PI) + 2*M_PI;
    	for(Segment *s : p->segments) {
            segmentAngleDict[s][p] = optimalAngle;
    		optimalAngle = fmod(optimalAngle+(2*M_PI)/n, 2*M_PI);
    	}
    }
    for(auto segmentAngle : segmentAngleDict) {
        Segment *s = segmentAngle.first;
        std::map<Point*, Real> angleDict = segmentAngle.second;
        Real angle;
        if(s->b->segments.size() == 1)
            angle = angleDict[s->a];
        else if(s->a->segments.size() == 1)
            angle = fmod(angleDict[s->b] + M_PI, 2*M_PI);
        else {
            Real angleA = angleDict[s->a];
            Real angleB = fmod(angleDict[s->b] + M_PI, 2*M_PI);
            angle = (angleA + angleB) / 2;
            if(fabs(angleA - angle) > M_PI/2)
                angle = fmod(angle + M_PI, 2*M_PI);
        }
        s->optimalAngle = angle;
    }
}

void RoadNetwork::calculateOctilinearAngles() {
    std::map<Segment*, std::map<Point*, Real>> segmentAngleDict;
    for(Point *p : points) {
    	const int segCount = p->segments.size();
    	minCostMaxFlow::MinCostMaxFlow mcmf(segCount+10);
    	for(int i = 0; i < segCount; i++) {
    		Point *b = p->segments[i]->oppositeOf(p);
    		Real angle = fmod(atan2(b->displaced.y - p->displaced.y,
    			b->displaced.x - p->displaced.x)+2*M_PI,2*M_PI);
    		for(int j = 0; j < 8; j++) {
    			Real angleDiff = fabs(angle - j * (M_PI/4))*180/M_PI;
                angleDiff = (angleDiff < (360-angleDiff)) ? angleDiff : 360-angleDiff;
                angleDiff *= 100;
    			mcmf.AddEdge(i, segCount+j, 1, angleDiff*angleDiff);
    		}
    		mcmf.AddEdge(segCount+8, i, 1, 0);
    	}
    	for(int i = 0; i < 8; i++) {
    		mcmf.AddEdge(segCount+i, segCount+9, 1, 0);
    	}
    	pair<long long, long long> solution = mcmf.GetMaxFlow(segCount+8, segCount+9);
    	for(int i = 0; i < segCount; i++) {
            Segment *s = p->segments[i];
    		for(int j = 0; j < 8; j++) {
    			if(mcmf.flow[i][segCount+j] > 0) {
                    segmentAngleDict[s][p] = j * (M_PI/4);
    				break;
    			}
    		}
    	}
    }
    // int disagreements = 0;
    for(auto segmentAngle : segmentAngleDict) {
        Segment *s = segmentAngle.first;
        std::map<Point*, Real> angleDict = segmentAngle.second;
        Real angle;
        if(s->b->segments.size() == 1)
            angle = angleDict[s->a];
        else if(s->a->segments.size() == 1)
            angle = fmod(angleDict[s->b] + M_PI, 2*M_PI);
        else {
            Real angleA = angleDict[s->a];
            Real angleB = fmod(angleDict[s->b] + M_PI, 2*M_PI);
            // if(angleA != angleB)
            //     disagreements++;
            angle = (angleA + angleB) / 2;
            if(fabs(angleA - angle) > M_PI/2)
                angle = fmod(angle + M_PI, 2*M_PI);
        }
        s->octilinearAngle = angle;
    }
    // cout << "Disagreements: " << disagreements << "/" << segments.size() << endl;
}

Point *RoadNetwork::closestPoint( Real x, Real y ) {
	Point query( x, y );
	Real closestDist = numeric_limits<Real>::max();
	Point *closestPoint = 0;
	for( auto pi=points.begin(); pi!=points.end(); ++pi ) {
        Point p = Point((*pi)->displaced.x, (*pi)->displaced.y);
		Real dist = Point::dist( &query, &p );
		if( dist<closestDist ) {
			closestDist = dist;
			closestPoint = *pi;
		}
	}
	return closestPoint;
}

Segment *RoadNetwork::closestSegment(Point *p) {
	Real closestDist = numeric_limits<Real>::max();
	Segment *closestSegment;
	for(Segment *s : segments) {
		Real dist = s->distToDisplacedSegment(p, displacementTime);
		if(dist < closestDist) {
			closestDist = dist;
			closestSegment = s;
		}
	}
	return closestSegment;
}

void RoadNetwork::markIntersections( Real t ) {
	// cout << "Marking intersections... ";
	// Wue::Timer tim;
	rasterMarker( t );
	// tim.report();
}

void RoadNetwork::rasterMarker( Real t ) {
	// clear buffer
	for( int i=0; i<W*W; ++i ) {
		buffer[i].clear();
	}
	// draw segments into buffer
	for( auto i=segments.begin(); i!=segments.end(); ++i ) {
		Segment *s = *i;
		s->isIntersecting = false;
		s->testx1 = lerp( s->a->original.x, s->a->displaced.x, t );
		s->testy1 = lerp( s->a->original.y, s->a->displaced.y, t );
		s->testx2 = lerp( s->b->original.x, s->b->displaced.x, t );
		s->testy2 = lerp( s->b->original.y, s->b->displaced.y, t );
		drawLine( s->testx1, s->testy1, s->testx2, s->testy2, s );
	}
	// handle bins
	for( int i=0; i<W*W; ++i ) {
		handleBin( buffer[i] );
	}

}
void RoadNetwork::handleBin( const vector<Segment*> &segs ) {
	// quadratic-time brute force
	int n = segs.size();
	for( int i=0; i<n; ++i ) {
		for( int j=i+1; j<n; ++j ) {
			markSegSeg( segs[i], segs[j] );
		}
	}
}
void RoadNetwork::markSegSeg( Segment *a, Segment *b ) {
	// Check if segments are already in intersections
	for(auto inter : intersections)
		if((inter.a == a && inter.b == b) || (inter.b == a && inter.a == b)) return;

	// marks segments a and b as intersecting if they do
	if( a->a == b->a ) return;
	if( a->b == b->a ) return;
	if( a->a == b->b ) return;
	if( a->b == b->b ) return;

	if( find(a->allowedIntersections.begin(),a->allowedIntersections.end(), b)!=a->allowedIntersections.end() ) {
		return;
	}

	Real p0_x = a->testx1;
	Real p0_y = a->testy1;
	Real p1_x = a->testx2;
	Real p1_y = a->testy2;
	Real p2_x = b->testx1;
	Real p2_y = b->testy1;
	Real p3_x = b->testx2;
	Real p3_y = b->testy2;

    Real s02_x, s02_y, s10_x, s10_y, s32_x, s32_y, s_numer, t_numer, denom, t;
    s10_x = p1_x - p0_x;
    s10_y = p1_y - p0_y;
    s32_x = p3_x - p2_x;
    s32_y = p3_y - p2_y;

    denom = s10_x * s32_y - s32_x * s10_y;
    if (denom == 0)
        return; // Collinear
    bool denomPositive = denom > 0;

    s02_x = p0_x - p2_x;
    s02_y = p0_y - p2_y;
    s_numer = s10_x * s02_y - s10_y * s02_x;
    if ((s_numer < 0) == denomPositive)
        return; // No collision

    t_numer = s32_x * s02_y - s32_y * s02_x;
    if ((t_numer < 0) == denomPositive)
        return; // No collision

    if (((s_numer > denom) == denomPositive) || ((t_numer > denom) == denomPositive))
        return; // No collision

	a->isIntersecting = true;
	b->isIntersecting = true;
	intersections.push_back( RoadNetwork::Intersection( a, b ) );
}
void RoadNetwork::drawPixel( int x, int y, Segment *s ) {
	// put segment into buffer bin
	buffer[y*W+x].push_back(s);
}
void RoadNetwork::drawLine( Real x0, Real y0, Real x1, Real y1, Segment *s ) {
	// in the buffer: mark any bin that contains the line
	x0 = (W-2) * worldToScreenX(x0) + 1;
	y0 = (W-2) * worldToScreenY(y0) + 1;
	x1 = (W-2) * worldToScreenX(x1) + 1;
	y1 = (W-2) * worldToScreenY(y1) + 1;
	if (x0 == x1) {
		int ix = floor(x0);
		if (y0 < y1)
			for(int i = floor(y0); i <= y1; i++)
				drawPixel(ix, i, s);
		else
			for(int i = floor(y1); i <= y0; i++)
				drawPixel(ix, i, s);
	}
	else {
		if (x0 > x1) {
			// Switch points
			Real tmp = x0;
			x0 = x1;
			x1 = tmp;
			tmp = y0;
			y0 = y1;
			y1 = tmp;
		}
		Real m = (y0-y1)/(x0-x1);
		Real b = (x0*y1 - x1*y0)/(x0-x1);
		int ix0 = floor(x0);
		int iy0 = floor(y0);
		// Retain numerical errors
		if(fabs(m) < 1.0e-14) {
			for(int i = ix0; i <= x1; i++)
				drawPixel(i, iy0, s);
		}
		else if(y0 < y1) {
			Real y = m*(ix0+1)+b;
			while(y < y1) {
				for(int i = iy0; i <= y; i++)
					drawPixel(ix0, i, s);
				iy0 = floor(y);
				ix0++;
				y += m;
			}
			for(int i = iy0; i <= y1; i++)
				drawPixel(ix0, i, s);
		}
		else if(y0 > y1) {
			Real y = m*(ix0+1)+b;
			while(y > y1) {
				for(int i = iy0; i > y-1; i--) {
					drawPixel(ix0, i, s);
				}
				iy0 = floor(y);
				ix0++;
				y += m;
			}
			for(int i = iy0; i > y1-1; i--)
				drawPixel(ix0, i, s);
		}
	}
}

void RoadNetwork::replaceIntersections() {
	std::map<Segment*, map<Real, Point*>> segInterMap;
	for(auto inter : net.intersections) {
		// Calculate intersection point
		Point *p = inter.a->a;
		Point *p2 = inter.a->b;
		const Real rx = p2->original.x - p->original.x;
		const Real ry = p2->original.y - p->original.y;
		Point *q = inter.b->a;
		Point *q2 = inter.b->b;
		const Real sx = q2->original.x - q->original.x;
		const Real sy = q2->original.y - q->original.y;
		const Real denom = rx*sy - ry*sx;
		const Real dx = q->original.x - p->original.x;
		const Real dy = q->original.y - p->original.y;
		const Real t = (dx*sy - dy*sx) / denom;
		const Real x = p->original.x + t*rx;
		const Real y = p->original.y + t*ry;
		Point *interP = makePoint(x,y,true);
		const Real u = (dx*ry - dy*rx) / denom;

		segInterMap[inter.a].insert(std::make_pair(t, interP));
		segInterMap[inter.b].insert(std::make_pair(u, interP));
	}
	// Create new segments
	for(auto i : segInterMap) {
		Segment *s = i.first;
		Point *p = s->a;
		for(auto j : i.second) {
			Point *p2 = j.second;
			Segment *seg = makeSegment(p, p2, s->strokeID);
			seg->metroLines.insert(s->metroLines.begin(), s->metroLines.end());
			p = p2;
		}
		Segment *seg = makeSegment(p, s->b, s->strokeID);
		seg->metroLines.insert(s->metroLines.begin(), s->metroLines.end());
	}
	// Remove intersecting segments
	for(auto inter : net.intersections) {
		removeSegment(inter.a);
		removeSegment(inter.b);
	}
	net.intersections.clear();
}

void RoadNetwork::removeSegment(Segment *s) {
	for(auto iter = segments.begin(); iter != segments.end(); iter++) {
		if(*iter == s) {
			segments.erase(iter);
			break;
		}
	}
	for(auto iter = s->a->segments.begin(); iter != s->a->segments.end(); iter++) {
		if(*iter == s) {
			s->a->segments.erase(iter);
			break;
		}
	}
	for(auto iter = s->b->segments.begin(); iter != s->b->segments.end(); iter++) {
		if(*iter == s) {
			s->b->segments.erase(iter);
			break;
		}
	}
}

int RoadNetwork::xid( Point *p ) {
	return 2*p->idx;
}
int RoadNetwork::yid( Point *p ) {
	return 2*p->idx + 1;
}

void RoadNetwork::calculateDisplacement(Real cParFactor, Real cPerpFactor,
	bool resolveAngles) {

	int numNodes = points.size();
	const int varsPerPoint = 2;

	int numFixPoints = 1;
	int numSegments = segments.size();

	if(useProximityConstraints) {
		numSegments += proximitySegments.size();
	}

	int numInterEvents = 0;
	for(Point *p : points) {
		if( p->hasEvent ) {
			++numInterEvents;
		}
	}

    int numProxiEvents = events.size();

	int rowsPerSegment = 4;
	const int rowsPerFixPoint = 2;
    const int rowsPerInterEvent = oldIntersectionConstraints ? 4 : 2;
	const int rowsPerProxiEvent = 2;
	int numRows = rowsPerSegment*numSegments + rowsPerFixPoint*numFixPoints + \
				  rowsPerInterEvent*numInterEvents + rowsPerProxiEvent*numProxiEvents;
	const int nonZeroesPerSegment = 4;
	const int nonZeroesPerFixPoint = 1;
    const int nonZeroesPerInterEventRow = 6;
	const int nonZeroesPerProxiEventRow = 6;
	int numNonZeroes = rowsPerSegment*nonZeroesPerSegment*numSegments + \
					   rowsPerFixPoint*nonZeroesPerFixPoint*numFixPoints + \
                       rowsPerInterEvent*nonZeroesPerInterEventRow*numInterEvents + \
					   rowsPerProxiEvent*nonZeroesPerProxiEventRow*numProxiEvents;

	VectorXd rhs( numRows );

	//cout << "Setting up trips..." << endl;
	Wue::Timer tripsTimer;

	typedef Triplet<double> Trip;
	vector<Trip> trips;
	trips.reserve( numNonZeroes );
	int currentRow = 0;
	if(resolveAngles || octilinearResolution) {
        // Loop over segments
        for(Segment *s : segments) {
            Point *a = s->a;
            Point *b = s->b;
            Real ax = a->original.x;
            Real ay = a->original.y;
            Real bx, by;
            if(octilinearResolution) {
                bx = ax + cos(s->octilinearAngle) * s->length;
                by = ay + sin(s->octilinearAngle) * s->length;
                // Assign adjusted weights and scale them to make length errors
                // for edges equally as bad as for intersection/proximity constraints
                Real W = 1.13797 / 1.00390;
                cParFactor = 1.13797 / W;
                cPerpFactor = 15.2343 / W;
            }
            else {
                bx = ax + cos(s->optimalAngle) * s->length;
                by = ay + sin(s->optimalAngle) * s->length;
            }
            Real denominator = (bx-ax)*(bx-ax)+(by-ay)*(by-ay);
            Real fParXcx, fParXcy, fParXdx, fParXdy, \
                 fParYcx, fParYcy, fParYdx, fParYdy, \
                 fPerpXcx, fPerpXcy, fPerpXdx, fPerpXdy, \
                 fPerpYcx, fPerpYcy, fPerpYdx, fPerpYdy;
            fParXdx = fPerpYdy = (bx-ax)*(bx-ax)/denominator;
            fParXdy = fParYdx = fPerpXcy = fPerpYcx = (bx-ax)*(by-ay)/denominator;
            fParXcx = fPerpYcy = -fParXdx;
            fParXcy = fParYcx = fPerpXdy = fPerpYdx = -fParXdy;
            fParYdy = fPerpXdx = (by-ay)*(by-ay)/denominator;
            fParYcy = fPerpXcx = -fParYdy;

            const Real cPar = sqrt(cParFactor / s->requestedLength);
            // Par X
            trips.push_back(Trip(currentRow, xid(b), cPar * fParXdx));
            trips.push_back(Trip(currentRow, yid(b), cPar * fParXdy));
            trips.push_back(Trip(currentRow, xid(a), cPar * fParXcx));
            trips.push_back(Trip(currentRow, yid(a), cPar * fParXcy));
            rhs[currentRow++] = cPar * (s->targetZoom() * (bx-ax));
            // Par Y
            trips.push_back(Trip(currentRow, xid(b), cPar * fParYdx));
            trips.push_back(Trip(currentRow, yid(b), cPar * fParYdy));
            trips.push_back(Trip(currentRow, xid(a), cPar * fParYcx));
            trips.push_back(Trip(currentRow, yid(a), cPar * fParYcy));
            rhs[currentRow++] = cPar * (s->targetZoom() * (by-ay));

            const Real cPerp = sqrt(cPerpFactor / s->requestedLength);
            // Perp X
            trips.push_back(Trip(currentRow, xid(b), cPerp * fPerpXdx));
            trips.push_back(Trip(currentRow, yid(b), cPerp * fPerpXdy));
            trips.push_back(Trip(currentRow, xid(a), cPerp * fPerpXcx));
            trips.push_back(Trip(currentRow, yid(a), cPerp * fPerpXcy));
            rhs[currentRow++] = 0;
            // Perp Y
            trips.push_back(Trip(currentRow, xid(b), cPerp * fPerpYdx));
            trips.push_back(Trip(currentRow, yid(b), cPerp * fPerpYdy));
            trips.push_back(Trip(currentRow, xid(a), cPerp * fPerpYcx));
            trips.push_back(Trip(currentRow, yid(a), cPerp * fPerpYcy));
            rhs[currentRow++] = 0;
        }
	}
	else {
		// Loop over segments
		for(Segment *s : segments) {
			Point *a = s->a;
			Point *b = s->b;
			Real ax = a->original.x;
			Real ay = a->original.y;
			Real bx = b->original.x;
			Real by = b->original.y;
			Real denominator = (bx-ax)*(bx-ax)+(by-ay)*(by-ay);
			Real fParXcx, fParXcy, fParXdx, fParXdy, \
				 fParYcx, fParYcy, fParYdx, fParYdy, \
				 fPerpXcx, fPerpXcy, fPerpXdx, fPerpXdy, \
				 fPerpYcx, fPerpYcy, fPerpYdx, fPerpYdy;
			fParXdx = fPerpYdy = (bx-ax)*(bx-ax)/denominator;
			fParXdy = fParYdx = fPerpXcy = fPerpYcx = (bx-ax)*(by-ay)/denominator;
			fParXcx = fPerpYcy = -fParXdx;
			fParXcy = fParYcx = fPerpXdy = fPerpYdx = -fParXdy;
			fParYdy = fPerpXdx = (by-ay)*(by-ay)/denominator;
			fParYcy = fPerpXcx = -fParYdy;

			const Real cPar = sqrt(cParFactor / s->requestedLength);
			// Par X
			trips.push_back(Trip(currentRow, xid(b), cPar * fParXdx));
			trips.push_back(Trip(currentRow, yid(b), cPar * fParXdy));
			trips.push_back(Trip(currentRow, xid(a), cPar * fParXcx));
			trips.push_back(Trip(currentRow, yid(a), cPar * fParXcy));
			rhs[currentRow++] = cPar * (s->targetZoom() * (bx-ax));
			// Par Y
			trips.push_back(Trip(currentRow, xid(b), cPar * fParYdx));
			trips.push_back(Trip(currentRow, yid(b), cPar * fParYdy));
			trips.push_back(Trip(currentRow, xid(a), cPar * fParYcx));
			trips.push_back(Trip(currentRow, yid(a), cPar * fParYcy));
			rhs[currentRow++] = cPar * (s->targetZoom() * (by-ay));

			const Real cPerp = sqrt(cPerpFactor / s->requestedLength);
			// Perp X
			trips.push_back(Trip(currentRow, xid(b), cPerp * fPerpXdx));
			trips.push_back(Trip(currentRow, yid(b), cPerp * fPerpXdy));
			trips.push_back(Trip(currentRow, xid(a), cPerp * fPerpXcx));
			trips.push_back(Trip(currentRow, yid(a), cPerp * fPerpXcy));
			rhs[currentRow++] = 0;
			// Perp Y
			trips.push_back(Trip(currentRow, xid(b), cPerp * fPerpYdx));
			trips.push_back(Trip(currentRow, yid(b), cPerp * fPerpYdy));
			trips.push_back(Trip(currentRow, xid(a), cPerp * fPerpYcx));
			trips.push_back(Trip(currentRow, yid(a), cPerp * fPerpYcy));
			rhs[currentRow++] = 0;
		}
	}

	// Fix point
	Point *p = closestPoint(net.halfsize, net.halfsize);
	trips.push_back(Trip(currentRow, xid(p), 1));
	rhs[currentRow++] = p->original.x;
	trips.push_back(Trip(currentRow, yid(p), 1));
	rhs[currentRow++] = p->original.y;

	// Loop over events
	if(useIntersectionConstraints) {
        if(!oldIntersectionConstraints) {
    		for(Point *p : points) {
    			if(!p->hasEvent) continue;
                Segment *s = p->event.s;
                Real param = p->event.param;
                Real t = p->event.t;

                Real axt = lerp(s->a->original.x, s->a->displaced.x, 0.99*t);
                Real ayt = lerp(s->a->original.y, s->a->displaced.y, 0.99*t);
                Real bxt = lerp(s->b->original.x, s->b->displaced.x, 0.99*t);
                Real byt = lerp(s->b->original.y, s->b->displaced.y, 0.99*t);
                Real pxt = lerp(p->original.x, p->displaced.x, 0.99*t);
                Real pyt = lerp(p->original.y, p->displaced.y, 0.99*t);
                Real arcAP = atan2(pyt - ayt, pxt - axt);
                Real arcAB = atan2(byt - ayt, bxt - axt);
                Real sgn = (fmod(arcAP - arcAB + 2*M_PI, 2*M_PI) <= M_PI) ? 1 : -1;
                Real dxt = bxt - axt;
                Real dyt = byt - ayt;
                Real lengtht = sqrt(dxt*dxt + dyt*dyt);
                Real nx = -dyt/lengtht;
                Real ny = dxt/lengtht;

                Real bx = (1.0-param)*axt + param*bxt;
                Real by = (1.0-param)*ayt + param*byt;
                Real ax = bx + sgn*nx;
                Real ay = by + sgn*ny;
        		Real denominator = (bx-ax)*(bx-ax)+(by-ay)*(by-ay);
    			Real fParXcx, fParXcy, fParXdx, fParXdy, \
    				 fParYcx, fParYcy, fParYdx, fParYdy, \
    				 fPerpXcx, fPerpXcy, fPerpXdx, fPerpXdy, \
    				 fPerpYcx, fPerpYcy, fPerpYdx, fPerpYdy;
    			fParXdx = fPerpYdy = (bx-ax)*(bx-ax)/denominator;
    			fParXdy = fParYdx = fPerpXcy = fPerpYcx = (bx-ax)*(by-ay)/denominator;
    			fParXcx = fPerpYcy = -fParXdx;
    			fParXcy = fParYcx = fPerpXdy = fPerpYdx = -fParXdy;
    			fParYdy = fPerpXdx = (by-ay)*(by-ay)/denominator;
    			fParYcy = fPerpXcx = -fParYdy;

    	        const Real cPar = sqrt(1.00390 / minDistance);
    			// Par X
    			trips.push_back(Trip(currentRow, xid(s->a), cPar * (1.0-param) * fParXdx));
    			trips.push_back(Trip(currentRow, yid(s->a), cPar * (1.0-param) * fParXdy));
    			trips.push_back(Trip(currentRow, xid(s->b), cPar * param * fParXdx));
    			trips.push_back(Trip(currentRow, yid(s->b), cPar * param * fParXdy));
    			trips.push_back(Trip(currentRow, xid(p), cPar * fParXcx));
    			trips.push_back(Trip(currentRow, yid(p), cPar * fParXcy));
    			rhs[currentRow++] = cPar * minDistance*(bx-ax);
    			// Par Y
    			trips.push_back(Trip(currentRow, xid(s->a), cPar * (1.0-param) * fParYdx));
    			trips.push_back(Trip(currentRow, yid(s->a), cPar * (1.0-param) * fParYdy));
    			trips.push_back(Trip(currentRow, xid(s->b), cPar * param * fParYdx));
    			trips.push_back(Trip(currentRow, yid(s->b), cPar * param * fParYdy));
    			trips.push_back(Trip(currentRow, xid(p), cPar * fParYcx));
    			trips.push_back(Trip(currentRow, yid(p), cPar * fParYcy));
    			rhs[currentRow++] = cPar * minDistance*(by-ay);
    		}
        }
        else {
            for(Point *p : points) {
                if(!p->hasEvent) continue;
                Segment *s = p->event.s;
                Real param = p->event.param;

                Real ax = p->original.x;
                Real ay = p->original.y;
                Real bx = (1.0-param)*s->a->original.x + param*s->b->original.x;
                Real by = (1.0-param)*s->a->original.y + param*s->b->original.y;
                Real denominator = (bx-ax)*(bx-ax)+(by-ay)*(by-ay);
                Real fParXcx, fParXcy, fParXdx, fParXdy, \
                     fParYcx, fParYcy, fParYdx, fParYdy, \
                     fPerpXcx, fPerpXcy, fPerpXdx, fPerpXdy, \
                     fPerpYcx, fPerpYcy, fPerpYdx, fPerpYdy;
                fParXdx = fPerpYdy = (bx-ax)*(bx-ax)/denominator;
                fParXdy = fParYdx = fPerpXcy = fPerpYcx = (bx-ax)*(by-ay)/denominator;
                fParXcx = fPerpYcy = -fParXdx;
                fParXcy = fParYcx = fPerpXdy = fPerpYdx = -fParXdy;
                fParYdy = fPerpXdx = (by-ay)*(by-ay)/denominator;
                fParYcy = fPerpXcx = -fParYdy;

                const Real length = sqrt(denominator);

                const Real W = weights.events;
                const Real cPar = sqrt(1.00390 / s->requestedLength) * W;
                // Par X
                trips.push_back(Trip(currentRow, xid(s->a), cPar * (1.0-param) * fParXdx));
                trips.push_back(Trip(currentRow, yid(s->a), cPar * (1.0-param) * fParXdy));
                trips.push_back(Trip(currentRow, xid(s->b), cPar * param * fParXdx));
                trips.push_back(Trip(currentRow, yid(s->b), cPar * param * fParXdy));
                trips.push_back(Trip(currentRow, xid(p), cPar * fParXcx));
                trips.push_back(Trip(currentRow, yid(p), cPar * fParXcy));
                rhs[currentRow++] = cPar * 0.5*(bx-ax);
                // Par Y
                trips.push_back(Trip(currentRow, xid(s->a), cPar * (1.0-param) * fParYdx));
                trips.push_back(Trip(currentRow, yid(s->a), cPar * (1.0-param) * fParYdy));
                trips.push_back(Trip(currentRow, xid(s->b), cPar * param * fParYdx));
                trips.push_back(Trip(currentRow, yid(s->b), cPar * param * fParYdy));
                trips.push_back(Trip(currentRow, xid(p), cPar * fParYcx));
                trips.push_back(Trip(currentRow, yid(p), cPar * fParYcy));
                rhs[currentRow++] = cPar * 0.5*(by-ay);

                const Real cPerp = sqrt(0.413051 / s->requestedLength) * W;
                // Perp X
                trips.push_back(Trip(currentRow, xid(s->a), cPerp * (1.0-param) * fPerpXdx));
                trips.push_back(Trip(currentRow, yid(s->a), cPerp * (1.0-param) * fPerpXdy));
                trips.push_back(Trip(currentRow, xid(s->b), cPerp * param * fPerpXdx));
                trips.push_back(Trip(currentRow, yid(s->b), cPerp * param * fPerpXdy));
                trips.push_back(Trip(currentRow, xid(p), cPerp * fPerpXcx));
                trips.push_back(Trip(currentRow, yid(p), cPerp * fPerpXcy));
                rhs[currentRow++] = 0;
                // Perp Y
                trips.push_back(Trip(currentRow, xid(s->a), cPerp * (1.0-param) * fPerpYdx));
                trips.push_back(Trip(currentRow, yid(s->a), cPerp * (1.0-param) * fPerpYdy));
                trips.push_back(Trip(currentRow, xid(s->b), cPerp * param * fPerpYdx));
                trips.push_back(Trip(currentRow, yid(s->b), cPerp * param * fPerpYdy));
                trips.push_back(Trip(currentRow, xid(p), cPerp * fPerpYcx));
                trips.push_back(Trip(currentRow, yid(p), cPerp * fPerpYcy));
                rhs[currentRow++] = 0;
            }
        }
	}
	if(useProximityConstraints) {
		for(Event e : events) {
	        Point *p = e.p;
	        Segment *s = e.s;
	        Real param = e.param;
	        Real t = e.t;

	        Real axt = lerp(s->a->original.x, s->a->displaced.x, t);
	        Real ayt = lerp(s->a->original.y, s->a->displaced.y, t);
	        Real bxt = lerp(s->b->original.x, s->b->displaced.x, t);
	        Real byt = lerp(s->b->original.y, s->b->displaced.y, t);
	        Real bx = (1.0-param)*axt + param*bxt;
	        Real by = (1.0-param)*ayt + param*byt;
	        Real pxt = lerp(p->original.x, p->displaced.x, t);
	        Real pyt = lerp(p->original.y, p->displaced.y, t);
	        Real arcAP = atan2(pyt - ayt, pxt - axt);
	        Real arcAB = atan2(byt - ayt, bxt - axt);
	        Real sgn = (fmod(arcAP - arcAB + 2*M_PI, 2*M_PI) <= M_PI) ? 1 : -1;
	        Real dxt = bxt - axt;
	        Real dyt = byt - ayt;
	        Real lengtht = sqrt(dxt*dxt + dyt*dyt);
	        Real nx = -dyt/lengtht;
	        Real ny = dxt/lengtht;
	        Real ax = bx + sgn*nx;
	        Real ay = by + sgn*ny;
	        Real denominator = (bx-ax)*(bx-ax)+(by-ay)*(by-ay);
	        Real fParXcx, fParXcy, fParXdx, fParXdy, \
	             fParYcx, fParYcy, fParYdx, fParYdy, \
	             fPerpXcx, fPerpXcy, fPerpXdx, fPerpXdy, \
	             fPerpYcx, fPerpYcy, fPerpYdx, fPerpYdy;
	        fParXdx = fPerpYdy = (bx-ax)*(bx-ax)/denominator;
	        fParXdy = fParYdx = fPerpXcy = fPerpYcx = (bx-ax)*(by-ay)/denominator;
	        fParXcx = fPerpYcy = -fParXdx;
	        fParXcy = fParYcx = fPerpXdy = fPerpYdx = -fParXdy;
	        fParYdy = fPerpXdx = (by-ay)*(by-ay)/denominator;
	        fParYcy = fPerpXcx = -fParYdy;

	        const Real cPar = sqrt(1.00390 / minDistance);
	        // Par X
	        trips.push_back(Trip(currentRow, xid(s->a), cPar * (1.0-param) * fParXdx));
	        trips.push_back(Trip(currentRow, yid(s->a), cPar * (1.0-param) * fParXdy));
	        trips.push_back(Trip(currentRow, xid(s->b), cPar * param * fParXdx));
	        trips.push_back(Trip(currentRow, yid(s->b), cPar * param * fParXdy));
	        trips.push_back(Trip(currentRow, xid(p), cPar * fParXcx));
	        trips.push_back(Trip(currentRow, yid(p), cPar * fParXcy));
	        rhs[currentRow++] = cPar * minDistance*(bx-ax);
	        // Par Y
	        trips.push_back(Trip(currentRow, xid(s->a), cPar * (1.0-param) * fParYdx));
	        trips.push_back(Trip(currentRow, yid(s->a), cPar * (1.0-param) * fParYdy));
	        trips.push_back(Trip(currentRow, xid(s->b), cPar * param * fParYdx));
	        trips.push_back(Trip(currentRow, yid(s->b), cPar * param * fParYdy));
	        trips.push_back(Trip(currentRow, xid(p), cPar * fParYcx));
	        trips.push_back(Trip(currentRow, yid(p), cPar * fParYcy));
	        rhs[currentRow++] = cPar * minDistance*(by-ay);
	    }
	}

	// cout << currentRow << " == " << numRows << endl;

	//cout << "Constructing matrix..." << endl;
	Wue::Timer constructTimer;
	SparseMatrix< double > A( numRows, varsPerPoint*numNodes );
	A.setFromTriplets( trips.begin(), trips.end() );
	int maxnz = 0;

	//cout << "Setting up solver..." << endl;
	Wue::Timer setupTimer;
	SparseMatrix<double> AtA = A.transpose() * A;
	VectorXd Atb = A.transpose() * rhs;
	//setupTimer.report();

	//cout << "Solving..." << endl;
	Wue::Timer solveTimer;
	SimplicialLDLT<SparseMatrix<double> > solver(AtA);
	ComputationInfo info = solver.info();
	// cout << "Before solve: " << info << endl;
	VectorXd x = solver.solve( Atb );
	info = solver.info();
	// cout << "After solve: " << info << endl;
	VectorXd discrep = A*x - rhs;
	//solveTimer.report();

	//cout << "discrepancy: " << discrep.squaredNorm() << endl;

	// Rescale
	Real maxCoordX = numeric_limits<Real>::lowest();
	Real maxCoordY = numeric_limits<Real>::lowest();
	Real minCoordX = numeric_limits<Real>::max();
	Real minCoordY = numeric_limits<Real>::max();
	for(Point *p : points) {
		Real px = x[xid(p)];
		Real py = x[yid(p)];
		if(px > maxCoordX)
			maxCoordX = px;
		if(py > maxCoordY)
			maxCoordY = py;
		if(px < minCoordX)
			minCoordX = px;
		if(py < minCoordY)
			minCoordY = py;
	}

	// calculate scaleFactor
	Real xScale = 2*halfsize / (maxCoordX - minCoordX);
	Real yScale = 2*halfsize / (maxCoordY - minCoordY);
	scaleFactor = (xScale < yScale) ? xScale : yScale;

	// write back solution from least-squares x into the actual Points
	Real xOffset = (2*halfsize - (maxCoordX - minCoordX)*scaleFactor) / 2;
	xOffset += midX - halfsize;
	Real yOffset = (2*halfsize - (maxCoordY - minCoordY)*scaleFactor) / 2;
	yOffset += midY - halfsize;
	for(Point *p : points) {
		p->displaced.x = ((x[xid(p)] - minCoordX) * scaleFactor) + xOffset;
		p->displaced.y = ((x[yid(p)] - minCoordY) * scaleFactor) + yOffset;
	}
}

void RoadNetwork::calculateDisplacement_nlls(bool resolveAngles) {
	const Real cRadFactor = 1;
	const Real cArcFactor = 4 / (M_PI*M_PI);

	int numNodes = points.size();
	const int varsPerPoint = 2;

	int numFixPoints = 1;
	int numSegments = segments.size();

	int numEvents = 0;
	for(Point *p : points) {
		if( p->hasEvent ) {
			++numEvents;
		}
	}
	numEvents += events.size();

	int rowsPerSegment = 2;
	const int rowsPerFixPoint = 2;
	const int rowsPerEvent = 2;
	int numRows = rowsPerSegment*numSegments + rowsPerFixPoint*numFixPoints + \
				  rowsPerEvent*numEvents;
	const int nonZeroesPerSegment = 4;
	const int nonZeroesPerFixPoint = 1;
	const int nonZeroesPerEventRow = 6;
	int numNonZeroes = rowsPerSegment*nonZeroesPerSegment*numSegments + \
					   rowsPerFixPoint*nonZeroesPerFixPoint*numFixPoints + \
					   rowsPerEvent*nonZeroesPerEventRow*numEvents;

	VectorXd rhs( numRows );

	typedef Triplet<double> Trip;
	vector<Trip> trips;
	trips.reserve( numNonZeroes );
	int currentRow = 0;
	if(resolveAngles || octilinearResolution) {
        // Loop over segments
        for(Segment *s : segments) {
            Point *a = s->a;
            Point *b = s->b;
            Real ax = a->original.x;
            Real ay = a->original.y;
            Real bx = b->original.x;
            Real by = b->original.y;

			Real denominator = (bx-ax)*(bx-ax)+(by-ay)*(by-ay);
			Real length = sqrt(denominator);

			Real fRadAx = -(bx-ax) / length;
			Real fRadAy = -(by-ay) / length;
			Real fRadBx = -fRadAx;
			Real fRadBy = -fRadAy;
			Real fArcAx = (by-ay) / denominator;
			Real fArcAy = -(bx-ax) / denominator;
			Real fArcBx = -fArcAx;
			Real fArcBy = -fArcAy;

			const Real cRad = sqrt(cRadFactor / s->requestedLength);
			// Rad
			trips.push_back(Trip(currentRow, xid(b), cRad * fRadBx));
			trips.push_back(Trip(currentRow, yid(b), cRad * fRadBy));
			trips.push_back(Trip(currentRow, xid(a), cRad * fRadAx));
			trips.push_back(Trip(currentRow, yid(a), cRad * fRadAy));
			rhs[currentRow++] = cRad * (s->targetZoom()-1) * length;

			Real angleAB = fmod(atan2(by-ay, bx-ax) + 2*M_PI, 2*M_PI);
			Real angleDiff;
			if(octilinearResolution)
				angleDiff = s->octilinearAngle - angleAB;
			else
				angleDiff = s->optimalAngle - angleAB;
			if(angleDiff > M_PI)
				angleDiff -= 2*M_PI;
			else if(angleDiff < -M_PI)
				angleDiff += 2*M_PI;

			const Real cArc = sqrt(cArcFactor * s->requestedLength);
			// Arc
			trips.push_back(Trip(currentRow, xid(b), cArc * fArcBx));
			trips.push_back(Trip(currentRow, yid(b), cArc * fArcBy));
			trips.push_back(Trip(currentRow, xid(a), cArc * fArcAx));
			trips.push_back(Trip(currentRow, yid(a), cArc * fArcAy));
			rhs[currentRow++] = cArc * angleDiff;
		}
	}
	else {
		// Loop over segments
		for(Segment *s : segments) {
			Point *a = s->a;
			Point *b = s->b;
			Real ax = a->original.x;
			Real ay = a->original.y;
			Real bx = b->original.x;
			Real by = b->original.y;

			Real denominator = (bx-ax)*(bx-ax)+(by-ay)*(by-ay);
			Real length = sqrt(denominator);

			Real fRadAx = -(bx-ax) / length;
			Real fRadAy = -(by-ay) / length;
			Real fRadBx = -fRadAx;
			Real fRadBy = -fRadAy;
			Real fArcAx = (by-ay) / denominator;
			Real fArcAy = -(bx-ax) / denominator;
			Real fArcBx = -fArcAx;
			Real fArcBy = -fArcAy;

			const Real cRad = sqrt(cRadFactor / s->requestedLength);
			// Rad
			trips.push_back(Trip(currentRow, xid(b), cRad * fRadBx));
			trips.push_back(Trip(currentRow, yid(b), cRad * fRadBy));
			trips.push_back(Trip(currentRow, xid(a), cRad * fRadAx));
			trips.push_back(Trip(currentRow, yid(a), cRad * fRadAy));
			rhs[currentRow++] = cRad * (s->targetZoom()-1) * length;

			const Real cArc = sqrt(cArcFactor * s->requestedLength);
			// Arc
			trips.push_back(Trip(currentRow, xid(b), cArc * fArcBx));
			trips.push_back(Trip(currentRow, yid(b), cArc * fArcBy));
			trips.push_back(Trip(currentRow, xid(a), cArc * fArcAx));
			trips.push_back(Trip(currentRow, yid(a), cArc * fArcAy));
			rhs[currentRow++] = 0;
		}
	}

	// Fix point
	Point *p = closestPoint(net.halfsize, net.halfsize);
	trips.push_back(Trip(currentRow, xid(p), 1));
	rhs[currentRow++] = p->original.x;
	trips.push_back(Trip(currentRow, yid(p), 1));
	rhs[currentRow++] = p->original.y;

	// Loop over events
	if(useIntersectionConstraints) {
        if(!oldIntersectionConstraints) {
    		for(Point *p : points) {
    			if(!p->hasEvent) continue;
    			Segment *s = p->event.s;
    			Real param = p->event.param;

    			Real ax = p->original.x;
    			Real ay = p->original.y;
    			Real bx = (1.0-param)*s->a->original.x + param*s->b->original.x;
    			Real by = (1.0-param)*s->a->original.y + param*s->b->original.y;

    			Real denominator = (bx-ax)*(bx-ax)+(by-ay)*(by-ay);
    			Real length = sqrt(denominator);

    			Real fRadAx = -(bx-ax) / length;
    			Real fRadAy = -(by-ay) / length;
    			Real fRadBx = -fRadAx;
    			Real fRadBy = -fRadAy;
    			Real fArcAx = (by-ay) / denominator;
    			Real fArcAy = -(bx-ax) / denominator;
    			Real fArcBx = -fArcAx;
    			Real fArcBy = -fArcAy;

    			const Real cRad = sqrt(cRadFactor / s->requestedLength);
    			// Rad
    			trips.push_back(Trip(currentRow, xid(s->a), cRad * (1.0-param) * fRadBx));
    			trips.push_back(Trip(currentRow, yid(s->a), cRad * (1.0-param) * fRadBy));
    			trips.push_back(Trip(currentRow, xid(s->b), cRad * param * fRadBx));
    			trips.push_back(Trip(currentRow, yid(s->b), cRad * param * fRadBy));
    			trips.push_back(Trip(currentRow, xid(p), cRad * fRadAx));
    			trips.push_back(Trip(currentRow, yid(p), cRad * fRadAy));
    			rhs[currentRow++] = cRad * (minDistance - length);

    			const Real cArc = sqrt(cArcFactor * s->requestedLength);
    			// Arc
    			trips.push_back(Trip(currentRow, xid(s->a), cArc * (1.0-param) * fArcBx));
    			trips.push_back(Trip(currentRow, yid(s->a), cArc * (1.0-param) * fArcBy));
    			trips.push_back(Trip(currentRow, xid(s->b), cArc * param * fArcBx));
    			trips.push_back(Trip(currentRow, yid(s->b), cArc * param * fArcBy));
    			trips.push_back(Trip(currentRow, xid(p), cArc * fArcAx));
    			trips.push_back(Trip(currentRow, yid(p), cArc * fArcAy));
    			rhs[currentRow++] = 0;
    		}
        }
        else {
            for(Point *p : points) {
                if(!p->hasEvent) continue;
                Segment *s = p->event.s;
                Real param = p->event.param;

                Real ax = p->original.x;
                Real ay = p->original.y;
                Real bx = (1.0-param)*s->a->original.x + param*s->b->original.x;
                Real by = (1.0-param)*s->a->original.y + param*s->b->original.y;

                Real denominator = (bx-ax)*(bx-ax)+(by-ay)*(by-ay);
                Real length = sqrt(denominator);

                Real fRadAx = -(bx-ax) / length;
                Real fRadAy = -(by-ay) / length;
                Real fRadBx = -fRadAx;
                Real fRadBy = -fRadAy;
                Real fArcAx = (by-ay) / denominator;
                Real fArcAy = -(bx-ax) / denominator;
                Real fArcBx = -fArcAx;
                Real fArcBy = -fArcAy;

                const Real W = weights.events;
                const Real cRad = sqrt(cRadFactor / s->requestedLength) * W;
                // Rad
                trips.push_back(Trip(currentRow, xid(s->a), cRad * (1.0-param) * fRadBx));
                trips.push_back(Trip(currentRow, yid(s->a), cRad * (1.0-param) * fRadBy));
                trips.push_back(Trip(currentRow, xid(s->b), cRad * param * fRadBx));
                trips.push_back(Trip(currentRow, yid(s->b), cRad * param * fRadBy));
                trips.push_back(Trip(currentRow, xid(p), cRad * fRadAx));
                trips.push_back(Trip(currentRow, yid(p), cRad * fRadAy));
                rhs[currentRow++] = cRad * (-0.5*length);

                const Real cArc = sqrt(cArcFactor * s->requestedLength) * W;
                // Arc
                trips.push_back(Trip(currentRow, xid(s->a), cArc * (1.0-param) * fArcBx));
                trips.push_back(Trip(currentRow, yid(s->a), cArc * (1.0-param) * fArcBy));
                trips.push_back(Trip(currentRow, xid(s->b), cArc * param * fArcBx));
                trips.push_back(Trip(currentRow, yid(s->b), cArc * param * fArcBy));
                trips.push_back(Trip(currentRow, xid(p), cArc * fArcAx));
                trips.push_back(Trip(currentRow, yid(p), cArc * fArcAy));
                rhs[currentRow++] = 0;
            }
        }
	}
	if(useProximityConstraints) {
		for(Event e : events) {
	        Point *p = e.p;
	        Segment *s = e.s;
	        Real param = e.param;
	        Real t = e.t;

	        Real ax = p->original.x;
			Real ay = p->original.y;
			Real bx = (1.0-param)*s->a->original.x + param*s->b->original.x;
			Real by = (1.0-param)*s->a->original.y + param*s->b->original.y;

	        Real denominator = (bx-ax)*(bx-ax)+(by-ay)*(by-ay);
			Real length = sqrt(denominator);

			Real fRadAx = -(bx-ax) / length;
			Real fRadAy = -(by-ay) / length;
			Real fRadBx = -fRadAx;
			Real fRadBy = -fRadAy;
			Real fArcAx = (by-ay) / denominator;
			Real fArcAy = -(bx-ax) / denominator;
			Real fArcBx = -fArcAx;
			Real fArcBy = -fArcAy;

			const Real cRad = sqrt(cRadFactor / s->requestedLength);
			// Rad
			trips.push_back(Trip(currentRow, xid(s->a), cRad * (1.0-param) * fRadBx));
			trips.push_back(Trip(currentRow, yid(s->a), cRad * (1.0-param) * fRadBy));
			trips.push_back(Trip(currentRow, xid(s->b), cRad * param * fRadBx));
			trips.push_back(Trip(currentRow, yid(s->b), cRad * param * fRadBy));
			trips.push_back(Trip(currentRow, xid(p), cRad * fRadAx));
			trips.push_back(Trip(currentRow, yid(p), cRad * fRadAy));
			rhs[currentRow++] = cRad * (minDistance - length);

			Real axt = lerp(s->a->original.x, s->a->displaced.x, t);
	        Real ayt = lerp(s->a->original.y, s->a->displaced.y, t);
	        Real bxt = lerp(s->b->original.x, s->b->displaced.x, t);
	        Real byt = lerp(s->b->original.y, s->b->displaced.y, t);
	        Real pxt = lerp(p->original.x, p->displaced.x, t);
	        Real pyt = lerp(p->original.y, p->displaced.y, t);
	        Real angleAPt = atan2(pyt - ayt, pxt - axt);
	        Real angleABt = atan2(byt - ayt, bxt - axt);
	        Real sgn = (fmod(angleAPt - angleABt + 2*M_PI, 2*M_PI) <= M_PI) ? -1 : 1;
	        Real requestedAngle = fmod(angleABt + sgn*M_PI/2 + 4*M_PI, 2*M_PI);
			Real angleAB = fmod(atan2(by-ay, bx-ax) + 2*M_PI, 2*M_PI);
			Real angleDiff = requestedAngle - angleAB;
			if(angleDiff > M_PI)
				angleDiff -= 2*M_PI;
			else if(angleDiff < -M_PI)
				angleDiff += 2*M_PI;

			const Real cArc = sqrt(cArcFactor * s->requestedLength);
			// Arc
			trips.push_back(Trip(currentRow, xid(s->a), cArc * (1.0-param) * fArcBx));
			trips.push_back(Trip(currentRow, yid(s->a), cArc * (1.0-param) * fArcBy));
			trips.push_back(Trip(currentRow, xid(s->b), cArc * param * fArcBx));
			trips.push_back(Trip(currentRow, yid(s->b), cArc * param * fArcBy));
			trips.push_back(Trip(currentRow, xid(p), cArc * fArcAx));
			trips.push_back(Trip(currentRow, yid(p), cArc * fArcAy));
			rhs[currentRow++] = cArc * angleDiff;
	    }
	}

    // cout << currentRow << " == " << numRows << endl;

	SparseMatrix< double > A( numRows, varsPerPoint*numNodes );
	A.setFromTriplets( trips.begin(), trips.end() );

	SparseMatrix<double> AtA = A.transpose() * A;
	VectorXd Atb = A.transpose() * rhs;

	SimplicialLDLT<SparseMatrix<double> > solver(AtA);
	ComputationInfo info = solver.info();
	// cout << "Before solve: " << info << endl;
	VectorXd x = solver.solve( Atb );
	info = solver.info();
	// cout << "After solve: " << info << endl;

	// Rescale
	Real maxCoordX = numeric_limits<Real>::lowest();
	Real maxCoordY = numeric_limits<Real>::lowest();
	Real minCoordX = numeric_limits<Real>::max();
	Real minCoordY = numeric_limits<Real>::max();
	for(Point *p : points) {
		Real px = p->original.x + x[xid(p)];
		Real py = p->original.y + x[yid(p)];
		if(px > maxCoordX)
			maxCoordX = px;
		if(py > maxCoordY)
			maxCoordY = py;
		if(px < minCoordX)
			minCoordX = px;
		if(py < minCoordY)
			minCoordY = py;
	}

	// calculate scaleFactor
	Real xScale = 2*halfsize / (maxCoordX - minCoordX);
	Real yScale = 2*halfsize / (maxCoordY - minCoordY);
	scaleFactor = (xScale < yScale) ? xScale : yScale;

	// write back solution from least-squares x into the actual Points
	Real xOffset = (2*halfsize - (maxCoordX - minCoordX)*scaleFactor) / 2;
	xOffset += midX - halfsize;
	Real yOffset = (2*halfsize - (maxCoordY - minCoordY)*scaleFactor) / 2;
	yOffset += midY - halfsize;
	for(Point *p : points) {
		p->displaced.x = ((p->original.x + x[xid(p)] - minCoordX) * scaleFactor) + xOffset;
		p->displaced.y = ((p->original.y + x[yid(p)] - minCoordY) * scaleFactor) + yOffset;
	}
}

void RoadNetwork::calculateDisplacement_netDistance() {
	Real cParFactor = 1.00390;
	Real cPerpFactor = 0.413051;

	int numNodes = points.size();
	const int varsPerPoint = 2;

	int numFixPoints = 1;
	int numSegments = (numNodes-1)*numNodes/2;

	int rowsPerSegment = 4;
	const int rowsPerFixPoint = 2;
	int numRows = rowsPerSegment*numSegments + rowsPerFixPoint*numFixPoints;
	const int nonZeroesPerSegment = 4;
	const int nonZeroesPerFixPoint = 1;
	int numNonZeroes = rowsPerSegment*nonZeroesPerSegment*numSegments + \
					   rowsPerFixPoint*nonZeroesPerFixPoint*numFixPoints;

	VectorXd rhs( numRows );

	typedef Triplet<double> Trip;
	vector<Trip> trips;
	trips.reserve(numNonZeroes);
	int currentRow = 0;
	// Loop over segments
	for(int i = 0; i < points.size()-1; i++) {
		for(int j = i+1; j < points.size(); j++) {
			Point *a = points[i];
			Point *b = points[j];
			Real ax = a->original.x;
			Real ay = a->original.y;
			Real bx = b->original.x;
			Real by = b->original.y;
			Real denominator = (bx-ax)*(bx-ax)+(by-ay)*(by-ay);
			Real fParXcx, fParXcy, fParXdx, fParXdy, \
				 fParYcx, fParYcy, fParYdx, fParYdy, \
				 fPerpXcx, fPerpXcy, fPerpXdx, fPerpXdy, \
				 fPerpYcx, fPerpYcy, fPerpYdx, fPerpYdy;
			fParXdx = fPerpYdy = (bx-ax)*(bx-ax)/denominator;
			fParXdy = fParYdx = fPerpXcy = fPerpYcx = (bx-ax)*(by-ay)/denominator;
			fParXcx = fPerpYcy = -fParXdx;
			fParXcy = fParYcx = fPerpXdy = fPerpYdx = -fParXdy;
			fParYdy = fPerpXdx = (by-ay)*(by-ay)/denominator;
			fParYcy = fPerpXcx = -fParYdy;

			const Real requestedLength = (Real) netDistances[a][b];
			const Real targetZoom = requestedLength / Point::dist(a,b);

            Real W;
            if(netDistanceDisplacementWeighting == "constant") {
                W = 1;
            }
            else if(netDistanceDisplacementWeighting == "linear") {
                W = -0.05*requestedLength + 1.05;
                if(W < 0.05) W = 0.05;
            }
            else if(netDistanceDisplacementWeighting == "reciprocal") {
                W = sqrt(1.0 / requestedLength);
            }
            else if(netDistanceDisplacementWeighting == "delayed reciprocal") {
                W = sqrt(1.0 / (max(3.0, requestedLength) - 2.0));
            }
            else {
                W = sqrt(pow(1.0 / requestedLength, 3.0));
            }
            const Real cPar = sqrt(cParFactor / requestedLength) * W;
			// Par X
			trips.push_back(Trip(currentRow, xid(b), cPar * fParXdx));
			trips.push_back(Trip(currentRow, yid(b), cPar * fParXdy));
			trips.push_back(Trip(currentRow, xid(a), cPar * fParXcx));
			trips.push_back(Trip(currentRow, yid(a), cPar * fParXcy));
			rhs[currentRow++] = cPar * (targetZoom * (bx-ax));
			// Par Y
			trips.push_back(Trip(currentRow, xid(b), cPar * fParYdx));
			trips.push_back(Trip(currentRow, yid(b), cPar * fParYdy));
			trips.push_back(Trip(currentRow, xid(a), cPar * fParYcx));
			trips.push_back(Trip(currentRow, yid(a), cPar * fParYcy));
			rhs[currentRow++] = cPar * (targetZoom * (by-ay));

			const Real cPerp = sqrt(cPerpFactor / requestedLength) * W;
			// Perp X
			trips.push_back(Trip(currentRow, xid(b), cPerp * fPerpXdx));
			trips.push_back(Trip(currentRow, yid(b), cPerp * fPerpXdy));
			trips.push_back(Trip(currentRow, xid(a), cPerp * fPerpXcx));
			trips.push_back(Trip(currentRow, yid(a), cPerp * fPerpXcy));
			rhs[currentRow++] = 0;
			// Perp Y
			trips.push_back(Trip(currentRow, xid(b), cPerp * fPerpYdx));
			trips.push_back(Trip(currentRow, yid(b), cPerp * fPerpYdy));
			trips.push_back(Trip(currentRow, xid(a), cPerp * fPerpYcx));
			trips.push_back(Trip(currentRow, yid(a), cPerp * fPerpYcy));
			rhs[currentRow++] = 0;
		}
	}

	// Fix point
	Point *p = closestPoint(net.halfsize, net.halfsize);
	trips.push_back(Trip(currentRow, xid(p), 1));
	rhs[currentRow++] = p->original.x;
	trips.push_back(Trip(currentRow, yid(p), 1));
	rhs[currentRow++] = p->original.y;

	// cout << currentRow << " == " << numRows << endl;

	SparseMatrix< double > A( numRows, varsPerPoint*numNodes );
	A.setFromTriplets( trips.begin(), trips.end() );

	SparseMatrix<double> AtA = A.transpose() * A;
	VectorXd Atb = A.transpose() * rhs;

	SimplicialLDLT<SparseMatrix<double> > solver(AtA);
	ComputationInfo info = solver.info();
	// cout << "Before solve: " << info << endl;
	VectorXd x = solver.solve( Atb );
	info = solver.info();
	// cout << "After solve: " << info << endl;

	// Rescale
    Real maxCoordX = numeric_limits<Real>::lowest();
    Real maxCoordY = numeric_limits<Real>::lowest();
    Real minCoordX = numeric_limits<Real>::max();
    Real minCoordY = numeric_limits<Real>::max();
    for(Point *p : points) {
        Real px = x[xid(p)];
        Real py = x[yid(p)];
        if(px > maxCoordX)
            maxCoordX = px;
        if(py > maxCoordY)
            maxCoordY = py;
        if(px < minCoordX)
            minCoordX = px;
        if(py < minCoordY)
            minCoordY = py;
    }

    // calculate scaleFactor
    Real xScale = 2*halfsize / (maxCoordX - minCoordX);
    Real yScale = 2*halfsize / (maxCoordY - minCoordY);
    scaleFactor = (xScale < yScale) ? xScale : yScale;

    // write back solution from least-squares x into the actual Points
    Real xOffset = (2*halfsize - (maxCoordX - minCoordX)*scaleFactor) / 2;
    xOffset += midX - halfsize;
    Real yOffset = (2*halfsize - (maxCoordY - minCoordY)*scaleFactor) / 2;
    yOffset += midY - halfsize;
    for(Point *p : points) {
        p->displaced.x = ((x[xid(p)] - minCoordX) * scaleFactor) + xOffset;
        p->displaced.y = ((x[yid(p)] - minCoordY) * scaleFactor) + yOffset;
    }
}

Real RoadNetwork::determineDistortion() {
	Real distortion = 0;
	for( auto si=segments.begin(); si!=segments.end(); ++si ) {
		Segment *s = *si;
		distortion += s->distortion();
	}
	return distortion;
}

Real RoadNetwork::determineError(Real t, bool resolveAngles) {
	error = 0;
	maxArcError = maxRadError = avgArcError = avgRadError = 0;
	for(Segment* s : segments)
		error += segmentError(s, t);
	avgArcError /= segments.size();
	avgRadError /= segments.size();

    // std::vector<Real> x;
    // std::vector<Real> y;
    // for(Segment *s : segments) {
    //     x.push_back(s->requestedLength);
    //     Point *a = s->a;
    //     Point *b = s->b;
    //     Real ax = lerp(a->original.x, a->displaced.x, t);
    //     Real ay = lerp(a->original.y, a->displaced.y, t);
    //     Real bx = lerp(b->original.x, b->displaced.x, t);
    //     Real by = lerp(b->original.y, b->displaced.y, t);
    //     Point displacedA = Point(ax, ay);
    //     Point displacedB = Point(bx, by);
    //     Real length = Point::dist(&displacedA, &displacedB) / scaleFactor;
    //     y.push_back(length);
    // }
    // corr = correlation(x,y);

	return error;
}

Real RoadNetwork::segmentError(Segment *s, Real t, bool resolveAngles) {
	Point *a = s->a;
	Point *b = s->b;
	Real ax = lerp(a->original.x, a->displaced.x, t);
	Real ay = lerp(a->original.y, a->displaced.y, t);
	Real bx = lerp(b->original.x, b->displaced.x, t);
	Real by = lerp(b->original.y, b->displaced.y, t);

	Point displacedA = Point(ax, ay);
	Point displacedB = Point(bx, by);

    Real arcError;
    if(octilinearResolution || resolveAngles) {
        Real direction = atan2(by - ay, bx - ax);
        direction = fmod(direction + 2*M_PI, 2*M_PI);
        Real requestedAngle;
        if(octilinearResolution) {
            requestedAngle = s->octilinearAngle;
        }
        else {
            requestedAngle = s->optimalAngle;
        }
        Real dirDiff = fabs(direction - requestedAngle);
        if(dirDiff > M_PI)
            arcError = 2*M_PI - dirDiff;
        else
            arcError = dirDiff;
    }
    else
        arcError = s->arcOriginalDisplaced(t);

	// const Real arcError = s->arcOriginalDisplaced(t);
	const Real cArc = 4*s->requestedLength / (M_PI*M_PI);

    const Real length = Point::dist(&displacedA, &displacedB) / scaleFactor;
    const Real radError = length - s->requestedLength;
    const Real cRad = 1 / s->requestedLength;

	if(arcError > maxArcError) maxArcError = arcError;
	Real radErrorFactor = fabs(radError) / s->requestedLength;
	if(radErrorFactor > maxRadError) maxRadError = radErrorFactor;
	avgArcError += arcError;
	avgRadError += radErrorFactor;

	return cArc * arcError*arcError + cRad * radError*radError;
}

pair<Real,Real> RoadNetwork::preferredPlacement( Point *p ) {
	Real x = p->original.x;
	Real y = p->original.y;
	x -= focusX;
	y -= focusY;
	x *= settings.focusScaleTarget;
	y *= settings.focusScaleTarget;
	x += focusX;
	y += focusY;
	return make_pair( x, y );
}


Real RoadNetwork::getMinX() {
	return minX;
}
Real RoadNetwork::getMaxX() {
	return maxX;
}
Real RoadNetwork::getMinY() {
	return minY;
}
Real RoadNetwork::getMaxY() {
	return maxY;
}

Real RoadNetwork::screenToWorldX( Real x ) {
	x *= 2*halfsize;
	x += midX-halfsize;
	return x;
}
Real RoadNetwork::screenToWorldY( Real y ) {
	y *= 2*halfsize;
	y += midY-halfsize;
	return y;
}
Real RoadNetwork::worldToScreenX( Real x ) {
	x -= midX-halfsize;
	x /= 2*halfsize;
	return x;
}
Real RoadNetwork::worldToScreenY( Real y ) {
	y -= midY-halfsize;
	y /= 2*halfsize;
	return y;
}
Real RoadNetwork::screenToWorldScale( Real x ) {
	return 2*halfsize*x;
}
Real RoadNetwork::worldToScreenScale( Real x ) {
	x /= 2*halfsize;
	return x;
}

void RoadNetwork::clearEvents() {
	for( auto pi=points.begin(); pi!=points.end(); ++pi ) {
		Point *p = *pi;
		p->hasEvent = false;
	}
	clearProximityEvents();
}

void RoadNetwork::clearProximityEvents() {
	events.clear();
	for(Segment *s : segments)
		s->proximitySegments.clear();
	proximitySegments.clear();
}

void RoadNetwork::resetMap(bool clearEventsFlag) {
	for(Point *p : points) {
		p->current.x = p->displaced.x = p->original.x;
		p->current.y = p->displaced.y = p->original.y;
		p->current.scale = p->displaced.scale = 1;
		p->isFocus = false;
	}
	for(Segment *s : segments) {
		s->isFocus = false;
		s->requestedLength = s->length;
	}
	maxArcError = maxRadError = avgArcError = avgRadError = error = 0;
	scaleFactor = 1.0;
	if(clearEventsFlag) clearEvents();
}

int RoadNetwork::solve(Real cParFactor, Real cPerpFactor, bool resolveAngles,
	bool nllsDisplacement) {
	working = true;

	Real minRequestedLength = numeric_limits<Real>::max();

    for(Segment *s : segments) {
		if(s->requestedLength < minRequestedLength)
			minRequestedLength = s->requestedLength;
	}
	minDistance = minRequestedLength;

	Real timeProgress = 0;
	Real progressScale = 1;
	int maxIterations = 15;
	int stepCount = 0;
	int rewinds = 0;
	if(netDistanceDisplacement) {
		timeProgress = 1;
		stepCount = 1;
		calculateDisplacement_netDistance();
		displacementTime = 1;
	}
	else if(useIntersectionConstraints) {
        Real eventNumBefore;
		Real proximitySegmentsNumBefore;
		do {
            eventNumBefore = events.size();
			proximitySegmentsNumBefore = proximitySegments.size();
			rewinds += step(cParFactor, cPerpFactor, resolveAngles, nllsDisplacement);
			++stepCount;
			if(displacementTime > timeProgress) {
				timeProgress = displacementTime;
				progressScale = scaleFactor;
				for(Point *p : points) {
					p->cache.x = p->displaced.x;
					p->cache.y = p->displaced.y;
				}
			}
		} while((displacementTime<0.9
            || eventNumBefore != events.size())
			// || proximitySegmentsNumBefore != proximitySegments.size())
			&& stepCount<maxIterations);

		if(timeProgress > displacementTime) {
			displacementTime = timeProgress;
			scaleFactor = progressScale;
			for(Point *p : points) {
				p->displaced.x = p->cache.x;
				p->displaced.y = p->cache.y;
			}
		}
	}
	else {
		timeProgress = 1;
		stepCount = 1;
		if(nllsDisplacement)
			calculateDisplacement_nlls(resolveAngles);
		else
			calculateDisplacement(cParFactor, cPerpFactor, resolveAngles);
		displacementTime = 1;
	}

	working = false;
	// cout << "TIME! " << displacementTime << endl;
	// cout << "================================================================================" << endl;

	return stepCount;
}

int RoadNetwork::step(Real cParFactor, Real cPerpFactor, bool resolveAngles,
	bool nllsDisplacement) {
	bool done = false;
	if(nllsDisplacement)
		calculateDisplacement_nlls(resolveAngles);
	else
		calculateDisplacement(cParFactor, cPerpFactor, resolveAngles);

	Events tempEvents;
	displacementTime = 1;
	Real t = displacementTime;
	int maxIter = 100;
	int iter = 0;
	intersections.push_back(Intersection(0,0)); // dummy to get into loop; gets cleared
	Real previousT = 42; // invalid dummy value
	while(iter++ < maxIter && t > 0.01 && !intersections.empty()) {
		t = displacementTime * 0.99; // back off a little bit to prevent numerical problems
		if(t==previousT) break; // probably not needed because of previous line
		previousT = t;
		// cout << "t=" << displacementTime << endl;

		intersections.clear();
		markIntersections(t);

		tempEvents = newEvents;
		newEvents.clear();
		for(Intersection i : intersections)
			intersectionTime(i.a, i.b);
	}
	// select events to actually insert
	for(Event event : tempEvents)
		if(event.t < eventAcceptFactor * displacementTime)
			event.p->considerEvent(event);

	if(useProximityConstraints) {
		getProximityDataAndCreateEvents(displacementTime);
		
	}

	return iter;
}

void RoadNetwork::intersectionTime(Segment *a, Segment *b) {
	// finds earliest intersection time between segment a and b
	// if exists: pushes it into newEvents

	// try all combinations for a point and a segment

	//if( a->a==b->a || a->a==b->b || a->b==b->a || a->b==b->b ) return;
	intersectionPointSegment( a, b->a );
	intersectionPointSegment( a, b->b );
	intersectionPointSegment( b, a->a );
	intersectionPointSegment( b, a->b );
}

void RoadNetwork::intersectionPointSegment(Segment *s, Point *p) {
	// do some normalisation and pass on
	intersectionSegmentOrigin(
		s,
		p,
		-p->original.x      + s->a->original.x,
		-p->original.y      + s->a->original.y,
		-p->displaced.x    + s->a->displaced.x,
		-p->displaced.y    + s->a->displaced.y,
		-p->original.x      + s->b->original.x,
		-p->original.y      + s->b->original.y,
		-p->displaced.x    + s->b->displaced.x,
		-p->displaced.y    + s->b->displaced.y
	);
}

void RoadNetwork::intersectionSegmentOrigin(Segment *s, Point *p,
	const Real a1x, const Real a1y, const Real a2x, const Real a2y,
	const Real b1x, const Real b1y, const Real b2x,	const Real b2y) {
	// intersection time between a point and a segment
	// solve using quadratic equation

	const Real a = a1x;
	const Real b = a1y;
	const Real c = a2x;
	const Real d = a2y;
	const Real f = b1x;
	const Real g = b1y;
	const Real h = b2x;
	const Real i = b2y;

	// figure out coefficients, then solve using ABC-formula
	Real A = a*g - a*i - b*f + b*h - c*g + c*i + d*f - d*h;
	Real B = -2*a*g + a*i + 2*b*f - b*h + c*g - d*f;
	Real C = a*g - b*f;

	Real discr = B*B - 4*A*C;

	if(A==0) {
		if(B==0) {
			return;
		}

		Real t = -C/B;
		insertEventIfValid(s, p, a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y, t);
	}
	else if( discr>0 ) {
		Real sqrtDiscr = sqrt(discr);

		Real t1 = (-B + sqrtDiscr) /	(2*A);
		insertEventIfValid(s, p, a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y, t1);

		Real t2 = (-B - sqrtDiscr) /	(2*A);
		insertEventIfValid(s, p, a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y, t2);
	}
	else if(discr==0) {
		Real t = B / (2*A);
		insertEventIfValid(s, p, a1x, a1y, a2x, a2y, b1x, b1y, b2x, b2y, t);
	}
}

inline void RoadNetwork::insertEventIfValid(Segment *s, Point *p,
	const Real a1x, const Real a1y, const Real a2x, const Real a2y,
	const Real b1x,	const Real b1y,	const Real b2x,	const Real b2y,
	const Real t) {
	if(t > 0 && t < 1) {
		bool xvalid = valid(a1x, a2x, b1x, b2x, t);
		bool yvalid = valid(a1y, a2y, b1y, b2y, t);
		if( xvalid && yvalid ) {
			Real axt = lerp(a1x, a2x, t);
			Real ayt = lerp(a1y, a2y, t);
			Real bxt = lerp(b1x, b2x, t);
			Real byt = lerp(b1y, b2y, t);
			Real alen = sqrt(axt*axt + ayt*ayt);
			Real dx = bxt-axt;
			Real dy = byt-ayt;
			Real ablen = sqrt(dx*dx + dy*dy);
			Real param = alen / ablen;
			newEvents.push_back(Event(s, p, t, param));
			if(t < displacementTime) {
				displacementTime = t;
			}
		}
	}
}

bool RoadNetwork::valid(Real a1, Real a2, Real b1, Real b2, Real t) {
    Real at = lerp(a1, a2, t);
    Real bt = lerp(b1, b2, t);
    // Values smaller than epsilon are assumed to be 0 for numeric reasons
    Real epsilon = 1.0e-12;
    if(fabs(at) < epsilon && fabs(bt) < epsilon)
        return true;
	return (at < 0) ^ (bt < 0);
}

void RoadNetwork::getProximityDataAndCreateEvents(Real t) {
	// clearProximityEvents();
    int boxWindowSize = ceil(minDistance*scaleFactor / (2*halfsize/(W-2))) + 1;
    if (boxWindowSize > W)
    	boxWindowSize = W;
    for(int i = 0; i < W-boxWindowSize+1; i++) {
        for(int j = 0; j < W-boxWindowSize+1; j++) {
            std::set<Segment*> segs;
            for(int si = i; si < i + boxWindowSize; si++) {
                for(int sj = j; sj < j + boxWindowSize; sj++) {
                    segs.insert(buffer[sj*W+si].begin(), buffer[sj*W+si].end());
                }
            }
            createProximityEvents(segs, t);
        }
    }
}

void RoadNetwork::createProximityEvents(const std::set<Segment*> &segs, Real t) {
	for(int i = 0; i < segs.size(); i++) {
		for(int j = i+1; j < segs.size(); j++) {
            set<Segment*>::iterator it = segs.begin();
            std::advance(it, i);
			Segment *s0 = *it;
            std::advance(it, j-i);
			Segment *s1 = *it;

			if(std::find(s0->proximitySegments.begin(), s0->proximitySegments.end(), s1)
		    	!= s0->proximitySegments.end())
				continue;

			std::vector<std::pair<Point*, Segment*>> pairs;
			pairs.push_back(make_pair(s0->a, s1));
			pairs.push_back(make_pair(s0->b, s1));
			pairs.push_back(make_pair(s1->a, s0));
			pairs.push_back(make_pair(s1->b, s0));

            Real minStretch = numeric_limits<Real>::max();
			Real minLength = numeric_limits<Real>::max();
            int minIndex = -1;
			Real minParam = -1;
			for(int k = 0; k < pairs.size(); k++) {
				Point *p = pairs[k].first;
				Segment *s = pairs[k].second;
			    if(s->a == p || s->b == p)
			    	continue;
                if(p->hasEvent && p->event.s == s)
                    continue;
				Real length = pointToSegDistance(p, s, t, minParam);
                Real stretch = length/scaleFactor / min(netDistances[p][s->a]
                    + minParam, netDistances[p][s->b] + (1-minParam));
				if(stretch < minStretch) {
					minStretch = stretch;
                    minLength = length;
					minIndex = k;
				}
			}

			if(minIndex == -1) return;
			if(minStretch < 0.05 && minLength < minDistance*scaleFactor) {
				Point *p = pairs[minIndex].first;
				Segment *s = pairs[minIndex].second;
                events.push_back(Event(s, p, t, minParam));
                s0->proximitySegments.push_back(s1);
                s1->proximitySegments.push_back(s0);
			}
		}
	}
}

Real RoadNetwork::pointToSegDistance(Point *p, Segment *s, Real t, Real &param) {
	const Real x = lerp(p->original.x, p->displaced.x, t);
    const Real y = lerp(p->original.y, p->displaced.y, t);
    const Real ax = lerp(s->a->original.x, s->a->displaced.x, t);
    const Real ay = lerp(s->a->original.y, s->a->displaced.y, t);
    const Real bx = lerp(s->b->original.x, s->b->displaced.x, t);
    const Real by = lerp(s->b->original.y, s->b->displaced.y, t);
    Real dx = bx - ax;
    Real dy = by - ay;
    param = ((x - ax)*dx + (y - ay)*dy) / (dx*dx + dy*dy);
    Real Fx, Fy;
    if(param < 0) {
        param = 0;
        Fx = ax;
        Fy = ay;
    }
    else if(param > 1) {
        param = 1;
        Fx = bx;
        Fy = by;
    }
    else {
        Fx = ax + param*dx;
        Fy = ay + param*dy;
    }
    dx = Fx - x;
    dy = Fy - y;
    return sqrt(dx*dx + dy*dy);
}

void RoadNetwork::writeShp( const string &original_fname ) {

	string outfname = original_fname + "-out";
	SHPHandle hSHP = SHPCreate( outfname.c_str(), SHPT_ARC );
	DBFHandle hDBF = DBFCreate( outfname.c_str() );

	int scaleFID = DBFAddField( hDBF, "meaningOfLife", FTInteger, 3, 0 );

	for( auto si=segments.begin(); si!=segments.end(); ++si ) {

		Segment *s = *si;
		double xs[2];
		double ys[2];
		xs[0] = s->a->displaced.x;
		xs[1] = s->b->displaced.x;
		ys[0] = s->a->displaced.y;
		ys[1] = s->b->displaced.y;

		SHPObject *obj = SHPCreateSimpleObject( SHPT_ARC, 2, xs, ys, 0 );
		int entityID = SHPWriteObject( hSHP, -1, obj );
		SHPDestroyObject( obj );

		DBFWriteIntegerAttribute( hDBF, entityID, scaleFID, 42 );

	}

	SHPClose( hSHP );
	DBFClose( hDBF );

	string distfname = original_fname + "-scales";
	hSHP = SHPCreate( distfname.c_str(), SHPT_POINT );
	hDBF = DBFCreate( distfname.c_str() );

	scaleFID = DBFAddField( hDBF, "scale", FTDouble, 33, 16 );

	for( auto pi=points.begin(); pi!=points.end(); ++pi ) {

		Point *p = *pi;

		double xs[1];
		double ys[1];
		xs[0] = p->displaced.x;
		ys[0] = p->displaced.y;

		SHPObject *obj = SHPCreateSimpleObject( SHPT_POINT, 1, xs, ys, 0 );
		int entityID = SHPWriteObject( hSHP, -1, obj );
		SHPDestroyObject( obj );

		DBFWriteDoubleAttribute( hDBF, entityID, scaleFID, p->displaced.scale );

	}


	SHPClose( hSHP );
	DBFClose( hDBF );
}

void RoadNetwork::writeIpe(const string &original_fname, bool markFocusEdges) {
	stringstream fname;
	if(original_fname != "") {
		fname << original_fname << ".ipe";
	}
	else {
		static int frameNum = 1;
		fname << "frame";
		fname << setfill('0') << setw(5) << frameNum++;
		fname << ".ipe";
	}

	string outfname = fname.str();
	ofstream out(outfname);

	ifstream header("ipe_header.txt");
	string line;
	while(getline(header, line)) {
		out << line << "\n";
	}
	header.close();

	out << "<page>" << endl;
	out << "<layer name=\"original\"/>" << endl;
	out << "<layer name=\"displaced\"/>" << endl;
	out << "<layer name=\"maxTime\"/>" << endl;
	out << "<view layers=\"original\" active=\"original\"/>" << endl;
	out << "<view layers=\"displaced\" active=\"displaced\"/>" << endl;
	out << "<view layers=\"maxTime\" active=\"maxTime\"/>" << endl;

	Real scale = 400;
	Real offset = 10;

	for(Point *p : points) {
		p->cache.x = lerp(p->original.x, p->displaced.x, displacementTime);
		p->cache.y = lerp(p->original.y, p->displaced.y, displacementTime);
	}

	for(Segment *s : segments) {
		if(markFocusEdges && s->isFocus)
			out << "<path layer=\"original\" stroke=\"black\" pen=\"heavier\">" << endl;
		else
			out << "<path layer=\"original\" stroke=\"black\">" << endl;
		out << scale*worldToScreenX(s->a->original.x) + offset << " " << scale*worldToScreenY(s->a->original.y) + offset << " m " << endl;
		out << scale*worldToScreenX(s->b->original.x) + offset << " " << scale*worldToScreenY(s->b->original.y) + offset << " l" << endl;
		out << "</path>" << endl;
	}
	for(Point *p : points) {
		if(p->intersectionPoint) continue;
		out << "<use layer=\"original\" name=\"mark/fdisk(sfx)\" pos=\"";
		out << scale*worldToScreenX(p->original.x) + offset << " " << scale*worldToScreenY(p->original.y) + offset;
		out << "\" size=\"small\" stroke=\"black\" fill=\"white\"/>" << endl;
	}

	for(Segment *s : segments) {
		if(markFocusEdges && s->isFocus)
			out << "<path layer=\"displaced\" stroke=\"black\" pen=\"heavier\">" << endl;
		else
			out << "<path layer=\"displaced\" stroke=\"black\">" << endl;
		out << scale*worldToScreenX(s->a->cache.x) + offset << " " << scale*worldToScreenY(s->a->cache.y) + offset << " m ";
		out << scale*worldToScreenX(s->b->cache.x) + offset << " " << scale*worldToScreenY(s->b->cache.y) + offset << " l";
		out << "</path>" << endl;
	}
	for(Point *p : points) {
		if(p->intersectionPoint) continue;
		out << "<use layer=\"displaced\" name=\"mark/fdisk(sfx)\" pos=\"";
		out << scale*worldToScreenX(p->cache.x) + offset << " " << scale*worldToScreenY(p->cache.y) + offset;
		out << "\" size=\"small\" stroke=\"black\" fill=\"white\"/>" << endl;
	}

	for(Segment *s : segments) {
		if(markFocusEdges && s->isFocus)
			out << "<path layer=\"maxTime\" stroke=\"black\" pen=\"heavier\">" << endl;
		else
			out << "<path layer=\"maxTime\" stroke=\"black\">" << endl;
		out << scale*worldToScreenX(s->a->displaced.x) + offset << " " << scale*worldToScreenY(s->a->displaced.y) + offset << " m ";
		out << scale*worldToScreenX(s->b->displaced.x) + offset << " " << scale*worldToScreenY(s->b->displaced.y) + offset << " l";
		out << "</path>" << endl;
	}
	for(Point *p : points) {
		if(p->intersectionPoint) continue;
		out << "<use layer=\"maxTime\" name=\"mark/fdisk(sfx)\" pos=\"";
		out << scale*worldToScreenX(p->displaced.x) + offset << " " << scale*worldToScreenY(p->displaced.y) + offset;
		out << "\" size=\"small\" stroke=\"black\" fill=\"white\"/>" << endl;
	}

	out << "</page>" << endl;
	out << "</ipe>" << endl;
	out.close();
}
