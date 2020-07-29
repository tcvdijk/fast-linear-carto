#include "sweepline.h"
#include "Segment.h"
#include "Timer.h"

#include <list>
#include <map>
#include <memory>
#include <vector>
#include <iostream>
using namespace std;


//==================================================================================================================================================================================

#include <CGAL/Sweep_line_2_algorithms.h>
#include <CGAL/Sweep_line_2/Sweep_line_event.h>
#include <CGAL/Sweep_line_2/Sweep_line_subcurve.h>
#include <CGAL/Sweep_line_2/Sweep_line_2_utils.h>
#include <CGAL/Sweep_line_empty_visitor.h>
using namespace CGAL;





template< typename Traits_ >
class MyVisitor : public Sweep_line_empty_visitor<Traits_> {
public:

	typedef Traits_                              MyTraits_2;
	typedef MyVisitor<MyTraits_2>                  Self;
	typedef Sweep_line_2<MyTraits_2, Self>   Sweep_line_2;

	typename const MyTraits_2::X_monotone_curve_2 *originalSegs;

	MyVisitor( typename const MyTraits_2::X_monotone_curve_2 *originalSegs ) : originalSegs(originalSegs) {}
	
	template< class CurveIterator >
	void sweep( CurveIterator begin, CurveIterator end ) {
		// jump through some hoops to start the sweep
		Sweep_line_2* sl = reinterpret_cast<Sweep_line_2*>(this->sweep_line());
		sl->sweep( begin, end );
	}

	void update_event (Event* e, const Point_2& end_point, const X_monotone_curve_2& cv, Arr_curve_end cv_end, bool is_new ) {}
	void update_event( Event*, Subcurve* ) {} // this method is only for infinite curves? we don't have those, so ignore.
	void update_event (Event* e, Subcurve* sc1, Subcurve* sc2, bool is_new ) {
		// if event is intersection, mark both segments as intersecting
		if( e->is_intersection() ) {
			Segment *s1 = translate( sc1 );
			if( s1 ) s1->isIntersecting = true;
			Segment *s2 = translate( sc2 );
			if( s2 ) s2->isIntersecting = true;
			if( s1!=0 && s2!=0 ) {
				net.intersections.push_back( RoadNetwork::Intersection( s1, s2, MyPoint_2() ) );
			}
		}		
	}

	Segment *translate( Subcurve *sc ) {
		int offset = (sc->get_original() - originalSegs);
		if( offset<0 || offset>=net.cgalSegments.size() ) return 0;
		else return net.segments[ offset ];
	}
	
};


//==================================================================================================================================================================================

void testSweepline( vector<Segment*> &segs ) {
	MyTraits_2 traits;
	typedef MyVisitor<MyTraits_2> Visitor;
	typedef Sweep_line_2< MyTraits_2, MyVisitor<MyTraits_2> > Sweep_line;
	Visitor visitor( net.cgalSegments.data() );
	Sweep_line sweep_line( &traits, &visitor );
	visitor.sweep( net.cgalSegments.begin(), net.cgalSegments.end() );

}
