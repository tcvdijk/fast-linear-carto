#include <iostream>
#include <sstream>

#include <FL/Fl.H>
#include <Fl/fl_ask.H>

#include "RoadNetwork.h"
#include "Segment.h"
#include "UIWindow.h"

using namespace std;

int main(int argc, char **argv) {
	// Read commandline arguments if present
	if(argc < 2) {
		fl_alert("Usage: main.exe [path-to-map-data]");
		return 0;
	}
	stringstream args(argv[1]);
	string fname;
	args >> fname;
	cout << "Reading file: " << fname << endl;
	string ext = fname.substr(fname.find_last_of(".") + 1);
	cout << ext << endl;
	if(ext == "ipe") {
		net.loadIpe(fname);
	}
	else if(ext == "shp") {
		net.loadShp(fname);
	}
	else {
		net.loadGML(fname);
	}
	net.scaleFactor = 1.0;
	net.borderWidth = 0.0;
	net.displacementTime = 1;

	// Set up window
	int windowSize = (Fl::w() < Fl::h()-50) ? Fl::w() : Fl::h()-50;
	UIWindow *window = new UIWindow(0, 0, windowSize+200, windowSize, "Cartogram Window");

    return Fl::run();
}
