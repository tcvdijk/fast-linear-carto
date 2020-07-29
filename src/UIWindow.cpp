#include <cmath>

#include "UIWindow.h"

#include <iostream>
#include <string>
#include <set>

#include "RoadNetwork.h"
#include "Segment.h"
#include "Point.h"

using namespace std;

void UIWindow::resetWeights() {
    cParFactor_ = 1.00390;
    cPerpFactor_ = 0.413051;
    controls->cParFactor(1.00390);
    controls->cPerpFactor(0.413051);
}

int UIWindow::handle(int event) {
    switch(event) {
        case FL_KEYUP:
            switch(Fl::event_key()) {
                case 114: // R
                    randomEdgeLengths();
                    return 1;

                case 113: // Q
                    if(net.segmentTargetZooms.size() > 0) {
                        setTargetZooms();
                        return 1;
                    }
                    else
                        return Fl_Window::handle(event);

                case 97: // A
                    randomStrokesCongestion();
                    return 1;

                case 122: // Z
                    randomPointCongestion();
                    return 1;

                case 121: // Y
                    netDistanceDisplacement();
                    return 1;

                case 110: // N
                    toggleNetDistanceDisplacementWeighting();
                    return 1;

                case 103: // G
                    if(net.metroLines.size() > 0) {
                        cyclicMetroLineCongestion();
                        return 1;
                    }
                    else
                        return Fl_Window::handle(event);

                case 117: // U
                    toggleNllsDisplacement();
                    return 1;

                case 106: // J
                    toggleIntersectionConstraints();
                    return 1;

                case 120: // X
                    map->dropNode(Fl::event_x(), Fl::event_y());
                    map->redraw_map();
                    return 1;

                case 115: // S
                    map->screenshot();
                    return 1;

                case 101: // E
                    map->showEvents(!map->showEvents());
                    cout << "Show events: " << (map->showEvents()?"true":"false") << endl;
                    map->redraw_map();
                    return 1;

                case 109: // M
                    map->showMaxTime(!map->showMaxTime());
                    cout << "Show max time: " << (map->showMaxTime()?"true":"false") << endl;
                    map->redraw_map();
                    return 1;

                case 100: // D
                    map->showDistortion(!map->showDistortion());
                    cout << "Show distortion: " << (map->showDistortion()?"true":"false") << endl;
                    map->redraw_map();
                    return 1;

                case 102: // F
                    map->showFocus(!map->showFocus());
                    cout << "Show focus: " << (map->showFocus()?"true":"false") << endl;
                    map->redraw_map();
                    return 1;

                case 111: // O
                    cout << "Writing output... ";
                    net.writeShp("output");
                    cout << "done." << endl;
                    return 1;

                case 112: // P
                    cout << "Writing to ipe... ";
                    net.writeIpe("output");
                    cout << "done." << endl;
                    return 1;

                case 108: // L
                    cout << "Writing to ipe... ";
                    net.writeIpe("output", true);
                    cout << "done." << endl;
                    return 1;

            }
            return Fl_Window::handle(event);

        case FL_KEYDOWN:
            switch(Fl::event_key()) {
                case 120: // X
                    if(map->pickedUpNode == nullptr) {
                        map->pickUpNode(Fl::event_x(), Fl::event_y());
                        return 1;
                    }
            }
            return Fl_Window::handle(event);

        default:
            return Fl_Window::handle(event);
    }
}

void UIWindow::randomEdgeLengths() {
    net.resetMap(true);
    for(Segment *s : net.segments)
        s->requestedLength = Real(double(rand())/double(RAND_MAX)*3.0 + 1.0) * s->length;
    oldSolve();

    for(Segment *s : net.segments) {
        const Real error = net.segmentError(s, net.displacementTime, resolveAngles());
        // Color blue if error bigger than 20% length error
        if (error > 0.04*s->requestedLength)
            s->isFocus = true;
    }

    map->randomEdgeLengthsAnimation();
}

void UIWindow::calculateMetromap() {
    net.resetMap(true);
    for(int i = 0; i < net.segments.size(); i++)
        net.segments[i]->requestedLength = 1;
    solve();

    for(Segment *s : net.segments) {
        const Real error = net.segmentError(s, net.displacementTime, resolveAngles());
        // Color blue if error bigger than 20% length error
        if (error > 0.04*s->requestedLength)
            s->isFocus = true;
    }

    map->calculateMetromapAnimation();
}

void UIWindow::setTargetZooms() {
    net.resetMap(true);
    for(int i = 0; i < net.segments.size(); i++)
        net.segments[i]->requestedLength = net.segmentTargetZooms[i] \
            * net.segments[i]->length;
    oldSolve();

    for(Segment *s : net.segments) {
        const Real error = net.segmentError(s, net.displacementTime, resolveAngles());
        // Color blue if error bigger than 20% length error
        if (error > 0.04*s->requestedLength)
            s->isFocus = true;
    }

    map->setTargetZoomsAnimation();
}

void UIWindow::randomStrokesCongestion() {
    net.resetMap(true);
    std::set<int> strokeIDs;
    for(Segment *s : net.segments) {
        if(s->strokeID != -1)
            strokeIDs.insert(s->strokeID);
    }
    if(strokeIDs.size() == 0) return;

    std::set<int> randomStrokeNumbers;
    while(randomStrokeNumbers.size() < int(strokeIDs.size()*0.02)) {
        randomStrokeNumbers.insert(rand()%strokeIDs.size());
    }
    for(int strokeNumber : randomStrokeNumbers) {
        std::set<int>::const_iterator it(strokeIDs.begin());
        advance(it, strokeNumber);
        int randomStrokeID = *it;
        for(Segment *s : net.segments) {
            if(s->strokeID == randomStrokeID) {
                s->requestedLength = 2 * s->length;
                s->isFocus = true;
            }
        }
    }
    cout << "Congesting " << randomStrokeNumbers.size() << " strokes..." << endl;
    oldSolve();

    map->randomStrokesCongestionAnimation();
}

void UIWindow::randomPointCongestion() {
    net.resetMap(true);
    int congestedSegs = 0;
    int iter = 0;
    while(congestedSegs == 0 && iter < 100) {
        iter++;
        Real x = double(rand())/double(RAND_MAX)*0.93 + 0.035;
        Real y = double(rand())/double(RAND_MAX)*0.93 + 0.035;
        for(Segment *s : net.segments) {
            Point *a = s->a;
            Point *b = s->b;
            a->cache.x = net.worldToScreenX(a->original.x);
            a->cache.y = net.worldToScreenY(a->original.y);
            b->cache.x = net.worldToScreenX(b->original.x);
            b->cache.y = net.worldToScreenY(b->original.y);
            Real dx = b->cache.x - a->cache.x;
            Real dy = b->cache.y - a->cache.y;
            Real delta = ((x - a->cache.x)*dx + (y - a->cache.y)*dy) / (dx*dx + dy*dy);
            Real Fx, Fy;
            if(delta < 0) {
                Fx = a->cache.x;
                Fy = a->cache.y;
            }
            else if(delta > 1) {
                Fx = b->cache.x;
                Fy = b->cache.y;
            }
            else {
                Fx = a->cache.x + delta*dx;
                Fy = a->cache.y + delta*dy;
            }
            dx = Fx - x;
            dy = Fy - y;
            Real length = sqrt(dx*dx + dy*dy);
            if(length <= 0.05) {
                s->requestedLength = 2 * s->length;
                s->isFocus = true;
                congestedSegs++;
            }
        }
    }
    cout << "Congesting " << congestedSegs << " segments..." << endl;
    oldSolve();

    map->randomPointCongestionAnimation();
}

void UIWindow::netDistanceDisplacement() {
    net.resetMap(true);
    for(int i = 0; i < net.segments.size(); i++)
        net.segments[i]->requestedLength = 1;
    net.netDistanceDisplacement = true;
    solve();
    net.netDistanceDisplacement = false;

    map->netDistanceDisplacementAnimation();
}

void UIWindow::toggleNetDistanceDisplacementWeighting() {
    if(net.netDistanceDisplacementWeighting == "cubed reciprocal")
        net.netDistanceDisplacementWeighting = "reciprocal";
    else if(net.netDistanceDisplacementWeighting == "reciprocal")
        net.netDistanceDisplacementWeighting = "delayed reciprocal";
    else if(net.netDistanceDisplacementWeighting == "delayed reciprocal")
        net.netDistanceDisplacementWeighting = "linear";
    else if(net.netDistanceDisplacementWeighting == "linear")
        net.netDistanceDisplacementWeighting = "constant";
    else if(net.netDistanceDisplacementWeighting == "constant")
        net.netDistanceDisplacementWeighting = "cubed reciprocal";
    else
        net.netDistanceDisplacementWeighting = "cubed reciprocal";
    cout << "Net distance weighting: " << net.netDistanceDisplacementWeighting << endl;
}

void UIWindow::cyclicMetroLineCongestion(bool reset) {
    static int lineID = 0;
    if(reset) {
        lineID = 0;
        return;
    }
    net.resetMap(true);
    set<string>::const_iterator iter = net.metroLines.begin();
    advance(iter, lineID);
    string metroLine = *iter;
    lineID = (lineID+1) % net.metroLines.size();

    for(Segment *s : net.segments) {
        if(s->metroLines.find(metroLine) != s->metroLines.end()) {
            s->requestedLength = 2;
            s->isFocus = true;
        }
        else
            s->requestedLength = 1;
    }

    cout << "Congesting line " << metroLine << "..." << endl;
    solve();

    map->cyclicMetroLineCongestionAnimation();
}

void UIWindow::toggleResolveAngles() {
    resolveAngles_ = !resolveAngles_;

    cout << "Resolve angles: " << (resolveAngles()?"true":"false") << endl;

    solve();

    map->toggleResolveAnglesAnimation();
}

void UIWindow::toggleOctilinearResolution() {
    octilinearResolution_ = !octilinearResolution_;
    net.octilinearResolution = octilinearResolution_;

    cout << "Octilinear resolution: " << (octilinearResolution()?"true":"false") << endl;

    solve(true);

    map->toggleResolveAnglesAnimation();
}

void UIWindow::toggleNllsDisplacement() {
    nllsDisplacement = !nllsDisplacement;

    cout << "NLLS displacement: " << (nllsDisplacement?"true":"false") << endl;

    solve();

    map->toggleNllsDisplacementAnimation();
}

void UIWindow::toggleIntersectionConstraints() {
    net.useIntersectionConstraints = !net.useIntersectionConstraints;

    cout << "Intersection constraints: " \
    << (net.useIntersectionConstraints?"true":"false") << endl;

    solve();

    map->toggleIntersectionConstraintsAnimation();
}

void UIWindow::toggleProximityConstraints() {
    net.useProximityConstraints = !net.useProximityConstraints;

    cout << "Proximity constraints: " \
    << (proximityConstraints()?"true":"false") << endl;

    solve();

    map->toggleProximityConstraintsAnimation();
}

void UIWindow::solve(bool octilinearToggle) {
    for(Point *p : net.points) {
        p->current.x = lerp(p->original.x, p->displaced.x, net.displacementTime);
        p->current.y = lerp(p->original.y, p->displaced.y, net.displacementTime);
        p->current.scale = p->displaced.scale;
    }

    if(!octilinearToggle && octilinearResolution()) {
        net.octilinearResolution = false;
        net.clearEvents();
        net.solve(cParFactor(), cPerpFactor(), resolveAngles(), nllsDisplacement);
        net.octilinearResolution = true;
    }
    if(octilinearResolution())
        net.calculateOctilinearAngles();
    net.clearEvents();
    net.solve(cParFactor(), cPerpFactor(), resolveAngles(), nllsDisplacement);
    net.determineError(net.displacementTime, resolveAngles());
    updateLabels();
}

void UIWindow::oldSolve() {
    net.oldIntersectionConstraints = true;
    proximityConstraints(false);
    solve();
    net.oldIntersectionConstraints = false;
}

void UIWindow::reset() {
    net.resetMap(true);
    resetWeights();
    resolveAngles(false);
    octilinearResolution(false);
    nllsDisplacement = false;
    net.useIntersectionConstraints = true;
    proximityConstraints(false);
    cyclicMetroLineCongestion(true);
    updateLabels();
    map->pickedUpNode = nullptr;
    map->redraw_map();
}

UIWindow::UIWindow(int X, int Y, int W, int H, const char *L)
    : Fl_Window(X, Y, W, H, L) {
    map = new UIMap(0, 0, w()-200, h());
    resizable(map);
    controls = new UIControls(map->w(), 0, 200, h());
    end();
    show();
}

void UIWindow::updateLabels() {
    controls->maxArcError(net.maxArcError);
    controls->maxRadError(net.maxRadError);
    controls->avgArcError(net.avgArcError);
    controls->avgRadError(net.avgRadError);
    controls->error(net.error);
    controls->scaleFactor(net.scaleFactor);
    controls->displacementTime(net.displacementTime);
}

void UIWindow::cParFactor(Real cPar) {
    cParFactor_ = cPar;
    solve();
    map->changeWeightsAnimation();
}

void UIWindow::cPerpFactor(Real cPerp) {
    cPerpFactor_ = cPerp;
    solve();
    map->changeWeightsAnimation();
}
