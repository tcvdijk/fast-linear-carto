#include <iostream>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <iomanip>
#include <cmath>
#include <string>

#include "RoadNetwork.h"
#include "Segment.h"
#include "Point.h"
#include "Timer.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace std;

string mapname = "";
string mode = "";
Real cParFactor = 1.00390;
Real cPerpFactor = 0.413051;
bool resolveAngles = false;
bool nllsDisplacement = false;

void reset() {
    cParFactor = 1.00390;
    cPerpFactor = 0.413051;
    resolveAngles = false;
    nllsDisplacement = false;
    net.oldIntersectionConstraints = false;
    net.useIntersectionConstraints = true;
    net.octilinearResolution = false;
    net.useProximityConstraints = false;
}

void displayProgress(int currIter, int maxIter) {
    cout << "\rProgress: " << currIter << "/" << maxIter;
}

void solve(bool maxTime = false) {
    net.clearEvents();
    net.solve(cParFactor, cPerpFactor, resolveAngles, nllsDisplacement);
    if(maxTime)
        net.determineError(1, resolveAngles);
    else
        net.determineError(net.displacementTime, resolveAngles);
}

void calculateMetromap() {
    net.resetMap(true);
    for(int i = 0; i < net.segments.size(); i++)
        net.segments[i]->requestedLength = 1;
}

void setRandomEdgeLengths() {
    net.resetMap(true);
    for(Segment *s : net.segments) {
        s->requestedLength = Real(double(rand())/double(RAND_MAX)*3.0 + 1.0) * s->length;
    }
}

void setRandomStrokesCongestion(std::vector<Segment*> &segs) {
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
                segs.push_back(s);
            }
        }
    }
}

void setRandomPointCongestion(std::vector<Segment*> &segs) {
    net.resetMap(true);
    int iter = 0;
    while(segs.size() < 10 && iter < 100) {
        iter++;
        segs.clear();
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
                segs.push_back(s);
            }
        }
    }
}

void setCyclicMetroLineCongestion() {
    static int lineID = 0;
    net.resetMap(true);
    set<string>::const_iterator iter = net.metroLines.begin();
    advance(iter, lineID);
    string metroLine = *iter;
    lineID = (lineID+1) % net.metroLines.size();
    for(Segment *s : net.segments) {
        if(s->metroLines.find(metroLine) != s->metroLines.end())
            s->requestedLength = 2;
        else
            s->requestedLength = 1;
    }
}

Real calculateCorrelation(std::vector<Segment*> segs, Real t) {
    std::vector<Real> x;
    std::vector<Real> y;
    for(Segment *s : segs) {
        x.push_back(s->requestedLength);
        Point *a = s->a;
        Point *b = s->b;
        Real ax = lerp(a->original.x, a->displaced.x, t);
        Real ay = lerp(a->original.y, a->displaced.y, t);
        Real bx = lerp(b->original.x, b->displaced.x, t);
        Real by = lerp(b->original.y, b->displaced.y, t);
        Point displacedA = Point(ax, ay);
        Point displacedB = Point(bx, by);
        Real length = Point::dist(&displacedA, &displacedB) / net.scaleFactor;
        y.push_back(length);
    }
    return correlation(x,y);
}

pair<Real, Real> segmentErrors(Segment *s, Real t) {
    Point *a = s->a;
    Point *b = s->b;
    Real ax = lerp(a->original.x, a->displaced.x, t);
    Real ay = lerp(a->original.y, a->displaced.y, t);
    Real bx = lerp(b->original.x, b->displaced.x, t);
    Real by = lerp(b->original.y, b->displaced.y, t);

    Point displacedA = Point(ax, ay);
    Point displacedB = Point(bx, by);

    const Real directionError = s->arcOriginalDisplaced(t);
    const Real Wdir = 4 * s->requestedLength / (M_PI * M_PI);
    const Real weightingDirectionError = Wdir * directionError*directionError;

    const Real length = Point::dist(&displacedA, &displacedB) / net.scaleFactor;
    const Real lengthError = length - s->requestedLength;
    const Real Wlen = 1 / s->requestedLength;
    const Real weightedLengthError = Wlen * lengthError*lengthError;

    return make_pair(weightingDirectionError, weightedLengthError);
}

pair<Real, Real> avgErrors(std::vector<Segment*> segs, Real t) {
    Real directionError = 0;
    Real lengthError = 0;
    for(Segment *s : segs) {
        pair<Real, Real> segErrors = segmentErrors(s, t);
        directionError += segErrors.first;
        lengthError += segErrors.second;
    }
    directionError /= segs.size();
    lengthError /= segs.size();
    return make_pair(directionError, lengthError);
}

pair<Real, Real> relativeSegmentErrors(Segment *s, Real t, string mode = "") {
    Point *a = s->a;
    Point *b = s->b;
    Real ax = lerp(a->original.x, a->displaced.x, t);
    Real ay = lerp(a->original.y, a->displaced.y, t);
    Real bx = lerp(b->original.x, b->displaced.x, t);
    Real by = lerp(b->original.y, b->displaced.y, t);

    Point displacedA = Point(ax, ay);
    Point displacedB = Point(bx, by);

    Real directionError;
    if(mode != "") {
        Real direction = atan2(by - ay, bx - ax);
        direction = fmod(direction + 2*M_PI, 2*M_PI);
        Real requestedAngle;
        if(mode == "angleRes")
            requestedAngle = s->optimalAngle;
        else if(mode == "octilinearity")
            requestedAngle = s->octilinearAngle;
        else {
            cout << "Undefined mode for relativeSegmentErrors!" << endl;
            std::exit(EXIT_FAILURE);
        }
        Real dirDiff = fabs(direction - requestedAngle);
        if(dirDiff > M_PI)
            directionError = 2*M_PI - dirDiff;
        else
            directionError = dirDiff;
    }
    else
        directionError = s->arcOriginalDisplaced(t);

    const Real length = Point::dist(&displacedA, &displacedB) / net.scaleFactor;
    const Real lengthError = length - s->requestedLength;
    const Real relLengthError = fabs(lengthError) / s->requestedLength;
    return make_pair(directionError, relLengthError);
}

pair<Real, Real> avgRelativeErrors(std::vector<Segment*> segs, Real t, string mode = "") {
    Real avgDirectionError = 0;
    Real avgLengthError = 0;
    for(Segment *s : segs) {
        pair<Real, Real> errors = relativeSegmentErrors(s, t, mode);
        avgDirectionError += errors.first;
        avgLengthError += errors.second;
    }
    avgDirectionError /= segs.size();
    avgLengthError /= segs.size();
    return make_pair(avgDirectionError, avgLengthError);
}



///////////////////////////////////////////////////
/////////////// GENERAL EXPERIMENTS ///////////////
///////////////////////////////////////////////////
void runtimeExperiment(bool noInter = false, bool nlls = false,
    bool angleRes = false, bool octilinearity = false) {
    net.oldIntersectionConstraints = true;
    net.useIntersectionConstraints = (angleRes || octilinearity) ? false : !noInter;
    resolveAngles = angleRes;
    nllsDisplacement = nlls;
    Real timeElapsed = 0;
    for(int i = 0; i < 100; i++) {
        displayProgress(i+1, 100);
        setRandomEdgeLengths();
        net.clearEvents();
        Wue::Timer solveTimer;
        net.solve(cParFactor, cPerpFactor, resolveAngles, nllsDisplacement);
        if(octilinearity) {
            net.octilinearResolution = true;
            net.calculateOctilinearAngles();
            solve();
            net.octilinearResolution = false;
        }
        timeElapsed += solveTimer.elapsed();
    }
    net.oldIntersectionConstraints = false;
    cout << "\n#Nodes: " << net.points.size() << endl;
    cout << "#Edges: " << net.segments.size() << endl;
    cout << "Average solve time: " << ((timeElapsed/100)*1000) << " ms" << endl;
    reset();
}

void nllsExperiment(bool noInter, bool maxTime) {
    net.oldIntersectionConstraints = true;
    net.useIntersectionConstraints = !noInter;
    const int iterations = 100;
    std::vector<Real> directionError;
    directionError.reserve(iterations);
    std::vector<Real> directionErrorNLLS;
    directionErrorNLLS.reserve(iterations);
    std::vector<Real> lengthError;
    lengthError.reserve(iterations);
    std::vector<Real> lengthErrorNLLS;
    lengthErrorNLLS.reserve(iterations);
    for(int i = 0; i < iterations; i++) {
        displayProgress(i+1, iterations);
        setRandomEdgeLengths();
        solve(maxTime);
        pair<Real, Real> errors = avgErrors(net.segments,
            maxTime ? 1 : net.displacementTime);
        directionError.push_back(errors.first);
        lengthError.push_back(errors.second);

        nllsDisplacement = true;
        solve(maxTime);
        errors = avgErrors(net.segments, maxTime ? 1 : net.displacementTime);
        directionErrorNLLS.push_back(errors.first);
        lengthErrorNLLS.push_back(errors.second);
        nllsDisplacement = false;
    }
    net.oldIntersectionConstraints = false;
    // Write results to file
    ofstream file;
    string filename = mapname + "-nlls-comparison";
    if(mode != "") filename += "-" + mode;
    filename += ".txt";
    file.open(filename);
    file << std::fixed;
    file << "#directionError directionErrorNLLS lengthError lengthErrorNLLS overallError overallErrorNLLS\n";
    for(int i = 0; i < iterations; i++) {
        file << directionError[i] << " " << directionErrorNLLS[i] << " ";
        file << lengthError[i] << " " << lengthErrorNLLS[i] << " ";
        file << directionError[i] + lengthError[i] << " ";
        file << directionErrorNLLS[i] + lengthErrorNLLS[i] << "\n";
    }
    file.close();
    cout << "\nWritten results to " << filename << endl;
    Real avgDirError = 0, avgDirErrorNLLS = 0, avgLenError = 0, avgLenErrorNLLS = 0;
    for(int i = 0; i < iterations; i++) {
        avgDirError += directionError[i];
        avgDirErrorNLLS += directionErrorNLLS[i];
        avgLenError += lengthError[i];
        avgLenErrorNLLS += lengthErrorNLLS[i];
    }
    cout << "Avg. Direction Error: " << (avgDirError/iterations) << endl;
    cout << "Avg. Direction Error NLLS: " << (avgDirErrorNLLS/iterations) << endl;
    cout << "Avg. Length Error: " << (avgLenError/iterations) << endl;
    cout << "Avg. Length Error NLLS: " << (avgLenErrorNLLS/iterations) << endl;
    cout << "Avg. Overall Error: " << ((avgDirError+avgLenError)/iterations) << endl;
    cout << "Avg. Overall Error NLLS: " << ((avgDirErrorNLLS+avgLenErrorNLLS)/iterations) << endl;
    reset();
}

void radWeightingExperiment(bool noInter, bool maxTime) {
    net.oldIntersectionConstraints = true;
    net.useIntersectionConstraints = !noInter;
    const int steps = 31;
    const Real stepWidth = 3.0/double(steps-1);
    std::vector<Real> avgRadErrors(steps, 0);
    std::vector<Real> maxRadErrors(steps, 0);
    std::vector<Real> cParFactors;
    cParFactors.reserve(steps);
    std::vector<Real> cPerpFactors;
    cPerpFactors.reserve(steps);
    for(int i = 0; i < steps; i++) {
        cParFactors[i] = 0.694059 * i*stepWidth + 0.764513 * (4/(M_PI*M_PI));
        cPerpFactors[i] = 0.115959 * i*stepWidth + 0.733046 * (4/(M_PI*M_PI));
    }
    const int iterations = 100;
    for(int i = 0; i < iterations; i++) {
        displayProgress(i+1, iterations);
        setRandomEdgeLengths();
        for(int j = 0; j < steps; j++) {
            cParFactor = cParFactors[j];
            cPerpFactor = cPerpFactors[j];
            solve(maxTime);
            avgRadErrors[j] += net.avgRadError;
            maxRadErrors[j] += net.maxRadError;
        }
    }
    for(int i = 0; i < steps; i++) {
        avgRadErrors[i] /= 100;
        maxRadErrors[i] /= 100;
    }
    net.oldIntersectionConstraints = false;
    // Write results to file
    ofstream file;
    string filename = mapname + "-radWeighting";
    if(mode != "") filename += "-" + mode;
    filename += ".txt";
    file.open(filename);
    file << std::fixed;
    file << "#radWeighting avgRadError maxRadError\n";
    for(int i = 0; i < steps; i++) {
        file << std::setprecision(2);
        file << i*stepWidth << " ";
        file << std::setprecision(6);
        file << avgRadErrors[i] << " " << maxRadErrors[i] << "\n";
    }
    file.close();
    cout << "\nWritten results to " << filename << endl;
    reset();
}

void arcWeightingExperiment(bool noInter, bool maxTime) {
    net.oldIntersectionConstraints = true;
    net.useIntersectionConstraints = !noInter;
    const int steps = 31;
    const Real stepWidth = 3.0/double(steps-1);
    std::vector<Real> avgArcErrors(steps, 0);
    std::vector<Real> maxArcErrors(steps, 0);
    std::vector<Real> cParFactors;
    cParFactors.reserve(steps);
    std::vector<Real> cPerpFactors;
    cPerpFactors.reserve(steps);
    for(int i = 0; i < steps; i++) {
        cParFactors[i] = 0.694059 * 1 + 0.764513 * (i*stepWidth*4/(M_PI*M_PI));
        cPerpFactors[i] = 0.115959 * 1 + 0.733046 * (i*stepWidth*4/(M_PI*M_PI));
    }
    const int iterations = 100;
    for(int i = 0; i < iterations; i++) {
        displayProgress(i+1, iterations);
        setRandomEdgeLengths();
        for(int j = 0; j < steps; j++) {
            cParFactor = cParFactors[j];
            cPerpFactor = cPerpFactors[j];
            solve(maxTime);
            avgArcErrors[j] += net.avgArcError;
            maxArcErrors[j] += net.maxArcError;
        }
    }
    for(int i = 0; i < steps; i++) {
        avgArcErrors[i] /= 100;
        maxArcErrors[i] /= 100;
    }
    net.oldIntersectionConstraints = false;
    // Write results to file
    ofstream file;
    string filename = mapname + "-arcWeighting";
    if(mode != "") filename += "-" + mode;
    filename += ".txt";
    file.open(filename);
    file << std::fixed;
    file << "#arcWeighting avgArcError maxArcError\n";
    for(int i = 0; i < steps; i++) {
        file << std::setprecision(2);
        file << i*stepWidth << " ";
        file << std::setprecision(6);
        file << avgArcErrors[i] << " " << maxArcErrors[i] << "\n";
    }
    file.close();
    cout << "\nWritten results to " << filename << endl;
    reset();
}

void weightingExperiment(bool noInter, bool maxTime) {
    net.oldIntersectionConstraints = true;
    net.useIntersectionConstraints = !noInter;
    const int steps = 41;
    std::vector<Real> directionError(steps, 0);
    std::vector<Real> lengthError(steps, 0);
    const int iterations = 100;
    for(int i = 0; i < iterations; i++) {
        displayProgress(i+1, iterations);
        setRandomEdgeLengths();
        for(int step = 0; step < steps; step++) {
            if(step-20 < 0)
                cParFactor = 1.00390 / (1.0 + (20.0-step)/10.0);
            else
                cParFactor = 1.00390 * (1.0 + (step-20.0)/10.0);
            solve(maxTime);
            pair<Real, Real> errors = avgErrors(net.segments,
                maxTime ? 1 : net.displacementTime);
            directionError[step] += errors.first;
            lengthError[step] += errors.second;
        }
    }
    for(int step = 0; step < steps; step++) {
        directionError[step] /= 100;
        lengthError[step] /= 100;
    }
    net.oldIntersectionConstraints = false;
    // Write results to file
    ofstream file;
    string filename = mapname + "-weighting";
    if(mode != "") filename += "-" + mode;
    filename += ".txt";
    file.open(filename);
    file << std::fixed;
    file << "#step directionError lengthError\n";
    for(int step = 0; step < steps; step++) {
        file << step << " ";
        file << directionError[step] << " " << lengthError[step] << "\n";
    }
    file.close();
    cout << endl << "Written results to " << filename << endl;
    reset();
}

void displacementTimeExperiment() {
    net.oldIntersectionConstraints = true;
    Real avgDisplacementTime = 0;
    for(int i = 0; i < 100; i++) {
        displayProgress(i+1, 100);
        setRandomEdgeLengths();
        solve();
        avgDisplacementTime += net.displacementTime;
    }
    net.oldIntersectionConstraints = false;
    cout << "\nAvg. Displacement Time: " << (avgDisplacementTime/100) << endl;
    reset();
}

void strokesExperiment(bool noInter, bool maxTime) {
    net.oldIntersectionConstraints = true;
    net.useIntersectionConstraints = !noInter;
    Real avgUnaffectedDirError = 0;
    Real avgAffectedDirError = 0;
    Real avgUnaffectedLenError = 0;
    Real avgAffectedLenError = 0;
    for(int i = 0; i < 100; i++) {
        displayProgress(i+1, 100);
        std::vector<Segment*> segs;
        setRandomStrokesCongestion(segs);
        solve(maxTime);
        Real t = maxTime ? 1 : net.displacementTime;
        std::vector<Segment *> unaffectedSegs(net.segments);
        for(Segment *s : segs) {
            for(auto iter = unaffectedSegs.begin(); iter != unaffectedSegs.end(); iter++) {
                if(*iter == s) {
                    unaffectedSegs.erase(iter);
                    break;
                }
            }
        }
        pair<Real, Real> errors = avgRelativeErrors(unaffectedSegs, t);
        avgUnaffectedDirError += errors.first;
        avgUnaffectedLenError += errors.second;
        errors = avgRelativeErrors(segs, t);
        avgAffectedDirError += errors.first;
        avgAffectedLenError += errors.second;
    }
    net.oldIntersectionConstraints = false;

    cout << "\nAvg. unaffected Direction Error: " << (avgUnaffectedDirError/100*180/M_PI) << endl;
    cout << "Avg. affected Direction Error: " << (avgAffectedDirError/100*180/M_PI) << endl;
    cout << "Avg. unaffected relative Length Error: " << (avgUnaffectedLenError/100) << endl;
    cout << "Avg. affected relative Length Error: " << (avgAffectedLenError/100) << endl;
    reset();
}

void pointExperiment(bool noInter, bool maxTime) {
    net.oldIntersectionConstraints = true;
    net.useIntersectionConstraints = !noInter;
    Real avgUnaffectedDirError = 0;
    Real avgAffectedDirError = 0;
    Real avgUnaffectedLenError = 0;
    Real avgAffectedLenError = 0;
    for(int i = 0; i < 100; i++) {
        displayProgress(i+1, 100);
        std::vector<Segment*> segs;
        setRandomPointCongestion(segs);
        solve(maxTime);
        Real t = maxTime ? 1 : net.displacementTime;
        std::vector<Segment *> unaffectedSegs(net.segments);
        for(Segment *s : segs) {
            for(auto iter = unaffectedSegs.begin(); iter != unaffectedSegs.end(); iter++) {
                if(*iter == s) {
                    unaffectedSegs.erase(iter);
                    break;
                }
            }
        }
        pair<Real, Real> errors = avgRelativeErrors(unaffectedSegs, t);
        avgUnaffectedDirError += errors.first;
        avgUnaffectedLenError += errors.second;
        errors = avgRelativeErrors(segs, t);
        avgAffectedDirError += errors.first;
        avgAffectedLenError += errors.second;
    }
    net.oldIntersectionConstraints = false;

    cout << "\nAvg. unaffected Direction Error: " << (avgUnaffectedDirError/100*180/M_PI) << endl;
    cout << "Avg. affected Direction Error: " << (avgAffectedDirError/100*180/M_PI) << endl;
    cout << "Avg. unaffected relative Length Error: " << (avgUnaffectedLenError/100) << endl;
    cout << "Avg. affected relative Length Error: " << (avgAffectedLenError/100) << endl;
    reset();
}




///////////////////////////////////////////////////
//////////////// METRO EXPERIMENTS ////////////////
///////////////////////////////////////////////////
void runtimeMetroExperiment(bool noInter, bool nlls) {
    net.useIntersectionConstraints = !noInter;
    nllsDisplacement = nlls;
    // resolveAngles = true;
    // net.useProximityConstraints = true;
    // bool octilinearity = true;
    Real timeElapsed = 0;
    for(int i = 0; i < 100; i++) {
        displayProgress(i+1, 100);
        calculateMetromap();
        net.clearEvents();
        Wue::Timer solveTimer;
        net.solve(cParFactor, cPerpFactor, resolveAngles, nllsDisplacement);
        // if(octilinearity) {
        //     net.octilinearResolution = true;
        //     net.calculateOctilinearAngles();
        //     solve();
        //     net.octilinearResolution = false;
        // }
        timeElapsed += solveTimer.elapsed();
    }
    cout << "\n#Nodes: " << net.points.size() << endl;
    cout << "#Edges: " << net.segments.size() << endl;
    cout << "Average solve time: " << ((timeElapsed/100)*1000) << " ms" << endl;
    reset();
}

void runtimeNetDistanceExperiment() {
    Real timeElapsed = 0;
    for(int i = 0; i < 100; i++) {
        displayProgress(i+1, 100);
        net.resetMap(true);
        for(int i = 0; i < net.segments.size(); i++)
            net.segments[i]->requestedLength = 1;
        net.netDistanceDisplacement = true;
        net.clearEvents();
        Wue::Timer solveTimer;
        net.solve(cParFactor, cPerpFactor, resolveAngles, nllsDisplacement);
        timeElapsed += solveTimer.elapsed();
        net.netDistanceDisplacement = false;
    }
    cout << "\n#Nodes: " << net.points.size() << endl;
    cout << "#Edges: " << net.segments.size() << endl;
    cout << "Average solve time: " << ((timeElapsed/100)*1000) << " ms" << endl;
    reset();
}

void edgeLengthExperiment(bool angleRes) {
    resolveAngles = angleRes;
    calculateMetromap();
    solve();

    // Write results to file
    ofstream file;
    string filename = mapname + "-metro-edge-length";
    if(mode == "angleRes") filename += "-" + mode;
    filename += ".txt";
    file.open(filename);
    file << std::fixed;
    file << "#edgeLength\n";
    Real t = net.displacementTime;
    for(Segment *s : net.segments) {
        Point *a = s->a;
        Point *b = s->b;
        Real ax = lerp(a->original.x, a->displaced.x, t);
        Real ay = lerp(a->original.y, a->displaced.y, t);
        Real bx = lerp(b->original.x, b->displaced.x, t);
        Real by = lerp(b->original.y, b->displaced.y, t);

        Point displacedA = Point(ax, ay);
        Point displacedB = Point(bx, by);
        const Real length = Point::dist(&displacedA, &displacedB) / net.scaleFactor;
        file << length << "\n";
    }
    file.close();
    cout << "\nWritten results to " << filename << endl;
    reset();
}

void angleResolutionExperiment(bool metroMap, bool original) {
    if(!original) {
        resolveAngles = true;
        if(metroMap)
            calculateMetromap();
        solve();
    }

    // Calculate ratio optimal resolution to actual resolution for every point
    std::vector<Real> ratioVector;
    for(Point *p : net.points) {
        if(p->segments.size() == 1) continue;
        Real biggestDiff = 0;
        Real optAngle = 2*M_PI / p->segments.size();
        for(int i = 0; i < p->segments.size(); i++) {
            Segment *s = p->segments[i];
            Segment *s2 = p->segments[(i+1)%p->segments.size()];
            Point *u = s->oppositeOf(p);
            Point *v = s2->oppositeOf(p);
            Real pxt, pyt, uxt, uyt, vxt, vyt;
            if(original) {
                pxt = p->original.x;
                pyt = p->original.y;
                uxt = u->original.x;
                uyt = u->original.y;
                vxt = v->original.x;
                vyt = v->original.y;
            }
            else {
                Real t = net.displacementTime;
                pxt = lerp(p->original.x, p->displaced.x, t);
                pyt = lerp(p->original.y, p->displaced.y, t);
                uxt = lerp(u->original.x, u->displaced.x, t);
                uyt = lerp(u->original.y, u->displaced.y, t);
                vxt = lerp(v->original.x, v->displaced.x, t);
                vyt = lerp(v->original.y, v->displaced.y, t);
            }
            Real anglePU = atan2(uyt - pyt, uxt - pxt);
            Real anglePV = atan2(vyt - pyt, vxt - pxt);
            Real angle = fmod(anglePV - anglePU + 4*M_PI, 2*M_PI);
            Real diff = fabs(optAngle - angle);
            if(diff > biggestDiff)
                biggestDiff = diff;
        }
        Real ratio = biggestDiff / optAngle;
        ratioVector.push_back(ratio);
    }

    // Write results to file
    ofstream file;
    string filename = mapname + "-metro-angle-resolution";
    if(mode == "metroMap") filename += "-" + mode;
    else if(mode == "original") filename += "-" + mode;
    filename += ".txt";
    file.open(filename);
    file << std::fixed;
    file << "#ratioDiffOptActual\n";
    for(Real ratio : ratioVector)
        file << ratio << "\n";
    file.close();
    cout << "\nWritten results to " << filename << endl;
    reset();
}

void metroLineExperiment() {
    resolveAngles = true;
    net.useProximityConstraints = true;
    Real avgDirError = 0;
    Real avgLenError = 0;
    int iterations = net.metroLines.size();
    for(int i = 0; i < iterations; i++) {
        displayProgress(i+1, iterations);
        setCyclicMetroLineCongestion();
        solve();
        net.octilinearResolution = true;
        net.calculateOctilinearAngles();
        solve();
        net.octilinearResolution = false;
        pair<Real, Real> errors = avgRelativeErrors(net.segments, net.displacementTime,
            "octilinearity");
        avgDirError += errors.first;
        avgLenError += errors.second;
    }

    cout << "\nMetro line count: " << iterations << endl;
    cout << "Avg. Direction Error: " << (avgDirError/iterations*180/M_PI) << endl;
    cout << "Avg. relative Length Error: " << (avgLenError/iterations) << endl;
    reset();
}

void metromapExperiment(bool octilinearity) {
    resolveAngles = true;
    net.useProximityConstraints = true;
    calculateMetromap();
    solve();
    if(octilinearity) {
        net.octilinearResolution = true;
        net.calculateOctilinearAngles();
        solve();
    }
    pair<Real, Real> errors = avgRelativeErrors(net.segments, net.displacementTime,
        octilinearity ? "octilinearity" : "");
    Real avgDirError = errors.first;
    Real avgLenError = errors.second;

    cout << "Avg. Direciton Error: " << (avgDirError*180/M_PI) << endl;
    cout << "Avg. relative Length Error: " << avgLenError << endl;
    reset();
}



int main(int argc, char **argv) {
    // Read commandline arguments if present
    if(argc < 2) {
        cout << "Usage: experiments.exe [experiment-to-run] [path-to-map-data]" << endl;
        return 0;
    }

    if(argc >= 3) {
        stringstream maparg(argv[2]);
        string fname;
        maparg >> fname;
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
        size_t mapnameStart = fname.find_last_of("/\\")+1;
        size_t mapnameEnd = fname.find_last_of(".");
        mapname = fname.substr(mapnameStart, mapnameEnd - mapnameStart);
        net.scaleFactor = 1.0;
        net.borderWidth = 0.0;
        net.displacementTime = 1;
    }

    bool noInter = false;
    bool maxTime = false;
    bool angleRes = false;
    bool metroMap = false;
    bool original = false;
    bool octilinearity = false;
    if(argc >= 4) {
        stringstream arg(argv[3]);
        arg >> mode;
        if(mode == "noInter")
            noInter = true;
        else if(mode == "maxTime")
            maxTime = true;
        else if(mode == "angleRes")
            angleRes = true;
        else if(mode == "metroMap")
            metroMap = true;
        else if(mode == "original")
            original = true;
        else if(mode == "octilinearity")
            octilinearity = true;
        else
            mode = "";
    }

    stringstream exparg(argv[1]);
    string experiment;
    exparg >> experiment;
    ///// General Experiments /////
    if(experiment == "runtime")
        runtimeExperiment(noInter, false, angleRes, octilinearity);
    else if(experiment == "runtimeNlls")
        runtimeExperiment(noInter, true, angleRes, octilinearity);
    else if(experiment == "nlls")
        nllsExperiment(noInter, maxTime);
    else if(experiment == "radWeighting")
        radWeightingExperiment(noInter, maxTime);
    else if(experiment == "arcWeighting")
        arcWeightingExperiment(noInter, maxTime);
    else if(experiment == "weighting")
        weightingExperiment(noInter, maxTime);
    else if(experiment == "displacementTime")
        displacementTimeExperiment();
    else if(experiment == "strokes")
        strokesExperiment(noInter, maxTime);
    else if(experiment == "point")
        pointExperiment(noInter, maxTime);

    ////// Metro Experiments //////
    else if(experiment == "runtimeMetro")
        runtimeMetroExperiment(noInter, false);
    else if(experiment == "runtimeMetroNlls")
        runtimeMetroExperiment(noInter, true);
    else if(experiment == "runtimeNetDistance")
        runtimeNetDistanceExperiment();
    // else if(experiment == "metroNlls")
    //     nllsMetroExperiment(noInter, maxTime);
    else if(experiment == "edgeLength")
        edgeLengthExperiment(angleRes);
    else if(experiment == "angleResolution")
        angleResolutionExperiment(metroMap, original);
    else if(experiment == "metroLine")
        metroLineExperiment();
    else if(experiment == "metromap")
        metromapExperiment(octilinearity);
    else {
        cout << "Invalid experiment name: " << experiment << endl;
        return 0;
    }
}
