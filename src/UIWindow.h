#ifndef INCLUDED_UIWINDOW
#define INCLUDED_UIWINDOW

#include <FL/Fl_Window.H>

#include "UIMap.h"
#include "UIControls.h"

class UIWindow : public Fl_Window {
    UIMap *map;
    UIControls *controls;

    Real cParFactor_ = 1.00390;
    Real cPerpFactor_ = 0.413051;
    bool resolveAngles_ = false;
    bool nllsDisplacement = false;
    bool octilinearResolution_ = false;

    void resetWeights();

    int handle(int event);
    void randomEdgeLengths();
    void setTargetZooms();
    void randomStrokesCongestion();
    void randomPointCongestion();
    void netDistanceDisplacement();
    void toggleNetDistanceDisplacementWeighting();
    void cyclicMetroLineCongestion(bool reset = false);
    void toggleNllsDisplacement();
    void toggleIntersectionConstraints();

    void updateLabels();

public:
    UIWindow(int X, int Y, int W, int H, const char *L = 0);

    void calculateMetromap();
    void toggleResolveAngles();
    void toggleProximityConstraints();
    void toggleOctilinearResolution();

    void solve(bool octilinearToggle = false);
    void oldSolve();
    void reset();

    inline Real cParFactor() {return cParFactor_;}
    void cParFactor(Real cPar);
    inline Real cPerpFactor() {return cPerpFactor_;}
    void cPerpFactor(Real cPerp);

    inline bool resolveAngles() {return resolveAngles_;}
    inline void resolveAngles(bool val) {
        resolveAngles_ = val;
        controls->angleResolution(val);
    }

    inline bool proximityConstraints() {return net.useProximityConstraints;}
    inline void proximityConstraints(bool val) {
        net.useProximityConstraints = val;
        controls->proximityConstraints(val);
    }

    inline bool octilinearResolution() {return octilinearResolution_;}
    inline void octilinearResolution(bool val) {
        octilinearResolution_ = val;
        net.octilinearResolution = val;
        controls->octilinearResolution(val);
    }
};

#endif //ndef INCLUDED_UIWINDOW
