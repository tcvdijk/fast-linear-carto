#ifndef INCLUDED_UIMAP
#define INCLUDED_UIMAP

#include <FL/Fl.H>
#include <FL/Fl_Gl_Window.H>

#include <string>

#include "Point.h"
#include "Util.h"

class UIMap : public Fl_Gl_Window {
    struct AnimationData {
        double duration;
        int fps;
        double delay;
        int currentStep;

        AnimationData(double duration = 1.0, int fps = 60, double delay = 0.0)
            : duration(duration), fps(fps), delay(delay), currentStep(0) {}
    };
    AnimationData linearInterpolationData;
    Real currentTime = 1;
    Real viewTime = 1;

    bool showNodes_ = true;
    bool showEvents_ = false;
    bool showMaxTime_ = false;
    bool showDistortion_ = false;
    bool showFocus_ = true;

    void draw();
    void DrawRoadNetwork();
    void reportGLError();
    void resize(int X, int Y, int W, int H);
    static void linearInterpolation(void *data);
    void abortAnimation();

    int handle(int event);
    void leftMouseButtonDown(int X, int Y);
    void rightMouseButtonDown(int X, int Y);
    void altLeftMouseButtonDown(int X, int Y);

    Real mouseX(int X);
    Real mouseY(int Y);

public:
    UIMap(int X, int Y, int W, int H, const char *L = 0);

    void screenshot(const std::string &filenameBase = "screenshot");

    void randomEdgeLengthsAnimation();
    void calculateMetromapAnimation();
    void setTargetZoomsAnimation();
    void randomStrokesCongestionAnimation();
    void randomPointCongestionAnimation();
    void netDistanceDisplacementAnimation();
    void cyclicMetroLineCongestionAnimation();
    void changeWeightsAnimation();
    void toggleResolveAnglesAnimation();
    void toggleNllsDisplacementAnimation();
    void toggleIntersectionConstraintsAnimation();
    void toggleProximityConstraintsAnimation();

    Point* pickedUpNode = nullptr;
    void pickUpNode(int X, int Y);
    void dropNode(int X, int Y);

    void redraw_map();

    inline bool showNodes() {return showNodes_;}
    inline void showNodes(bool show) {showNodes_ = show;}
    inline bool showEvents() {return showEvents_;}
    inline void showEvents(bool show) {showEvents_ = show;}
    inline bool showMaxTime() {return showMaxTime_;}
    inline void showMaxTime(bool show) {showMaxTime_ = show;}
    inline bool showDistortion() {return showDistortion_;}
    inline void showDistortion(bool show) {showDistortion_ = show;}
    inline bool showFocus() {return showFocus_;}
    inline void showFocus(bool show) {showFocus_ = show;}
};

#endif //ndef INCLUDED_UIMAP
