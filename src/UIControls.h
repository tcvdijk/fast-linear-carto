#ifndef INCLUDED_UICONTROLS
#define INCLUDED_UICONTROLS

#include <FL/Fl_Group.H>
#include <FL/Fl_Widget.H>
#include <FL/Fl_Button.H>
#include <Fl/Fl_Check_Button.H>

#include "UISlider.h"
#include "UILabel.h"

class UIControls : public Fl_Group {
    UISlider *cParSlider;
    UISlider *cPerpSlider;
    UILabel *maxArcErrorLabel;
    UILabel *maxRadErrorLabel;
    UILabel *avgArcErrorLabel;
    UILabel *avgRadErrorLabel;
    UILabel *scaleFactorLabel;
    UILabel *errorLabel;
    UILabel *displacementTimeLabel;
    Fl_Button *resetButton;
    Fl_Button *metromapButton;
    Fl_Check_Button *angleResolutionButton;
    Fl_Check_Button *proximityButton;
    Fl_Check_Button *octilinearityButton;

    static void updateCPar(UISlider *slider, void *data);
    static void updateCPerp(UISlider *slider, void *data);

    static void reset_cb(Fl_Widget *wigdet, void *data);
    static void metromap_cb(Fl_Widget *widget, void *data);
    static void angleResolution_cb(Fl_Widget *widget, void *data);
    static void proximity_cb(Fl_Widget *widget, void *data);
    static void octilinearity_cb(Fl_Widget *widget, void *data);

public:
    UIControls(int X, int Y, int W, int H, const char*L = 0);

    inline void cParFactor(double cPar) {cParSlider->value(cPar);}
    inline void cPerpFactor(double cPerp) {cPerpSlider->value(cPerp);}
    inline void maxArcError(double error) {maxArcErrorLabel->label(error);}
    inline void maxRadError(double error) {maxRadErrorLabel->label(error);}
    inline void avgArcError(double error) {avgArcErrorLabel->label(error);}
    inline void avgRadError(double error) {avgRadErrorLabel->label(error);}
    inline void error(double e) {errorLabel->label(e);}
    inline void scaleFactor(double scale) {scaleFactorLabel->label(scale);}
    inline void displacementTime(double t) {displacementTimeLabel->label(t);}

    inline void angleResolution(bool val) {angleResolutionButton->value(val ? 1 : 0);}
    inline void proximityConstraints(bool val) {proximityButton->value(val ? 1 : 0);}
    inline void octilinearResolution(bool val) {octilinearityButton->value(val ? 1 : 0);}
};

#endif //ndef INCLUDED_UICONTROLS
