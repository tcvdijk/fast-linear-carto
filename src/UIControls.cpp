#include "UIControls.h"

#include "UIWindow.h"

void UIControls::updateCPar(UISlider *slider, void *data) {
    UIControls *controls = (UIControls *) data;
    UIWindow *window = (UIWindow *) controls->parent();
    window->cParFactor(slider->value());
}

void UIControls::updateCPerp(UISlider *slider, void *data) {
    UIControls *controls = (UIControls *) data;
    UIWindow *window = (UIWindow *) controls->parent();
    window->cPerpFactor(slider->value());
}

void UIControls::reset_cb(Fl_Widget *widget, void *data) {
    UIControls *controls = (UIControls *) data;
    UIWindow *window = (UIWindow *) controls->parent();
    window->reset();
}

void UIControls::metromap_cb(Fl_Widget *widget, void *data) {
    UIControls *controls = (UIControls *) data;
    UIWindow *window = (UIWindow *) controls->parent();
    window->calculateMetromap();
}

void UIControls::angleResolution_cb(Fl_Widget *widget, void *data) {
    UIControls *controls = (UIControls *) data;
    UIWindow *window = (UIWindow *) controls->parent();
    window->toggleResolveAngles();
}

void UIControls::proximity_cb(Fl_Widget *widget, void *data) {
    UIControls *controls = (UIControls *) data;
    UIWindow *window = (UIWindow *) controls->parent();
    window->toggleProximityConstraints();
}

void UIControls::octilinearity_cb(Fl_Widget *widget, void *data) {
    UIControls *controls = (UIControls *) data;
    UIWindow *window = (UIWindow *) controls->parent();
    window->toggleOctilinearResolution();
}

UIControls::UIControls(int X, int Y, int W, int H, const char *L)
    : Fl_Group(X, Y, W, H, L) {
    UIWindow *window = (UIWindow *) parent();
    cParSlider = new UISlider(X, Y, W, 40, "cPar", 0.0, 3.0,
        window->cParFactor());
    cParSlider->callback(updateCPar, (void *)this);
    cPerpSlider = new UISlider(X, cParSlider->y() + cParSlider->h(), W, 40,
        "cPerp", 0.0, 3.0, window->cPerpFactor());
    cPerpSlider->callback(updateCPerp, (void *)this);
    maxArcErrorLabel = new UILabel(X, cPerpSlider->y() + cPerpSlider->h(),
        W, 20, "Max. Arc Error", 0.0);
    maxRadErrorLabel = new UILabel(X, maxArcErrorLabel->y() + maxArcErrorLabel->h(),
        W, 20, "Max. Rad Error", 0.0);
    avgArcErrorLabel = new UILabel(X, maxRadErrorLabel->y() + maxRadErrorLabel->h(),
        W, 20, "Avg. Arc Error", 0.0);
    avgRadErrorLabel = new UILabel(X, avgArcErrorLabel->y() + avgArcErrorLabel->h(),
        W, 20, "Avg. Rad Error", 0.0);
    errorLabel = new UILabel(X, avgRadErrorLabel->y() + avgRadErrorLabel->h(),
        W, 20, "Error", 0.0);
    scaleFactorLabel = new UILabel(X, errorLabel->y() + errorLabel->h(),
        W, 20, "Scale Factor", 1.0);
    displacementTimeLabel = new UILabel(X, scaleFactorLabel->y() + scaleFactorLabel->h(),
        W, 20, "Displacement Time", 1.0);

    resetButton = new Fl_Button(X, displacementTimeLabel->y() +
        displacementTimeLabel->h(), W, 20, "Reset");
    resetButton->shortcut(' ');
    resetButton->callback(reset_cb, (void *)this);
    resetButton->clear_visible_focus();

    metromapButton = new Fl_Button(X, resetButton->y() +
        resetButton->h(), W, 20, "Calclate metro map");
    metromapButton->shortcut('t');
    metromapButton->callback(metromap_cb, (void *)this);
    metromapButton->clear_visible_focus();

    angleResolutionButton = new Fl_Check_Button(X, metromapButton->y() + metromapButton->h(),
        W, 20, "Angular resolution");
    angleResolutionButton->shortcut('i');
    angleResolutionButton->callback(angleResolution_cb, (void *)this);
    angleResolutionButton->clear_visible_focus();

    proximityButton = new Fl_Check_Button(X, angleResolutionButton->y() +
        angleResolutionButton->h(), W, 20, "Proximity constraints");
    proximityButton->shortcut('k');
    proximityButton->callback(proximity_cb, (void *)this);
    proximityButton->clear_visible_focus();

    octilinearityButton = new Fl_Check_Button(X, proximityButton->y() + proximityButton->h(),
        W, 20, "Octilinearity");
    octilinearityButton->shortcut('h');
    octilinearityButton->callback(octilinearity_cb, (void *)this);
    octilinearityButton->clear_visible_focus();

    end();
}
