#include "UISlider.h"

#include <sstream>
#include <iomanip>

void UISlider::FlSliderCallback(Fl_Widget *widget, void *data) {
    UISlider *slider = (UISlider*) data;
    slider->updateLabel();
    if(slider->callback_)
        slider->callback_(slider, slider->data_);
}

void UISlider::updateLabel() {
    std::stringstream ss;
    if(label())
        ss << label() << ": ";
    ss << std::fixed << slider->value();
    box->copy_label(ss.str().c_str());
}

UISlider::UISlider(int X, int Y, int W, int H, const char *L, double min,
    double max, double val) : Fl_Group(X, Y, W, H), label_(L),
    callback_(nullptr), data_(nullptr) {
    slider = new Fl_Hor_Slider(X, Y, W, H/2);
    slider->bounds(min, max);
    slider->value(val);
    box = new Fl_Box(X, Y+slider->h(), W, H/2);
    updateLabel();
    slider->callback(FlSliderCallback, (void *)this);
    end();
}

void UISlider::value(double v) {
    slider->value(v);
    updateLabel();
}
