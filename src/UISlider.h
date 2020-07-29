#ifndef INCLUDED_UISLIDER
#define INCLUDED_UISLIDER

#include <FL/Fl_Widget.H>
#include <FL/Fl_Group.H>
#include <FL/Fl_Hor_Slider.H>
#include <FL/Fl_Box.H>

class UISlider : public Fl_Group {
    const char *label_;
    Fl_Hor_Slider *slider;
    Fl_Box *box;

    typedef void (*UISliderCB)(UISlider*, void*);
    UISliderCB callback_;
    void *data_;

    static void FlSliderCallback(Fl_Widget *widget, void *data);

    void updateLabel();

public:
    UISlider(int X, int Y, int W, int H, const char *L = 0, double min = 0.0,
        double max = 1.0, double val = 0.0);

    inline void label(const char *L) {label_ = L;}
    inline const char *label() {return label_;}
    inline void bounds(double a, double b) {slider->bounds(a, b);}
    void value(double v);
    inline double value() {return slider->value();}
    inline void callback(UISliderCB callback, void *data) {
        callback_ = callback;
        data_ = data;
    }
};

#endif //ndf INCLUDED_UISLIDER
