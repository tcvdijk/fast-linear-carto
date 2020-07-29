#ifndef INCLUDED_UILABEL
#define INCLUDED_UILABEL

#include <FL/Fl_Box.H>

class UILabel : public Fl_Box {
    const char *tag_;

public:
    UILabel(int X, int Y, int W, int H, const char *t = 0, double v = 0.0)
        : Fl_Box(X, Y, W, H), tag_(t) {label(v);}

    inline const char *tag() {return tag_;}
    inline void tag(const char *t) {tag_ = t;}
    void label(const char *text);
    void label(double v);
};

#endif //ndef INCLUDED_UILABEL
