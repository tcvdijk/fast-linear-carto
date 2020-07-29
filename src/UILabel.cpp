#include "UILabel.h"

#include <sstream>

void UILabel::label(const char *text) {
    std::stringstream ss;
    if(tag())
        ss << tag() << ": ";
    ss << text;
    Fl_Box::copy_label(ss.str().c_str());
}

void UILabel::label(double v) {
    std::stringstream ss;
    if(tag())
        ss << tag() << ": ";
    ss << std::fixed << v;
    Fl_Box::copy_label(ss.str().c_str());
}
