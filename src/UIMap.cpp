#include "UIMap.h"

#include <iostream>
#include <math.h>
#include <sstream>
#include <iomanip>

#include <FL/gl.h>
#include <FL/fl_ask.H>

#include "UIWindow.h"
#include "RoadNetwork.h"
#include "Segment.h"
#include "lodepng.h"

using namespace std;

void UIMap::draw() {
    if(!valid()) {
        glEnable(GL_POINT_SMOOTH);
        glEnable(GL_LINE_SMOOTH);

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        glEnable(GL_DEPTH_TEST);
        glDepthMask(GL_TRUE);
        glDepthFunc(GL_LEQUAL);
        glDepthRange(0.0f, 1.0f);

        glLineWidth(3);
        glClearColor(1,1,1,1);

        glLoadIdentity();
        glOrtho(net.midX - net.halfsize*1.02, net.midX + net.halfsize*1.02,
            net.midY - net.halfsize*1.02, net.midY + net.halfsize*1.02, -1, 1);
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    reportGLError();
    DrawRoadNetwork();
    reportGLError();
}

void UIMap::DrawRoadNetwork() {
    if(showMaxTime())
        viewTime = 1;
    else
        viewTime = net.displacementTime;

    // Draw line segments
    glBegin(GL_LINES);
        for(Segment *s : net.segments) {
            Point *a = s->a;
            Point *b = s->b;

            // Figure out the color of this segment
            double red = 0, green = 0, blue = 0;
            if(showFocus() && ((s->a->isFocus || s->b->isFocus) || s->isFocus))
                blue = 1;
            if(s->isIntersecting) red = 1;
            if(showDistortion()) red = 0.6 * s->distortion();
            glColor3d(red, green, blue);

            const Real axt = lerp(a->original.x, a->displaced.x, viewTime);
            const Real ayt = lerp(a->original.y, a->displaced.y, viewTime);
            const Real bxt = lerp(b->original.x, b->displaced.x, viewTime);
            const Real byt = lerp(b->original.y, b->displaced.y, viewTime);
            // Draw the segment
            glVertex2d(lerp(a->current.x, axt, currentTime),
                       lerp(a->current.y, ayt, currentTime));
            glVertex2d(lerp(b->current.x, bxt, currentTime),
                       lerp(b->current.y, byt, currentTime));
        }
    glEnd();

    // Draw nodes
    if(showNodes()) {
        glColor3f(0,0,0);
        glPointSize(12);
        glBegin(GL_POINTS);
            for(Point *p : net.points) {
                const Real xt = lerp(p->original.x, p->displaced.x, viewTime);
                const Real yt = lerp(p->original.y, p->displaced.y, viewTime);
                Real x = lerp(p->current.x, xt, currentTime);
                Real y = lerp(p->current.y, yt, currentTime);
                glVertex2d(x, y);
            }
        glEnd();
        glColor3f(1,1,1);
        glPointSize(7);
        glBegin(GL_POINTS);
            for(Point *p : net.points) {
                const Real xt = lerp(p->original.x, p->displaced.x, viewTime);
                const Real yt = lerp(p->original.y, p->displaced.y, viewTime);
                Real x = lerp(p->current.x, xt, currentTime);
                Real y = lerp(p->current.y, yt, currentTime);
                glVertex2d(x, y);
            }
        glEnd();
    }

    // Draw dot at mouse position (useful when recording movies)
    if(!net.working && pickedUpNode == nullptr) {
        glPointSize(16);
        if(Fl::event_button1()) glColor3f(1,0,0);
        else glColor3f(0.51,0.51,0.51);
        glBegin(GL_POINTS);
            glVertex2d(net.screenToWorldX(mouseX(Fl::event_x())),
                       net.screenToWorldY(mouseY(Fl::event_y())));
        glEnd();
    }

    // Draw debug information about events
    if(showEvents()) {
        glColor3f(1.51,0.21,0.21);
        glPointSize(8);
        glBegin(GL_POINTS);
            // Intersection events
            for(Point *p : net.points) {
                if( p->hasEvent ) {
                    Segment *s = p->event.s;
                    Point *a = s->a;
                    Point *b = s->b;
                    Real param = p->event.param;
                    const Real axt = lerp(a->original.x, a->displaced.x, viewTime);
                    const Real ayt = lerp(a->original.y, a->displaced.y, viewTime);
                    const Real bxt = lerp(b->original.x, b->displaced.x, viewTime);
                    const Real byt = lerp(b->original.y, b->displaced.y, viewTime);
                    Real ax = lerp(a->current.x, axt, currentTime);
                    Real ay = lerp(a->current.y, ayt, currentTime);
                    Real bx = lerp(b->current.x, bxt, currentTime);
                    Real by = lerp(b->current.y, byt, currentTime);
                    glVertex2d(lerp(ax, bx, param), lerp(ay, by, param));
                }
            }
            // Proximity events points
            glColor3f(0.0,1.0,0.0);
            for(auto e : net.events) {
                Point *p = e.p;
                const Real xt = lerp(p->original.x, p->displaced.x, viewTime);
                const Real yt = lerp(p->original.y, p->displaced.y, viewTime);
                Real x = lerp(p->current.x, xt, currentTime);
                Real y = lerp(p->current.y, yt, currentTime);
                glVertex2d(x, y);
            }
        glEnd();
        // Proximity events segments
        glColor3f(0.0,1.0,0.0);
        glBegin(GL_LINES);
            for(auto e : net.events) {
                Segment *s = e.s;
                Point *a = s->a;
                Point *b = s->b;

                const Real axt = lerp(a->original.x, a->displaced.x, viewTime);
                const Real ayt = lerp(a->original.y, a->displaced.y, viewTime);
                const Real bxt = lerp(b->original.x, b->displaced.x, viewTime);
                const Real byt = lerp(b->original.y, b->displaced.y, viewTime);
                // Draw the segment
                glVertex2d(lerp(a->current.x, axt, currentTime),
                           lerp(a->current.y, ayt, currentTime));
                glVertex2d(lerp(b->current.x, bxt, currentTime),
                           lerp(b->current.y, byt, currentTime));
            }
        glEnd();
        // Proximity segments
        glColor3f(0.0,1.0,0.0);
        glBegin(GL_LINES);
            for(Segment *s : net.proximitySegments) {
                Point *a = s->a;
                Point *b = s->b;

                const Real axt = lerp(a->original.x, a->displaced.x, viewTime);
                const Real ayt = lerp(a->original.y, a->displaced.y, viewTime);
                const Real bxt = lerp(b->original.x, b->displaced.x, viewTime);
                const Real byt = lerp(b->original.y, b->displaced.y, viewTime);
                // Draw the segment
                glVertex2d(lerp(a->current.x, axt, currentTime),
                           lerp(a->current.y, ayt, currentTime));
                glVertex2d(lerp(b->current.x, bxt, currentTime),
                           lerp(b->current.y, byt, currentTime));
            }
        glEnd();
    }
}

void UIMap::reportGLError() {
    GLenum error = glGetError();
    if(error)
        fl_alert("GL Error!");
}

void UIMap::resize(int X, int Y, int W, int H) {
    Fl_Gl_Window::resize(X, Y, W, H);
    glViewport(0, 0, W, H);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    glMatrixMode(GL_MODELVIEW);
    redraw();
}

void UIMap::linearInterpolation(void *data) {
    UIMap *map = (UIMap*) data;
    double duration = map->linearInterpolationData.duration;
    int fps = map->linearInterpolationData.fps;
    double delay = map->linearInterpolationData.delay;
    int step = ++map->linearInterpolationData.currentStep;

    Real currentTime_ = step/(duration*fps) - delay;
    if(currentTime_ < 0)
        map->currentTime = 0;
    else if(currentTime_ > 1)
        map->currentTime = 1;
    else
        map->currentTime = currentTime_;
    map->redraw();

    if(step < (delay + duration) * fps)
        Fl::repeat_timeout(1.0/fps, linearInterpolation, data);
    else
        map->currentTime = 1;
}

void UIMap::abortAnimation() {
    Fl::remove_timeout(linearInterpolation, (void *)this);
    currentTime = 1;
}

int UIMap::handle(int event) {
    switch(event) {
        case FL_PUSH:
            {
            int event_button = Fl::event_button();
            if(event_button == FL_LEFT_MOUSE) {
                if(Fl::event_key(FL_Alt_L)) {
                    altLeftMouseButtonDown(Fl::event_x(), Fl::event_y());
                    return 1;
                }
                else {
                    leftMouseButtonDown(Fl::event_x(), Fl::event_y());
                    return 1;
                }
            }
            else if(event_button == FL_RIGHT_MOUSE) {
                rightMouseButtonDown(Fl::event_x(), Fl::event_y());
                return 1;
            }
            return Fl_Gl_Window::handle(event);
            }

        case FL_MOVE:
        case FL_DRAG:
            if(pickedUpNode != nullptr) {
                pickedUpNode->displaced.x = pickedUpNode->current.x
                                          = net.screenToWorldX(mouseX(Fl::event_x()));
                pickedUpNode->displaced.y = pickedUpNode->current.y
                                          = net.screenToWorldY(mouseY(Fl::event_y()));
                redraw();
            }
            else if(!Fl::has_timeout(linearInterpolation, (void *)this))
                redraw();
            return 1;

        default:
            return Fl_Gl_Window::handle(event);
    }
}

void UIMap::leftMouseButtonDown(int X, int Y) {
    Point mouse = Point(net.screenToWorldX(mouseX(X)), net.screenToWorldY(mouseY(Y)));
    Segment *s = net.closestSegment(&mouse);
    s->isFocus = true;
    s->requestedLength *= 2;

    UIWindow *window = (UIWindow *) parent();
    window->oldSolve();

    abortAnimation();
    redraw();
}

void UIMap::rightMouseButtonDown(int X, int Y) {
    Point mouse = Point(net.screenToWorldX(mouseX(X)), net.screenToWorldY(mouseY(Y)));
    Segment *s = net.closestSegment(&mouse);
    s->isFocus = true;
    s->requestedLength /= 2;

    UIWindow *window = (UIWindow *) parent();
    window->oldSolve();

    abortAnimation();
    redraw();
}

void UIMap::altLeftMouseButtonDown(int X, int Y) {
    const Real x = mouseX(X);
    const Real y = mouseY(Y);

    net.resetMap(true);
    int congestedSegs = 0;
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
    cout << "Congesting " << congestedSegs << " segments..." << endl;

    UIWindow *window = (UIWindow *) parent();
    window->oldSolve();

    randomPointCongestionAnimation();
}

Real UIMap::mouseX(int X) {
    return (Real(X) / w())*1.02 - 0.01;
}

Real UIMap::mouseY(int Y) {
    return 1.01 - 1.02*(Real(Y)/h());
}

UIMap::UIMap(int X, int Y, int W, int H, const char *L)
    : Fl_Gl_Window(X, Y, W, H, L) {
    end();
}

void UIMap::screenshot( const string &filenameBase ) {
    static int count = 0;
    count++;
    int width = 1000;
    int height = 1000;
    unsigned char *imageData = new unsigned char[ width * height * 4 ];
    glReadPixels( 0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, imageData );
    int stride = 4*width;
    for( int x=0; x<width; ++x ) {
        for( int y=0; y<height/2; ++y ) {
            swap( imageData[ y*stride + x*4 + 0 ], imageData[ (height-y-1)*stride + x*4 + 0 ] );
            swap( imageData[ y*stride + x*4 + 1 ], imageData[ (height-y-1)*stride + x*4 + 1 ] );
            swap( imageData[ y*stride + x*4 + 2 ], imageData[ (height-y-1)*stride + x*4 + 2 ] );
            swap( imageData[ y*stride + x*4 + 3 ], imageData[ (height-y-1)*stride + x*4 + 3 ] );
        }
    }
    stringstream fname;
    fname << filenameBase;
    fname << setw(4) << setfill('0') << count;
    fname << ".png";
    unsigned int error = lodepng_encode32_file(fname.str().c_str(), imageData, 1000, 1000);
    if( error )
        cout << "Error writing png: " << error << " : " << lodepng_error_text(error);
}

void UIMap::randomEdgeLengthsAnimation() {
    abortAnimation();
    double duration = 1.0;
    int fps = 60;
    double delay = 0.3;
    linearInterpolationData = AnimationData(duration, fps, delay);
    Fl::add_timeout(1.0/fps, linearInterpolation, (void *)this);
}

void UIMap::calculateMetromapAnimation() {
    abortAnimation();
    double duration = 1.0;
    int fps = 60;
    double delay = 0.3;
    linearInterpolationData = AnimationData(duration, fps, delay);
    Fl::add_timeout(1.0/fps, linearInterpolation, (void *)this);
}

void UIMap::setTargetZoomsAnimation() {
    abortAnimation();
    double duration = 1.0;
    int fps = 60;
    double delay = 0.3;
    linearInterpolationData = AnimationData(duration, fps, delay);
    Fl::add_timeout(1.0/fps, linearInterpolation, (void *)this);
}

void UIMap::randomStrokesCongestionAnimation() {
    abortAnimation();
    double duration = 1.0;
    int fps = 60;
    double delay = 0.3;
    linearInterpolationData = AnimationData(duration, fps, delay);
    Fl::add_timeout(1.0/fps, linearInterpolation, (void *)this);
}

void UIMap::randomPointCongestionAnimation() {
    abortAnimation();
    double duration = 1.0;
    int fps = 60;
    double delay = 0.3;
    linearInterpolationData = AnimationData(duration, fps, delay);
    Fl::add_timeout(1.0/fps, linearInterpolation, (void *)this);
}

void UIMap::netDistanceDisplacementAnimation() {
    abortAnimation();
    double duration = 1.0;
    int fps = 60;
    double delay = 0.3;
    linearInterpolationData = AnimationData(duration, fps, delay);
    Fl::add_timeout(1.0/fps, linearInterpolation, (void *)this);
}

void UIMap::cyclicMetroLineCongestionAnimation() {
    abortAnimation();
    double duration = 1.0;
    int fps = 60;
    double delay = 0.3;
    linearInterpolationData = AnimationData(duration, fps, delay);
    Fl::add_timeout(1.0/fps, linearInterpolation, (void *)this);
}

void UIMap::changeWeightsAnimation() {
    abortAnimation();
    double duration = 0.5;
    int fps = 60;
    double delay = 0.0;
    linearInterpolationData = AnimationData(duration, fps, delay);
    Fl::add_timeout(1.0/fps, linearInterpolation, (void *)this);
}

void UIMap::toggleResolveAnglesAnimation() {
    abortAnimation();
    double duration = 1.0;
    int fps = 60;
    double delay = 0.0;
    linearInterpolationData = AnimationData(duration, fps, delay);
    Fl::add_timeout(1.0/fps, linearInterpolation, (void *)this);
}

void UIMap::toggleNllsDisplacementAnimation() {
    abortAnimation();
    double duration = 1.0;
    int fps = 60;
    double delay = 0.0;
    linearInterpolationData = AnimationData(duration, fps, delay);
    Fl::add_timeout(1.0/fps, linearInterpolation, (void *)this);
}

void UIMap::toggleIntersectionConstraintsAnimation() {
    abortAnimation();
    double duration = 1.0;
    int fps = 60;
    double delay = 0.0;
    linearInterpolationData = AnimationData(duration, fps, delay);
    Fl::add_timeout(1.0/fps, linearInterpolation, (void *)this);
}

void UIMap::toggleProximityConstraintsAnimation() {
    abortAnimation();
    double duration = 1.0;
    int fps = 60;
    double delay = 0.0;
    linearInterpolationData = AnimationData(duration, fps, delay);
    Fl::add_timeout(1.0/fps, linearInterpolation, (void *)this);
}

void UIMap::pickUpNode(int X, int Y) {
    pickedUpNode = net.closestPoint(net.screenToWorldX(mouseX(X)),
                                    net.screenToWorldY(mouseY(Y)));
    redraw();
}

void UIMap::dropNode(int X, int Y) {
    if(pickedUpNode == nullptr) return;
    pickedUpNode->displaced.x = pickedUpNode->current.x = net.screenToWorldX(mouseX(X));
    pickedUpNode->displaced.y = pickedUpNode->current.y = net.screenToWorldY(mouseY(Y));
    pickedUpNode = nullptr;
}

void UIMap::redraw_map() {
    abortAnimation();
    redraw();
}
