
UNAME := $(shell uname)

CC = gcc
CCFLAGS = -g -Wall -O2

CPP = g++
CPP_FLAGS = -std=c++11 -O2

FLTK_CPP_FLAGS = `fltk-config --cxxflags | sed -e "s/-mwindows //"`
FLTK_FLAGS = `fltk-config --use-gl --ldflags | sed -e "s/-mwindows //"`

GL_FLAG =
ifeq ($(UNAME),Linux)
	GL_FLAG = -lGL
endif

OBJ = pugixml/pugixml.o shapelib/shpopen.o shapelib/shptree.o shapelib/safileio.o shapelib/dbfopen.o Point.o RoadNetwork.o Segment.o Util.o lodepng.o UIMap.o UIControls.o UISlider.o UILabel.o UIWindow.o

default: $(OBJ) Timer.h.gch main.exe experiments.exe

static: CPP_FLAGS += -static
static: FLTK_FLAGS = `fltk-config --use-gl --ldstaticflags | sed -e "s/-mwindows //"`
static: GL_FLAG = -lgdi32
static: default

pugixml/pugixml.o: pugixml/pugixml.cpp pugixml/pugixml.hpp pugixml/pugiconfig.hpp
	$(CPP) $(CPP_FLAGS) -o pugixml/pugixml.o -c pugixml/pugixml.cpp

shapelib/shpopen.o: shapelib/shpopen.c shapelib/shapefil.h
	$(CC) $(CCFLAGS) -o shapelib/shpopen.o -c shapelib/shpopen.c

shapelib/shptree.o: shapelib/shptree.c shapelib/shapefil.h
	$(CC) $(CCFLAGS) -o shapelib/shptree.o -c shapelib/shptree.c

shapelib/dbfopen.o: shapelib/dbfopen.c shapelib/shapefil.h
	$(CC) $(CCFLAGS) -o shapelib/dbfopen.o -c shapelib/dbfopen.c

shapelib/safileio.o: shapelib/safileio.c shapelib/shapefil.h
	$(CC) $(CCFLAGS) -o shapelib/safileio.o -c shapelib/safileio.c

Point.o: Point.cpp Point.h RoadNetwork.h Util.h pugixml/pugixml.hpp
	$(CPP) $(CPP_FLAGS) -c Point.cpp

RoadNetwork.o: RoadNetwork.cpp RoadNetwork.h Point.h Segment.h Timer.h.gch Util.h pugixml/pugixml.hpp Eigen/Sparse shapelib/shapefil.h
	$(CPP) $(CPP_FLAGS) -c RoadNetwork.cpp

Segment.o: Segment.cpp Segment.h RoadNetwork.h Point.h Util.h
	$(CPP) $(CPP_FLAGS) -c Segment.cpp

Util.o: Util.cpp Util.h lodepng.h
	$(CPP) $(CPP_FLAGS) -c Util.cpp

lodepng.o: lodepng.cpp lodepng.h
	$(CPP) $(CPP_FLAGS) -c lodepng.cpp

Timer.h.gch: Timer.h
	$(CPP) $(CPP_FLAGS) -c Timer.h

UIMap.o: UIMap.cpp UIMap.h RoadNetwork.h Segment.h Point.h lodepng.h Util.h
	$(CPP) $(CPP_FLAGS) $(FLTK_CPP_FLAGS) -c UIMap.cpp

UIControls.o: UIControls.cpp UIControls.h UISlider.h UILabel.h UIWindow.h
	$(CPP) $(CPP_FLAGS) $(FLTK_CPP_FLAGS) -c UIControls.cpp

UISlider.o: UISlider.cpp UISlider.h
	$(CPP) $(CPP_FLAGS) $(FLTK_CPP_FLAGS) -c UISlider.cpp

UILabel.o: UILabel.cpp UILabel.h
	$(CPP) $(CPP_FLAGS) $(FLTK_CPP_FLAGS) -c UILabel.cpp

UIWindow.o: UIWindow.cpp UIWindow.h UIMap.h UIControls.h RoadNetwork.h Segment.h Point.h
	$(CPP) $(CPP_FLAGS) $(FLTK_CPP_FLAGS) -c UIWindow.cpp

main.exe: main.cpp $(OBJ) Timer.h.gch
	$(CPP) $(CPP_FLAGS) $(FLTK_CPP_FLAGS) -o main.exe main.cpp $(OBJ) $(FLTK_FLAGS) $(GL_FLAG)

experiments.exe: experiments.cpp $(OBJ) Timer.h.gch
	$(CPP) $(CPP_FLAGS) $(FLTK_CPP_FLAGS) -o experiments.exe experiments.cpp $(OBJ) $(FLTK_FLAGS) $(GL_FLAG)

.Phony: clean
clean:
	rm $(OBJ) Timer.h.gch main.exe experiments.exe
