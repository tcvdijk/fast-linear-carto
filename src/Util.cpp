#include "Util.h"

#include <iostream>
#include <cmath>
#include <cstdlib>

Real lerp( Real a, Real b, Real t ) {
	return (1.0-t)*a + t*b;
}

Real correlation(std::vector<Real> x, std::vector<Real> y) {
    if(x.size() != y.size()) {
        std::cout << "Error: Vector sizes don't match!" << std::endl;
        exit(EXIT_FAILURE);
    }
    else if(x.size() == 0) {
        std::cout << "Error: Vector size 0!" << std::endl;
        exit(EXIT_FAILURE);
    }
    Real xAvg = 0;
    Real yAvg = 0;
    for(int i = 0; i < x.size(); i++) {
        xAvg += x[i];
        yAvg += y[i];
    }
    xAvg /= x.size();
    yAvg /= y.size();
    Real ss_xx = 0;
    Real ss_yy = 0;
    Real ss_xy = 0;
    for(int i = 0; i < x.size(); i++) {
        ss_xx += pow(x[i]-xAvg, 2);
        ss_yy += pow(y[i]-yAvg, 2);
        ss_xy += (x[i]-xAvg)*(y[i]-yAvg);
    }
    if(ss_xx == 0 && ss_yy == 0) {
        std::cout << "Error: Standard deviation 0!" << std::endl;
        exit(EXIT_FAILURE);
    }
    return sqrt(pow(ss_xy, 2) / (ss_xx * ss_yy));
}
