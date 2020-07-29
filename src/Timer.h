#ifndef INCLUDED_TIMER
#define INCLUDED_TIMER

#include <ctime>
#include <iostream>

namespace Wue {
	class Timer {
	public:

		Timer() {
			start = std::clock();
		}
		double elapsed() const {
			return (std::clock()-start) / CLOCKS_PER_SEC;
		}
		double report() {
			double t = elapsed();
			std::cout << "Time: " << int(t*1000) << " ms" << std::endl;
			return t;
		}

	protected:
		double start;

	};
}

#endif //ndef INCLUDED_TIMER