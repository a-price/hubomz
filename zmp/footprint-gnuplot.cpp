#include <math.h>
#include <vector>
#include <iostream>
#include "footprint.h"

int main() {
    double radius = 1;
    double distance = 2;
    double width = .2;
    double max_length = .5;
    double max_angle = M_PI / 6;

    Footprint stance_foot = Footprint(0, width, 0, true);
    std::vector<Footprint> footprints;

    footprints = walkCircle(radius,
                            distance,
                            width,
                            max_length,
                            max_angle,
                            stance_foot);

    for(std::vector<Footprint>::iterator it = footprints.begin(); it < footprints.end(); it++) {
        // std::cout << "[" << it->x << ", " << it->y << " @ " << it->theta << "]" << std::endl;
        std::cout
            << it->x() << ", "
            << it->y() << ", "
            << .3 * cos(it->theta()) << ", "
            << .3 * sin(it->theta())
            << std::endl;
    }
	return 0;
}

