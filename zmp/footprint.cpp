#include <stdlib.h>
#include <math.h>
#include <vector>
#include "footprint.h"
#include <Eigen/Dense>
#include <iostream>

using namespace std;

bool is_even(int i) {
  return (i%2) == 0;
}

vector<Footprint> walkLine(double dist, double width, double max_step_length) {
  // Solve simple equation
  const int K = int(ceil(dist/max_step_length) + 1e-10);
  const double L = dist/K;
  const int N = K+4;
  vector<Footprint> res(N);

  // Do all steps
  for ( int i = 0; i < N; i++ ) {
    res[i].x = (i-1)*L;
    res[i].is_left = is_even(i);
    res[i].y = width * (res[i].is_left-0.5)*2;
  }
  res[0].x = res[1].x = 0.0;
  res[N-2].x = res[N-1].x = dist;
  return res;
}

vector<Footprint> walkCircle(double radius,
                             double distance,
                             double width,
                             double max_step_length,
                             double max_step_angle,
                             Footprint* init_left,
                             Footprint* init_right,
                             stance_t stance_handedness) {
    // select stance foot, fill out transforms
    Footprint* stance_foot;
    bool left_is_stance_foot = stance_handedness == SINGLE_LEFT || stance_handedness == DOUBLE_LEFT;
    if (left_is_stance_foot) stance_foot = init_left;
    else stance_foot = init_right;
    Eigen::Transform<double,2,Eigen::Affine> T_stance_to_world;
    T_stance_to_world =
        Eigen::Translation<double, 2>(stance_foot->x, stance_foot->y)
        * Eigen::Rotation2D<double>(stance_foot->theta)
        * Eigen::Translation<double, 2>(0, left_is_stance_foot?-width:width);
    
    // minimize K subject to conditions, compute resulting dTheta
    int K = ceil(distance / max_step_angle * abs((radius - width) / radius));
    double dTheta = distance / (K * radius);
    if (abs(dTheta) > max_step_angle) {
        K = ceil(distance / abs(radius) * max_step_angle);
        dTheta = distance / (K * radius);
    }
    
    // init results list
    vector<Footprint> result;
    
    // fill out results
    for(int i = 2; i < K + 1; i++) {
        double theta_i = dTheta * (i - 1);
        if (is_even(i) xor left_is_stance_foot) { // i odd means this step is for the stance foot
            result.push_back(Footprint((radius - width) * sin(theta_i),
                                       radius - ((radius - width) * cos(theta_i)),
                                       theta_i,
                                       true));
        } 
        else {
            result.push_back(Footprint((radius + width) * sin(theta_i),
                                       radius - ((radius + width) * cos(theta_i)),
                                       theta_i,
                                       false));
        }
    }

    // fill out the last two footsteps
    double theta_last = dTheta * K;
    if (is_even(K) xor left_is_stance_foot) { // K even means we end on the stance foot
        result.push_back(Footprint((radius + width) * sin(theta_last),
                                   radius - ((radius + width) * cos(theta_last)),
                                   theta_last,
                                   false));
        result.push_back(Footprint((radius - width) * sin(theta_last),
                                   radius - ((radius - width) * cos(theta_last)),
                                   theta_last,
                                   true));
    } 
    else {
        result.push_back(Footprint((radius - width) * sin(theta_last),
                                   radius - ((radius - width) * cos(theta_last)),
                                   theta_last,
                                   true));
        result.push_back(Footprint((radius + width) * sin(theta_last),
                                   radius - ((radius + width) * cos(theta_last)),
                                   theta_last,
                                   false));
    }
    result.insert(result.begin(), Footprint(*stance_foot));
    
    // run through results transforming them back into the original frame of reference
    for(std::vector<Footprint>::iterator it = result.begin(); it < result.end(); it++) {
        Eigen::Vector2d t(it->x, it->y);
        t = T_stance_to_world * t;
        it->x = t.x();
        it->y = t.y();
        it->theta = it->theta + stance_foot->theta;
    }
    
    // return the result
    return result;
}
                             

int main() {
    double radius = 1;
    double distance = 2;
    double width = .2;
    double max_length = .5;
    double max_angle = M_PI / 6;

    Footprint* foot_l = new Footprint(0, width, 0, true);
    Footprint* foot_r = new Footprint(0, -width, 0, false);
    stance_t stance_handedness = SINGLE_LEFT;        // start on right foot
    std::vector<Footprint> footprints;

    footprints = walkCircle(radius,
                            distance,
                            width,
                            max_length,
                            max_angle,
                            foot_l,
                            foot_r,
                            stance_handedness);
    
    for(std::vector<Footprint>::iterator it = footprints.begin(); it < footprints.end(); it++) {
        // std::cout << "[" << it->x << ", " << it->y << " @ " << it->theta << "]" << std::endl;
        std::cout
            << it->x << ", "
            << it->y << ", "
            << .3 * cos(it->theta) << ", "
            << .3 * sin(it->theta)
            << std::endl;
    }
}
