#include <stdlib.h>
#include <math.h>
#include <vector>
#include "footprint.h"
#include <iostream>
#include "mzcommon/quat.h"

using namespace std;
using namespace fakerave;

Footprint::Footprint(Transform3 t, bool is_left) {
    this->transform = t;
    this->is_left = is_left;
}
Footprint::Footprint(double x, double y, double theta, bool is_left) {
    vec3 translation(x, y, 0);
    Transform3 transform(quat::fromAxisAngle(vec3(0.0,0.0,1.0), theta), vec3(x, y, 0));
    this->transform = transform;
    this->is_left = is_left;
}

double Footprint::x() const { return this->transform.translation().x(); }
double Footprint::y() const{ return this->transform.translation().y(); }
double Footprint::theta() const {
    vec3 forward = this->transform.rotFwd() * vec3(1.0, 0.0, 0.0);
    return atan2(forward.y(), forward.x());
}


Footprint::Footprint(){
    Footprint( 0.0, 0.0, 0.0, false );
}

Transform3 Footprint::getTransform3() const {

    return transform;
}

void Footprint::setTransform3(Transform3 transform){
    this->transform = transform;
}

Transform3 Footprint::getMidTransform3(double width) const {
  return getTransform3() * Transform3(quat(), vec3(0, is_left?-width:width, 0));
}

bool is_even(int i) {
    return (i%2) == 0;
}

bool is_odd(int i) {
    return !is_even(i);
}

vector<Footprint> walkLine(double distance,
                           double width,
                           double max_step_length,
                           Footprint stance_foot) {
    return walkCircle(1e13,
                      distance,
                      width,
                      max_step_length,
                      3.14,
                      stance_foot);
}

vector<Footprint> walkCircle(double radius,
                             double distance,
                             double width,
                             double max_step_length,
                             double max_step_angle,
                             Footprint stance_foot) {
    assert(width > 0);
    assert(max_step_length > 0);
    assert(max_step_angle >= -M_PI);
    assert(max_step_angle < M_PI);
    if(radius == 0) radius = 1e13;

    bool left_is_stance_foot = stance_foot.is_left;
    int sign = distance < 0? -1 : 1;

    // select stance foot, fill out transforms
    Transform3 T_circle_to_world =
        stance_foot.transform
        * Transform3(quat(), vec3(0, left_is_stance_foot?-width:width, 0));

    // alpha = angle of arc to walk
    double alpha = abs(distance) / abs(radius);
    double outer_dist = (abs(radius) + width) * alpha;
    int K_steps = ceil(outer_dist / max_step_length);
    int K_angle = ceil(alpha / max_step_angle);
    int K = max(K_steps, K_angle);
    double dTheta = alpha/K * (radius > 0 ? 1 : -1 );

#ifdef DEBUG
    cout << "outer_dist IS " << outer_dist << endl;
    cout << outer_dist / max_step_length << endl;
    cout << "Ksl IS " << K_steps << endl;
    cout << "Kang IS " << K_angle << endl;
    cout << "K IS " << K << endl;
    cout << "dTheta IS " << dTheta << endl;
#endif

    // init results list
    vector<Footprint> result;

    // fill out results
    for(int i = 2; i < K + 1; i++) {
        double theta_i = dTheta * (i - 1);
        if (is_even(i) xor left_is_stance_foot) { // i odd means this step is for the stance foot
            // add left footstep
            result.push_back(Footprint(sign*(radius - width) * sin(theta_i),
                                       radius - ((radius - width) * cos(theta_i)),
                                       theta_i,
                                       true));
        }
        else {
            // add right footstep
            result.push_back(Footprint(sign*(radius + width) * sin(theta_i),
                                       radius - ((radius + width) * cos(theta_i)),
                                       theta_i,
                                       false));
        }
    }

    // fill out the last two footsteps
    double theta_last = dTheta * K;
    if (is_even(K) xor left_is_stance_foot) { // K even means we end on the stance foot
        // add right footstep
        result.push_back(Footprint(sign*(radius + width) * sin(theta_last),
                                   radius - ((radius + width) * cos(theta_last)),
                                   theta_last,
                                   false));
        // add left footstep
        result.push_back(Footprint(sign*(radius - width) * sin(theta_last),
                                   radius - ((radius - width) * cos(theta_last)),
                                   theta_last,
                                   true));
    }
    else {
        // add left footstep
        result.push_back(Footprint(sign*(radius - width) * sin(theta_last),
                                   radius - ((radius - width) * cos(theta_last)),
                                   theta_last,
                                   true));
        // add right footstep
        result.push_back(Footprint(sign*(radius + width) * sin(theta_last),
                                   radius - ((radius + width) * cos(theta_last)),
                                   theta_last,
                                   false));
    }

    // run through results transforming them back into the original frame of reference
    for(std::vector<Footprint>::iterator it = result.begin(); it < result.end(); it++) {
        it->transform = T_circle_to_world * it->transform;
    }

//    for(std::vector<Footprint>::iterator it = result.begin(); it < result.end(); it++)
//      std::cout << "[" << it->x() << ", " << it->y() << "\n";

    // return the result
    return result;
}

vector<Footprint> sidestep(double dist,
                           double halfWidth,
                           double max_step_length,
                           Footprint stance_foot) {
    assert(halfWidth > 0);
    assert(max_step_length > 0);

    //FIXME TODO when going from walking forward to sidestepping, x gets doubled each trajectory
    // select stance foot, fill out transforms
    Transform3 T_line_to_world =
        stance_foot.transform
        * Transform3(quat(), vec3(0, stance_foot.is_left?-halfWidth:halfWidth, 0));

    // compute number of steps and step length
    int nSteps = ceil(abs(2*dist) / max_step_length);
    double stepLength = 2*dist / nSteps;

    // init results list
    vector<Footprint> result;
//    std::cout << "Stance foot is left? " << stance_foot.is_left << "\n";
    // get initial y-position of both feet
    double yL = halfWidth; //stance_foot.is_left == true ? stance_foot.y() : stance_foot.y() + (2*halfWidth);
    double yR = -halfWidth; //yL - (2*halfWidth);
    double theta_i = 0;

    // fill out results
    for(int i = 0; i < nSteps; i++) {
        if (is_even(i) xor stance_foot.is_left) { // i odd means this step is for the stance foot
            // add left footstep
            yL += stepLength;
            result.push_back(Footprint(0,
                                       yL,
                                       theta_i,
                                       true));
        }
        else {
            // add right footstep
            yR += stepLength;
            result.push_back(Footprint(0,
                                       yR,
                                       theta_i,
                                       false));
        }
    }

    // run through results transforming them back into the original frame of reference
    for(std::vector<Footprint>::iterator it = result.begin(); it < result.end(); it++) {
        it->transform = T_line_to_world * it->transform;
    }

//    for(std::vector<Footprint>::iterator it = result.begin(); it < result.end(); it++)
//      std::cout << "[" << it->x() << ", " << it->y() << "\n";

    // return the result
    return result;
}

vector<Footprint> turnInPlace(
                         double desired_angle, /// The desired angle
                         double width, /// The desired angle
                         double max_step_angle, /// The maximum HALF angle between successive steps
                         Footprint from /// Where we start from. Note that this exact foot will be repeated in the output
    )
{
  assert(max_step_angle >= -M_PI);
  assert(max_step_angle <   M_PI);
  assert(desired_angle >=  -M_PI);
  assert(desired_angle <    M_PI);
  assert(from.theta() >=   -M_PI);
  assert(from.theta() <     M_PI);

  double goal_angle = desired_angle-from.theta();
  if(goal_angle >= M_PI) goal_angle -= M_PI;
  else if( goal_angle < -M_PI) goal_angle -= M_PI;
  double eps = 1e-10;
  return walkCircle(eps, eps*goal_angle, width, 1000, max_step_angle, from);
}

Transform3 compensate(double width, bool is_left) {
  return Transform3(quat(), vec3(0, is_left?-width:width, 0));

}

vector<Footprint> walkTo(
    double width, /// The maximum HALF angle between successive steps
    double max_step_length, /// The maximum HALF allowed length the robot may step
    double max_step_angle, /// The maximum HALF angle between successive steps
    Footprint from, /// Where we start from. Note that this exact foot will be repeated in the output
    Footprint to /// Where we should end at. Note that this exact foot will be repeated in the output
    )
{
  /* double walk_angle = to. */
  Transform3 T_delta = from.getMidTransform3(width).inverse() * to.getMidTransform3(width);
// #define PRINT(var) cout << #var << ": " << var << endl
#define PRINT(var)
  PRINT(T_delta);
  /* to.y(), to.x() */
  vec3 trans = T_delta.translation();
  PRINT(trans);
  double walk_angle = atan2(trans.y(), trans.x());
  PRINT(walk_angle);

  vector<Footprint> turn1 = turnInPlace(walk_angle, width, max_step_angle, from);
  Footprint f1 = turn1.back();
  turn1.pop_back();
  PRINT(f1);

  double walk_length = trans.norm();
  PRINT(walk_length);
  vector<Footprint> turn2 = walkLine(walk_length, width, max_step_length, f1);
  Footprint f2 = turn2.back();
  turn2.pop_back();
  PRINT(f2);

  PRINT(to.theta());
  vector<Footprint> turn3 = turnInPlace(to.theta(), width, max_step_angle, f2);
  PRINT(turn3.back());

  vector<Footprint> total = turn1;
  total.insert(total.end(), turn2.begin(), turn2.end());
  total.insert(total.end(), turn3.begin(), turn3.end());
  return total;
}

/*
vector<Footprint> walkLine(double distance,
                           double halfFeetSep,
                           double max_step_length,
                           Footprint stance_foot) {

    // make sure inputs are valid
    assert(distance > 0);
    assert(halfFeetSep > 0);
    assert(max_step_length > 0);

    // select stance foot, fill out transforms
    Transform3 T_line_to_world = stance_foot.transform * Transform3(quat(), vec3(0, stance_foot.is_left?-halfFeetSep:halfFeetSep, 0));

    // calculate number of steps and step length for distance
    double numSteps = ceil(distance/max_step_length);
    double stepLength = distance/numSteps;
    double theta = 0;

    // instatiate result list
    vector<Footprint> result;
    double curX = stance_foot.x();

    // fill in result with footsteps
    for(int i = 2; i < numSteps + 1; i++) {
        curX += stepLength;
        // add left footstep
        if (is_even(i) xor stance_foot.is_left) { // i odd means this step is for the stance foot
            result.push_back(Footprint(curX, halfFeetSep, theta, true));
        }
        // add right footstep
        else {
            result.push_back(Footprint(curX, -halfFeetSep, theta, false));
        }
    }

    // fill in last two footprints
    // add left footstep
    if (is_even(numSteps) xor stance_foot.is_left) { // i odd means this step is for the stance foot
        result.push_back(Footprint(curX, halfFeetSep, theta, true));
    }
    // add right footstep
    else {
        result.push_back(Footprint(curX, -halfFeetSep, theta, false));
    }

    std::cout << "before transformed\n";
    for(int i = 0; i < (int)result.size(); i++)
    {
        std::cout << result.at(i).x() << ", " << result.at(i).y() << "\n";
    }

    // run through results transforming them back into the original frame of reference
    for(std::vector<Footprint>::iterator it = result.begin(); it < result.end(); it++) {
        it->transform = T_line_to_world * it->transform;
    }

    std::cout << "# of Footsteps: " << result.size() << std::endl;

    for(int i = 0; i < (int)result.size(); i++)
    {
        std::cout << result.at(i).x() << ", " << result.at(i).y() << "\n";
    }

    // return the result
    return result;
}
*/

