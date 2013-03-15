#ifndef  FOOTPRINT_INC
#define  FOOTPRINT_INC

#include <vector>

class Footprint {
  Eigen::Transform3d transform;
  stance_t is_left;

  Footprint(double x, double y, double theta, stance_t is_left);
  Footprint();

  Eigen::Transform3d getTransform3d();
  void setTransform3d(Eigen::Transform3d transform);

};

std::vector<Footprint> walkLine(double dist, /// The distance to walk in meters
    double width, /// The ground distance between the center of the robot to the center of a foot
    double max_step_length /// The maximum allowed length the robot may step
  );

std::vector<Footprint> walkCircle(double radius, /// The radius of the circle to walk in
                             double distance, /// The distance to walk along the circle
                             double width, /// The distance between the center of the robot and the center of a foot
                             double max_step_length, /// The maximum allowed length the robot may step
                             double max_step_angle, /// The maximum angle between successive steps
                             Footprint* init_left, /// The position the left foot starts in
                             Footprint* init_right, /// The position the right foot starts in
                             stance_t stance_handedness /// Which foot the robot will be starting off standing on
    );

#endif   /* ----- #ifndef FOOTPRINT_INC  ----- */

/* Local Variables: */
/* mode: c++ */
/* c-basic-offset: 2 */
/* End: */
