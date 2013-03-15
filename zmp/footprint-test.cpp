#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include "zmp/footprint.h"
#include <math.h>

using namespace std;

TEST(Footprint, lineWalking)
{
  vector<Footprint> res = walkLine(10, 1, 3);
  EXPECT_FALSE(res.empty());
  EXPECT_LT(res.size(), 10000);
  int N = res.size();
  EXPECT_DOUBLE_EQ(res[0].x, 0);
  EXPECT_DOUBLE_EQ(res[1].x, 0);
  EXPECT_DOUBLE_EQ(res[2].x, 2.5);
  EXPECT_DOUBLE_EQ(res[3].x, 5);
  EXPECT_DOUBLE_EQ(res[4].x, 7.5);
  EXPECT_DOUBLE_EQ(res[N-2].x, 10);
  EXPECT_DOUBLE_EQ(res[N-1].x, 10);
  EXPECT_LT(res[N-3].x, 9);

  EXPECT_TRUE(res[0].is_left);
  EXPECT_FALSE(res[1].is_left);

  EXPECT_FALSE(res[N-2].is_left);
  EXPECT_TRUE(res[N-1].is_left);


  for ( int i = 0; i < N; i++ ) {
    Footprint &fp = res[i];
    EXPECT_DOUBLE_EQ(fp.y, (fp.is_left-0.5)*2);
  }
}

#define EXPECT_FEET_EQ(foot, _x, _y, _theta, _is_left) \
  EXPECT_NEAR((foot).x, _x, 1e-12); \
  EXPECT_NEAR((foot).y, _y, 1e-12); \
  EXPECT_DOUBLE_EQ((foot).theta, _theta); \
  EXPECT_EQ((foot).is_left, _is_left); \

#define EXPECT_FEET_EQ2(foot1, foot2) \
  EXPECT_FEET_EQ(foot1, (foot2).x, (foot2).y, (foot2).theta, (foot2).is_left);

const bool LEFT = true;
const bool RIGHT = false;
const double pi = atan(1.0)*4;

TEST(Footprint, circleWalking)
{
  double radius;
  double distance;
  double width;
  double max_step_length;
  double max_step_angle;
  Footprint* init_left;
  Footprint* init_right;
  bool left_is_stance_foot;


  vector<Footprint> res = walkCircle(
      radius = 10,
      distance = 10*pi, // Walk half the circle
      width = 0.1,
      max_step_length = pi+1e-15,
      max_step_angle = pi, // We don't care about it for now
      init_left = new Footprint(0, 0.1, 0, true),
      init_right = new Footprint(0, -0.1, 0, false),
      left_is_stance_foot = false // We start moving the left foot
    );

  EXPECT_FALSE(res.empty());
  EXPECT_LT(res.size(), 10000);
  int N = res.size();

  ASSERT_EQ(N, 12);
  EXPECT_FEET_EQ(res[0]  , 0        , -width   , 0    , RIGHT);
  EXPECT_FEET_EQ(res[5]  , 10-width , 10       , pi/2 , LEFT);
  EXPECT_FEET_EQ(res[10] , 0        , 20+width , pi   , RIGHT);
  EXPECT_FEET_EQ(res[11] , 0        , 20-width , pi   , LEFT);
}



struct TestHelper {
  double radius;
  double distance;
  double width;
  double max_step_length;
  double max_step_angle;
  Footprint* init_left;
  Footprint* init_right;
  bool left_is_stance_foot;
  vector<Footprint> res;

  /* TestHelper () {} */
  TestHelper (
      double radius,
      double distance,
      double width,
      double max_step_length,
      double max_step_angle,
      Footprint* init_left,
      Footprint* init_right,
      bool left_is_stance_foot
  ) : radius(radius), distance(distance), width(width),
             max_step_length(max_step_length), max_step_angle(max_step_angle),
             init_left(init_left), init_right(init_right),
             left_is_stance_foot(left_is_stance_foot) {
    calcAndSetRes();
  }

  void setRandom() {

  }

  void calcAndSetRes() {
    res = walkCircle(
      radius,
      distance,
      width,
      max_step_length,
      max_step_angle,
      init_left,
      init_right,
      left_is_stance_foot
    );
  }

  void checkAllProperties(){
    propertyFirstFootIsCorrect();
    propertyStepLengthAndAngle();
  }

  void propertyFirstFootIsCorrect() {
    EXPECT_EQ(res[0].is_left, left_is_stance_foot);
    Footprint *foot = left_is_stance_foot ? init_left : init_right;
    EXPECT_FEET_EQ2(res[0], *foot);
  }

  void propertyStepLengthAndAngle() {
    vector<Footprint> lefts, rights;
    for(unsigned int i = 0; i < res.size(); i++) {
      (res[i].is_left ? lefts : rights).push_back(res[i]);
    }
    propertyOnesStepLength(lefts);
    propertyOnesStepLength(rights);
    propertyOnesAngle(lefts);
    propertyOnesAngle(rights);
  }

  void propertyOnesStepLength(vector<Footprint> fps) {
    // Here, we check that the footsteps for the SAME foot are correct.
    for(unsigned int i = 0; i < fps.size() - 1; i++) {
      // TODO Much pretteier when using Eigen later
      double dx = res[i+1].x - res[i].x;
      double dy = res[i+1].y - res[i].y;
      EXPECT_LT(dx*dx + dy*dy, max_step_length*max_step_length);
    }
  }

  void propertyOnesAngle(vector<Footprint> fps) {
    for(unsigned int i = 0; i < fps.size() - 1; i++) {
      EXPECT_LT(abs(res[i+1].theta - res[i].theta), max_step_angle);
    }
  }
};

TEST(Footprint, circleWalking2)
{
  double width;
  TestHelper th(-10, //radius
      10*pi, //distance
      width = 0.1, //width
      10000, // max_step_length
      pi/10+1e-14, //max_step_angle
      new Footprint(0, 0.1, 0, true), // init_left
      NULL, // init_right
      true // left_is_stance_foot
    );
  th.checkAllProperties();
  vector<Footprint> res(th.res);

  EXPECT_FALSE(res.empty());
  EXPECT_LT(res.size(), 10000);
  int N = res.size();

  EXPECT_EQ(N, 12);
  ASSERT_GE(N, 12);
  /* EXPECT_FEET_EQ(res[0]  , 0        , width     , 0     , LEFT); */
  EXPECT_FEET_EQ(res[5]  , 10-width , -10       , -pi/2 , RIGHT);
  /* EXPECT_FEET_EQ(res[10] , 0        , -20-width , -pi   , LEFT); */
  /* EXPECT_FEET_EQ(res[11] , 0        , -20+width , -pi   , RIGHT); */
}

/* TEST(Footprint, circleWalking4) */
/* { */
/*   TestHelper th(-10, //radius */
/*       10*pi, //distance */
/*       0.1, //width */
/*       10000, // max_step_length */
/*       pi/10+1e-14, //max_step_angle */
/*       new Footprint(0, -0.1, 0, true), // init_left */
/*       NULL, // init_right */
/*       true // left_is_stance_foot */
/*     ); */
/*   th.checkAllProperties(); */
/*   /1* vector<Footprint> res(th.res); *1/ */
/* } */

