#include <gtest/gtest.h>
#include <iostream>
#include <vector>
#include "zmp/footprint.h"

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

TEST(Footprint, circleWalking)
{
  /* vector<Footprint> res = walkLine(10, 1, 3); */
  /* int N = res.size(); */
  /* EXPECT_GT(N, 1); */
  /* EXPECT_LT(res.size(), 10000); */
}
