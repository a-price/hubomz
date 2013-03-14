#include "gait.h"
#include <iostream>

using namespace Eigen;
using namespace std;

const int stance_foot_table[4] = { 0, 1, 0, 1 };
const int swing_foot_table[4] = { -1, -1, 1, 0 };

const stance_t next_stance_table[4] = {
		SINGLE_LEFT,
		SINGLE_RIGHT,
		DOUBLE_RIGHT,
		DOUBLE_LEFT
};

vector<step_t> steps(int num_steps, bool firstleft) {
	vector<step_t> step_seq;
	step_seq.reserve(num_steps + 2);
	step_t step;

	double sway = 0.085;
	double forward = 0.15;

	int side = 1;
	stance_t stance = DOUBLE_LEFT; // Always start with left foot; w/e

	// Start step in double stance
	// pos will determine zmp in this step
	step.pos << 0,0;  // x, y: forward, side
	step.support = stance;
	step_seq.push_back(step);
	for(int i=0; i < num_steps; i++) {
		stance = next_stance_table[stance]; // SINGLE type stance
		if(i == 0) {
			// First step is a half step
			step.pos(0) += forward / 2;
			step.pos(1) = side * sway;
		} else if(i < num_steps-1) {
			// Middle steps are full steps
			step.pos(0) += forward;
			step.pos(1) = side * sway;
		} else {
			// Last step is also a half step
			step.pos(0) += forward / 2;
			step.pos(1) = side * sway;
		}
		step.support = stance;
		step_seq.push_back(step);
		// update vars to next step
		side *= -1;
		stance = next_stance_table[stance]; // DOUBLE type stance
	}
	// End step in double stance
	step.pos(1) = 0; // set zmp to center
	step.support = stance; // Should be DOUBLE type
	step_seq.push_back(step);

	return step_seq;
}

step_traj_t step_times(vector<step_t> step_seq, step_timer_t timer) {
	EIGEN_V_VEC2D zmpref; // b/c cannot dynamically resize matrixxd
	vector<stance_t> stance;

	int n_steps = step_seq.size(); // number of real foot steps

	assert(n_steps >= 0); // first and last step are stationary steps

	size_t curr_ticks = 0;
	size_t step_ticks;
	size_t double_support_ticks = 0;
	size_t single_support_ticks = 0;
	stance_t s = DOUBLE_LEFT;
	Vector2d zmp;
	for(int i=0; i < n_steps; i++) {
		s = step_seq[i].support;
		// Always start step in double support phase
		//
		if(s == SINGLE_LEFT) {
			s = DOUBLE_LEFT;
		}
		if(s == SINGLE_RIGHT) {
			s = DOUBLE_RIGHT;
		}
		zmp = step_seq[i].pos;

		if(i == 0) {
			// hold starting position
			double_support_ticks = timer.compute_startup();
			single_support_ticks = 0;
		} else if(i < n_steps - 1) {
			// real foot steps
			double_support_ticks = timer.compute_double(0,0,0);
			single_support_ticks = timer.compute_single(0,0,0);
		} else {
			// hold ending position
			double_support_ticks = timer.compute_shutdown();
			single_support_ticks = 0;
		}
		step_ticks = double_support_ticks + single_support_ticks;

		// reserve space in arrays
		zmpref.reserve(curr_ticks + step_ticks);
		stance.reserve(curr_ticks + step_ticks);

		for(size_t j=0; j < double_support_ticks; j++) {
			zmpref.push_back(zmp);
			stance.push_back(s);
		}
		s = next_stance_table[s];
		curr_ticks += double_support_ticks;
		for(size_t j=0; j < single_support_ticks; j++) {
			zmpref.push_back(zmp);
			stance.push_back(s);
		}
		s = next_stance_table[s];
		curr_ticks += single_support_ticks;
	}

	step_traj_t traj;
	traj.zmpref = zmpref;
	traj.stance = stance;

	return traj;
}


