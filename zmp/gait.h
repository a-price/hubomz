#pragma once

#include "hubo-zmp.h"
#include <Eigen/Dense>
#include <Eigen/StdVector>
#include <vector>
#include <iostream>

typedef std::vector< Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > EIGEN_V_VEC2D;

struct step_t {
	Eigen::Vector2d pos;
	stance_t support;
};

struct step_traj_t {
	EIGEN_V_VEC2D zmpref;
	std::vector<stance_t> stance;
};

struct step_timer_t {
	step_timer_t()
		: single_support_time(0.70),
		  double_support_time(0.05),
		  startup_time(1.0),
		  shutdown_time(1.0),
		  duty_factor(0),
		  dist_gain(0),
		  theta_gain(0),
		  height_gain(0) {
	}

	double single_support_time; // minimum time for single support
	double double_support_time; // minimum time for double support
	double startup_time;
	double shutdown_time;
	// duty factor = support / support + transfer
	// (duty factor - 0.5) * 2 is the % of total time
	// spent in double support phase
	double duty_factor;

	double dist_gain; // time gain based on distance
	double theta_gain;
	double height_gain;

	void set_duty_cycle(double df, double step_time) {
		duty_factor = df;
		double_support_time = (df - 0.5) * 2 * step_time;
		single_support_time = (1 - df) * step_time;
	}

	void set_duty_single(double df, double single_time) {
		set_duty_cycle(df, single_time/(1-df));
	}

	void set_duty_double(double df, double double_time) {
		set_duty_cycle(df, double_time/((df-0.5)*2));
	}

	size_t seconds_to_ticks(double s) {
	  return size_t(round(s*TRAJ_FREQ_HZ));
	}

	size_t compute_startup() {
		return seconds_to_ticks(startup_time);
	}
	size_t compute_shutdown() {
		return seconds_to_ticks(shutdown_time);
	}
	size_t compute_double(double dist, double theta, double height) {
		double double_time = double_support_time;
		double_time += height_gain * height;
		double_time += dist_gain * dist;
		double_time += theta_gain * theta;
		return seconds_to_ticks(double_time);
	}
	size_t compute_single(double dist, double theta, double height) {
		double single_time = single_support_time;
		single_time += height_gain * height;
		single_time += dist_gain * dist;
		single_time += theta_gain * theta;
		return seconds_to_ticks(single_time);
	}

	friend std::ostream& operator<<(std::ostream& out, step_timer_t& t) {
		return
		out << "step-timer:\n"
			<< "start up time: " << t.startup_time << " s\n"
			<< "shutdown time: " << t.shutdown_time << " s\n"
		    << "single support time: " << t.single_support_time << " s\n"
		    << "double support time: " << t.double_support_time << " s\n"
		    << "duty factor set: " << t.duty_factor << "\n"
		    << "duty factor calc: " << (t.single_support_time + t.double_support_time)/(2*t.single_support_time + t.double_support_time) << "\n"
		    << "dist gain:   " << t.dist_gain << " s/m\n"
		    << "theta gain:  " << t.theta_gain << " s/m\n"
		    << "height gain: " << t.height_gain << " s/m\n";
	}
};

// generates a stepping pattern
std::vector<step_t> steps(int, bool);

// compute step times from step pattern
step_traj_t step_times(std::vector<step_t>, step_timer_t timer);

