#pragma once

#include <UGM/UGM.h>
#include <Eigen/Core>

struct CanvasData {
	std::vector<Ubpa::pointf2> points;
	Ubpa::valf2 scrolling{ 0.f,0.f };
	bool opt_enable_grid{ true };
	bool opt_enable_context_menu{ true };
	bool adding_line{ false };

	bool lagrange_interpolation{ false };
	bool rbf_interpolation{ false };
	bool linear_regression{ false };
	bool ridge_regression{ false };

	Eigen::VectorXf w_rbf;
	Eigen::VectorXf w_lr;
	Eigen::VectorXf w_rr;

	int n_order = 3;
	float sigma = 100;
	float lambda = 0;
};

#include "details/CanvasData_AutoRefl.inl"
