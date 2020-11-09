#pragma once

#include <UGM/UGM.h>
#include <Eigen/Core>

constexpr auto insert = 0;
constexpr auto edit   = 1;

constexpr auto unselected = -1;

struct CanvasData {
	std::vector<Ubpa::pointf2> points;
	Ubpa::valf2 scrolling{ 0.f,0.f };
	bool opt_enable_grid{ true };
	bool opt_enable_context_menu{ true };
	bool adding_line{ false };

	int mode = insert;
	bool update{ false };
	int selected = unselected;
	
	Eigen::VectorXf parameterization;
	Eigen::MatrixXf coeff;

};

#include "details/CanvasData_AutoRefl.inl"
