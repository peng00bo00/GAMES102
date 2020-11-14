#pragma once

#include <UGM/UGM.h>
#include <Eigen/Core>

constexpr auto insert = 0;
constexpr auto edit   = 1;

constexpr auto unselected = -1;

constexpr auto C2 = 2;
constexpr auto G0 = 0;
constexpr auto G1 = 1;

constexpr auto _point = 0;
constexpr auto _Lcp = 1;
constexpr auto _Rcp = 2;


struct CanvasData {
	std::vector<Ubpa::pointf2> points;
	int selected = unselected;
	int selected_type = _point;
	std::vector<int> points_type;

	std::vector<Ubpa::pointf2> cpL;
	std::vector<Ubpa::pointf2> cpR;

	Ubpa::valf2 scrolling{ 0.f,0.f };
	bool opt_enable_grid{ true };
	bool opt_enable_context_menu{ true };
	bool adding_line{ false };

	int mode = insert;
	bool update{ false };
	
	Eigen::VectorXf parameterization;
	Eigen::MatrixXf coeff;

};

#include "details/CanvasData_AutoRefl.inl"
