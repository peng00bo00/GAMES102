#pragma once

#include <UGM/UGM.h>
#include <Eigen/Core>


struct CanvasData {
	std::vector<Ubpa::pointf2> points;
	Ubpa::valf2 scrolling{ 0.f,0.f };
	bool opt_enable_grid{ true };
	bool opt_enable_context_menu{ true };
	bool adding_line{ false };

	bool drawUniformParameterization = false;
	bool drawChordalParameterization = false;
	bool drawCentripetalParameterization = false;
	bool drawFoleyParameterization = false;

	Eigen::VectorXf uniformParameterization;
	Eigen::VectorXf chordalParameterization;
	Eigen::VectorXf centripetalParameterization;
	Eigen::VectorXf foleyParameterization;
};

#include "details/CanvasData_AutoRefl.inl"
