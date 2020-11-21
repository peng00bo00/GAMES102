#pragma once

#include <UGM/UGM.h>

constexpr auto insert = 0;
constexpr auto display= 1;

struct CanvasData {
	std::vector<Ubpa::pointf2> points;
	Ubpa::valf2 scrolling{ 0.f,0.f };
	bool opt_enable_grid{ true };
	bool opt_enable_context_menu{ true };
	// bool adding_line{ false };

	int mode{insert};
	float alpha{0.1};
	int order{4};

	bool draw_interpolate{ false };
	bool draw_chaikin{ false };
	bool draw_bspline{ false };

	std::vector<Ubpa::pointf2> interpolate_points;
	std::vector<Ubpa::pointf2> chaikin_points;
	std::vector<Ubpa::pointf2> bspline_points;
};

#include "details/CanvasData_AutoRefl.inl"
