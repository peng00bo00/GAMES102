#include "CanvasSystem.h"

#include "../Components/CanvasData.h"

#include <_deps/imgui/imgui.h>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>

using namespace Ubpa;

float LagrangeInterpolation(float x, const CanvasData* data);
float RBFInterpolation(float x, const CanvasData* data);
float LinearRegression(float x, const CanvasData* data);
float RidgeRegression(float x, const CanvasData* data);

Eigen::VectorXf getRBFWeights(const CanvasData* data);
Eigen::VectorXf getLRWeights(const CanvasData* data);
Eigen::VectorXf getRRWeights(const CanvasData* data);

void CanvasSystem::OnUpdate(Ubpa::UECS::Schedule& schedule) {
	schedule.RegisterCommand([](Ubpa::UECS::World* w) {
		auto data = w->entityMngr.GetSingleton<CanvasData>();
		if (!data)
			return;


		if (ImGui::Begin("Canvas")) {
			ImGui::Checkbox("Enable grid", &data->opt_enable_grid);
			ImGui::Checkbox("Enable context menu", &data->opt_enable_context_menu);
			// Drawing options
			ImGui::Checkbox("Lagrange Interpolation", &data->lagrange_interpolation); ImGui::SameLine();
			ImGui::Checkbox("RBF Interpolation", &data->rbf_interpolation); ImGui::SameLine();
			ImGui::Checkbox("Linear Regression", &data->linear_regression); ImGui::SameLine();
			ImGui::Checkbox("Ridge Regression", &data->ridge_regression);
			
			ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.25);
			ImGui::SliderFloat("Sigma", &data->sigma, 1.0f, 500.0f); ImGui::SameLine();
			ImGui::PushItemWidth(ImGui::GetWindowWidth()*0.25);
			ImGui::SliderInt("Order", &data->n_order, 0, 5); ImGui::SameLine();
			ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.25);
			ImGui::SliderFloat("Lambda", &data->lambda, 0.0f, 10.0f);
			ImGui::PopItemWidth();

			// ImGui::Text("Mouse Left: drag to add lines,\nMouse Right: drag to scroll, click for context menu.");
			ImGui::Text("Mouse Left: click to add points,\nMouse Right: drag to scroll, click for context menu.");

			// Typically you would use a BeginChild()/EndChild() pair to benefit from a clipping region + own scrolling.
			// Here we demonstrate that this can be replaced by simple offsetting + custom drawing + PushClipRect/PopClipRect() calls.
			// To use a child window instead we could use, e.g:
			//      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));      // Disable padding
			//      ImGui::PushStyleColor(ImGuiCol_ChildBg, IM_COL32(50, 50, 50, 255));  // Set a background color
			//      ImGui::BeginChild("canvas", ImVec2(0.0f, 0.0f), true, ImGuiWindowFlags_NoMove);
			//      ImGui::PopStyleColor();
			//      ImGui::PopStyleVar();
			//      [...]
			//      ImGui::EndChild();

			// Using InvisibleButton() as a convenience 1) it will advance the layout cursor and 2) allows us to use IsItemHovered()/IsItemActive()
			ImVec2 canvas_p0 = ImGui::GetCursorScreenPos();      // ImDrawList API uses screen coordinates!
			ImVec2 canvas_sz = ImGui::GetContentRegionAvail();   // Resize canvas to what's available
			if (canvas_sz.x < 50.0f) canvas_sz.x = 50.0f;
			if (canvas_sz.y < 50.0f) canvas_sz.y = 50.0f;
			ImVec2 canvas_p1 = ImVec2(canvas_p0.x + canvas_sz.x, canvas_p0.y + canvas_sz.y);

			// Draw border and background color
			ImGuiIO& io = ImGui::GetIO();
			ImDrawList* draw_list = ImGui::GetWindowDrawList();
			draw_list->AddRectFilled(canvas_p0, canvas_p1, IM_COL32(50, 50, 50, 255));
			draw_list->AddRect(canvas_p0, canvas_p1, IM_COL32(255, 255, 255, 255));

			// This will catch our interactions
			ImGui::InvisibleButton("canvas", canvas_sz, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
			const bool is_hovered = ImGui::IsItemHovered(); // Hovered
			const bool is_active = ImGui::IsItemActive();   // Held
			const ImVec2 origin(canvas_p0.x + data->scrolling[0], canvas_p0.y + data->scrolling[1]); // Lock scrolled origin
			const pointf2 mouse_pos_in_canvas(io.MousePos.x - origin.x, io.MousePos.y - origin.y);

			// Add first and second point
			//if (is_hovered && !data->adding_line && imgui::ismouseclicked(imguimousebutton_left))
			//{
			//	data->points.push_back(mouse_pos_in_canvas);
			//	data->points.push_back(mouse_pos_in_canvas);
			//	data->adding_line = true;
			//}
			//if (data->adding_line)
			//{
			//	data->points.back() = mouse_pos_in_canvas;
			//	if (!imgui::ismousedown(imguimousebutton_left))
			//		data->adding_line = false;
			//}

			// Add points by clicking
			if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
			{
				int i = 0;
				int n = data->points.size();
				if (data->points.empty()) {
					i = 0;
				}
				else if (data->points[0][0] > mouse_pos_in_canvas[0]) {
					i = 0;
				}
				else if (data->points[n-1][0] < mouse_pos_in_canvas[0]) {
					i = n;
				}
				else {
					for (i = 1; i <= n-1; i++)
					{
						if (data->points[i-1][0] < mouse_pos_in_canvas[0] && mouse_pos_in_canvas[0] < data->points[i][0]) {
							break;
						}
					}
				}
				data->points.insert(data->points.begin() + i, mouse_pos_in_canvas);
			}

			// Pan (we use a zero mouse threshold when there's no context menu)
			// You may decide to make that threshold dynamic based on whether the mouse is hovering something etc.
			const float mouse_threshold_for_pan = data->opt_enable_context_menu ? -1.0f : 0.0f;
			if (is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Right, mouse_threshold_for_pan))
			{
				data->scrolling[0] += io.MouseDelta.x;
				data->scrolling[1] += io.MouseDelta.y;
			}

			// Context menu (under default mouse threshold)
			ImVec2 drag_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
			if (data->opt_enable_context_menu && ImGui::IsMouseReleased(ImGuiMouseButton_Right) && drag_delta.x == 0.0f && drag_delta.y == 0.0f)
				ImGui::OpenPopupContextItem("context");
			if (ImGui::BeginPopup("context"))
			{
				if (data->adding_line)
					data->points.resize(data->points.size() - 2);
				data->adding_line = false;
				//if (ImGui::MenuItem("Remove one", NULL, false, data->points.size() > 0)) { data->points.resize(data->points.size() - 2); }
				if (ImGui::MenuItem("Remove one", NULL, false, data->points.size() > 0)) { data->points.resize(data->points.size() - 1); }
				if (ImGui::MenuItem("Remove all", NULL, false, data->points.size() > 0)) { data->points.clear(); }
				ImGui::EndPopup();
			}

			// Draw grid + all lines in the canvas
			draw_list->PushClipRect(canvas_p0, canvas_p1, true);
			if (data->opt_enable_grid)
			{
				const float GRID_STEP = 64.0f;
				for (float x = fmodf(data->scrolling[0], GRID_STEP); x < canvas_sz.x; x += GRID_STEP)
					draw_list->AddLine(ImVec2(canvas_p0.x + x, canvas_p0.y), ImVec2(canvas_p0.x + x, canvas_p1.y), IM_COL32(200, 200, 200, 40));
				for (float y = fmodf(data->scrolling[1], GRID_STEP); y < canvas_sz.y; y += GRID_STEP)
					draw_list->AddLine(ImVec2(canvas_p0.x, canvas_p0.y + y), ImVec2(canvas_p1.x, canvas_p0.y + y), IM_COL32(200, 200, 200, 40));
			}

			// Draw points
			for (size_t i = 0; i < data->points.size(); i++)
			{
				draw_list->AddCircleFilled(ImVec2(origin.x + data->points[i][0], origin.y + data->points[i][1]), 3.0f, IM_COL32(255, 255, 0, 255));
			}

			// Update parameters
			data->w_rbf = getRBFWeights(data);
			data->w_lr = getLRWeights(data);
			data->w_rr = getRRWeights(data);

			int n_samples = 1000;
			
			if (data->points.size() > 1) {
				float x_prev = data->points[0][0];
				float y_prev = data->points[0][1];
				float y_lag_prev = data->points[0][1];
				float y_rbf_prev = data->points[0][1];
				float y_lr_prev;
				float y_rr_prev;

				if (data->points.size() > data->n_order) {
					y_lr_prev = LinearRegression(x_prev, data);
					y_rr_prev = RidgeRegression(x_prev, data);
				}

				for (size_t n = 0; n < data->points.size()-1; n++)
				{
					float x1 = data->points[n][0];
					float x2 = data->points[n+1][0];
					float y1 = data->points[n][1];
					float y2 = data->points[n+1][1];

					float dx = (x2 - x1) / n_samples;
					float dy = (y2 - y1) / n_samples;

					// sample points on the curve
					for (size_t i = 1; i <= n_samples; i++)
					{
						float x = x_prev + dx;
						float y = y_prev + dy;

						draw_list->AddLine(ImVec2(origin.x+x_prev, origin.y+y_prev),
														ImVec2(origin.x+x, origin.y+y),
														IM_COL32(255, 255, 0, 255), 2.0f);

						// Lagrange Interpolation
						if (data->lagrange_interpolation) {
							float y_lag = LagrangeInterpolation(x, data);

							draw_list->AddLine(ImVec2(origin.x+x_prev, origin.y+y_lag_prev),
															ImVec2(origin.x+x, origin.y+y_lag),
															IM_COL32(255, 0, 0, 255), 2.0f);

							y_lag_prev = y_lag;
						}

						// RBF Interpolation
						if (data->rbf_interpolation) {
							float y_rbf = RBFInterpolation(x, data);

							draw_list->AddLine(ImVec2(origin.x+x_prev, origin.y+y_rbf_prev),
															ImVec2(origin.x+x, origin.y+y_rbf),
															IM_COL32(0, 255, 0, 255), 2.0f);

							y_rbf_prev = y_rbf;
						}

						// Linear Regression
						if (data->linear_regression && data->points.size() > data->n_order)
						{
							float y_lr = LinearRegression(x, data);

							draw_list->AddLine(ImVec2(origin.x+x_prev, origin.y+y_lr_prev),
															ImVec2(origin.x+x, origin.y+y_lr),
															IM_COL32(0, 0, 255, 255), 2.0f);

							y_lr_prev = y_lr;
						}

						// Ridge Regression
						if (data->ridge_regression && data->points.size() > data->n_order)
						{
							float y_rr = RidgeRegression(x, data);

							draw_list->AddLine(ImVec2(origin.x+x_prev, origin.y+y_rr_prev),
															ImVec2(origin.x+x, origin.y+y_rr),
															IM_COL32(0, 255, 255, 255), 2.0f);

							y_rr_prev = y_rr;
						}

						x_prev = x;
						y_prev = y;
					}

					//draw_list->AddLine(ImVec2(x1, y1), 
					//							    ImVec2(x2, y2), 
					//								IM_COL32(255, 255, 0, 255), 2.0f);
				}
			}
			
			draw_list->PopClipRect();
		}

		ImGui::End();
	});
}

float LagrangeInterpolation(float x, const CanvasData* data) {
	auto points = data->points;

	float y = 0.0;

	for (size_t j = 0; j < points.size(); j++)
	{
		float l = 1.0;
		for (size_t i = 0; i < points.size(); i++) {
			if (j == i) continue;
			l = l * (x - points[i][0]) / (points[j][0] - points[i][0]);
		}

		y = y + points[j][1] * l;
	}

	return y;
}

Eigen::VectorXf getRBFWeights(const CanvasData* data) {
	auto points = data->points;

	Eigen::VectorXf w_rbf;
	if (points.empty()) return w_rbf;

	int n = points.size();
	Eigen::VectorXf b = Eigen::VectorXf::Zero(n+1);
	Eigen::MatrixXf A = Eigen::MatrixXf::Zero(n+1, n+1);

	for (size_t i = 0; i < n; i++)
	{
		float xi = points[i][0];
		float yi = points[i][1];

		for (size_t j = 0; j < n; j++)
		{
			float xj = points[j][0];

			//A(i, j) = exp(-pow(xi - xj, 2) / (2*pow(data->sigma, 2)));
			A(i, j) = exp(-pow((xi - xj) / data->sigma, 2) / 2);
		}

		A(i, n) = 1;

		b(i) = yi;
	}
	
	for (size_t i = 0; i < n; i++)
	{
		A(n, i) = 1;
	}

	// solve normal equation
	w_rbf = A.colPivHouseholderQr().solve(b);
	std::cout << w_rbf << std::endl;

	return w_rbf;
}

float RBFInterpolation(float x, const CanvasData* data) {
	float y = 0;
	for (size_t i = 0; i < data->points.size(); i++)
	{
		float g = exp(-pow((x - data->points[i][0]) / data->sigma, 2) / 2);
		y = y + data->w_rbf(i) * g;
	}

	y = y + data->w_rbf(data->points.size());

	return y;
}

Eigen::VectorXf getLRWeights(const CanvasData* data) {
	auto points = data->points;

	Eigen::VectorXf w_lr;
	if (points.size() < data->n_order+1) return w_lr;

	int n = points.size();
	int n_order = data->n_order;
	Eigen::VectorXf b(n);
	Eigen::MatrixXf A(n, n_order+1);

	for (size_t i = 0; i < n; i++)
	{
		float xi = points[i][0];
		float yi = points[i][1];

		for (size_t j = 0; j < n_order+1; j++)
		{
			A(i, j) = pow(xi, j);
		}

		b(i) = yi;
	}

	// solve normal equation
	w_lr = (A.transpose() * A).inverse() * A.transpose() * b;

	return w_lr;
}

float LinearRegression(float x, const CanvasData* data) {
	float y = 0;
	for (size_t i = 0; i < data->n_order+1; i++)
	{
		float g = pow(x, i);
		y = y + data->w_lr(i) * g;
	}

	return y;
}

Eigen::VectorXf getRRWeights(const CanvasData* data) {
	auto points = data->points;

	Eigen::VectorXf w_rr;
	if (points.size() < data->n_order + 1) return w_rr;

	int n = points.size();
	int n_order = data->n_order;
	Eigen::VectorXf b(n);
	Eigen::MatrixXf A(n, n_order + 1);

	for (size_t i = 0; i < n; i++)
	{
		float xi = points[i][0];
		float yi = points[i][1];

		for (size_t j = 0; j < n_order + 1; j++)
		{
			A(i, j) = pow(xi, j);
		}

		b(i) = yi;
	}

	// solve normal equation
	w_rr = (A.transpose() * A + data->lambda * Eigen::MatrixXf::Identity(n_order+1, n_order+1)).inverse() * A.transpose() * b;

	return w_rr;
}

float RidgeRegression(float x, const CanvasData* data) {
	float y = 0;
	for (size_t i = 0; i < data->n_order + 1; i++)
	{
		float g = pow(x, i);
		y = y + data->w_rr(i) * g;
	}

	return y;
}