#include "CanvasSystem.h"

#include "../Components/CanvasData.h"

#include <_deps/imgui/imgui.h>

#include <Eigen/Core>

#include <cmath>

# define M_PI 3.14159265358979323846

using namespace Ubpa;

void updateParameterization(CanvasData* data);

void updateUniformParameterization(CanvasData* data);
void updateChordalParameterization(CanvasData* data);
void updateCentripetalParameterization(CanvasData* data);
void updateFoleyParameterization(CanvasData* data);

void drawParameterization(CanvasData* data, ImDrawList* draw_list, int num_samples, const ImVec2 origin);

ImVec2 lagrangeInterpolation(float t, const Eigen::VectorXf& parameterization, const std::vector<Ubpa::pointf2>& points);

float _getAlpha(int i, const std::vector<Ubpa::pointf2>& points);

void CanvasSystem::OnUpdate(Ubpa::UECS::Schedule& schedule) {
	schedule.RegisterCommand([](Ubpa::UECS::World* w) {
		auto data = w->entityMngr.GetSingleton<CanvasData>();
		if (!data)
			return;

		if (ImGui::Begin("Canvas")) {
			ImGui::Checkbox("Enable grid", &data->opt_enable_grid);
			ImGui::Checkbox("Enable context menu", &data->opt_enable_context_menu);
			// ImGui::Text("Mouse Left: drag to add lines,\nMouse Right: drag to scroll, click for context menu.");
			ImGui::Text("Mouse Left: click to add points,\nMouse Right: drag to scroll, click for context menu.");

			// Drawing options
			ImGui::Checkbox("Uniform Parameterization",     &data->drawUniformParameterization); ImGui::SameLine();
			ImGui::Checkbox("Chordal Parameterization",     &data->drawChordalParameterization); ImGui::SameLine();
			ImGui::Checkbox("Centripetal Parameterization", &data->drawCentripetalParameterization); ImGui::SameLine();
			ImGui::Checkbox("Foley-Nielson Parameterization",		&data->drawFoleyParameterization);

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

			// Legend
			float font_size = 15.0;
			draw_list->AddText(NULL, font_size, ImVec2(canvas_p1.x - 400, canvas_p0.y + 20 - font_size / 2), IM_COL32(255, 255, 255, 255), "Uniform Parameterization");
			draw_list->AddLine(ImVec2(canvas_p1.x - 130, canvas_p0.y + 20), ImVec2(canvas_p1.x - 50, canvas_p0.y + 20), IM_COL32(255, 0, 0, 255));

			draw_list->AddText(NULL, font_size, ImVec2(canvas_p1.x - 400, canvas_p0.y + 40 - font_size / 2), IM_COL32(255, 255, 255, 255), "Chordal Parameterization");
			draw_list->AddLine(ImVec2(canvas_p1.x - 130, canvas_p0.y + 40), ImVec2(canvas_p1.x - 50, canvas_p0.y + 40), IM_COL32(0, 255, 0, 255));

			draw_list->AddText(NULL, font_size, ImVec2(canvas_p1.x - 400, canvas_p0.y + 60 - font_size / 2), IM_COL32(255, 255, 255, 255), "Centripetal Parameterization");
			draw_list->AddLine(ImVec2(canvas_p1.x - 130, canvas_p0.y + 60), ImVec2(canvas_p1.x - 50, canvas_p0.y + 60), IM_COL32(0, 255, 255, 255));

			draw_list->AddText(NULL, font_size, ImVec2(canvas_p1.x - 400, canvas_p0.y + 80 - font_size / 2), IM_COL32(255, 255, 255, 255), "Foley-Nielson Parameterization");
			draw_list->AddLine(ImVec2(canvas_p1.x - 130, canvas_p0.y + 80), ImVec2(canvas_p1.x - 50, canvas_p0.y + 80), IM_COL32(255, 0, 255, 255));

			// This will catch our interactions
			ImGui::InvisibleButton("canvas", canvas_sz, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
			const bool is_hovered = ImGui::IsItemHovered(); // Hovered
			const bool is_active = ImGui::IsItemActive();   // Held
			const ImVec2 origin(canvas_p0.x + data->scrolling[0], canvas_p0.y + data->scrolling[1]); // Lock scrolled origin
			const pointf2 mouse_pos_in_canvas(io.MousePos.x - origin.x, io.MousePos.y - origin.y);

			// Add first and second point
			//if (is_hovered && !data->adding_line && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
			//{
			//	data->points.push_back(mouse_pos_in_canvas);
			//	data->points.push_back(mouse_pos_in_canvas);
			//	data->adding_line = true;
			//}
			//if (data->adding_line)
			//{
			//	data->points.back() = mouse_pos_in_canvas;
			//	if (!ImGui::IsMouseDown(ImGuiMouseButton_Left))
			//		data->adding_line = false;
			//}

			// Add points to draw
			if (is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
			{
				data->points.push_back(mouse_pos_in_canvas);
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
			//for (int n = 0; n < data->points.size(); n += 2)
			//	draw_list->AddLine(ImVec2(origin.x + data->points[n][0], origin.y + data->points[n][1]), ImVec2(origin.x + data->points[n + 1][0], origin.y + data->points[n + 1][1]), IM_COL32(255, 255, 0, 255), 2.0f);
			for (size_t i = 0; i < data->points.size(); i++)
			{
				draw_list->AddCircleFilled(ImVec2(origin.x + data->points[i][0], origin.y + data->points[i][1]), 3.0f, IM_COL32(255, 255, 0, 255));
			}

			// Update Parameterization
			updateParameterization(data);

			// Draw Curves
			drawParameterization(data, draw_list, 1000, origin);
			
			draw_list->PopClipRect();
		}

		ImGui::End();
	});
}

void updateParameterization(CanvasData* data) {
	if (data->points.empty()) return;

	// Uniform Parameterization
	updateUniformParameterization(data);

	// Chordal Parameterization
	updateChordalParameterization(data);

	// Centripetal Parameterization
	updateCentripetalParameterization(data);

	// Foley-Nielson Parameterization
	updateFoleyParameterization(data);
}

void updateUniformParameterization(CanvasData* data) {
	int n = data->points.size();
	data->uniformParameterization = Eigen::VectorXf::LinSpaced(n, 0, 1);
}

void updateChordalParameterization(CanvasData* data) {
	int n = data->points.size();
	data->chordalParameterization = Eigen::VectorXf::Zero(n);
	if (n == 1 || n == 2)
	{
		data->chordalParameterization(n - 1) = 1;
		return;
	}

	for (size_t i = 0; i < n - 1; i++)
	{
		float dx = data->points[i + 1][0] - data->points[i][0];
		float dy = data->points[i + 1][1] - data->points[i][1];
		float chord = sqrt(dx * dx + dy * dy);
		data->chordalParameterization[i + 1] = data->chordalParameterization[i] + chord;
	}
	// data->chordalParameterization = data->chordalParameterization / data->chordalParameterization[n - 1];
}

void updateCentripetalParameterization(CanvasData* data) {
	int n = data->points.size();
	data->centripetalParameterization = Eigen::VectorXf::Zero(n);
	if (n == 1 || n == 2)
	{
		data->centripetalParameterization(n - 1) = 1;
		return;
	}

	for (size_t i = 0; i < n - 1; i++)
	{
		float dx = data->points[i + 1][0] - data->points[i][0];
		float dy = data->points[i + 1][1] - data->points[i][1];
		float chord = sqrt(dx * dx + dy * dy);
		chord = sqrt(chord);
		data->centripetalParameterization[i + 1] = data->centripetalParameterization[i] + chord;
	}
	// data->centripetalParameterization = data->centripetalParameterization / data->centripetalParameterization[n - 1];
}

void updateFoleyParameterization(CanvasData* data) {
	int n = data->points.size();
	data->foleyParameterization = Eigen::VectorXf::Zero(n);
	if (n == 1 || n == 2)
	{
		data->foleyParameterization(n - 1) = 1;
		return;
	}

	float d_prev = 0, d_next = 0;
	for (size_t i = 0; i < n - 1; i++)
	{
		float dx = data->points[i + 1][0] - data->points[i][0];
		float dy = data->points[i + 1][1] - data->points[i][1];
		float chord = sqrt(dx * dx + dy * dy);

		if (i == n - 2) d_next = 0;
		else {
			float dx = data->points[i + 2][0] - data->points[i + 1][0];
			float dy = data->points[i + 2][1] - data->points[i + 1][1];

			d_next = sqrt(dx * dx + dy * dy);
		}

		float factor = 1.0;
		if (i == 0)
		{
			float theta_next = fminf(M_PI / 2, _getAlpha(i + 1, data->points));
			factor = 1 + 1.5 * (theta_next * d_next) / (chord + d_next);
		}
		else if (i == n - 2) {
			float theta = fminf(M_PI / 2, _getAlpha(i, data->points));
			factor = 1 + 1.5 * (theta * d_prev) / (d_prev + chord);
		}
		else {
			float theta = fminf(M_PI / 2, _getAlpha(i, data->points));
			float theta_next = fminf(M_PI / 2, _getAlpha(i + 1, data->points));

			factor = 1 + 1.5 * (theta * d_prev) / (d_prev + chord) + 1.5 * (theta_next * d_next) / (chord + d_next);
		}

		data->foleyParameterization[i + 1] = data->foleyParameterization[i] + chord * factor;

		d_prev = chord;
	}

	data->foleyParameterization = data->foleyParameterization / data->foleyParameterization[n - 1];
}

float _getAlpha(int i, const std::vector<Ubpa::pointf2>& points) {
	float dx, dy;
	// d_prev
	dx = points[i][0] - points[i - 1][0];
	dy = points[i][1] - points[i - 1][1];
	float d_prev = sqrt(dx * dx + dy * dy);

	// d_next
	dx = points[i + 1][0] - points[i][0];
	dy = points[i + 1][1] - points[i][1];
	float d_next = sqrt(dx * dx + dy * dy);

	// l2
	dx = points[i + 1][0] - points[i - 1][0];
	dy = points[i + 1][1] - points[i - 1][1];
	float l2 = dx * dx + dy * dy;
	
	float alpha = M_PI - acos((d_prev*d_prev + d_next*d_next - l2/(2*d_next*d_prev)));

	return alpha;
}

void drawParameterization(CanvasData* data, ImDrawList* draw_list, int num_samples, const ImVec2 origin) {
	if (data->points.empty()) return;

	Eigen::VectorXf T = Eigen::VectorXf::LinSpaced(num_samples, 0, 1);

	// Uniform Parameterization
	if (data->drawUniformParameterization) {
		std::vector<ImVec2> uniformResult;
		for (size_t i = 0; i < num_samples; i++)
		{
			ImVec2 point = lagrangeInterpolation(T[i], data->uniformParameterization, data->points);
			uniformResult.push_back(ImVec2(origin.x + point.x, origin.y + point.y));
		}
		draw_list->AddPolyline(uniformResult.data(), uniformResult.size(), IM_COL32(255, 0, 0, 255), false, 1.0f);
	}

	// Chordal Parameterization
	if (data->drawChordalParameterization)
	{
		std::vector<ImVec2> chordalResult;
		T = Eigen::VectorXf::LinSpaced(num_samples, 0, data->chordalParameterization.tail(1)(0));
		for (size_t i = 0; i < num_samples; i++)
		{
			ImVec2 point = lagrangeInterpolation(T[i], data->chordalParameterization, data->points);
			chordalResult.push_back(ImVec2(origin.x + point.x, origin.y + point.y));
		}
		draw_list->AddPolyline(chordalResult.data(), chordalResult.size(), IM_COL32(0, 255, 0, 255), false, 1.0f);
	}

	// Centripetal Parameterization
	if (data->drawCentripetalParameterization)
	{
		std::vector<ImVec2> centripetalResult;
		T = Eigen::VectorXf::LinSpaced(num_samples, 0, data->centripetalParameterization.tail(1)(0));
		for (size_t i = 0; i < num_samples; i++)
		{
			ImVec2 point = lagrangeInterpolation(T[i], data->centripetalParameterization, data->points);
			centripetalResult.push_back(ImVec2(origin.x + point.x, origin.y + point.y));
		}
		draw_list->AddPolyline(centripetalResult.data(), centripetalResult.size(), IM_COL32(0, 255, 255, 255), false, 1.0f);
	}
	
	// Foley-Nielson Parameterization
	if (data->drawFoleyParameterization)
	{
		std::vector<ImVec2> foleyResult;
		T = Eigen::VectorXf::LinSpaced(num_samples, 0, data->foleyParameterization.tail(1)(0));
		for (size_t i = 0; i < num_samples; i++)
		{
			ImVec2 point = lagrangeInterpolation(T[i], data->foleyParameterization, data->points);
			foleyResult.push_back(ImVec2(origin.x+point.x, origin.y+point.y));
		}
		draw_list->AddPolyline(foleyResult.data(), foleyResult.size(), IM_COL32(255, 0, 255, 255), false, 1.0f);
	}
}

ImVec2 lagrangeInterpolation(float t, const Eigen::VectorXf& parameterization, const std::vector<Ubpa::pointf2>& points) {
	int n = points.size();
	for (size_t i = 0; i < n; i++)
	{
		if (t == parameterization[i]) return ImVec2(points[i][0], points[i][1]);
	}

	float x = 0, y = 0;
	for (size_t j = 0; j < n; j++)
	{
		float l = 1.0;
		for (size_t i = 0; i < n; i++) {
			if (j == i) continue;
			l = l * (t - parameterization[i]) / (parameterization[j] - parameterization[i]);
		}

		x = x + points[j][0] * l;
		y = y + points[j][1] * l;
	}

	return ImVec2(x, y);
}