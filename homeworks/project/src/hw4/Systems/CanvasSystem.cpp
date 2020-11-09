#include "CanvasSystem.h"

#include "../Components/CanvasData.h"

#include <_deps/imgui/imgui.h>

#include <Eigen/Core>
#include <Eigen/Sparse>

using namespace Ubpa;

void updateParameterization(CanvasData* data);
void updateCoefficients(CanvasData* data);
void drawCurve(CanvasData* data, ImDrawList* draw_list, const ImVec2 origin);
Eigen::MatrixXf naturalCubicSpline(Eigen::VectorXf x, Eigen::VectorXf y);

// float e = 2.71828182846;
int numSamples = 1000;

void CanvasSystem::OnUpdate(Ubpa::UECS::Schedule& schedule) {
	schedule.RegisterCommand([](Ubpa::UECS::World* w) {
		auto data = w->entityMngr.GetSingleton<CanvasData>();
		if (!data)
			return;

		if (ImGui::Begin("Canvas")) {
			ImGui::Checkbox("Enable grid", &data->opt_enable_grid);
			ImGui::Checkbox("Enable context menu", &data->opt_enable_context_menu);
			// ImGui::Text("Mouse Left: drag to add lines,\nMouse Right: drag to scroll, click for context menu.");
			if (data->mode == insert) ImGui::Text("Mouse Left: click to add points,\nMouse Right: drag to scroll, click for context menu.");
			else ImGui::Text("  \n  ");

			// Drawing options
			ImGui::RadioButton("Insert Mode", &data->mode, insert); ImGui::SameLine();
			ImGui::RadioButton("Edit Mode", &data->mode, edit);

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
			if (data->mode == insert && is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
			{
				data->points.push_back(mouse_pos_in_canvas);
				data->update = true;
			}

			// Pan (we use a zero mouse threshold when there's no context menu)
			// You may decide to make that threshold dynamic based on whether the mouse is hovering something etc.
			const float mouse_threshold_for_pan = data->opt_enable_context_menu ? -1.0f : 0.0f;
			if (is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Right, mouse_threshold_for_pan))
			{
				data->scrolling[0] += io.MouseDelta.x;
				data->scrolling[1] += io.MouseDelta.y;
			}

			// Edit point location
			if (data->mode == edit) {
				float dmin = INFINITY;

				// select the nearest point
				for (size_t i = 0; i < data->points.size(); i++)
				{
					pointf2 point = data->points[i];
					float dx = point[0] - mouse_pos_in_canvas[0];
					float dy = point[1] - mouse_pos_in_canvas[1];
					float d = dx * dx + dy * dy;

					if (d < dmin && d <= 400)
					{
						data->selected = i;
						dmin = d;
					}
				}

				// update the location
				if (data->selected != unselected && is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
				{
					data->points[data->selected] = mouse_pos_in_canvas;
					data->update = true;
				}
				
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
				if (ImGui::MenuItem("Remove one", NULL, false, data->points.size() > 0)) { 
					if (data->selected == data->points.size() - 1) { data->selected = unselected; }
						data->points.resize(data->points.size() - 1);
					}
				if (ImGui::MenuItem("Remove all", NULL, false, data->points.size() > 0)) { 
						data->points.clear();
						data->selected = unselected;
					}
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

			// Draw points
			for (size_t i = 0; i < data->points.size(); i++)
			{
				draw_list->AddCircleFilled(ImVec2(origin.x + data->points[i][0], origin.y + data->points[i][1]), 3.0f, IM_COL32(255, 255, 0, 255));
			}
			// draw the selected point
			if (data->mode == edit && data->selected != unselected)
			{
				draw_list->AddCircleFilled(ImVec2(origin.x + data->points[data->selected][0], origin.y + data->points[data->selected][1]), 3.0f, IM_COL32(255, 0, 0, 255));
				// data->selected = unselected;
			}

			// Update Parameterization
			updateParameterization(data);

			// Update Coefficients
			updateCoefficients(data);

			// Draw Curve
			drawCurve(data, draw_list, origin);

			draw_list->PopClipRect();
		}

		ImGui::End();
	});
}


void updateParameterization(CanvasData* data) {
	if (data->update && !data->points.empty()) {
		// uniform parameterization
		//int n = data->points.size();
		//data->parameterization = Eigen::VectorXf::LinSpaced(n, 0, 1);

		// chordal parameterization
		int n = data->points.size();
		data->parameterization = Eigen::VectorXf::Zero(n);
		if (n == 1 || n == 2)
		{
			data->parameterization(n - 1) = 1;
			return;
		}

		for (size_t i = 0; i < n - 1; i++)
		{
			float dx = data->points[i + 1][0] - data->points[i][0];
			float dy = data->points[i + 1][1] - data->points[i][1];
			float chord = sqrt(dx * dx + dy * dy);
			data->parameterization[i + 1] = data->parameterization[i] + chord;
		}
		// data->parameterization = data->parameterization / data->parameterization[n - 1];
	}
}

void updateCoefficients(CanvasData* data) {
	
	if (data->update) {
		if (data->points.size() > 2) {
			int n = data->points.size();

			Eigen::VectorXf t = Eigen::VectorXf(n);
			Eigen::VectorXf x = Eigen::VectorXf(n);
			Eigen::VectorXf y = Eigen::VectorXf(n);

			Eigen::MatrixXf coeff = Eigen::MatrixXf::Zero(n - 1, 8);

			for (size_t i = 0; i < n; i++)
			{
				t(i) = data->parameterization(i);
				x(i) = data->points[i][0];
				y(i) = data->points[i][1];
			}

			coeff.block(0, 0, n - 1, 4) = naturalCubicSpline(t, x);
			coeff.block(0, 4, n - 1, 4) = naturalCubicSpline(t, y);
			data->coeff = coeff;
		}
	}
}

Eigen::MatrixXf naturalCubicSpline(Eigen::VectorXf x, Eigen::VectorXf y) {
	int n = int(x.size()) - 1;

	// step1
	Eigen::VectorXf h = Eigen::VectorXf::Zero(n);
	for (size_t i = 0; i < n; i++) h(i) = x(i + 1) - x(i);

	// step2
	Eigen::VectorXf alpha = Eigen::VectorXf::Zero(n);
	for (size_t i = 1; i < n; i++)
	{
		alpha(i) = 3 / h(i) * (y(i + 1) - y(i)) - 3 / h(i - 1) * (y(i) - y(i - 1));
	}

	// step3
	Eigen::VectorXf l  = Eigen::VectorXf::Zero(n+1);
	Eigen::VectorXf mu = Eigen::VectorXf::Zero(n);
	Eigen::VectorXf z  = Eigen::VectorXf::Zero(n+1);

	l(0) = 1; mu(0) = 0; z(0) = 0;

	// step4
	for (size_t i = 1; i < n; i++)
	{
		l(i)  = 2 * (x(i + 1) - x(i - 1)) - h(i - 1) * mu(i - 1);
		mu(i) = h(i) / l(i);
		z(i)  = (alpha(i) - h(i - 1)*z(i - 1)) / l(i);
	}

	// step5
	Eigen::VectorXf c = Eigen::VectorXf::Zero(n + 1);
	Eigen::VectorXf b = Eigen::VectorXf::Zero(n + 1);
	Eigen::VectorXf d = Eigen::VectorXf::Zero(n + 1);
	l(n) = 1; z(n) = 0; c(n) = 0;

	// step6
	for (int j = n-1; j >= 0; j--)
	{
		c(j) = z(j) - mu(j) * c(j + 1);
		b(j) = (y(j + 1) - y(j)) / h(j) - h(j) * (c(j + 1) + 2 * c(j)) / 3;
		d(j) = (c(j + 1) - c(j)) / (3 * h(j));
	}

	// step7
	Eigen::MatrixXf coeff = Eigen::MatrixXf::Zero(n, 4);
	for (size_t j = 0; j < n; j++)
	{
		coeff(j, 0) = y(j);
		coeff(j, 1) = b(j);
		coeff(j, 2) = c(j);
		coeff(j, 3) = d(j);
	}

	return coeff;
}

void drawCurve(CanvasData* data, ImDrawList* draw_list, const ImVec2 origin) {
	if (data->points.size() < 3) return;

	int n = data->points.size();
	std::vector<ImVec2> curve;

	int j = 0;
	float a, b, c, d, t;
	float x, y;
	Eigen::VectorXf T = Eigen::VectorXf::LinSpaced(numSamples, 0, data->parameterization(n-1));
	for (size_t i = 0; i < numSamples; i++)
	{
		if (T(i) > data->parameterization(j+1)) j++;

		t = T(i) - data->parameterization(j);

		// x-coordinate
		a = data->coeff(j, 0);
		b = data->coeff(j, 1);
		c = data->coeff(j, 2);
		d = data->coeff(j, 3);

		x = a + b * t + c * t * t + d * t * t * t;

		// y-coordinate
		a = data->coeff(j, 4);
		b = data->coeff(j, 5);
		c = data->coeff(j, 6);
		d = data->coeff(j, 7);

		y = a + b * t + c * t * t + d * t * t * t;

		curve.push_back(ImVec2(x+origin.x, y+origin.y));
	}

	draw_list->AddPolyline(curve.data(), curve.size(), IM_COL32(0, 255, 0, 255), false, 1.0f);

	data->update = false;
}