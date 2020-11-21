#include "CanvasSystem.h"

#include "../Components/CanvasData.h"

#include <_deps/imgui/imgui.h>


using namespace Ubpa;


std::vector<pointf2> interpolatesubdiv(std::vector<pointf2> points, int order, float alpha);
std::vector<pointf2> chaikin(std::vector<pointf2> points, int order);
std::vector<pointf2> bsplinesubdiv(std::vector<pointf2> points, int order);

void CanvasSystem::OnUpdate(Ubpa::UECS::Schedule& schedule) {
	schedule.RegisterCommand([](Ubpa::UECS::World* w) {
		auto data = w->entityMngr.GetSingleton<CanvasData>();
		if (!data)
			return;

		if (ImGui::Begin("Canvas")) {
			ImGui::Checkbox("Enable grid", &data->opt_enable_grid);
			ImGui::Checkbox("Enable context menu", &data->opt_enable_context_menu);
			ImGui::Text("Mouse Left: drag to add lines,\nMouse Right: drag to scroll, click for context menu.");

			// Drawing options
			ImGui::RadioButton("Insert Mode", &data->mode, insert); ImGui::SameLine();
			ImGui::RadioButton("Display Mode", &data->mode, display);

			ImGui::Checkbox("Chaikin Subdivision", &data->draw_chaikin); ImGui::SameLine();
			ImGui::Checkbox("B-Spline Subdivision", &data->draw_bspline); ImGui::SameLine();
			ImGui::Checkbox("Interpolating Subdivision", &data->draw_interpolate);

			ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.25);
			ImGui::SliderInt("Order", &data->order, 0, 10); ImGui::SameLine();
			ImGui::PushItemWidth(ImGui::GetWindowWidth() * 0.25);
			ImGui::SliderFloat("Alpha", &data->alpha, 0.0f, 0.125f);
			ImGui::PopItemWidth();

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
			draw_list->AddText(NULL, font_size, ImVec2(canvas_p1.x - 400, canvas_p0.y + 20 - font_size / 2), IM_COL32(255, 255, 255, 255), "Chaikin Subdivision");
			draw_list->AddLine(ImVec2(canvas_p1.x - 130, canvas_p0.y + 20), ImVec2(canvas_p1.x - 50, canvas_p0.y + 20), IM_COL32(0, 255, 255, 255));

			draw_list->AddText(NULL, font_size, ImVec2(canvas_p1.x - 400, canvas_p0.y + 40 - font_size / 2), IM_COL32(255, 255, 255, 255), "B-Spline Subdivision");
			draw_list->AddLine(ImVec2(canvas_p1.x - 130, canvas_p0.y + 40), ImVec2(canvas_p1.x - 50, canvas_p0.y + 40), IM_COL32(255, 0, 255, 255));

			draw_list->AddText(NULL, font_size, ImVec2(canvas_p1.x - 400, canvas_p0.y + 60 - font_size / 2), IM_COL32(255, 255, 255, 255), "Interpolating Subdivision");
			draw_list->AddLine(ImVec2(canvas_p1.x - 130, canvas_p0.y + 60), ImVec2(canvas_p1.x - 50, canvas_p0.y + 60), IM_COL32(0, 255, 0, 255));

			// This will catch our interactions
			ImGui::InvisibleButton("canvas", canvas_sz, ImGuiButtonFlags_MouseButtonLeft | ImGuiButtonFlags_MouseButtonRight);
			const bool is_hovered = ImGui::IsItemHovered(); // Hovered
			const bool is_active = ImGui::IsItemActive();   // Held
			const ImVec2 origin(canvas_p0.x + data->scrolling[0], canvas_p0.y + data->scrolling[1]); // Lock scrolled origin
			const pointf2 mouse_pos_in_canvas(io.MousePos.x - origin.x, io.MousePos.y - origin.y);

			//// Add first and second point
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

			if (data->mode == insert && is_hovered && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
			{
				data->points.push_back(mouse_pos_in_canvas);
			}
			if (data->mode == display) {
				data->interpolate_points = interpolatesubdiv(data->points, data->order, data->alpha);
				data->chaikin_points = chaikin(data->points, data->order);
				data->bspline_points = bsplinesubdiv(data->points, data->order);
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
				//if (data->adding_line)
				//	data->points.resize(data->points.size() - 2);
				//data->adding_line = false;
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
			// Draw points
			for (size_t i = 0; i < data->points.size(); i++)
			{
				draw_list->AddCircleFilled(ImVec2(origin.x + data->points[i][0], origin.y + data->points[i][1]), 3.0f, IM_COL32(255, 255, 0, 255));
			}

			if (data->mode == display) {

				if (data -> draw_interpolate) {
					for (size_t i = 0; i < data->interpolate_points.size(); i++)
					{
						draw_list->AddCircleFilled(ImVec2(origin.x + data->interpolate_points[i][0], origin.y + data->interpolate_points[i][1]), 1.0f, IM_COL32(0, 255, 0, 255));
					}

				}

				if (data->draw_chaikin)
				{
					for (size_t i = 0; i < data->chaikin_points.size(); i++)
					{
						draw_list->AddCircleFilled(ImVec2(origin.x + data->chaikin_points[i][0], origin.y + data->chaikin_points[i][1]), 1.0f, IM_COL32(0, 255, 255, 255));
					}

				}

				if (data->draw_bspline) {
					for (size_t i = 0; i < data->bspline_points.size(); i++)
					{
						draw_list->AddCircleFilled(ImVec2(origin.x + data->bspline_points[i][0], origin.y + data->bspline_points[i][1]), 1.0f, IM_COL32(255, 0, 255, 255));
					}

				}

			}
			

			draw_list->PopClipRect();
		}

		ImGui::End();
	});
}

std::vector<pointf2> interpolatesubdiv(std::vector<pointf2> points, int order, float alpha) {
	if (points.size() < 4 || order == 0) return points;

	std::vector<pointf2> subdiv, _points;
	_points = points;
	for (size_t i = 1; i+2 < points.size(); i++)
	{
		float px = (points[i][0] + points[i + 1][0]) / 2 + alpha * ((points[i][0] + points[i + 1][0]) / 2 - (points[i - 1][0] + points[i + 2][0]) / 2);
		float py = (points[i][1] + points[i + 1][1]) / 2 + alpha * ((points[i][1] + points[i + 1][1]) / 2 - (points[i - 1][1] + points[i + 2][1]) / 2);

		subdiv.push_back(pointf2(px, py));
	}

	for (size_t i = 0; i < subdiv.size(); i++)
	{
		_points.insert(_points.begin()+(2*i+2), subdiv[i]);
	}

	return interpolatesubdiv(_points, order - 1, alpha);
}

std::vector<pointf2> chaikin(std::vector<pointf2> points, int order) {
	if (points.size() < 2 || order == 0) return points;
	std::vector<pointf2> subdiv, _points;

	// split
	for (size_t i = 0; i+1 < points.size(); i++)
	{
		float px = (points[i][0] + points[i + 1][0]) / 2;
		float py = (points[i][1] + points[i + 1][1]) / 2;

		subdiv.push_back(pointf2(px, py));
	}

	for (size_t i = 0; i < subdiv.size(); i++)
	{
		points.insert(points.begin() + (2 * i + 1), subdiv[i]);
	}

	// average
	for (size_t i = 0; i+1 < points.size(); i++)
	{
		float px = (points[i][0] + points[i+1][0]) / 2;
		float py = (points[i][1] + points[i+1][1]) / 2;

		_points.push_back(pointf2(px, py));
	}

	return chaikin(_points, order - 1);
}

std::vector<pointf2> bsplinesubdiv(std::vector<pointf2> points, int order) {
	if (points.size() < 3 || order == 0) return points;
	std::vector<pointf2> subdiv, _points;

	// split
	for (size_t i = 1; i + 1 < points.size(); i++)
	{
		float px = (points[i - 1][0] + 6 * points[i][0] + points[i + 1][0]) / 8;
		float py = (points[i - 1][1] + 6 * points[i][1] + points[i + 1][1]) / 8;

		subdiv.push_back(pointf2(px, py));
	}

	// average
	for (size_t i = 0; i + 1 < points.size(); i++)
	{
		float px = (points[i][0] + points[i + 1][0]) / 2;
		float py = (points[i][1] + points[i + 1][1]) / 2;

		_points.push_back(pointf2(px, py));
	}

	for (size_t i = 0; i < subdiv.size(); i++)
	{
		_points.insert(_points.begin() + (2 * i + 1), subdiv[i]);
	}

	return bsplinesubdiv(_points, order - 1);
}