#include "CanvasSystem.h"

#include "../Components/CanvasData.h"

#include <_deps/imgui/imgui.h>

#include <Eigen/Core>
#include <Eigen/Dense>

using namespace Ubpa;

void updateParameterization(CanvasData* data);
void updateCoefficients(CanvasData* data);
void updateControlPoints(CanvasData* data);
void drawCurve(CanvasData* data, ImDrawList* draw_list, const ImVec2 origin);
void drawEdit(CanvasData* data, ImDrawList* draw_list, const ImVec2 origin);
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

			// Drawing options
			ImGui::RadioButton("Insert Mode", &data->mode, insert); ImGui::SameLine();
			ImGui::RadioButton("Edit Mode", &data->mode, edit);

			// ImGui::Text("Mouse Left: drag to add lines,\nMouse Right: drag to scroll, click for context menu.");
			if (data->mode == insert) ImGui::Text("Mouse Left: click to add points,\nMouse Right: drag to scroll, click for context menu.");
			else {
				if (data->selected != unselected) {
					int idx = data->selected;
					if (data->selected_type == _Lcp) idx = data->selected + 1;
					ImGui::RadioButton("C2", &data->points_type[idx], C2); ImGui::SameLine();
					ImGui::RadioButton("G0", &data->points_type[idx], G0); ImGui::SameLine();
					ImGui::RadioButton("G1", &data->points_type[idx], G1);
				}
			};

			

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
				data->points_type.push_back(C2);
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
			if (data->mode == edit && !data->points.empty()) {
				// Update Control Points
				updateControlPoints(data);

				float dmin = INFINITY;
				int n = data->points.size();

				// select the nearest point
				for (size_t i = 0; i < n; i++)
				{
					pointf2 point = data->points[i];
					float dx = point[0] - mouse_pos_in_canvas[0];
					float dy = point[1] - mouse_pos_in_canvas[1];
					float d = dx * dx + dy * dy;

					if (d < dmin && d <= 400 && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
					{
						data->selected = i;
						data->selected_type = _point;
						dmin = d;
					}
				}

				// select the nearest control point
				for (size_t i = 0; i < n - 1; i++)
				{
					int idx = i;
					pointf2 cp = data->cpR[i];
					cp[0] = 50 * cp[0] + data->points[idx][0];
					cp[1] = 50 * cp[1] + data->points[idx][1];

					float dx = cp[0] - mouse_pos_in_canvas[0];
					float dy = cp[1] - mouse_pos_in_canvas[1];
					float d = dx * dx + dy * dy;

					if (d < dmin && d <= 400 && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
					{
						data->selected = i;
						data->selected_type = _Rcp;
						dmin = d;
					}
				}

				for (size_t i = 0; i < n - 1; i++)
				{
					int idx = i+1;
					pointf2 cp = data->cpL[i];
					cp[0] = -50 * cp[0] + data->points[idx][0];
					cp[1] = -50 * cp[1] + data->points[idx][1];

					float dx = cp[0] - mouse_pos_in_canvas[0];
					float dy = cp[1] - mouse_pos_in_canvas[1];
					float d = dx * dx + dy * dy;

					if (d < dmin && d <= 400 && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
					{
						data->selected = i;
						data->selected_type = _Lcp;
						dmin = d;
					}
				}

				// update the (control) point location
				if (data->selected != unselected && is_active && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
				{
					// edit point
					if (data->selected_type == _point && data->points_type[data->selected] == C2) {
						data->points[data->selected] = mouse_pos_in_canvas;
						data->update = true;
						//int idx = data->selected;
						//if (data->selected < n-1) {
						//	int seg_idx = idx;

						//	// update coefficients
						//	float h = data->parameterization[idx + 1] - data->parameterization[idx];
						//	float h2 = h * h;
						//	float h3 = h2 * h;

						//	float ax = data->coeff(seg_idx, 0);
						//	//float bx = data->coeff(seg_idx, 1);
						//	float cx = data->coeff(seg_idx, 2);
						//	float dx = data->coeff(seg_idx, 3);

						//	Eigen::MatrixXf A = Eigen::MatrixXf::Zero(2, 2);
						//	A(0, 0) = 1 / h;
						//	A(0, 1) = 1;
						//	A(1, 0) = 2 / h;
						//	A(1, 1) = 3;

						//	Eigen::MatrixXf b = Eigen::Vector2f::Zero(2, 1);
						//	b(0) = (ax - mouse_pos_in_canvas[0])/h3+cx/h+dx;
						//	b(1) = 2 * cx / h + 3 * dx ;

						//	Eigen::VectorXf x = A.colPivHouseholderQr().solve(b);

						//	ax = mouse_pos_in_canvas[0];
						//	//bx = fx;
						//	cx = x(0);
						//	dx = x(1);

						//	data->coeff(seg_idx, 0) = ax;
						//	//data->coeff(idx, 1) = bx;
						//	data->coeff(seg_idx, 2) = cx;
						//	data->coeff(seg_idx, 3) = dx;

						//	float ay = data->coeff(seg_idx, 4);
						//	//float by = data->coeff(seg_idx, 5);
						//	float cy = data->coeff(seg_idx, 6);
						//	float dy = data->coeff(seg_idx, 7);

						//	A(0, 0) = 1 / h;
						//	A(0, 1) = 1;
						//	A(1, 0) = 2 / h;
						//	A(1, 1) = 3;

						//	b(0) = (ay - mouse_pos_in_canvas[1]) / h3 + cy / h + dy;
						//	b(1) = 2 * cy / h + 3 * dy;

						//	x = A.colPivHouseholderQr().solve(b);

						//	ay = mouse_pos_in_canvas[1];
						//	//bx = fx;
						//	cy = x(0);
						//	dy = x(1);

						//	data->coeff(seg_idx, 4) = ay;
						//	//data->coeff(idx, 1) = bx;
						//	data->coeff(seg_idx, 6) = cy;
						//	data->coeff(seg_idx, 7) = dy;

						//	if (data->points_type[idx + 1] == C2) data->points_type[idx + 1] = G1;
						//}
						//if (data->selected > 0) {
						//	int seg_idx = idx - 1;

						//	// update coefficients
						//	float h = data->parameterization[idx] - data->parameterization[idx-1];
						//	float h2 = h * h;
						//	float h3 = h2 * h;

						//	float ax = data->coeff(seg_idx, 0);
						//	float bx = data->coeff(seg_idx, 1);
						//	float cx = data->coeff(seg_idx, 2);
						//	float dx = data->coeff(seg_idx, 3);

						//	Eigen::MatrixXf A = Eigen::MatrixXf::Zero(2, 2);
						//	A(0, 0) = 1 / h;
						//	A(0, 1) = 1;
						//	A(1, 0) = 2 / h;
						//	A(1, 1) = 3;

						//	Eigen::MatrixXf b = Eigen::Vector2f::Zero(2, 1);
						//	b(0) = (mouse_pos_in_canvas[0] - ax) / h3 - bx / h2;
						//	b(1) = 2 * cx / h + 3 * dx;

						//	Eigen::VectorXf x = A.colPivHouseholderQr().solve(b);
						//	cx = x(0);
						//	dx = x(1);

						//	data->coeff(seg_idx, 2) = cx;
						//	data->coeff(seg_idx, 3) = dx;

						//	float ay = data->coeff(seg_idx, 4);
						//	float by = data->coeff(seg_idx, 5);
						//	float cy = data->coeff(seg_idx, 6);
						//	float dy = data->coeff(seg_idx, 7);

						//	A = Eigen::MatrixXf::Zero(2, 2);
						//	A(0, 0) = 1 / h;
						//	A(0, 1) = 1;
						//	A(1, 0) = 2 / h;
						//	A(1, 1) = 3;

						//	b = Eigen::Vector2f::Zero(2, 1);
						//	b(0) = (mouse_pos_in_canvas[1] - ay) / h3 - by / h2;
						//	b(1) = 2 * cy / h + 3 * dy;

						//	x = A.colPivHouseholderQr().solve(b);
						//	cy = x(0);
						//	dy = x(1);

						//	data->coeff(seg_idx, 6) = cy;
						//	data->coeff(seg_idx, 7) = dy;

						//	if (data->points_type[idx - 1] == C2) data->points_type[idx - 1] = G1;
						//}

						//if (data->points_type[idx] == C2) data->points_type[idx] = G1;
					}

					// edit right control point
					if (data->selected_type == _Rcp && data->points_type[data->selected] != C2) {
						int idx = data->selected;
						int seg_idx = idx;
						pointf2 cp = mouse_pos_in_canvas;

						float fx = (cp[0] - data->points[idx][0]) / 50;
						float fy = (cp[1] - data->points[idx][1]) / 50;

						data->cpR[data->selected] = pointf2(fx, fy);
						// update coefficients
						float h = data->parameterization[idx + 1] - data->parameterization[idx];
						float h2 = h * h;

						//float ax = data->coeff(seg_idx, 0);
						float bx = data->coeff(seg_idx, 1);
						float cx = data->coeff(seg_idx, 2);
						float dx = data->coeff(seg_idx, 3);

						Eigen::MatrixXf A = Eigen::MatrixXf::Zero(2, 2);
						A(0, 0) = 1/h;
						A(0, 1) = 1;
						A(1, 0) = 2/h;
						A(1, 1) = 3;

						Eigen::MatrixXf b = Eigen::Vector2f::Zero(2, 1);
						b(0) = bx/h2 + cx/h + dx - fx/h2;
						b(1) = bx/h2 + 2 * cx / h + 3 * dx - fx/h2;

						Eigen::VectorXf x = A.colPivHouseholderQr().solve(b);
						bx = fx;
						cx = x(0);
						dx = x(1);

						data->coeff(seg_idx, 1) = bx;
						data->coeff(seg_idx, 2) = cx;
						data->coeff(seg_idx, 3) = dx;

						//float ay = data->coeff(seg_idx, 4);
						float by = data->coeff(seg_idx, 5);
						float cy = data->coeff(seg_idx, 6);
						float dy = data->coeff(seg_idx, 7);

						A = Eigen::MatrixXf::Zero(2, 2);
						A(0, 0) = 1/h;
						A(0, 1) = 1;
						A(1, 0) = 2/h;
						A(1, 1) = 3;

						b = Eigen::Vector2f::Zero(2, 1);
						b(0) = by/h2 + cy/h+ dy - fy/h2;
						b(1) = by/h2 + 2 * cy /h + 3 * dy - fy/h2;

						x = A.colPivHouseholderQr().solve(b);
						by = fy;
						cy = x(0);
						dy = x(1);

						data->coeff(seg_idx, 5) = by;
						data->coeff(seg_idx, 6) = cy;
						data->coeff(seg_idx, 7) = dy;

						if (data->points_type[idx + 1] == C2) data->points_type[idx + 1] = G1;

						// G1
						if (data->points_type[data->selected] == G1 && idx > 0) {
							int seg_idx = idx - 1;
							float fx = data->cpL[idx-1][0];
							float fy = data->cpL[idx-1][1];

							float C = fx * fx + fy * fy;

							float ffx, ffy;

							if (bx != 0) {
								float k = by / bx;

								ffx = sqrt(C / (1 + k * k));
								if (bx < 0) ffx = -ffx;
								ffy = k * ffx;
							}

							else {
								ffx = 0;
								ffy = sqrt(C);
								if (by < 0) ffy = -ffy;
							}

							data->cpL[idx-1] = pointf2(ffx, ffy);

							// update coefficients
							float h = data->parameterization[idx] - data->parameterization[idx - 1];
							float h2 = h * h;

							//float ax = data->coeff(seg_idx, 0);
							float bx = data->coeff(seg_idx, 1);
							float cx = data->coeff(seg_idx, 2);
							float dx = data->coeff(seg_idx, 3);

							Eigen::MatrixXf A = Eigen::MatrixXf::Zero(2, 2);
							A(0, 0) = 1/h;
							A(0, 1) = 1;
							A(1, 0) = 2/h;
							A(1, 1) = 3;

							Eigen::MatrixXf b = Eigen::Vector2f::Zero(2, 1);
							b(0) = cx/h + dx;
							b(1) = (ffx - bx)/h2;

							Eigen::VectorXf x = A.colPivHouseholderQr().solve(b);
							cx = x(0);
							dx = x(1);

							data->coeff(seg_idx, 2) = cx;
							data->coeff(seg_idx, 3) = dx;

							//float ay = data->coeff(seg_idx, 4);
							float by = data->coeff(seg_idx, 5);
							float cy = data->coeff(seg_idx, 6);
							float dy = data->coeff(seg_idx, 7);

							A = Eigen::MatrixXf::Zero(2, 2);
							A(0, 0) = 1/h;
							A(0, 1) = 1;
							A(1, 0) = 2/h;
							A(1, 1) = 3;

							b = Eigen::Vector2f::Zero(2, 1);
							b(0) = cy/h + dy;
							b(1) = (ffy - by)/h2;

							x = A.colPivHouseholderQr().solve(b);
							cy = x(0);
							dy = x(1);

							data->coeff(seg_idx, 6) = cy;
							data->coeff(seg_idx, 7) = dy;
						}

					}

					// edit left control point
					if (data->selected_type == _Lcp && data->points_type[data->selected+1] != C2) {
						int idx = data->selected + 1;
						int seg_idx = idx - 1;
						pointf2 cp = mouse_pos_in_canvas;

						float fx = -(cp[0] - data->points[idx][0]) / 50;
						float fy = -(cp[1] - data->points[idx][1]) / 50;

						data->cpL[data->selected] = pointf2(fx, fy);
						// update coefficients
						float h = data->parameterization[idx] - data->parameterization[idx - 1];
						float h2 = h * h;

						//float ax = data->coeff(seg_idx, 0);
						float bx = data->coeff(seg_idx, 1);
						float cx = data->coeff(seg_idx, 2);
						float dx = data->coeff(seg_idx, 3);

						Eigen::MatrixXf A = Eigen::MatrixXf::Zero(2, 2);
						A(0, 0) = 1/h;
						A(0, 1) = 1;
						A(1, 0) = 2/h;
						A(1, 1) = 3;


						Eigen::MatrixXf b = Eigen::Vector2f::Zero(2, 1);
						b(0) = cx/h + dx;
						b(1) = (fx - bx)/h2;

						Eigen::VectorXf x = A.colPivHouseholderQr().solve(b);
						cx = x(0);
						dx = x(1);

						data->coeff(seg_idx, 2) = cx;
						data->coeff(seg_idx, 3) = dx;

						//float ay = data->coeff(seg_idx, 4);
						float by = data->coeff(seg_idx, 5);
						float cy = data->coeff(seg_idx, 6);
						float dy = data->coeff(seg_idx, 7);

						A = Eigen::MatrixXf::Zero(2, 2);
						A(0, 0) = 1/h;
						A(0, 1) = 1;
						A(1, 0) = 2/h;
						A(1, 1) = 3;

						b = Eigen::Vector2f::Zero(2, 1);
						b(0) = cy/h + dy;
						b(1) = (fy - by)/h2;

						x = A.colPivHouseholderQr().solve(b);
						cy = x(0);
						dy = x(1);

						data->coeff(seg_idx, 6) = cy;
						data->coeff(seg_idx, 7) = dy;

						if (data->points_type[idx - 1] == C2) data->points_type[idx - 1] = G1;

						// G1
						if (data->points_type[data->selected+1] == G1 && idx < n-1) {
							int seg_idx = idx;

							float fx = data->cpR[idx][0];
							float fy = data->cpR[idx][1];

							float C = fx * fx + fy * fy;

							float ffx, ffy;

							bx = bx + 2 * cx * h + 3 * dx * h2;
							by = by + 2 * cy * h + 3 * dy * h2;

							if (bx != 0) {
								float k = by / bx;

								ffx = sqrt(C / (1 + k * k));
								if (bx < 0) ffx = -ffx;
								ffy = k * ffx;
							}

							else {
								ffx = 0;
								ffy = sqrt(C);
								if (by < 0) ffy = -ffy;
							}

							data->cpR[idx] = pointf2(ffx, ffy);

							// update coefficients
							float h = data->parameterization[idx + 1] - data->parameterization[idx];
							float h2 = h * h;

							//float ax = data->coeff(seg_idx, 0);
							float bx = data->coeff(seg_idx, 1);
							float cx = data->coeff(seg_idx, 2);
							float dx = data->coeff(seg_idx, 3);

							Eigen::MatrixXf A = Eigen::MatrixXf::Zero(2, 2);
							A(0, 0) = 1/h;
							A(0, 1) = 1;
							A(1, 0) = 2/h;
							A(1, 1) = 3;

							Eigen::MatrixXf b = Eigen::Vector2f::Zero(2, 1);
							b(0) = bx/h2 + cx/h + dx - ffx/h2;
							b(1) = bx/h2 + 2 * cx/h + 3 * dx - ffx/h2;

							Eigen::VectorXf x = A.colPivHouseholderQr().solve(b);
							bx = ffx;
							cx = x(0);
							dx = x(1);

							data->coeff(seg_idx, 1) = bx;
							data->coeff(seg_idx, 2) = cx;
							data->coeff(seg_idx, 3) = dx;

							float ay = data->coeff(seg_idx, 4);
							float by = data->coeff(seg_idx, 5);
							float cy = data->coeff(seg_idx, 6);
							float dy = data->coeff(seg_idx, 7);

							A = Eigen::MatrixXf::Zero(2, 2);
							A(0, 0) = 1/h;
							A(0, 1) = 1;
							A(1, 0) = 2/h;
							A(1, 1) = 3;

							b = Eigen::Vector2f::Zero(2, 1);
							b(0) = by/h2 + cy/h + dy - ffy/h2;
							b(1) = by/h2 + 2 * cy/h + 3 * dy - ffy/h2;

							x = A.colPivHouseholderQr().solve(b);
							by = ffy;
							cy = x(0);
							dy = x(1);

							data->coeff(seg_idx, 5) = by;
							data->coeff(seg_idx, 6) = cy;
							data->coeff(seg_idx, 7) = dy;

						}
					}
				}
				
			}

			// Context menu (under default mouse threshold)
			ImVec2 drag_delta = ImGui::GetMouseDragDelta(ImGuiMouseButton_Right);
			if (data->opt_enable_context_menu && ImGui::IsMouseReleased(ImGuiMouseButton_Right) && drag_delta.x == 0.0f && drag_delta.y == 0.0f) {
				ImGui::OpenPopupContextItem("context");
				data->selected = unselected;
			}
			if (ImGui::BeginPopup("context"))
			{
				if (data->adding_line)
					data->points.resize(data->points.size() - 2);
				data->adding_line = false;
				if (ImGui::MenuItem("Remove one", NULL, false, data->points.size() > 0)) { 
					data->points.resize(data->points.size() - 1);
					data->points_type.resize(data->points_type.size() - 1);
					data->cpL.resize(data->cpL.size() - 1);
					data->cpR.resize(data->cpR.size() - 1);
					if (data->mode == insert) data->update = true;
					}
				if (ImGui::MenuItem("Remove all", NULL, false, data->points.size() > 0)) { 
						data->points.clear();
						data->points_type.clear();
						data->cpL.clear();
						data->cpR.clear();
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

			// Update Parameterization
			updateParameterization(data);

			// Update Coefficients
			updateCoefficients(data);

			// Draw Curve
			drawCurve(data, draw_list, origin);
			drawEdit(data, draw_list, origin);

			draw_list->PopClipRect();
		}

		ImGui::End();
	});
}


void updateParameterization(CanvasData* data) {

	if (data->update && !data->points.empty()) {
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
		data->update = false;
	}
}

void updateControlPoints(CanvasData* data) {

	if (data->points.size() > 2) {
		data->cpL.clear();
		data->cpR.clear();

		int n = data->points.size();

		float bx, by, cx, cy, dx, dy;
		float fx, fy;
		float h;

		pointf2 cp;

		for (size_t i = 0; i < n - 1; i++)
		{
			bx = data->coeff(i, 1);
			by = data->coeff(i, 5);

			fx = bx;
			fy = by;

			cp = pointf2(fx, fy);
			data->cpR.push_back(cp);
		}

		for (size_t i = 1; i < n; i++)
		{
			bx = data->coeff(i - 1, 1);
			cx = data->coeff(i - 1, 2);
			dx = data->coeff(i - 1, 3);

			by = data->coeff(i - 1, 5);
			cy = data->coeff(i - 1, 6);
			dy = data->coeff(i - 1, 7);

			h = data->parameterization[i] - data->parameterization[i - 1];

			fx = (bx + 2 * cx * h + 3 * dx * h * h);
			fy = (by + 2 * cy * h + 3 * dy * h * h);

			cp = pointf2(fx, fy);
			data->cpL.push_back(cp);
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

}

void drawEdit(CanvasData* data, ImDrawList* draw_list, const ImVec2 origin) {
	if (data->mode != edit || data->points.size() < 3) return;

	int n = data->points.size();

	// draw right control points
	for (size_t i = 0; i < n - 1; i++)
	{

		int idx = i;
		pointf2 cp = data->cpR[i];
		cp[0] = 50 * cp[0] + data->points[idx][0];
		cp[1] = 50 * cp[1] + data->points[idx][1];

		draw_list->AddCircleFilled(ImVec2(origin.x + cp[0], origin.y + cp[1]), 3.0f, IM_COL32(255, 255, 255, 255));
		draw_list->AddLine(ImVec2(origin.x + data->points[idx][0], origin.y + data->points[idx][1]), ImVec2(origin.x + cp[0], origin.y + cp[1]), IM_COL32(255, 255, 255, 255));
	}

	// draw left control points
	for (size_t i = 0; i < n - 1; i++)
	{
		int idx = i + 1;
		pointf2 cp = data->cpL[i];
		cp[0] = -50 * cp[0] + data->points[idx][0];
		cp[1] = -50 * cp[1] + data->points[idx][1];

		draw_list->AddCircleFilled(ImVec2(origin.x + cp[0], origin.y + cp[1]), 3.0f, IM_COL32(255, 255, 255, 255));
		draw_list->AddLine(ImVec2(origin.x + data->points[idx][0], origin.y + data->points[idx][1]), ImVec2(origin.x + cp[0], origin.y + cp[1]), IM_COL32(255, 255, 255, 255));
	}

	// draw the selected point
	if (data->selected != unselected)
	{
		if (data->selected_type == _point) draw_list->AddCircleFilled(ImVec2(origin.x + data->points[data->selected][0], origin.y + data->points[data->selected][1]), 3.0f, IM_COL32(255, 0, 0, 255));
		else {
			pointf2 cp;
			if (data->selected_type == _Rcp) {

				int idx = data->selected;
				cp = data->cpR[data->selected];
				cp[0] = 50 * cp[0] + data->points[idx][0];
				cp[1] = 50 * cp[1] + data->points[idx][1];

				draw_list->AddCircleFilled(ImVec2(origin.x + cp[0], origin.y + cp[1]), 3.0f, IM_COL32(255, 0, 0, 255));
			}
			else if (data->selected_type == _Lcp) {
				int idx = data->selected + 1;
				cp = data->cpL[data->selected];
				cp[0] = -50 * cp[0] + data->points[idx][0];
				cp[1] = -50 * cp[1] + data->points[idx][1];

				draw_list->AddCircleFilled(ImVec2(origin.x + cp[0], origin.y + cp[1]), 3.0f, IM_COL32(255, 0, 0, 255));
			}
		}
	}
}