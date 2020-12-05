#include "DenoiseSystem.h"

#include "../Components/DenoiseData.h"

#include <_deps/imgui/imgui.h>

#include <spdlog/spdlog.h>


const float  PI_F = 3.14159265358979f;

using namespace Ubpa;

float area(Vertex* const A, Vertex* const B, Vertex* const C);
float VoronoiArea(Vertex* const A, Vertex* const B, Vertex* const C);
float cot(Vertex* const A, Vertex* const B, Vertex* const C);
float theta(Vertex* const A, Vertex* const B, Vertex* const C);

void DenoiseSystem::OnUpdate(Ubpa::UECS::Schedule& schedule) {
	schedule.RegisterCommand([](Ubpa::UECS::World* w) {
		auto data = w->entityMngr.GetSingleton<DenoiseData>();
		if (!data)
			return;

		if (ImGui::Begin("Denoise")) {
			if (ImGui::Button("Mesh to HEMesh")) {
				data->heMesh->Clear();
				[&]() {
					if (!data->mesh) {
						spdlog::warn("mesh is nullptr");
						return;
					}

					if (data->mesh->GetSubMeshes().size() != 1) {
						spdlog::warn("number of submeshes isn't 1");
						return;
					}

					data->copy = *data->mesh;

					std::vector<size_t> indices(data->mesh->GetIndices().begin(), data->mesh->GetIndices().end());
					data->heMesh->Init(indices, 3);
					if (!data->heMesh->IsTriMesh())
						spdlog::warn("HEMesh init fail");
					
					for (size_t i = 0; i < data->mesh->GetPositions().size(); i++)
						data->heMesh->Vertices().at(i)->position = data->mesh->GetPositions().at(i);

					spdlog::info("Mesh to HEMesh success");
				}();
			}

			if (ImGui::Button("Add Noise")) {
				[&]() {
					if (!data->heMesh->IsTriMesh()) {
						spdlog::warn("HEMesh isn't triangle mesh");
						return;
					}

					for (auto* v : data->heMesh->Vertices()) {
						v->position += data->randomScale * (
							2.f * Ubpa::vecf3{ Ubpa::rand01<float>(),Ubpa::rand01<float>() ,Ubpa::rand01<float>() } - Ubpa::vecf3{ 1.f }
						);
					}

					spdlog::info("Add noise success");
				}();
			}

			if (ImGui::Button("Denoise")) {
				[&]() {
					if (!data->mesh) {
						spdlog::warn("mesh is nullptr");
						return;
					}

					for (size_t i = 0; i < data->num_step; i++)
					{

						std::vector<Ubpa::pointf3> pos;
						// loop over all the vertices
						for (auto* P : data->heMesh->Vertices()) {

							// skip vertex on boundary
							if (P->IsOnBoundary()) {
								pos.push_back(P->position);
							}
							else {
								// loop over all the adjacent vertices
								auto* he = P->HalfEdge();

								float A = 0.0;
								Ubpa::valf3 Hn{ 0.f };

								do {
									auto* Q = he->End();

									auto* V1 = he->Next()->End();
									auto* V2 = he->Pair()->Next()->End();

									float cot1 = cot(P, Q, V1);
									float cot2 = cot(P, Q, V2);

									Hn += (cot1 + cot2) * (P->position - Q->position);
									A += VoronoiArea(Q, V1, P);

									//Hn += Q->position;
									//A += 1;

									he = he->Pair()->Next();
								} while (he != P->HalfEdge());

								Hn = Hn / (-4*A);

								//Hn =  Hn / A;
								//Hn -= P->position;

								pos.push_back(P->position + data->lambda * Hn);
							};
						}

						// update vertices position
						for (size_t j = 0; j < pos.size(); j++)
						{
							data->heMesh->Vertices().at(j)->position = pos[j];
						}

						spdlog::info("Update vertex positions");
					}

				}();
			}

			if (ImGui::Button("Set Normal to Color")) {
				[&]() {
					if (!data->mesh) {
						spdlog::warn("mesh is nullptr");
						return;
					}

					data->mesh->SetToEditable();
					const auto& normals = data->mesh->GetNormals();
					std::vector<rgbf> colors;
					for (const auto& n : normals)
						colors.push_back((n.as<valf3>() + valf3{ 1.f }) / 2.f);
					data->mesh->SetColors(std::move(colors));

					spdlog::info("Set Normal to Color Success");
				}();
			}

			if (ImGui::Button("Set Mean Curvature to Color")) {
				[&]() {
					if (!data->mesh) {
						spdlog::warn("mesh is nullptr");
						return;
					}

					data->mesh->SetToEditable();
					std::vector<rgbf> colors;

					for (auto* P : data->heMesh->Vertices()) {
						valf3 color{ 0.f };

						if (P->IsOnBoundary()) {
							colors.push_back(color);
						}
						else {
							auto* he = P->HalfEdge();
							float A = 0.0;

							do {
								auto* Q = he->End();

								auto* V1 = he->Next()->End();
								auto* V2 = he->Pair()->Next()->End();

								float cot1 = cot(P, Q, V1);
								float cot2 = cot(P, Q, V2);

								color += (cot1 + cot2) * (P->position - Q->position);
								A += VoronoiArea(Q, V1, P);

								he = he->Pair()->Next();
							} while (he != P->HalfEdge());

							color /= 2*A;

							colors.push_back(color);
						}
					}

					data->mesh->SetColors(std::move(colors));

					spdlog::info("Set Mean Curvature to Color Success");
				}();
			}

			if (ImGui::Button("Set Gaussian Curvature to Color")) {
				[&]() {
					if (!data->mesh) {
						spdlog::warn("mesh is nullptr");
						return;
					}

					data->mesh->SetToEditable();
					std::vector<rgbf> colors;

					for (auto* P : data->heMesh->Vertices()) {
						valf3 color{ 0.f };

						if (P->IsOnBoundary()) {
							colors.push_back(color);
						}
						else {
							auto* he = P->HalfEdge();
							float A = 0.0;

							do {
								auto* Q = he->End();

								auto* V = he->Next()->End();

								A += VoronoiArea(Q, V, P);
								color += theta(Q, V, P);

								he = he->Pair()->Next();
							} while (he != P->HalfEdge());


							color = (valf3{ 2 * PI_F } - color) / (2*A);

							colors.push_back(color);
						}
					}

					data->mesh->SetColors(std::move(colors));

					spdlog::info("Set Gaussian Curvature to Color Success");
					

				}();
			}

			if (ImGui::Button("HEMesh to Mesh")) {
				[&]() {
					if (!data->mesh) {
						spdlog::warn("mesh is nullptr");
						return;
					}

					if (!data->heMesh->IsTriMesh() || data->heMesh->IsEmpty()) {
						spdlog::warn("HEMesh isn't triangle mesh or is empty");
						return;
					}

					data->mesh->SetToEditable();

					const size_t N = data->heMesh->Vertices().size();
					const size_t M = data->heMesh->Polygons().size();
					std::vector<Ubpa::pointf3> positions(N);
					std::vector<uint32_t> indices(M * 3);
					for (size_t i = 0; i < N; i++)
						positions[i] = data->heMesh->Vertices().at(i)->position;
					for (size_t i = 0; i < M; i++) {
						auto tri = data->heMesh->Indices(data->heMesh->Polygons().at(i));
						indices[3 * i + 0] = static_cast<uint32_t>(tri[0]);
						indices[3 * i + 1] = static_cast<uint32_t>(tri[1]);
						indices[3 * i + 2] = static_cast<uint32_t>(tri[2]);
					}
					data->mesh->SetPositions(std::move(positions));
					data->mesh->SetIndices(std::move(indices));
					data->mesh->SetSubMeshCount(1);
					data->mesh->SetSubMesh(0, { 0, M * 3 });
					data->mesh->GenNormals();
					data->mesh->GenTangents();

					spdlog::info("HEMesh to Mesh success");
				}();
			}

			if (ImGui::Button("Recover Mesh")) {
				[&]() {
					if (!data->mesh) {
						spdlog::warn("mesh is nullptr");
						return;
					}
					if (data->copy.GetPositions().empty()) {
						spdlog::warn("copied mesh is empty");
						return;
					}

					*data->mesh = data->copy;

					spdlog::info("recover success");
				}();
			}
		}
		ImGui::End();
	});
}

float area(Vertex *const A, Vertex* const B, Vertex* const C) {

	vecf3 AB = A->position - B->position;
	vecf3 BC = B->position - C->position;
	vecf3 CA = C->position - A->position;

	return 0.5f * (AB.cross(BC)).norm();
}

float cot(Vertex* const A, Vertex* const B, Vertex* const C) {

	float c = (A->position - B->position).norm();
	float a = (B->position - C->position).norm();
	float b = (C->position - A->position).norm();

	float cos = (a * a + b * b - c * c) / (2.f * a * b);
	float sin = std::sqrt(1 - cos * cos);

	return cos / sin;
}

float theta(Vertex* const A, Vertex* const B, Vertex* const C) {

	float c = (A->position - B->position).norm();
	float a = (B->position - C->position).norm();
	float b = (C->position - A->position).norm();

	float cos = (a * a + b * b - c * c) / (2.f * a * b);
	float t = acos(cos);

	return acos(cos);
}

float VoronoiArea(Vertex* const A, Vertex* const B, Vertex* const C) {
	float Av = 0.f;

	float c = (A->position - B->position).norm();
	float a = (B->position - C->position).norm();
	float b = (C->position - A->position).norm();

	float cosA = (b * b + c * c - a * a) / (2.f * b * c);
	float cosB = (a * a + c * c - b * b) / (2.f * a * c);
	float cosC = (a * a + b * b - c * c) / (2.f * a * b);

	if (cosA > 0 && cosB > 0 && cosC > 0) {
		float cotB = cosB / sqrt(1 - cosB * cosB);
		float cotA = cosA / sqrt(1 - cosA * cosA);
		Av = (b*b*cotB + a*a*cotA) / 8.f;
	}
	else {
		float sinC = sqrt(1 - cosC * cosC);
		float area = 0.5 * a * b * sinC;

		if (cosC < 0) Av = area / 2.f;
		else Av = area / 4.f;
	}

	return Av;
}