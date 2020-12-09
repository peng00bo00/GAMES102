#include "DenoiseSystem.h"

#include "../Components/DenoiseData.h"

#include <_deps/imgui/imgui.h>

#include <spdlog/spdlog.h>

#include <Eigen/Sparse>
#include <Eigen/SparseQR>
# define M_PI           3.14159265358979323846

using namespace Ubpa;

typedef Eigen::SparseMatrix<float> SpMat;
typedef Eigen::SparseVector<float> SpVec;

float cot(Vertex* const A, Vertex* const B, Vertex* const C);
int findVertex(std::vector<Vertex*> const vector, Vertex* const P);

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

			if (ImGui::Button("Minimal Surface")) {
				[&]() {
					if (!data->mesh) {
						spdlog::warn("mesh is nullptr");
						return;
					}

					// initialization
					size_t N = data->heMesh->Vertices().size();
					SpMat L(N, N);
					SpVec dx(N), dy(N), dz(N);

					// loop over all the vertices
					for (size_t i = 0; i < N; i++)
					{
						Vertex* P = data->heMesh->Vertices().at(i);

						// boundary vertices
						if (P->IsOnBoundary())
						{
							L.insert(i, i) = 1.f;
							dx.insert(i) = P->position[0];
							dy.insert(i) = P->position[1];
							dz.insert(i) = P->position[2];
						}
						// inner vertices
						else {
							auto* he = P->HalfEdge();
							float wi = 0.f;
							do {
								Vertex* Q = he->End();
								int j = findVertex(data->heMesh->Vertices(), Q);

								Vertex* V1 = he->Next()->End();
								Vertex* V2 = he->Pair()->Next()->End();

								float wj = cot(P, Q, V1) + cot(P, Q, V2);

								wi += wj;
								L.insert(i, j) = -wj;

								he = he->Pair()->Next();
							} while (he != P->HalfEdge());

							L.insert(i, i) = wi;
						}
					}

					// solve the equation
					L.makeCompressed();
					Eigen::SparseQR<SpMat, Eigen::COLAMDOrdering<int>> solver;
					solver.compute(L);

					Eigen::VectorXf xx = solver.solve(dx);
					Eigen::VectorXf xy = solver.solve(dy);
					Eigen::VectorXf xz = solver.solve(dz);

					// update coordinates
					for (size_t i = 0; i < N; i++)
					{
						data->heMesh->Vertices().at(i)->position = valf3{xx[i], xy[i], xz[i]};
					}

					spdlog::info("Update coordinates succeed!");
				}();
			}

			if (ImGui::Button("Surface Parameterization")) {
				[&]() {
					if (!data->mesh) {
						spdlog::warn("mesh is nullptr");
						return;
					}

					// initialization
					size_t N = data->heMesh->Vertices().size();
					SpMat L(N, N);
					SpVec dx(N), dy(N);

					// fix boundary vertices
					std::vector<int> boundary_idx;
					auto boundaries = data->heMesh->Boundaries()[0];

					for (auto boundary : boundaries->NextLoop()) {
						int idx = findVertex(data->heMesh->Vertices(), boundary->Origin());
						boundary_idx.push_back(findVertex(data->heMesh->Vertices(), boundary->Origin()));
					}

					for (size_t j = 0; j < boundary_idx.size(); j++)
					{
						int i = boundary_idx[j];

						dx.insert(i) = cos(2 * M_PI * j / boundary_idx.size());
						dy.insert(i) = sin(2 * M_PI * j / boundary_idx.size());
					}

					// loop over all the vertices
					for (size_t i = 0; i < N; i++)
					{
						Vertex* P = data->heMesh->Vertices().at(i);

						// boundary vertices
						if (P->IsOnBoundary())
						{
							L.insert(i, i) = 1.f;
						}
						// inner vertices
						else {
							auto* he = P->HalfEdge();
							float wi = 0.f;
							do {
								Vertex* Q = he->End();
								int j = findVertex(data->heMesh->Vertices(), Q);

								Vertex* V1 = he->Next()->End();
								Vertex* V2 = he->Pair()->Next()->End();

								float wj = cot(P, Q, V1) + cot(P, Q, V2);

								wi += wj;
								L.insert(i, j) = -wj;

								he = he->Pair()->Next();
							} while (he != P->HalfEdge());

							L.insert(i, i) = wi;
						}
					}

					// solve the equation
					L.makeCompressed();
					Eigen::SparseQR<SpMat, Eigen::COLAMDOrdering<int>> solver;
					solver.compute(L);

					Eigen::VectorXf xx = solver.solve(dx);
					Eigen::VectorXf xy = solver.solve(dy);

					// update coordinates
					for (size_t i = 0; i < N; i++)
					{
						data->heMesh->Vertices().at(i)->position = valf3{ xx[i], xy[i], 0.f };
					}

					spdlog::info("Update coordinates succeed!");

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


float cot(Vertex* const A, Vertex* const B, Vertex* const C) {
	valf3 CA = A->position - C->position;
	valf3 CB = B->position - C->position;

	if (CA.cross(CB).norm() < 1e-7) {
		return 0.f;
	}

	return CA.dot(CB) / CA.cross(CB).norm();
}

int findVertex(std::vector<Vertex*> const vector, Vertex* const P) {
	
	for (int i = 0; i < vector.size(); i++)
	{
		if (vector[i] == P) return i;
	}

	return -1;
}