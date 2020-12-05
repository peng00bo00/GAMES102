#pragma once

#include <UHEMesh/HEMesh.h>

#include <UGM/UGM.h>

struct Vertex;
struct Edge;
struct Triangle;
struct HalfEdge;

using HEMeshXTraits = Ubpa::HEMeshTraits<Vertex, Edge, Triangle, HalfEdge>;

struct Vertex : Ubpa::TVertex<HEMeshXTraits> {
	// you can add any attributes and mothods to Vertex
	Ubpa::pointf3 position{ 0.f };
};

struct Edge : Ubpa::TEdge<HEMeshXTraits> {
	// you can add any attributes and mothods to Edge

	// [example]

	 //Ubpa::pointf3 midpoint() const {
	 //    auto* p = HalfEdge()->Origin();
  //       auto* q = HalfEdge()->End();
	 //    return Ubpa::pointf3::combine(std::array{ p,q }, 0.5f);
	 //}
};

struct Triangle : Ubpa::TPolygon<HEMeshXTraits> {
	// you can add any attributes and mothods to Triangle

	// [example]
	// 
	//float area{ 0.f };
	// 
	//bool istriangle() const {
	//	return Degree() == 3;
	//}
	// 
	//void updatearea() {
	//    assert(istriangle());
	//    auto* p0 = HalfEdge()->Origin();
	//    auto* p1 = HalfEdge()->Next()->Origin();
	//    auto* p2 = HalfEdge()->Next()->Next()->Origin();
	//    auto d01 = p1 - p0;
	//    auto d02 = p2 - p0;
	//    area = 0.5f * d02.cross(d01);
	//}
};

struct HalfEdge : Ubpa::THalfEdge<HEMeshXTraits> {
	// you can add any attributes and mothods to HalfEdge

};

struct HEMeshX : Ubpa::HEMesh<HEMeshXTraits> {
	// you can add any attributes and mothods to HEMeshX
};
