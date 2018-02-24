//
//  dalauney_triangulation.h
//  World Maker
//
//  Created by Jerome Johnson on 14/10/17.
//  Copyright © 2017 Jerome Johnson. All rights reserved.
//

#ifndef dalauney_triangulation_h
#define dalauney_triangulation_h

#include "vector2.h"
#include "iterable_object_pool.h"
#include "half_edge.h"
#include "triangle.h"
#include "brtree.h"
#include "mesh.h"

namespace motu{

	struct EmptyProgressMonitor {
		void setTotal(size_t total) {}
		void increment() {}
	};
    
    template <class FaceData = EmptyData, class VertexData = EmptyData/*, class ProgressMonitor = EmptyProgressMonitor*/>
    class DalauneyTriangulation{
    public:
        typedef HalfEdge<Vector2, FaceData, VertexData> HalfEdgeType;
        typedef typename HalfEdgeType::Face Face;
        typedef typename HalfEdgeType::Vertex Vertex;
        typedef IterableObjectPool<HalfEdgeType> HalfEdges;
        typedef IterableObjectPool<Face> Faces;
        typedef IterableObjectPool<Vertex> Vertices;
        
        DalauneyTriangulation(const std::vector<Vector2> &points);

		DalauneyTriangulation(const Mesh &mesh) {
			mHalfEdges = new HalfEdges(mesh.vertices.size() << 1);
			mFaces = new Faces(mesh.triangles.size() / 3);
			mVertices = new Vertices(mesh.vertices.size());
			VertexMap<Vector2, HalfEdges, Vertices, Faces> builder(mHalfEdges, mVertices, mFaces);
			for (size_t i = 2; i < mesh.triangles.size(); i += 3) {
				Triangle tri(
					mesh.vertices[mesh.triangles[i - 2]].toVector2(),
					mesh.vertices[mesh.triangles[i - 1]].toVector2(),
					mesh.vertices[mesh.triangles[i]].toVector2()
				);
				builder.addPolygon(tri.vertices, tri.vertices + 3);
			}
			mExternalFace = builder.bind();
		}
        
        ~DalauneyTriangulation(){
            delete mHalfEdges;
            delete mFaces;
            delete mVertices;
        }
        
        const HalfEdges &halfEdges() const{
            return *mHalfEdges;
        }

		HalfEdges &halfEdges() {
			return *mHalfEdges;
		}
        
        const Faces &faces() const{
            return *mFaces;
        }

		Faces &faces() {
			return *mFaces;
		}
        
        const Vertices &vertices() const{
            return *mVertices;
        }

		Vertices &vertices() {
			return *mVertices;
		}
        
        const Face &externalFace() const{
            return *mExternalFace;
        }

		Face &externalFace() {
			return *mExternalFace;
		}
        
    private:
        HalfEdges *mHalfEdges;
        Faces *mFaces;
        Vertices *mVertices;
        Face *mExternalFace;
        
        struct TriangleWithCircumcircle{
            Triangle triangle;
            Circle circumcircle;
            
            TriangleWithCircumcircle(){}
            
            TriangleWithCircumcircle(const Triangle &triangle) : triangle(triangle), circumcircle(triangle.circumcircle()){}
            
            template <class Itr>
            void findUnsharedEdges(Itr first, Itr last, std::vector<Edge> &output){
                Edge myEdges[3];
                triangle.getEdges(myEdges);
                for (int i = 0; i != 3; ++i){
                    bool hasEdge = false;
                    for (Itr j = first; j != last; ++j){
                        if (*j == *this){
                            continue;
                        }
                        if (j->triangle.shares(myEdges[i])){
                            hasEdge = true;
                            break;
                        }
                    }
                    if (!hasEdge){
                        output.push_back(myEdges[i]);
                    }
                }
            }
            
            bool operator==(const TriangleWithCircumcircle &other) const{
                return triangle == other.triangle;
            }
        };
        
        typedef BRTree<BoundingRectangle, TriangleWithCircumcircle> RTree;
        
        static Triangle CreateBoundingTriangle(const std::vector<Vector2> &points){
            BoundingRectangle bounds(points.begin(), points.end());
            bounds.topLeft += Vector2 (-1.0f, -1.0f);
            bounds.bottomRight += Vector2 (1.0f, 1.0f);
            float width = bounds.width(), height = bounds.height();
            Vector2 apex(bounds.topLeft.x + width * 0.5f, bounds.topLeft.y - width * 0.6667f);
            float spread = height * 0.6667f;
            Vector2 left(bounds.topLeft.x - spread, bounds.bottomRight.y);
            Vector2 right(bounds.bottomRight.x + spread, bounds.bottomRight.y);
            return Triangle (left, apex, right);
        }
        
        static bool OfSuperTriangle(const Triangle &super, const Triangle &triangle){
            for (int i = 0; i != 3; ++i){
                for (int j = 0; j != 3; ++j){
                    if (super.vertices[i] == triangle.vertices[j]){
                        return true;
                    }
                }
            }
            return false;
        }
        
        static bool AddBadTriangles(const Vector2 &point, std::vector<TriangleWithCircumcircle> &badTriangles, const RTree &rTree, typename RTree::Values &buf){
            auto pair = rTree.containing(point, buf);
            while (pair.first != pair.second){
                if (pair.first->triangle.hasVertex(point)){
                    return false;
                }
                if (pair.first->circumcircle.contains (point)) {
                    badTriangles.push_back(*pair.first);
                }
                ++pair.first;
            }
            return true;
        }
    };
    
    template <class FaceData, class VertexData>
    DalauneyTriangulation<FaceData, VertexData>::DalauneyTriangulation(const std::vector<Vector2> &points){
        typedef HalfEdge<FaceData, VertexData> HalfEdgeType;
        int expectedTriangles = points.size() - 2;
        mHalfEdges = new HalfEdges(expectedTriangles * 3);
        mFaces = new Faces(expectedTriangles);
        mVertices = new Vertices(points.size());
        RTree rTree(expectedTriangles);
        TriangleWithCircumcircle super(CreateBoundingTriangle(points));
        rTree.add(super.circumcircle.boundingRectangle(), super);
        std::vector<TriangleWithCircumcircle> badTriangles;
        typename RTree::Values searchBuffer;
        searchBuffer.reserve(expectedTriangles);
        std::vector<Edge> poly;
        for (auto i = points.begin(); i != points.end(); ++i){
            if (!AddBadTriangles(*i, badTriangles, rTree, searchBuffer)){
                continue;
            }
            for (auto i = badTriangles.begin(); i != badTriangles.end(); ++i){
                i->findUnsharedEdges(badTriangles.begin(), badTriangles.end(), poly);
            }
            for (auto bad : badTriangles){
                rTree.remove(bad.circumcircle.boundingRectangle(), bad);
            }
            for (auto edge : poly){
                TriangleWithCircumcircle tri(Triangle(edge.endA, edge.endB, *i));
                if (!tri.triangle.isClockwise()){
                    tri.triangle.flipRotation();
                }
                rTree.add(tri.circumcircle.boundingRectangle(), tri);
            }
            poly.clear();
            badTriangles.clear();
        }
        auto pair = rTree.all(searchBuffer);
        VertexMap<Vector2, HalfEdges, Vertices, Faces> builder(mHalfEdges, mVertices, mFaces);
        while (pair.first != pair.second){
            if (!OfSuperTriangle(super.triangle, pair.first->triangle)){
                builder.addPolygon(pair.first->triangle.vertices, pair.first->triangle.vertices + 3);
            }
            ++pair.first;
        }
        mExternalFace = builder.bind();
    }
}

#endif /* dalauney_triangulation_h */
