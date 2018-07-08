#include <map>
#include <unordered_map>
#include <vector>

namespace motu {
	template<class VertexType>
	class Graph {
	public:
		typedef std::pair<const int*, const int*> Offsets;

		std::vector<VertexType> vertices;

		Offsets neighbours(int vertex) const{
			const int* start = neighbourOffsets.data() + neighbourMap[vertex];
			int num = *start++;
			return std::make_pair(start, start + num);
		}

		size_t numFaces() const {
			return faceLookup.size();
		}

		Offsets face(int offset) const {
			const int* start = faces.data() + faceLookup[offset];
			int num = *start++;
			return std::make_pair(start, start + num);
		}

		Offsets neighbouringFaces(int vertex) const {
			const int *start = faceOffsets + faceMap[vertex];
			int num = *start++;
			return std::make_pair(start, start + num);
		}

		template<class VertexType, class Hasher = std::hash<VertexType> > friend class GraphBuilder;
	private:
		std::vector<int> faces, faceLookup, neighbourOffsets, neighbourMap, faceOffsets, faceMap;
	};

	template<class VertexType, class Hasher = std::hash<VertexType> >
	class GraphBuilder {
	public:

		GraphBuilder(Graph<VertexType> &graph) : graph(graph) {}

		template<class itr>
		void addPolygon(itr from, itr to) {
			int face = graph.faces.size();
			graph.faces.push_back(to - from);
			int last = getVertexOffset(*from++);
			faces.emplace(last, face);
			graph.faces.push_back(last);
			while (from != to) {
				int next = getVertexOffset(*from++);
				faces.emplace(next, face);
				graph.faces.push_back(next);
				neighbours.emplace(last, next);
				neighbours.emplace(next, last);
			}
		}

		Graph<VertexType> &complete() {
			graph.neighbourMap.resize(graph.vertices.size());
			graph.faceMap.resize(graph.vertices.size());
			complete(neighbours, graph.neighbourMap, graph.neighbourOffsets);
			for (size_t i = 0; i < graph.faces.size();) {
				graph.faceLookup.push_back(i);
				i += graph.faces[i] + 1;
			}
			complete(faces, graph.faceMap, graph.faceOffsets);
			return graph;
		}

	private:
		std::unordered_map<VertexType, int, Hasher> mapper;
		std::multimap<int, int> neighbours, faces;
		Graph<VertexType> &graph;
		std::unordered_set<int> added;

		void addNeighbours(std::multimap<int, int>::const_iterator from, std::multimap<int, int>::const_iterator to, std::vector<int> &map, std::vector<int> &offsets) {
			added.clear();
			int offset = from->first;
			while (from != to) {
				added.insert(from->second);
				++from;
			}
			map[offset] = offsets.size();
			offsets.push_back(added.size());
			for (int i : added) {
				offsets.push_back(i);
			}
		}

		int getVertexOffset(const VertexType &vert) {
			auto i = mapper.find(vert);
			if (i == mapper.end()) {
				int out = graph.vertices.size();
				graph.vertices.push_back(vert);
				mapper.emplace(vert, out);
				return out;
			}
			return i->second;
		}

		void complete(const std::multimap<int, int> &mm, std::vector<int> &map, std::vector<int> &offsets) {
			auto start = mm.begin();
			for (auto i = mm.begin(); i != mm.end(); ++i) {
				if (i->first != start->first) {
					addNeighbours(start, i, map, offsets);
					start = i;
				}
			}
			addNeighbours(start, mm.end(), map, offsets);
		}
	};
}