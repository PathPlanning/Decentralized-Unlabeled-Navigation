#ifndef CTP_SWAP_NODE_HPP
#define CTP_SWAP_NODE_HPP

#include <cstdint>

#include "xtensor/xtensor.hpp"



//std::pair<int64_t, int64_t>


struct GridPosHash {
	inline std::int64_t operator()(const std::pair<int64_t, int64_t> &item) const {
		return item.first * 1000 + item.second;
	}
};

template <typename T>
struct Node {
	T node_id;
	double g = 0;
	double len = 0;
	std::shared_ptr<Node<T>> parent = nullptr;
};

template <typename T>
class NodeCompare {
  public:
	bool operator()(std::shared_ptr<Node<T>> first, std::shared_ptr<Node<T>> second) {
		return first->g > second->g;
	};
};

#endif // CTP_SWAP_NODE_HPP