#pragma once

#include <math.h>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <algorithm>
#include <functional>

#ifdef __cpp_lib_filesystem
    #include <filesystem>
    using std::filesystem::exists;
    using std::optional;
    using std::nullopt;
#elif __cpp_lib_experimental_filesystem
    #include <experimental/filesystem>
    using std::experimental::filesystem::create_directories;
    using std::experimental::filesystem::path;
    using std::experimental::filesystem::exists;
    using std::experimental::optional;
    using std::experimental::nullopt;
#else
    #include <boost/filesystem.hpp>
    #include <boost/optional/optional.hpp>
    #include <boost/none.hpp>
    using boost::filesystem::exists;
    using boost::optional;
    #define nullopt {}
    //using boost::optional<int> nullopt = boost::none ;
#endif
// #include <experimental/optional>
namespace prx
{

/**
 * @brief      A graph class. Does some memory management under the hood to maximize the amount of
 * utilized space.
 *
 * @tparam     Node A node type, that contains an `size_t index` member variable
 * @tparam     Edge An edge type, that contains `size_t index, size_t source_index, size_t
 * target_index` member variables
 */
template <typename Node, typename Edge>
class graph_t
{
  public:
    /**
     * @brief      Constructs a graph
     *
     * @param[in]  expected_nodes  Initial guess for number of nodes
     * @param[in]  expected_edges  Initial guess for number of edges
     */
    graph_t(size_t expected_nodes = 1000, size_t expected_edges = 1000 * log(1000))
      : num_valid_nodes(0), num_valid_edges(0), node_index_counter(0), edge_index_counter(0)
    {
        nodes.reserve(expected_nodes);
        edges.reserve(expected_edges);
    }

    /**
     * @brief      Make a new node, only populating the index
     *
     * @return     A ref wrapper around the node, so the user can update any information needed.
     */
    const std::reference_wrapper<Node> make_node()
    {
        if (available_node_positions.empty())
        {
            nodes.emplace_back(Node());
            nodes.back()->index                         = node_index_counter;
            node_index_to_position[nodes.back()->index] = node_index_counter;
            node_index_counter++;
            num_valid_nodes++;
            return std::ref(nodes.back().value());
        }
        else
        {
            auto position                              = available_node_positions.back();
            nodes[position]                            = Node();
            nodes[position]->index                     = node_index_counter;
            node_index_to_position[node_index_counter] = position;
            available_node_positions.pop_back();
            node_index_counter++;
            num_valid_nodes++;
            return std::ref(nodes[position].value());
        }
    }

    /**
     * @brief      Makes an edge, populating index, source, and target.
     *
     * @param[in]  source_vertex  The source vertex
     * @param[in]  target_vertex  The target vertex
     *
     * @return     A ref wrapper around the edge, so the user can update any information needed.
     */
    const std::reference_wrapper<Edge> make_edge(size_t source_vertex, size_t target_vertex)
    {
        if (available_edge_positions.empty())
        {
            edges.emplace_back(Edge(source_vertex, target_vertex));
            edges.back()->index                         = edge_index_counter;
            edge_index_to_position[edges.back()->index] = edge_index_counter;
            in_edges_map[target_vertex].insert(edges.back()->index);
            out_edges_map[source_vertex].insert(edges.back()->index);
            edge_index_counter++;
            num_valid_edges++;
            return std::ref(edges.back().value());
        }
        else
        {
            auto position                              = available_edge_positions.back();
            edges[position]                            = Edge(source_vertex, target_vertex);
            edges[position]->index                     = edge_index_counter;
            edge_index_to_position[edge_index_counter] = position;
            in_edges_map[target_vertex].insert(edges[position]->index);
            out_edges_map[source_vertex].insert(edges[position]->index);
            available_edge_positions.pop_back();
            edge_index_counter++;
            num_valid_edges++;
            return std::ref(edges[position].value());
        }
    }

    /**
     * @brief      Gets a node given its node index
     *
     * @param[in]  index  The index
     *
     * @return     A ref wrapper around the node
     */
    const std::reference_wrapper<Node> get_node(size_t index)
    {
        auto position = node_index_to_position[index];
        if (nodes[position])
        {
            return std::ref(nodes[position].value());
        }
        throw std::runtime_error("Trying to access an node that is unitialized");
    }

    /**
     * @brief      Gets the edge given its edge index
     *
     * @param[in]  index  The index
     *
     * @return     A ref wrapper around the edge
     */
    const std::reference_wrapper<Edge> get_edge(size_t index)
    {
        auto position = edge_index_to_position[index];
        if (edges[position])
        {
            return std::ref(edges[position].value());
        }
        throw std::runtime_error("Trying to access an edge that is unitialized");
    }

    /**
     * @brief      Removes a node.
     *
     * @param[in]  index  The node index to remove
     */
    void remove_node(size_t index)
    {
        auto position = node_index_to_position[index];
        node_index_to_position.erase(index);
        nodes[position] = nullopt;
        available_node_positions.push_back(position);
        num_valid_nodes--;
    }

    /**
     * @brief      Removes an edge.
     *
     * @param[in]  index  The edge index to remove
     */
    void remove_edge(size_t index)
    {
        auto position = edge_index_to_position[index];
        edge_index_to_position.erase(index);

        in_edges_map[edges[position]->target_index].erase(index);
        out_edges_map[edges[position]->source_index].erase(index);

        edges[position] = nullopt;
        available_edge_positions.push_back(position);
        num_valid_edges--;
    }

    /**
     * @brief      Get the number of nodes
     */
    size_t num_nodes() const { return num_valid_nodes; }

    /**
     * @brief      Get the number of edges
     */
    size_t num_edges() const { return num_valid_edges; }

    /**
     * @brief      Given a node index, get all edges that leave that node
     *
     * @param[in]  node_index  The node index
     *
     * @return     Vector of ref wrappers around edges
     */
    std::vector<std::reference_wrapper<Edge>> out_edges(size_t node_index)
    {
        std::vector<std::reference_wrapper<Edge>> out_edge_vector;

        auto &out_edge_indices = out_edges_map[node_index];
        std::transform(std::begin(out_edge_indices),
                       std::end(out_edge_indices),
                       std::back_inserter(out_edge_vector),
                       [this](auto edge_index) { return this->get_edge(edge_index); });
        return out_edge_vector;
    }

    /**
     * @brief      Given a node index, get all edges that enter that node
     *
     * @param[in]  node_index  The node index
     *
     * @return     Vector of ref wrappers around edges
     */
    std::vector<std::reference_wrapper<Edge>> in_edges(size_t node_index)
    {
        std::vector<std::reference_wrapper<Edge>> in_edge_vector;

        auto &in_edge_indices = in_edges_map[node_index];
        std::transform(std::begin(in_edge_indices),
                       std::end(in_edge_indices),
                       std::back_inserter(in_edge_vector),
                       [this](auto edge_index) { return this->get_edge(edge_index); });
        return in_edge_vector;
    }

    /**
     * @brief      Given a node index, get all nodes that have an out edge to this node
     *
     * @param[in]  node_index  The node index
     *
     * @return     Vector of ref wrappers around edges
     */
    std::vector<std::reference_wrapper<Node>> parents(size_t node_index)
    {
        std::vector<std::reference_wrapper<Node>> parent_nodes;

        auto in_edge_vec = in_edges(node_index);
        std::transform(std::begin(in_edge_vec),
                       std::end(in_edge_vec),
                       std::back_inserter(parent_nodes),
                       [this](auto in_edge) { return this->get_node(in_edge.get().source_index); });
        return parent_nodes;
    }

    /**
     * @brief      Given a node index, get all nodes that have an in edge from this node
     *
     * @param[in]  node_index  The node index
     *
     * @return     Vector of ref wrappers around edges
     */
    std::vector<std::reference_wrapper<Node>> children(size_t node_index)
    {
        std::vector<std::reference_wrapper<Node>> child_nodes;

        auto out_edge_vec = out_edges(node_index);
        std::transform(std::begin(out_edge_vec),
                       std::end(out_edge_vec),
                       std::back_inserter(child_nodes),
                       [this](auto out_edge) { return this->get_node(out_edge.get().target_index); });
        return child_nodes;
    }

    /**
     * @brief      Get all the valid nodes
     *
     * @return     The nodes.
     */
    std::vector<std::reference_wrapper<Node>> get_nodes()
    {
        std::vector<std::reference_wrapper<Node>> out_nodes;
        for (auto &node : nodes)
        {
            if (node)
            {
                out_nodes.push_back(std::ref(node.value()));
            }
        }
        return out_nodes;
    }

    /**
     * @brief      Get all the valid edges
     *
     * @return     The edges.
     */
    std::vector<std::reference_wrapper<Edge>> get_edges()
    {
        std::vector<std::reference_wrapper<Edge>> output_edges;
        for (auto &edge : edges)
        {
            if (edge)
            {
                output_edges.push_back(std::ref(edge.value()));
            }
        }
        return output_edges;
    }

  protected:
    size_t num_valid_nodes;
    size_t num_valid_edges;

    size_t node_index_counter;
    size_t edge_index_counter;

    std::vector<optional<Node>> nodes;
    std::vector<optional<Edge>> edges;

    std::vector<size_t> available_node_positions;
    std::vector<size_t> available_edge_positions;

    std::unordered_map<size_t, size_t> node_index_to_position;
    std::unordered_map<size_t, size_t> edge_index_to_position;

    std::unordered_map<size_t, std::unordered_set<size_t>> in_edges_map;
    std::unordered_map<size_t, std::unordered_set<size_t>> out_edges_map;
};

} // namespace prx
