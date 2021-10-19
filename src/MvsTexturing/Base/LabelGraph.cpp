//
// Created by Storm Phoenix on 2021/10/17.
//

#include "LabelGraph.h"

#include <list>
#include <limits>

namespace MvsTexturing {
    namespace Base {
        LabelGraph::LabelGraph(std::size_t n_nodes) {
            m_adj_lists.resize(n_nodes);
            m_labels.resize(n_nodes);
            m_n_edges = 0;
        }

        void LabelGraph::get_subgraphs(std::size_t label, std::vector<std::vector<std::size_t>> *subgraphs) const {
            std::vector<bool> used(m_adj_lists.size(), false);
            for (std::size_t i = 0; i < m_adj_lists.size(); ++i) {
                if (m_labels[i] == label && !used[i]) {
                    subgraphs->push_back(std::vector<std::size_t>());

                    std::list<std::size_t> queue;

                    queue.push_back(i);
                    used[i] = true;

                    while (!queue.empty()) {
                        std::size_t node = queue.front();
                        queue.pop_front();

                        subgraphs->back().push_back(node);

                        /* Add all unused neighbours with the same label to the queue. */
                        std::vector<std::size_t> const &adj_list = m_adj_lists[node];
                        for (std::size_t j = 0; j < adj_list.size(); ++j) {
                            std::size_t adj_node = adj_list[j];
                            assert(adj_node < m_labels.size() && adj_node < used.size());
                            if (m_labels[adj_node] == label && !used[adj_node]) {
                                queue.push_back(adj_node);
                                used[adj_node] = true;
                            }
                        }
                    }
                }
            }
        }
    }
}