//
// Created by Storm Phoenix on 2021/10/17.
//

#ifndef TEXTURINGPIPELINE_LABELGRAPH_H
#define TEXTURINGPIPELINE_LABELGRAPH_H

#include <cassert>
#include <algorithm>
#include <vector>

namespace MvsTexturing {
    namespace Base {
        class LabelGraph {
        public:
            LabelGraph(std::size_t n_nodes);

            void add_edge(std::size_t n1, std::size_t n2);

            void remove_edge(std::size_t n1, std::size_t n2);

            bool has_edge(std::size_t n1, std::size_t n2) const;

            std::size_t num_edges() const;

            std::size_t num_nodes() const;

            void set_label(std::size_t n, std::size_t label);

            std::size_t get_label(std::size_t n) const;

            void get_subgraphs(std::size_t label, std::vector<std::vector<std::size_t>> *subgraphs) const;

            std::vector<std::size_t> const &get_adj_nodes(std::size_t node) const;

        private:
            std::vector<std::vector<std::size_t>> m_adj_lists;
            std::vector<std::size_t> m_labels;
            std::size_t m_n_edges;
        };

        inline void
        LabelGraph::add_edge(std::size_t n1, std::size_t n2) {
            assert(n1 < num_nodes() && n2 < num_nodes());
            if (!has_edge(n1, n2)) {
                m_adj_lists[n1].push_back(n2);
                m_adj_lists[n2].push_back(n1);
                ++m_n_edges;
            }
        }

        inline void
        delete_element(std::vector<std::size_t> *vec, std::size_t element) {
            vec->erase(std::remove(vec->begin(), vec->end(), element), vec->end());
        }

        inline void
        LabelGraph::remove_edge(std::size_t n1, std::size_t n2) {
            assert(n1 < num_nodes() && n2 < num_nodes());
            if (has_edge(n1, n2)) {
                delete_element(&m_adj_lists[n1], n2);
                delete_element(&m_adj_lists[n2], n1);
                --m_n_edges;
            }
        }

        inline bool
        LabelGraph::has_edge(std::size_t n1, std::size_t n2) const {
            assert(n1 < num_nodes() && n2 < num_nodes());
            std::vector<std::size_t> const &adj_list = m_adj_lists[n1];
            return std::find(adj_list.begin(), adj_list.end(), n2) != adj_list.end();
        }

        inline std::size_t
        LabelGraph::num_edges() const {
            return m_n_edges;
        }

        inline std::size_t
        LabelGraph::num_nodes() const {
            return m_adj_lists.size();
        }

        inline void
        LabelGraph::set_label(std::size_t n, std::size_t label) {
            assert(n < num_nodes());
            m_labels[n] = label;
        }

        inline std::size_t
        LabelGraph::get_label(std::size_t n) const {
            assert(n < num_nodes());
            return m_labels[n];
        }

        inline std::vector<std::size_t> const &
        LabelGraph::get_adj_nodes(std::size_t node) const {
            assert(node < num_nodes());
            return m_adj_lists[node];
        }
    }
}

#endif //TEXTURINGPIPELINE_LABELGRAPH_H
