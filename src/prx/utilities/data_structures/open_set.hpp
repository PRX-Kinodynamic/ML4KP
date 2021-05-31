#pragma once

#include "prx/utilities/defs.hpp"
#include <unordered_map>
#include <vector>

namespace prx
{
    /**
     * A structure to represent a node on the A* open set. The main purpose 
     * is to provide a place to store vertices' f-values and some lightweight 
     * comparison and hashing functions.
     * 
     * @brief <b> A structure to represent a node on the A* open set. </b>
     * 
     * @author Athanasios Krontiris
     */
    class astar_node_t
    {

      public:
        node_index_t vertex;
        
        double g;
        double h;            
        double f;

        astar_node_t()
        {
            vertex = 0;
            g = h = f = 0;
        }

        astar_node_t(node_index_t vertex, double g, double h) : vertex(vertex), g(g), h(h), f(g+h)
        { }

        astar_node_t(node_index_t vertex, double g) : vertex(vertex), g(g), h(0), f(g)
        { }

        astar_node_t(const astar_node_t & n) : vertex(n.vertex), g(n.g), h(n.h), f(n.f){ }

        ~astar_node_t(){ }

        const astar_node_t& operator=(const astar_node_t& other)
        {
            vertex = other.vertex;
            g = other.g;
            h = other.h;
            f = other.f;
            return (*this);
        }

        /**
         * @brief Compares two A* nodes.
         * 
         * For two non-equal A* nodes, the node with the lower f-value is returned.
         * If two nodes share the same f-value, the one with the lower vertex number is returned.
         * */
        bool operator<(const astar_node_t & n) const
        {
            if( vertex == n.vertex )
                return false;
            if( f == n.f )
                return vertex < n.vertex;
            return f < n.f;
        }

        void set_costs(double in_g, double in_h)
        {
            g = in_g;
            h = in_h;
            f = g + h;
        }

        void set_cost(double in_g)
        {
            g = in_g;
            h = 0;
            f = g + h;
        }
    };

    /**
     * @anchor open_set_t
     *
     * This is the default open set implementation for A*. It is templated for
     * greater flexibility, but alternative implementations do not need to be.
     * As long as an open set implementation provides the same interface, A* will
     * attempt to use it. This implementation is based on the hashed heap originally
     * created for the NASA Ames version of Field D*.
     *
     * @brief <b> The default open set implementation for A*. </b>
     *
     * @author Justin Cardoza
     */
    class open_set_t
    {

      public:

        open_set_t()
        {
            heap.clear();
            heap.push_back(new astar_node_t());
            heap_size = 1;
        }

        ~open_set_t()
        {
            clear();
        }

        void insert(astar_node_t* element)
        {
            if( finder.find(element) != finder.end() )
            {
                return;
            }
            if( heap.size() > heap_size )
            {
                heap[heap_size] = element;
            }
            else
            {
                heap.push_back(element);
            }
            heap_size++;
            finder[element] = heap_size - 1;
            reheap_up(heap_size - 1);
        }

        astar_node_t* remove_min()
        {
            astar_node_t* return_val = NULL;

            if( heap_size > 1 )
            {
                return_val = heap[1];
                heap[1] = heap[heap_size - 1];
                finder[heap[1]] = 1;
                finder.erase(return_val);
                heap_size--;
                reheap_down(1);
            }

            return return_val;
        }

        void update( astar_node_t* element, astar_node_t* new_element )
        {
            prx_assert(finder.find(element) != finder.end(), "Trying to update a node that isn't in the queue");
            //Find where the element is currently in the hash.
            unsigned index = finder[element];
            //Update the element's information based on the new element
            *element = *new_element;
            //Reheap
            reheap_up( index );
            reheap_down( index );
        }

        void remove(astar_node_t* element)
        {
            if( finder.find(element) != finder.end() )
            {
                unsigned i = finder[element];
                while( i != 1 )
                {
                    swap(i, parent(i));
                    i = parent(i);
                }

                astar_node_t* node = remove_min();
            }
        }

        bool contains(astar_node_t* element)
        {
            return finder.find(element) != finder.end();
        }

        const astar_node_t* peek_min() const
        {
            return heap[1];
        }

        int size() const
        {
            return heap_size - 1;
        }

        bool empty() const
        {
            return size() == 0;
        }

        void clear()
        {
            heap_size = 1;
            finder.clear();
        }

        void get_items(std::vector<astar_node_t*>& items)
        {
            items.assign(heap.begin() + 1, heap.begin() + heap_size);
        }


      private:

        void reheap_up(unsigned index)
        {
            unsigned parent_index = parent(index);

            if( parent_index != 0 && *heap[index] < *heap[parent_index] )
            {
                swap(index, parent_index);
                reheap_up(parent_index);
            }
        }

        void reheap_down(unsigned index)
        {

            unsigned child1_index = child1(index);
            unsigned child2_index = child2(index);

            if( child1_index != 0 )
            {
                if( child2_index != 0 )
                {
                    if( ( *heap[child1_index] < *heap[index] ) || ( *heap[child2_index] < *heap[index] ) )
                    {
                        if( *heap[child1_index] < *heap[child2_index] )
                        {
                            swap(child1_index, index);
                            reheap_down(child1_index);
                        }
                        else
                        {
                            swap(child2_index, index);
                            reheap_down(child2_index);
                        }
                    }
                }
                else
                {
                    if( *heap[child1_index] < *heap[index] )
                    {
                        swap(child1_index, index);
                        reheap_down(child1_index);
                    }
                }
            }
        }

        unsigned parent(unsigned index)
        {
            return index / 2;
        }

        unsigned child1(unsigned index)
        {
            unsigned val = index * 2;
            if( val < heap_size )
                return val;
            return 0;
        }

        unsigned child2(unsigned index)
        {
            unsigned val = index * 2 + 1;
            if( val < heap_size )
                return val; 
            return 0;
        }

        void swap(unsigned val1, unsigned val2)
        {
            finder[heap[val1]] = val2;
            finder[heap[val2]] = val1;
            astar_node_t* temp = heap[val1];
            heap[val1] = heap[val2];
            heap[val2] = temp;
        }

        std::vector<astar_node_t*> heap;
        std::unordered_map<astar_node_t*, unsigned> finder;
        unsigned heap_size;
    };
}
