
#include "prx/utilities/data_structures/gnn.hpp"
#include "prx/utilities/data_structures/abstract_node.hpp"

#include <cassert>
#include <limits>

using namespace std;

namespace prx 
{ 
    proximity_node_t::proximity_node_t()
    {
        neighbors = (long unsigned*)malloc(INIT_CAP_NEIGHBORS*sizeof(long unsigned));
        nr_neighbors = 0;
        added_index = 0;
        cap_neighbors = INIT_CAP_NEIGHBORS;
        added_to_metric = false;
    }

    proximity_node_t::~proximity_node_t()
    {
        free( neighbors );
    }

    long unsigned proximity_node_t::get_prox_index()
    {
        return prox_index;
    }

    void proximity_node_t::set_index( long unsigned indx )
    {
        prox_index = indx;
    }

    long unsigned* proximity_node_t::get_neighbors( long unsigned* nr_neigh )
    {
        *nr_neigh = nr_neighbors;
        return neighbors;
    }

    void proximity_node_t::add_neighbor( long unsigned nd )
    {
        if( nr_neighbors >= cap_neighbors-1 )
        {
        cap_neighbors = 2*cap_neighbors;
        neighbors = (long unsigned*)realloc( neighbors, cap_neighbors*sizeof(long unsigned));
        }
        neighbors[nr_neighbors] = nd;
        nr_neighbors++;
    }

    void proximity_node_t::delete_neighbor( long unsigned nd )
    {
        int index_;
        for( index_=0; index_<nr_neighbors; index_++ )
        {
        if( neighbors[index_] == nd )
            break;
        }
        assert( index_ < nr_neighbors );

        for( int i=index_; i<nr_neighbors-1; i++ )
            neighbors[i] = neighbors[i+1];
        nr_neighbors--;
    }

    void proximity_node_t::replace_neighbor( long unsigned prev, long unsigned new_index )
    {
        int index_;
        for( index_=0; index_<nr_neighbors; index_++ )
        {
            if( neighbors[index_] == prev )
                break;
        }
        // prx_assert( index_ < nr_neighbors , "Didn't find original index: "<<prev<<" to replace.");
        assert(index_ < nr_neighbors);

        neighbors[index_] = new_index;
    }

    double graph_nearest_neighbors_t::node_distance(proximity_node_t* s1,proximity_node_t* s2)
    {
        return distance_function(static_cast<abstract_node_t*>(s1)->point,static_cast<abstract_node_t*>(s2)->point);
    }

    graph_nearest_neighbors_t::graph_nearest_neighbors_t(distance_function_t d)
    {
        distance_function = d;
        added_node_id = 0;
        nodes = (proximity_node_t**)malloc(INIT_NODE_SIZE*sizeof(proximity_node_t*));
        nr_nodes = 0;
        cap_nodes = INIT_NODE_SIZE;
        second_nodes = (proximity_node_t**)malloc( MAX_KK *sizeof(proximity_node_t*));
        second_distances = (double*)malloc( MAX_KK *sizeof(double));
        query_node = new abstract_node_t();
    }

    graph_nearest_neighbors_t::~graph_nearest_neighbors_t()
    {
        free( nodes );
        free( second_nodes );
        free( second_distances);
        delete query_node;
    }

    void graph_nearest_neighbors_t::add_node( proximity_node_t* graph_node )
    {    
        if(graph_node->added_to_metric)
        {
            prx_throw("Trying to add a node that is already added "<<graph_node->get_prox_index());
        }
        int k = percolation_threshold();

        int new_k = find_k_close( (proximity_node_t*)(graph_node), second_nodes, second_distances, k );

        if( nr_nodes >= cap_nodes-1 )
        {
            cap_nodes = 2 * cap_nodes;
            nodes = (proximity_node_t**)realloc(nodes, cap_nodes*sizeof(proximity_node_t*));
        }
        nodes[nr_nodes] = graph_node;

        graph_node->set_index(nr_nodes);
        nr_nodes++;

        for( int i=0; i<new_k; i++ )
        {
            graph_node->add_neighbor( second_nodes[i]->get_prox_index() );
            second_nodes[i]->add_neighbor( graph_node->get_prox_index() );
        }
        graph_node->added_to_metric = true;
    }

    void graph_nearest_neighbors_t::remove_node( proximity_node_t* graph_node )
    {
        long unsigned nr_neighbors;
        long unsigned* neighbors = graph_node->get_neighbors( &nr_neighbors );
        for( int i=0; i<nr_neighbors; i++ )
        {
            nodes[ neighbors[i] ]->delete_neighbor( graph_node->get_prox_index() );
        }
        graph_node->remove_all_neighbors();

        int index = graph_node->get_prox_index();
        if( index < nr_nodes-1 )
        {
            nodes[index] = nodes[nr_nodes-1];
            nodes[index]->set_index( index );

            neighbors = nodes[index]->get_neighbors( &nr_neighbors );
            for( int i=0; i<nr_neighbors; i++ )
            {
                nodes[ neighbors[i] ]->replace_neighbor( nr_nodes-1, index ); 
            }
        }
        nr_nodes--;
        graph_node->added_to_metric = false;

        if( nr_nodes < (cap_nodes-1)/2 )
        {
            cap_nodes *= 0.5;
            nodes = (proximity_node_t**)realloc(nodes,cap_nodes*sizeof(proximity_node_t*));
        }
    }
    void graph_nearest_neighbors_t::clear()
    {
        for(long unsigned i=0;i<nr_nodes;i++)
        {
            nodes[i]->nr_neighbors = 0;
        }
        nr_nodes = 0;
    }


    proximity_node_t* graph_nearest_neighbors_t::single_query(const space_point_t& point)
    {
        query_node->point = point;
        double distance;
        return find_closest(query_node,&distance);
    }

    std::vector<proximity_node_t*> graph_nearest_neighbors_t::multi_query(const space_point_t& point,int k)
    {
        query_node->point = point;
        int new_k = find_k_close( query_node, second_nodes, second_distances, k );
        std::vector<proximity_node_t*> ret(second_nodes,second_nodes+new_k);
        return std::move(ret);
    }

    std::vector<proximity_node_t*> graph_nearest_neighbors_t::radius_and_closest_query(const space_point_t& point,double rad)
    {
        query_node->point = point;
        int new_k = find_delta_close_and_closest( query_node, second_nodes, second_distances, rad );
        std::vector<proximity_node_t*> ret(second_nodes,second_nodes+new_k);
        return ret;

    }


    proximity_node_t* graph_nearest_neighbors_t::find_closest( proximity_node_t* state, double* the_distance )
    {
        long unsigned min_index = -1;
        return basic_closest_search( state, the_distance, &min_index );
    }     

    int graph_nearest_neighbors_t::find_k_close( proximity_node_t* state, proximity_node_t** close_nodes, double* distances, int k )
    {
        if( nr_nodes == 0 )
        {
            return 0;
        }

        if(k > MAX_KK)
        {
            k = MAX_KK;
        }
        else if( k >= nr_nodes )
        {
            for( int i=0; i<nr_nodes; i++ )
            {
                close_nodes[i] = nodes[i];
                distances[i] = node_distance(nodes[i],state );
            }        
            sort_proximity_nodes( close_nodes, distances, 0, nr_nodes-1 );
            return nr_nodes;
        }

        clear_added();

        long unsigned min_index = -1;
        close_nodes[0] = basic_closest_search( state, &(distances[0]), &min_index );
        nodes[min_index]->added_index = added_node_id;

        min_index = 0;
        int nr_elements = 1;
        double max_distance = distances[0];

        /* Find the neighbors of the closest node if they are not already in the set of k-closest nodes.
        If the distance to any of the neighbors is less than the distance to the k-th closest element,
        then replace the last element with the neighbor and resort the list. In order to decide the next
        node to pivot about, it is either the next node on the list of k-closest
        */
        do
        {
            long unsigned nr_neighbors;
            long unsigned* neighbors = nodes[ close_nodes[min_index]->get_prox_index() ]->get_neighbors( &nr_neighbors );
            int lowest_replacement = nr_elements;

            for( int j=0; j<nr_neighbors; j++ )
            {
                proximity_node_t* the_neighbor = nodes[neighbors[j]];
                if( does_node_exist( the_neighbor ) == false )
                {
                    the_neighbor->added_index = added_node_id;

                    double distance = node_distance(the_neighbor, state );
                    bool to_resort = false;
                    if( nr_elements < k )
                    {
                        close_nodes[nr_elements] = the_neighbor;
                        distances[nr_elements] = distance;
                        nr_elements++;
                        to_resort = true;
                    }
                    else if( distance < distances[k-1] )
                    {
                        close_nodes[k-1] = the_neighbor;
                        distances[k-1] = distance;
                        to_resort = true;
                    }

                    if( to_resort )
                    {
                        int test = resort_proximity_nodes( close_nodes, distances, nr_elements-1 );
                        lowest_replacement = (test<lowest_replacement?test:lowest_replacement);
                    }
                }
            }

            /* In order to decide the next node to pivot about, 
            it is either the next node on the list of k-closest (min_index)
            or one of the new neighbors in the case that it is closer than nodes already checked.
            */
            if(min_index < lowest_replacement)
            {
                min_index++;
            }
            else
            {
                min_index = lowest_replacement;
            }
        }
        while( min_index < nr_elements );

        return nr_elements;
    }

    int graph_nearest_neighbors_t::find_delta_close_and_closest( proximity_node_t* state, proximity_node_t** close_nodes, double* distances, double delta )
    {
        if( nr_nodes == 0 )
        {
            return 0;
        }

        clear_added();

        long unsigned min_index = -1;
        close_nodes[0] = basic_closest_search( state, &(distances[0]), &min_index );

        if( distances[0] > delta )
        {
            return 1;
        }

        nodes[min_index]->added_index = added_node_id;

        int nr_points = 1;
        for( int counter = 0; counter<nr_points; counter++ )
        {
            long unsigned nr_neighbors;
            long unsigned* neighbors = close_nodes[counter]->get_neighbors( &nr_neighbors ); 
            for( int j=0; j<nr_neighbors; j++ )
            {
                proximity_node_t* the_neighbor = nodes[neighbors[j]];
                if( does_node_exist( the_neighbor ) == false )
                {
                    the_neighbor->added_index = added_node_id;
                    double distance = node_distance(the_neighbor, state );
                    if( distance < delta && nr_points < MAX_KK)
                    {
                        close_nodes[ nr_points ] = the_neighbor;
                        distances[ nr_points ] = distance;
                        nr_points++;
                    }
                }
            }
        }

        if( nr_points > 0 )
        {
            sort_proximity_nodes( close_nodes, distances, 0, nr_points - 1 );
        }

        return nr_points;
    }

    int graph_nearest_neighbors_t::find_delta_close( proximity_node_t* state, proximity_node_t** close_nodes, double* distances, double delta )
    {
        if( nr_nodes == 0 )
        {
            return 0;
        }

        clear_added();

        long unsigned min_index = -1;
        close_nodes[0] = basic_closest_search( state, &(distances[0]), &min_index );

        if( distances[0] > delta )
        {
            return 0;
        }

        nodes[min_index]->added_index = added_node_id;

        int nr_points = 1;  
        for( int counter = 0; counter<nr_points; counter++ )
        {
            long unsigned nr_neighbors;
            long unsigned* neighbors = close_nodes[counter]->get_neighbors( &nr_neighbors ); 
            for( int j=0; j<nr_neighbors; j++ )
            {
                proximity_node_t* the_neighbor = nodes[neighbors[j]];
                if( does_node_exist( the_neighbor ) == false )
                {
                    the_neighbor->added_index = added_node_id;
                    double distance = node_distance(the_neighbor, state );
                    if( distance < delta && nr_points < MAX_KK)
                    {
                        close_nodes[ nr_points ] = the_neighbor;
                        distances[ nr_points ] = distance;
                        nr_points++;
                    }
                }
            }
        }

        if( nr_points > 0 )
        {
            sort_proximity_nodes( close_nodes, distances, 0, nr_points - 1 );
        }

        return nr_points;
    }

    void graph_nearest_neighbors_t::sort_proximity_nodes( proximity_node_t** close_nodes, double* distances, int low, int high )
    { 
        if( low < high )
        {
            int left, right, pivot;
            double pivot_distance = distances[low];
            proximity_node_t* pivot_node = close_nodes[low];

            double temp;
            proximity_node_t* temp_node;

            pivot = left = low;
            right = high;
            while( left < right )
            {
                while( left <= high && distances[left] <= pivot_distance )
                {
                    left++;
                }
                while( distances[right] > pivot_distance )
                {
                    right--;
                }
                if( left < right )
                {
                    temp = distances[left];
                    distances[left] = distances[right];
                    distances[right] = temp;

                    temp_node = close_nodes[left];
                    close_nodes[left] = close_nodes[right];
                    close_nodes[right] = temp_node;
                }
            }
            distances[low] = distances[right];
            distances[right] = pivot_distance;

            close_nodes[low] = close_nodes[right];
            close_nodes[right] = pivot_node;

            sort_proximity_nodes( close_nodes, distances, low, right-1 );
            sort_proximity_nodes( close_nodes, distances, right+1, high );
        }
    }

    int graph_nearest_neighbors_t::resort_proximity_nodes( proximity_node_t** close_nodes, double* distances, int index_ )
    {
        double temp;
        proximity_node_t* temp_node;

        while( index_ > 0 && distances[ index_ ] < distances[ index_-1 ] )
        {
            temp = distances[index_];
            distances[index_] = distances[index_-1];
            distances[index_-1] = temp;

            temp_node = close_nodes[index_];
            close_nodes[index_] = close_nodes[index_-1];
            close_nodes[index_-1] = temp_node;

            index_--;
        }
        return index_;
    }

    bool graph_nearest_neighbors_t::does_node_exist( proximity_node_t* query_node )
    {
        return query_node->added_index==added_node_id;
    }

    proximity_node_t* graph_nearest_neighbors_t::basic_closest_search( proximity_node_t* state, double* the_distance, long unsigned* the_index )
    {
        if( nr_nodes == 0 )
        {
            return nullptr;
        }

        long unsigned nr_samples = sampling_function();
        double min_distance = std::numeric_limits<double>::max();
        long unsigned min_index = -1;
        for( int i=0; i<nr_samples; i++ )
        {
            int index_ = rand() % nr_nodes;
            double distance = node_distance(nodes[index_], state );
            if( distance < min_distance )
            {
                min_distance = distance;
                min_index = index_;
            }
        }

        int old_min_index = min_index;
        do
        {
            old_min_index = min_index;
            long unsigned nr_neighbors;
            long unsigned* neighbors = nodes[min_index]->get_neighbors( &nr_neighbors );
            for( int j=0; j<nr_neighbors; j++ )
            {
                double distance = node_distance(nodes[ neighbors[j] ], state );
                if( distance < min_distance )
                {
                    min_distance = distance;
                    min_index = neighbors[j];
                }
            }
        }
        while( old_min_index != min_index );

        *the_distance = min_distance;
        *the_index = min_index;
        return nodes[min_index];
    }

    void graph_nearest_neighbors_t::clear_added()
    {
        added_node_id++;
    }
}