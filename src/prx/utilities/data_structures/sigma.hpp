#pragma once

#include <float.h>
#include <vector>
#include <unordered_map>

#include "prx/utilities/defs.hpp"
// #include "prx/utilities/spaces/space.hpp"
#include "prx/utilities/data_structures/tree.hpp"
#include "prx/planning/planner_functions/planner_functions.hpp"


namespace prx
{
    template<typename T> class sigma_t;
    /**
     * A sigma set for a resolution r.
     * 
     * @brief A sigma set for a resolution r.
     * 
     * @Author: Edgar Granados
     */
    template<typename T>
    class sigma_r_t
    {
        public:

            sigma_r_t(int r_init, space_point_t initial_state, int r_inc)
            {
                parent = nullptr;
                r = r_init;
                point = initial_state;
                r_increase = r_inc;
                node = nullptr;
            }

            sigma_r_t(std::shared_ptr<sigma_r_t<T>> parent_in)
            {
                parent = parent_in;
                r_increase = parent -> get_r_increase();
                r = parent -> get_r() + r_increase;
                point = nullptr;
                node = nullptr;
                // node = _n;
            }

            sigma_r_t(std::shared_ptr<sigma_r_t<T>> parent_in, T n, space_point_t pt)
            {
                parent = parent_in;
                r_increase = parent -> get_r_increase();
                r = parent -> get_r() + r_increase;
                point = pt;
                node = std::make_shared<T>(n);
                // node = Alloc.allocate(1);
            }

            std::shared_ptr<sigma_r_t<T>> get_parent()
            {
                return parent;
            }

            int get_r_increase()
            {
                return r_increase;
            }

            int get_r()
            {
                return r;
            }

            std::vector<int> get_equiv_class()
            {
                return equiv_class;
            }

            void set_node_and_point(T n, space_point_t new_point)
            {
                point = new_point;
                node = std::make_shared<T>(n);
            }

            std::pair<T, space_point_t> get_node_and_point()
            {
                return std::make_pair(*node, point);
            }

            friend std::ostream& operator<<(std::ostream& os, const sigma_r_t& obj)
            {
                std::cout << "Node: " << obj.node << std::endl;
                for (auto eqcl : obj.sigma_r)
                {
                    os << eqcl.second -> node << "," << eqcl.first << "," << *(eqcl.second -> point) << std::endl;
                }
                // os << std::endl;
                return os;
            }

        protected:
            std::shared_ptr<T> node;
            
            std::unordered_map<std::string, std::shared_ptr<sigma_r_t<T>>> sigma_r;
            int r;
            int r_increase;

            std::shared_ptr<sigma_r_t<T>> parent;
            std::vector<int> equiv_class; 
            space_point_t point;
            
            // template<typename Tt=T>

            friend class sigma_t<T>;

    };
    

    /**
     * The Sigma set used for GLC conditions. This set is the union of 
     * R sets, where R is the max resolution desired.
     * 
     * @brief  Sigma set for multiple resolutions
     * 
     * @Author: Edgar Granados
     */
    template<typename T>
    class sigma_t
    {

        public:

            sigma_t(eta_function_t eta_f, std::shared_ptr<space_t> _state_space, 
                space_point_t initial_state, int r_init, int r_increase)
            {
                eta = eta_f;
                root = std::make_shared<sigma_r_t<T>>(r_init, initial_state, r_increase);
                state_space = _state_space;

            }
            /**
             * @brief Finds pt in Sigma. Returns (true, index) if found, (false, 0) if not found.
             * @details Look for pt in the Sigma set. Returns a pair of (Found, node_index), where found is false
             * if there is no node for that equivalent class or true and the node that represents that equivalent class.
             * 
             * @param pt [description]
             * @return [description]
             */
            // std::pair<bool, T> find(space_point_t pt, int r);
            std::pair<bool, T> find(space_point_t pt, int r)
            {
                prx_assert(state_space -> is_point_in_space(pt), "Point of different state space");
                prx_assert(pt != nullptr, "Attempting to find a nullptr!?");
                prx_assert(r > 0, "r must be greater than 0");
                bool valid = false;
                T n = 0;
                space_point_t pt_found;

                std::shared_ptr<sigma_r_t<T>> s_r = root;
                std::vector<int> eqcl;
                std::string eqcl_str;

                for (int i = 1; i < r; ++i)
                {
                    eqcl = calc_equiv_class(pt, s_r -> get_r());
                    eqcl_str = equiv_class_as_string(eqcl);
            
                    if (s_r -> sigma_r.find(eqcl_str) == s_r -> sigma_r.end())
                    {
                        valid = false;
                        n = 0;
                        break;
                    }
                    else
                    {
                        s_r = s_r -> sigma_r[eqcl_str];
                        n = *(s_r -> node);
                        valid = true;
                    }

                }
                return std::make_pair(valid, n);
            }

            // void replace(T n_new, space_point_t pt_new, space_point_t pt_old, int r);
            void replace(T n_new, space_point_t pt_new, space_point_t pt_old, int r)
            {
                prx_assert(state_space -> is_point_in_space(pt_new), "Point of different state space");
                if (pt_old!=nullptr) prx_assert(state_space -> is_point_in_space(pt_old), "Point of different state space");
                std::shared_ptr<sigma_r_t<T>> s_r = root;
                std::vector<int> eqcl;
                std::string eqcl_str;
                int i = root -> get_r();
                do  
                {
                    if (pt_old == nullptr)
                    {
                        eqcl = calc_equiv_class(pt_new, s_r -> get_r());
                    }
                    else
                    {
                        eqcl = calc_equiv_class(pt_old, s_r -> get_r());
                    }
                    eqcl_str = equiv_class_as_string(eqcl);

                    if ( s_r -> sigma_r[eqcl_str] == nullptr )
                    { 
                        s_r -> sigma_r[eqcl_str] = std::make_shared<sigma_r_t<T>>(s_r, n_new, pt_new);
                        s_r -> sigma_r[eqcl_str] -> equiv_class = eqcl;
                    }
                    else
                    {
                        s_r -> sigma_r[eqcl_str] -> set_node_and_point(n_new, pt_new);
                    }

                    i += s_r -> get_r_increase();
                    s_r = s_r -> sigma_r[eqcl_str];
        
                }
                while(i <= r);
            };


            friend std::ostream& operator<< (std::ostream& os, const sigma_t<T>& obj) 
            {
                os << *(obj.root);
                return os;
            }

            std::string get_equiv_classes_grid(int R)
            {
                std::string res;
                
                std::shared_ptr<sigma_r_t<T>> s_r = root;
                int r_aux = 1;
                // for (auto eqcl : root -> sigma_r)
                // {
                //     res += "G,EQUIV_CLASS_CELL";
                //     for (auto e : eqcl.second -> equiv_class)
                //     {
                //         res += ",";
                //         res += std::to_string(e * (1.0 / eta(root -> get_r())));
                //     }
                //     res += "\n";
                // }
                return cell_to_str(R, root);
                //return res;
            }

            std::string cell_to_str(int R, std::shared_ptr<sigma_r_t<T>> s_r)
            {
                std::string res;
                printf("s_r -> r: %d\tR: %d\n", s_r -> get_r(), R );
                if ( s_r -> get_r() <= R)
                {
                    for (auto eqcl : s_r -> sigma_r)
                    {
                        res += cell_to_str(R, eqcl.second);
                        res += "G,EQUIV_CLASS_CELL";
                        for (auto e : eqcl.second -> equiv_class)
                        {
                            res += ",";
                            res += std::to_string(e * (1.0 / eta(s_r -> get_r())));
                        }
                        res += "\n";
                    }
                }
                // else
                // {
                //     res += "G,EQUIV_CLASS_CELL";
                //     for (auto e : s_r -> equiv_class)
                //     {
                //         res += ",";
                //         res += std::to_string(e * (1.0 / eta(s_r -> get_r())));
                //     }
                //     res += "\n";
                // }
                return res;

            }

            void set_sigma_limit(const int _sigma_limit)
            {
                sigma_limit = _sigma_limit;
            }

        protected:
            // std::string equiv_class_as_string(std::vector<int> equiv_class);
            std::string equiv_class_as_string(std::vector<int> equiv_class)
            {
                std::string str;
                bool first = true;
                for (auto p : equiv_class)
                {
                    str += first?"":","; first = false;
                    if (p == -0) // Necessary to handling the case of negative zero ==> -0 == 0
                    {
                        str += std::to_string(0);
                    }
                    else
                    {
                        str += std::to_string(p);
                    }
                }
                return str;
            }

            // std::vector<int> calc_equiv_class(space_point_t pt, int R);
            std::vector<int> calc_equiv_class(space_point_t pt, int R)
            {   
                std::vector<int> eqcl;
                double eta_val = eta(R);
                int aux;

                for (auto p : *pt)
                {
                    aux = static_cast<int>(std::floor(eta_val * p));
                    aux = aux==-0?0:aux;
                    eqcl.push_back(aux);
                }
                prx_assert(eqcl.size()==2,"Something wrong happened");
                return eqcl;
            }

            std::shared_ptr<space_t> state_space;
        private:
            std::shared_ptr<sigma_r_t<T>> root;
            eta_function_t eta;
            double sigma_limit;
    };
}
