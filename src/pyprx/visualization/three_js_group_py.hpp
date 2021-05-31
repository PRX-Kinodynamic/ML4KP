#include <iostream>
#include <boost/python.hpp>
#include "prx/visualization/three_js_group.hpp"


void (prx::three_js_group_t::*update_vis_infos_1_1)(prx::info_geometry_t, const prx::trajectory_t&, std::string, prx::space_t*, std::string color)  = &prx::three_js_group_t::add_vis_infos;
void (prx::three_js_group_t::*update_vis_infos_2_1)(prx::info_geometry_t, std::vector<prx::vector_t>, std::string, double)                          = &prx::three_js_group_t::add_vis_infos;
// void (prx::three_js_group_t::*update_vis_infos_2_2)(prx::info_geometry_t, std::vector<prx::vector_t>, std::string)                                  = &prx::three_js_group_t::add_vis_infos;
// void (prx::three_js_group_t::*update_vis_infos_2_3)(prx::info_geometry_t, std::vector<prx::vector_t>)                                               = &prx::three_js_group_t::add_vis_infos;



void pyprx_visualization_three_js_group_py()
{

	enum_<prx::info_geometry_t>("info_geometry")
        .value("LINE", prx::info_geometry_t::LINE)
        .value("QUAD", prx::info_geometry_t::QUAD)
        .value("FULL_LINE", prx::info_geometry_t::FULL_LINE)
        .value("CIRCLE", prx::info_geometry_t::CIRCLE)
        .export_values()
        ;

   	class_<prx::three_js_group_t>("three_js_group", init<std::vector<prx::system_ptr_t>, std::vector<std::shared_ptr<prx::movable_object_t>>>())
   		.def("output_html", &prx::three_js_group_t::output_html)
        .def("update_vis_infos", &prx::three_js_group_t::update_vis_infos)
        .def("add_vis_infos", update_vis_infos_1_1)
        // .def("add_vis_infos", update_vis_infos_1_2)
        .def("add_vis_infos", update_vis_infos_2_1)
        // .def("add_vis_infos", update_vis_infos_2_2)
        // .def("add_vis_infos", update_vis_infos_2_3)
        .def("add_detailed_vis_infos", &prx::three_js_group_t::add_detailed_vis_infos)
        .def("snapshot_state", &prx::three_js_group_t::snapshot_state)
   		;
}
