#pragma once
/**
 * @file obstacle_loader.hpp
 * @brief <b>A loader that loads obstacles from a file.</b>
 * @authors Zakary Littlefield, Aravind Sivaramakrishnan, Troy McMahon, Edgar Granados
 */

#include "prx/utilities/defs.hpp"
#include "prx/utilities/geometry/movable_object.hpp"

namespace prx
{
  /**
   * @brief <b>A loader that loads obstacles from a file.</b>
   * @param obstacles_file Name of the file to load obstacles from.
   * @return A mapping from obstacle names to pointers to the bodies.
   */
  std::pair<std::vector<std::string>,std::vector<std::shared_ptr<movable_object_t>>> load_obstacles(std::string obstacles_file);
}
