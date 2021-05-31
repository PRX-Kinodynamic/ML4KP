#pragma once

#include "prx/utilities/defs.hpp"

namespace prx
{
    class obtacle_map_t
    {
        public:

        obstacle_map_t(const std::string map_fname)
        {
            std::ifstream ifs(map_fname);
            std::string line;

            std::getline(ifs,line); // Assuming map type is octile, so skip

            std::getline(ifs,line);
            int i = 0;
            for (; i < line.length(); i++) if (std::isdigit(line[i])) break;
            line = line.substr(i,line.length() - i);
            height = std::atoi(line.c_str());
            std::cout << "Height: " << height;

            std::getline(ifs,line);
            for (i = 0; i < line.length(); i++) if (std::isdigit(line[i])) break;
            line = line.substr(i,line.length() - i);
            width = std::atoi(line.c_str());
            std::cout << " Width: " << width << std::endl;

            std::getline(ifs,line); // This is the line that says "map"

            for (i = 0; i < height; i++)
            {
                std::vector<bool> map_line;
                std::getline(ifs,line);
                for (int j = 0; j < width; j++)
                {
                    if (line[j] == '.') map_line.push_back(true);
                    else map_line.push_back(false);
                }
                extracted_map.push_back(map_line);
            }
        }

        private:
        std::vector<std::vector<bool>> extracted_map;
        int height, width;

    };
} // namespace prx
