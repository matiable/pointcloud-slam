#ifndef OCCUPANCY_UTILITY_H_
#define OCCUPANCY_UTILITY_H_

#include <iostream>
#include <sstream>
#include <iomanip>
#include <vector>
#include <math.h>

namespace occupancy_mapping
{

    enum grid_status
    {
        occ_status,
        free_status,
        unknown
    };

    inline std::string padZeros(int val, int num_digits = 6)
    {
        std::ostringstream out;
        out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
        return out.str();
    }

    struct GeneralLaserScan
    {
        std::vector<double> ranges;
        std::vector<double> angles;
    };

    struct GridIndex
    {
        int x;
        int y;

        GridIndex() : x(0), y(0) {}
        GridIndex(int x_, int y_) : x(x_), y(y_) {}

        void SetIndex(int x_, int y_)
        {
            x = x_;
            y = y_;
        }
    };

}

#endif