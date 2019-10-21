//
// Created by imbaguanxin on 9/26/2019.
//

#include "threeDModel.h"

namespace model {

    // Default constructor of a grid which set the grid a not blocked
    grid::grid()
            : blocked(false) {}

    // Constructor that sets the grid by given boolean value
    grid::grid(bool block)
            : blocked(block) {}

    // find whether a grid is blocked
    bool grid::getBlocked() {
        return blocked;
    }

    // set a grid by given boolean value
    void grid::setBlocked(bool block) {
        blocked = block;
    }
}