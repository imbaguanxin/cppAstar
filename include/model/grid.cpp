//
// Created by 11655 on 9/26/2019.
//

#include "threeDModel.h"
#include <iostream>

namespace model {
    grid::grid()
            : blocked(false) {}

    grid::grid(bool block)
            : blocked(block) {}

    bool grid::getBlocked() {
        return blocked;
    }

    void grid::setBlocked(bool block) {
        blocked = block;
    }
}