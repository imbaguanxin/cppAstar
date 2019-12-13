//
// Created by imbaguanxin on 9/26/2019.
//

#include <cmath>

#include "threeDModel.h"

namespace model {

    // Default constructor of a grid which set the grid a not blocked
    grid::grid()
            : status(THREE_D_GRID_EMPTY) {}

    // Constructor that sets the grid by given boolean value
    grid::grid(int gridStatus)
            : status(gridStatus) {}

    int grid::getStatus() {
        return status;
    }

    void grid::setStatus(int stat) {
        status = stat;
    }

    // father getter
    glm::vec3 grid::getFather() {
        return glm::vec3(parent);
    }

    // father setter
    void grid::setFather(float x, float y, float z) {
        parent = glm::vec3(std::floor(x), std::floor(y), std::floor(z));
    }
}