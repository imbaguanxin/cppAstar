//
// Created by imbaguanxin on 9/26/2019.
//

#ifndef ASTAR_CPP_THREEDMODEL_HPP
#define ASTAR_CPP_THREEDMODEL_HPP

#include <vector>
#include <glm/glm.hpp>

namespace model {

    // the 3D map model
    class threeDmodel;

    // the unit in a threeDmodel
    class grid;
}


class model::threeDmodel {

private:

    // The stored 3D model of the map
    std::vector<std::vector<std::vector<grid>>> space;
    // the 3D length of the map
    int xlength, ylength, zlength;

public:

    threeDmodel();

    // build map model with 3D constraint
    threeDmodel(int x, int y, int z);

    // Fill in the blocked grid
    void setGrid(float x, float y, float z, bool block);

    // Check whether a specific grid is blocked
    bool checkBlocked(float x, float y, float z);

    // Check whether a specific position is valid in the map model
    bool checkValidPos(float x, float y, float z);

    // Check whether a specific grid is blocked, input is a vec3
    bool checkBlocked(glm::vec3 pt);

    /**
     * Check whether a specific position is valid in the map model
     * @param pt This is a point represented by a glm vector
     */
    bool checkValidPos(glm::vec3 pt);

    // cout info of the model
    void printInfo();

    int getXlength();

    int getYlength();

    int getZlength();

};

// This is a specific grid. Similar to a Wrapper Class of boolean
class model::grid {

private:

    // Indicate whether a grid is blocked
    bool blocked;

public:

    // The default constructor, constructs a grid that is availabe to reach.
    grid();

    // constructs a grid according to given boolean.
    explicit grid(bool block);

    // grid getter
    bool getBlocked();

    // grid setter.
    void setBlocked(bool block);
};

#endif //ASTAR_CPP_THREEDMODEL_HPP
