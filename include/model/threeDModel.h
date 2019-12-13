//
// Created by imbaguanxin on 9/26/2019.
//

#ifndef ASTAR_CPP_THREEDMODEL_HPP
#define ASTAR_CPP_THREEDMODEL_HPP

#include <vector>
#include <glm/glm.hpp>

const int THREE_D_GRID_EMPTY = 0,
        THREE_D_GRID_BLOCKED = 1,
        THREE_D_GRID_SEARCHED = 2;

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
    void setGrid(float x, float y, float z, int gridStatus);

    // set parent
    void setFather(glm::vec3 child, glm::vec3 parent);

    // Check whether a specific position is valid in the map model
    bool checkValidPos(float x, float y, float z);

    /**
     * Check whether a specific position is valid in the map model
     * @param pt This is a point represented by a glm vector
     */
    bool checkValidPos(glm::vec3 pt);

    int checkStatus(float x, float y, float z);

    int checkStatus(glm::vec3 pt);

    glm::vec3 getFather(float x, float y, float z);

    glm::vec3 getFather(glm::vec3 pt);

    // cout info of the model
    void printInfo();

    int getXlength();

    int getYlength();

    int getZlength();

};

// This is a specific grid. Similar to a Wrapper Class of boolean
class model::grid {

private:

    // Indicate the status of a grid
    int status;

    // father
    glm::vec3 parent;

public:

    // The default constructor, constructs a grid that is available to reach.
    grid();

    // constructs a grid according to given boolean.
    explicit grid(int block);

    // grid status getter
    int getStatus();

    // grid status setter
    void setStatus(int stat);

    // father getter
    glm::vec3 getFather();

    // father setter
    void setFather(float x, float y, float z);

};

#endif //ASTAR_CPP_THREEDMODEL_HPP
