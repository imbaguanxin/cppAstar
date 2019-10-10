//
// Created by imbaguanxin on 9/26/2019.
//

#ifndef ASTAR_CPP_THREEDMODEL_HPP
#define ASTAR_CPP_THREEDMODEL_HPP

#include <vector>
#include <glm/glm.hpp>

namespace model {

    class threeDmodel;

    class grid;
}


class model::threeDmodel {

private:

    std::vector<std::vector<std::vector<grid>>> space;

    int xlength, ylength, zlength;

public:

    threeDmodel();

    threeDmodel(int x, int y, int z);

    void setGrid(float x, float y, float z, bool block);

    bool checkBlocked(float x, float y, float z);

    bool checkValidPos(float x, float y, float z);

    bool checkBlocked(glm::vec3 pt);

    bool checkValidPos(glm::vec3 pt);

    void printInfo();

};

class model::grid {

private:

    bool blocked;

public:
    grid();

    explicit grid(bool block);

    bool getBlocked();

    void setBlocked(bool block);
};

#endif //ASTAR_CPP_THREEDMODEL_HPP
