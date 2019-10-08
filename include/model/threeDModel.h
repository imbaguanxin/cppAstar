//
// Created by 11655 on 9/26/2019.
//

#ifndef CPPPRACTICE_THREEDMODEL_H
#define CPPPRACTICE_THREEDMODEL_H

#include <vector>

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

    void printInfo();

};

class model::grid {

private:

    bool blocked;

public:
    grid();

    grid(bool block);

    bool getBlocked();

    void setBlocked(bool block);
};


#endif //CPPPRACTICE_THREEDMODEL_H
