//
// Created by imbaguanxin on 9/26/2019.
//

#include "threeDModel.h"
#include <iostream>
#include <cmath>

using namespace std;

namespace model {

    // Default constructor sets a empty 3D map
    threeDmodel::threeDmodel()
            : xlength(0), ylength(0), zlength(0) {}

    // Constructs a 3D map with all grid unblocked
    threeDmodel::threeDmodel(int x, int y, int z)
            : xlength(max(x, 0)), ylength(max(y, 0)), zlength(max(z, 0)) {
        if (x > 0 && y > 0 && z > 0) {
            vector<vector<vector<grid>>> temp3D;
            for (int a = 0; a < x; a++) {
                vector<vector<grid>> temp2d;
                for (int b = 0; b < y; b++) {
                    vector<grid> temp1d;
                    for (int c = 0; c < z; c++) {
                        grid tempGrid = grid();
                        temp1d.push_back(tempGrid);
                    }
                    temp2d.push_back(temp1d);
                }
                temp3D.push_back(temp2d);
            }
            space = temp3D;
        } else {
            throw invalid_argument("invalid dimensions");
        }
    }

    // print out all information of a 3d model
    void threeDmodel::printInfo() {
        for (int x = 0; x < xlength; x++) {
            for (int y = 0; y < ylength; y++) {
                for (int z = 0; z < zlength; z++) {
                    cout << x << " , "
                         << y << " , "
                         << z << " , " <<
                         space.at(x).at(y).at(z).getBlocked()
                         << endl;
                }
            }
        }
    }

    // Grid setter
    void threeDmodel::setGrid(float x, float y, float z, bool block) {
        if (checkValidPos(x, y, z)) {
            space.at(floor(x)).at(floor(y)).at(floor(z)).setBlocked(block);
        } else {
            throw invalid_argument("Grid out of bound!");
        }
    }

    // check whether a position is valid in the map(won't go outside the map)
    bool threeDmodel::checkValidPos(float x, float y, float z) {
        return 0 <= x && x <= (float) xlength && 0 <= y && y <= (float) ylength && 0 <= z && z <= (float) zlength;
    }

    bool threeDmodel::checkBlocked(float x, float y, float z) {
        if (checkValidPos(x, y, z)) {
            return space.at(floor(x)).at(floor(y)).at(floor(z)).getBlocked();
        } else {
            throw invalid_argument("Grid out of bound!");
        }
    }

    bool threeDmodel::checkBlocked(glm::vec3 pt) {
        return checkBlocked(pt.x, pt.y, pt.z);
    }

    bool threeDmodel::checkValidPos(glm::vec3 pt) {
        return checkValidPos(pt.x, pt.y, pt.z);
    }
}
