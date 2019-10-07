//
// Created by 11655 on 9/26/2019.
//

#include "threeDModel.h"
#include <iostream>
#include <math.h>

using namespace std;

namespace model {

    threeDmodel::threeDmodel()
            : xlength(0), ylength(0), zlength(0) {}

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

    void threeDmodel::setGrid(float x, float y, float z, bool block) {
        if (checkValidPos(x, y, z)) {
            space.at(floor(x)).at(floor(y)).at(floor(z)).setBlocked(block);
        } else {
            throw invalid_argument("Grid out of bound!");
        }
    }

    bool threeDmodel::checkValidPos(float x, float y, float z) {
        return 0 <= x && x <= xlength && 0 <= y && y <= ylength && 0 <= z && z <= zlength;
    }

    bool threeDmodel::checkBlocked(float x, float y, float z) {
        if (checkValidPos(x, y, z)) {
            return space.at(floor(x)).at(floor(y)).at(floor(z)).getBlocked();
        } else {
            throw invalid_argument("Grid out of bound!");
        }
    }
}