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
                        tempGrid.setFather((float) a, (float) b, (float) c);
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
                    grid g = space.at(x).at(y).at(z);
                    glm::vec3 parent = g.getFather();
                    cout << x << " , "
                         << y << " , "
                         << z << " , " <<
                         g.getStatus() <<
                         " parent: " << "(" << parent.x << ", " << parent.y << ", " << parent.z << ")"
                         << endl;
                }
            }
        }
    }

    // print out all information of a 3d model
    void threeDmodel::printInfoSearched() {
        for (int x = 0; x < xlength; x++) {
            for (int y = 0; y < ylength; y++) {
                for (int z = 0; z < zlength; z++) {
                    grid g = space.at(x).at(y).at(z);
                    if (g.getStatus() == THREE_D_GRID_SEARCHED) {
                        glm::vec3 parent = g.getFather();
                        cout << x << " , "
                             << y << " , "
                             << z << " , " <<
                             g.getStatus() <<
                             " parent: " << "(" << parent.x << ", " << parent.y << ", " << parent.z << ")"
                             << endl;
                    }
                }
            }
        }
    }

    // Grid setter
    void threeDmodel::setGrid(float x, float y, float z, int gridStatus) {
        if (checkValidPos(x, y, z)) {
            space.at(floor(x)).at(floor(y)).at(floor(z)).setStatus(gridStatus);
        } else {
            throw invalid_argument("Grid out of bound!");
        }
    }

    // parent setter
    void threeDmodel::setFather(glm::vec3 child, glm::vec3 parent) {
        if (checkValidPos(child.x, child.y, child.z) && checkValidPos(parent.x, parent.y, parent.z)) {
            space.at(floor(child.x)).at(floor(child.y)).at(floor(child.z))
                    .setFather(parent.x, parent.y, parent.z);
        } else {
            throw invalid_argument("child or parent grid out of bound!");
        }
    }

    // check whether a position is valid in the map(won't go outside the map)
    bool threeDmodel::checkValidPos(float x, float y, float z) {
        return 0 <= x && x < (float) xlength && 0 <= y && y < (float) ylength && 0 <= z && z < (float) zlength;
    }

    // check whether a position is within the map
    bool threeDmodel::checkValidPos(glm::vec3 pt) {
        return checkValidPos(pt.x, pt.y, pt.z);
    }

    // check the status of a point in the form of glm vector.
    int threeDmodel::checkStatus(glm::vec3 pt) {
        return checkStatus(pt.x, pt.y, pt.z);
    }

    // check the status of a point
    int threeDmodel::checkStatus(float x, float y, float z) {
        if (checkValidPos(x, y, z)) {
            space.at(floor(x)).at(floor(y)).at(floor(z)).getStatus();
        } else {
            throw invalid_argument("Grid out of bound!");
        }
    }

    // check the father of a given point. If there is no father, it returns itself.
    glm::vec3 threeDmodel::getFather(float x, float y, float z) {
        if (checkValidPos(x, y, z)) {
            return space.at(floor(x)).at(floor(y)).at(floor(z)).getFather();
        } else {
            throw invalid_argument("Grid out of bound!");
        }
    }

    glm::vec3 threeDmodel::getFather(glm::vec3 pt) {
        return getFather(pt.x, pt.y, pt.z);
    }

    int threeDmodel::getXlength() {
        return xlength;
    }

    int threeDmodel::getYlength() {
        return ylength;
    }

    int threeDmodel::getZlength() {
        return zlength;
    }


}
