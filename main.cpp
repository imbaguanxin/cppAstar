#include <iostream>

#include "model/threeDModel.h"
#include "aStar/aStar.hpp"
#include "pathSimplifier/regressionSimplifier.hpp"
#include <glm/glm.hpp>
#include <fstream>
#include <wayPointsGenerator/wayPointsGenerator.hpp>

using namespace std;

int main() {
    float DRONE_SIZE = 1;
    model::threeDmodel demoModel(100, 100, 100);
    for (int i = 40; i < 50; ++i) {
        for (int j = 0; j < 20; ++j) {
            for (int k = 0; k < 100; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }

    for (int i = 0; i < 50; ++i) {
        for (int j = 40; j < 50; ++j) {
            for (int k = 0; k < 100; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }

    for (int i = 30; i < 40; ++i) {
        for (int j = 70; j < 100; ++j) {
            for (int k = 0; k < 100; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }

    for (int i = 60; i < 100; ++i) {
        for (int j = 60; j < 70; ++j) {
            for (int k = 0; k < 100; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }
    glm::vec3 startPoint(1, 1, 1);
    glm::vec3 destination(10, 80, 98);

    // test astar
    aStar astar(demoModel);
    astar.setDroneSize(DRONE_SIZE);
    if (astar.aStarPathPlan(startPoint, destination)) {
        cout << "astar plan finished" << endl;
        cout << "writing search and map data" << endl;
        fstream file;
        file.open("../dataScripts/data/status.csv", ios::out);
        if (file.is_open()) {
            file << "x,y,z,status" << endl;
            for(int x = 0; x < astar.referenceTable.getXlength(); ++x) {
                for(int y = 0; y < astar.referenceTable.getYlength(); ++y) {
                    for(int z = 0; z < astar.referenceTable.getZlength(); ++z) {
                        int num = std::max(astar.referenceTable.checkStatus(x,y,z), astar.model.checkStatus(x,y,z));
                        file << x << "," << y << "," << z << "," << num << endl;
                    }
                }
            }
            file.close();
        } else {
            cout << "unable to open file." << endl;
        }
        cout << "search and map data logging finished" << endl;
        cout << "writing path data" << endl;
        list<glm::vec3> temp = astar.getPath();
        file.open("../dataScripts/data/cpp_astar_path.csv", ios::out);
        if (file.is_open()) {
            file << "x,y,z" << endl;
            for (auto &ite : temp) {
                file << ite.x << "," << ite.y << "," << ite.z << endl;
            }
            file.close();
        } else {
            cout << "unable to open file." << endl;
        }
        cout << "path data finished writing" << endl;
        cout << "start path simplifying" << endl;
        // test regression simplifier
        vector<glm::vec3> tempVec = vector<glm::vec3>();
        for (auto vec : temp) {
            tempVec.emplace_back(vec);
        }
        regressionSimplifier rs = regressionSimplifier();
        rs.setDroneSize(DRONE_SIZE);
        vector<glm::vec3> regSimplified = rs.simplify(tempVec);
        cout << "regression simplifying finished, start logging to file" << endl;
        file.open("../dataScripts/data/cpp_reg_simplified.csv", ios::out);
        if (file.is_open()) {
            file << "x,y,z" << endl;
            for (auto &ite : regSimplified) {
                file << ite.x << "," << ite.y << "," << ite.z << endl;
            }
            file.close();
        } else {
            cout << "unable to open file." << endl;
        }
        cout << "regression logging finished." << endl;

        // test wayPointsGenerator
        cout << "Integrated Test starts" << endl;
        wayPointsGenerator wpg(demoModel, 1.0, DRONE_SIZE);
        vector<glm::vec3> wayPointsResult = wpg.genPoints(startPoint, destination);
        file.open("../dataScripts/data/cpp_wayPointGenerator.csv", ios::out);
        if (file.is_open()) {
            file << "x,y,z" << endl;
            for (auto &ite : wayPointsResult) {
                file << ite.x << "," << ite.y << "," << ite.z << endl;
            }
            file.close();
        } else {
            cout << "unable to open file." << endl;
        }
        cout << "Integrated Test finished, log to file" << endl;
    } else {
        cout << "path finding failed" << endl;
        return -1;
    }
    return 0;
}
