#include <iostream>

#include "model/threeDModel.h"
#include "aStar/aStar.hpp"
#include "pathSimplifier/regressionSimplifier.hpp"
#include <glm/glm.hpp>
#include <fstream>
#include <wayPointsGenerator/wayPointsGenerator.hpp>

using namespace std;

int main() {
    glm::vec3 empty = glm::vec3();
    cout << empty.x << empty.y << empty.z << endl;

    float DRONE_SIZE = 0.5;
    model::threeDmodel demoModel(9, 9, 9);
    demoModel.setGrid(3, 3, 3, THREE_D_GRID_BLOCKED);
//    for (int i = 11; i < 21; ++i) {
//        for (int j = 11; j < 21; ++j) {
//            for (int k = 11; k < 21; ++k) {
//                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
//            }
//        }
//    }
//    demoModel.printInfo();
    glm::vec3 startPoint(1, 1, 1);
    glm::vec3 destination(7, 7, 7);

    // test astar
    aStar astar(demoModel);
    astar.setDroneSize(DRONE_SIZE);
    bool res = astar.aStarPathPlan(startPoint, destination);
    if (res){
        astar.referenceTable.printInfo();
    }
    list<glm::vec3> temp = astar.getPath();
    fstream file;
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
//
//    // test regression simplifier
//    vector<glm::vec3> tempVec = vector<glm::vec3>();
//    for (auto vec : temp) {
//        tempVec.emplace_back(vec);
//    }
//    regressionSimplifier rs = regressionSimplifier();
//    rs.setDroneSize(DRONE_SIZE);
//    vector<glm::vec3> regSimplified = rs.simplify(tempVec);
//
//    file.open("../dataScripts/data/cpp_reg_simplified.csv", ios::out);
//    if (file.is_open()) {
//        file << "x,y,z" << endl;
//        for (auto &ite : regSimplified) {
//            file << ite.x << "," << ite.y << "," << ite.z << endl;
//        }
//        file.close();
//    } else {
//        cout << "unable to open file." << endl;
//    }
//
//    // test wayPointsGenerator
//    wayPointsGenerator wpg(demoModel, 1.0, DRONE_SIZE);
//    vector<glm::vec3> wayPointsResult = wpg.genPoints(startPoint, destination);
//    file.open("../dataScripts/data/cpp_wayPointGenerator.csv", ios::out);
//    if (file.is_open()) {
//        file << "x,y,z" << endl;
//        for (auto &ite : wayPointsResult) {
//            file << ite.x << "," << ite.y << "," << ite.z << endl;
//        }
//        file.close();
//    } else {
//        cout << "unable to open file." << endl;
//    }

    return 0;
}
