#include <iostream>

#include "model/threeDModel.h"
#include "aStar/aStarPathPlanner.h"
#include "pathSimplifier/blockCheckerSimplifier.hpp"
#include "pathSimplifier/regressionSimplifier.hpp"
#include <glm/glm.hpp>
#include <fstream>

using namespace std;

int main() {
    float STEP = 0.05, DRONE_SIZE = 0.3;
    model::threeDmodel demoModel(3, 3, 3);
    demoModel.setGrid(1.0, 1.0, 1.0, true);
    aStarPathPlanner planner(demoModel);
    planner.setStepLength(STEP);
    planner.setDroneSize(DRONE_SIZE);
    planner.planPath(glm::vec3(0.6, 0.6, 0.6),
                     glm::vec3(2.4f, 2.4f, 2.4f));
    vector<glm::vec3> temp = vector<glm::vec3>();
    planner.getPath(temp);
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

    blockCheckerSimplifier bs = blockCheckerSimplifier(demoModel);
    bs.setDroneSize(DRONE_SIZE);
    vector<glm::vec3> blockSimplified = bs.simplify(temp);

    file.open("../dataScripts/data/cpp_block_simplified.csv", ios::out);
    if (file.is_open()) {
        file << "x,y,z" << endl;
        for (auto &ite : blockSimplified) {
            file << ite.x << "," << ite.y << "," << ite.z << endl;
        }
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }


    regressionSimplifier rs = regressionSimplifier();
    rs.setDroneSize(DRONE_SIZE);
    vector<glm::vec3> regSimplified = rs.simplify(temp);

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
    return 0;
}
