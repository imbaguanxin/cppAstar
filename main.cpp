#include <iostream>

#include "model/threeDModel.h"
#include "aStar/aStarPathPlanner.h"
#include <glm/glm.hpp>
#include <glm/ext.hpp>

using namespace std;

int main() {
    model::threeDmodel demoModel(3, 3, 3);
    demoModel.setGrid(1.0, 1.0, 1.0, true);
    aStarPathPlanner planner(demoModel);
    planner.setStep(0.3);
    planner.planPath(glm::vec3(0.0, 0.0, 0.0),
                     glm::vec3(2.9f, 2.9f, 2.9f));
    vector<glm::vec3> temp = vector<glm::vec3>();
    planner.getPath(temp);
    for (auto &ite : temp) {
        cout << glm::to_string(ite) << endl;
    }

    return 0;
}