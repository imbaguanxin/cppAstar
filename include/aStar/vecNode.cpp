//
// Created by imbaguanxin on 2019/10/24.
//

#include "aStar/vecNode.hpp"

using namespace std;
using namespace glm;

vecNode::vecNode(float sco, glm::vec3 pos, vecNode *parentPos) :
        score(sco), position(pos), parentNode(parentPos) {}

float vecNode::getDistance() {
    return score;
}

vec3 vecNode::getPos() {
    return position;
}

vecNode *vecNode::getParent() {
    return parentNode;
}
