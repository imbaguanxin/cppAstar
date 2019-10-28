//
// Created by imbaguanxin on 2019/10/24.
//

#include "aStar/vecNode.hpp"

using namespace std;
using namespace glm;

/**
 * Build a node and all fields cannot be changed afterwards
 * @param sco The score of cost
 * @param pos  It's position in 3D
 * @param parentPos  It's parent
 */
vecNode::vecNode(float sco, glm::vec3 pos, vecNode *parent) :
        score(sco), position(pos), parentNode(parent) {}

float vecNode::getDistance() {
    return score;
}

vec3 vecNode::getPos() {
    return position;
}

vecNode *vecNode::getParent() {
    return parentNode;
}
