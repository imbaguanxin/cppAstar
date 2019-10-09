//
// Created by imbaguanxin on 2019/10/9.
//

#include "pathSimplifier.hpp"

#include <glm/glm.hpp>
#include <glm/gtx/transform.hpp>

#include <algorithm>

using namespace std;

pathSimplifier::pathSimplifier() {
    simplifiedPath = vector<glm::vec3>();
};

pathSimplifier::pathSimplifier(model::threeDmodel &m) :
        mapModel(m) {}

void pathSimplifier::setModel(model::threeDmodel &m) {
    mapModel = m;
}

void pathSimplifier::simplify(vector<glm::vec3> oriPath) {
    simplifiedPath = vector<glm::vec3>();
    simplifiedPath.emplace_back(glm::vec3(oriPath[0]));
    int beginIndex = 0;
    int endIndex = 1;
    if (oriPath.size() <= 1) {
        simplifiedPath = oriPath;
    } else {
        while (endIndex < oriPath.size()) {
            if (!checkValidPath(oriPath.at(beginIndex), oriPath.at(endIndex))) {
                simplifiedPath.emplace_back(glm::vec3(oriPath.at(endIndex - 1)));
                beginIndex = endIndex;
                endIndex++;
            } else {
                endIndex++;
            }
        }
    }
}

bool pathSimplifier::checkValidPath(glm::vec3 from, glm::vec3 to) {
    vector<glm::vec3> blockedPieces = findBlockedPieces(from, to);
    for (auto blocked : blockedPieces) {
        if (checkSingleBlock(from, to, blocked)) {
            return false;
        }
    }
    return true;
}


vector<glm::vec3> pathSimplifier::findBlockedPieces(glm::vec3 from, glm::vec3 to) {
    vector<glm::vec3> result = vector<glm::vec3>();
    pair<glm::vec3, glm::vec3> boxBound = findBoxBound(from, to);
    int boxLowX = (int) boxBound.first.x;
    int boxLowY = (int) boxBound.first.y;
    int boxLowZ = (int) boxBound.first.z;
    int boxHighX = (int) boxBound.second.x;
    int boxHighY = (int) boxBound.second.y;
    int boxHighZ = (int) boxBound.second.z;
    for (int x = boxLowX; x <= boxHighX; x++) {
        for (int y = boxLowY; y <= boxHighY; y++) {
            for (int z = boxLowZ; z <= boxHighZ; z++) {
                if (mapModel.checkBlocked(x, y, z)) {
                    result.emplace_back(glm::vec3(x, y, z));
                }
            }
        }
    }
    return result;
}


bool pathSimplifier::checkSingleBlock(glm::vec3 from, glm::vec3 to, glm::vec3 blockedPosition) {
    glm::vec4 from4 = glm::vec4(from.x, from.y, from.z, 1);
    glm::vec4 to4 = glm::vec4(to.x, to.y, to.z, 1);
    glm::mat4 modelView = glm::translate(glm::mat4(), glm::vec3(-blockedPosition.x - 0.5f, -blockedPosition.y - 0.5f,
                                                                -blockedPosition.z - 0.5f));
    glm::vec4 newFrom = modelView * from4;
    glm::vec4 newTo = modelView * to4;
    glm::vec4 dir = newTo - glm::vec4(newFrom);
    dir.w = 0;
    return checkSingleBlockedHelp(newFrom, dir);
}


bool pathSimplifier::checkSingleBlockedHelp(glm::vec4 s, glm::vec4 v) {
    float txMin = min((-0.5f - s.x) / v.x, (0.5f - s.x) / v.x);
    float txMax = max((-0.5f - s.x) / v.x, (0.5f - s.x) / v.x);
    float tyMin = min((-0.5f - s.y) / v.y, (0.5f - s.y) / v.y);
    float tyMax = max((-0.5f - s.y) / v.y, (0.5f - s.y) / v.y);
    float tzMin = min((-0.5f - s.z) / v.z, (0.5f - s.z) / v.z);
    float tzMax = max((-0.5f - s.z) / v.z, (0.5f - s.z) / v.z);

    float tMin = max(max(txMin, tyMin), tzMin);
    float tMax = min(min(txMax, tyMax), tzMax);

    return tMin < tMax && tMin <= 1 && tMin >= 0;
}


pair<glm::vec3, glm::vec3> pathSimplifier::findBoxBound(glm::vec3 from, glm::vec3 to) {
    double fromX = floor(from.x);
    double fromY = floor(from.y);
    double fromZ = floor(from.z);
    double toX = floor(to.x);
    double toY = floor(to.y);
    double toZ = floor(to.z);
    glm::vec3 boxLow = glm::vec3(min(fromX, toX), min(fromY, toY), min(fromZ, toZ));
    glm::vec3 boxHigh = glm::vec3(max(fromX, toX), max(fromY, toY), max(fromZ, toZ));
    return {boxLow, boxHigh};
}


