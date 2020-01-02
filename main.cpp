#include <iostream>
#include <iomanip>

#include "model/threeDModel.h"
#include <glm/glm.hpp>
#include <fstream>
#include <wayPointsGenerator/wayPointsGenerator.hpp>
#include <HGJPathSmooth/pathSmoother.h>


using namespace std;

/**
 * 构建一个简单的10m * 10m * 10m 分辨率为 0.1m的地图
 * @return threeDmodel类的地图
 */
model::threeDmodel genDemoModelMap1() {
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
    return demoModel;
}

/**
 * 构建一个4m * 4m * 4m 分辨率为0.1m的地图.
 * @return threeDmodel类的地图
 */
model::threeDmodel genDemoModelMap444() {
    model::threeDmodel demoModel(40, 40, 40);
    for (int i = 10; i < 30; ++i) {
        for (int j = 5; j < 10; ++j) {
            for (int k = 0; k < 30; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }
    for (int i = 10; i < 30; ++i) {
        for (int j = 25; j < 30; ++j) {
            for (int k = 10; k < 40; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }
    for (int i = 25; i < 30; ++i) {
        for (int j = 10; j < 25; ++j) {
            for (int k = 0; k < 40; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }
    return demoModel;
}

/**
 * 构建一个8m * 8m * 4m的地图.
 * @return threeDmodel类的地图
 */
model::threeDmodel genDemoModelMap884() {
    model::threeDmodel demoModel(80, 80, 40);
    for (int i = 20; i < 30; ++i) {
        for (int j = 10; j < 40; ++j) {
            for (int k = 0; k < 30; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }
    for (int i = 30; i < 50; ++i) {
        for (int j = 0; j < 20; ++j) {
            for (int k = 0; k < 40; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }
    for (int i = 50; i < 60; ++i) {
        for (int j = 10; j < 40; ++j) {
            for (int k = 0; k < 40; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }
    for (int i = 50; i < 60; ++i) {
        for (int j = 40; j < 70; ++j) {
            for (int k = 10; k < 40; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }
    for (int i = 30; i < 50; ++i) {
        for (int j = 60; j < 80; ++j) {
            for (int k = 0; k < 40; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }
    for (int i = 20; i < 30; ++i) {
        for (int j = 40; j < 70; ++j) {
            for (int k = 0; k < 40; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }
    return demoModel;
}

/**
 * 构建一个20m * 20m * 8m的地图
 * @return threeDmodel类的地图
 */
model::threeDmodel genDemoModelMap20208() {
    model::threeDmodel demoModel(200, 200, 80);
    //竖板,门在骗y上方位置
    for (int i = 20; i < 25; ++i) {
        for (int j = 0; j < 200; ++j) {
            for (int k = 0; k < 80; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }
    for (int i = 20; i < 25; ++i) {
        for (int j = 160; j < 190; ++j) {
            for (int k = 0; k < 80; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_EMPTY);
            }
        }
    }
    //竖板,门在骗y下方位置
    for (int i = 50; i < 55; ++i) {
        for (int j = 0; j < 200; ++j) {
            for (int k = 0; k < 80; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }
    for (int i = 50; i < 55; ++i) {
        for (int j = 10; j < 40; ++j) {
            for (int k = 0; k < 80; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_EMPTY);
            }
        }
    }
    //两竖板中间的门
    for (int i = 25; i < 50; ++i) {
        for (int j = 90; j < 110; ++j) {
            for (int k = 0; k < 80; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }
    for (int i = 30; i < 45; ++i) {
        for (int j = 90; j < 110; ++j) {
            for (int k = 0; k < 80; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_EMPTY);
            }
        }
    }
    //障碍物,在偏下面
    for (int i = 70; i < 80; ++i) {
        for (int j = 30; j < 70; ++j) {
            for (int k = 0; k < 40; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }
    //障碍物,在偏上面
    for (int i = 70; i < 80; ++i) {
        for (int j = 130; j < 170; ++j) {
            for (int k = 40; k < 80; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }

    //最大的墙板
    for (int i = 95; i < 105; ++i) {
        for (int j = 0; j < 200; ++j) {
            for (int k = 0; k < 80; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }
    for (int i = 95; i < 105; ++i) {
        for (int j = 40; j < 60; ++j) {
            for (int k = 0; k < 25; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_EMPTY);
            }
        }
    }
    for (int i = 95; i < 105; ++i) {
        for (int j = 140; j < 160; ++j) {
            for (int k = 0; k < 25; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_EMPTY);
            }
        }
    }
    for (int i = 95; i < 105; ++i) {
        for (int j = 30; j < 70; ++j) {
            for (int k = 55; k < 75; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_EMPTY);
            }
        }
    }
    for (int i = 95; i < 105; ++i) {
        for (int j = 130; j < 170; ++j) {
            for (int k = 40; k < 80; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_EMPTY);
            }
        }
    }

    //楼板
    for (int i = 105; i < 180; ++i) {
        for (int j = 0; j < 200; ++j) {
            for (int k = 35; k < 45; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }

    //楼墙
    for (int i = 105; i < 180; ++i) {
        for (int j = 95; j < 105; ++j) {
            for (int k = 0; k < 80; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_BLOCKED);
            }
        }
    }
    for (int i = 130; i < 160; ++i) {
        for (int j = 95; j < 105; ++j) {
            for (int k = 45; k < 75; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_EMPTY);
            }
        }
    }
    for (int i = 110; i < 150; ++i) {
        for (int j = 95; j < 105; ++j) {
            for (int k = 10; k < 25; ++k) {
                demoModel.setGrid(i, j, k, THREE_D_GRID_EMPTY);
            }
        }
    }

    return demoModel;

}
/**
 * 把glm::vec3按照要求写入csv文件
 * @param path 输出路径
 * @param data 需要打印的数据
 */
void printPointsFromVector(const string &path, vector<glm::vec3> &data) {
    fstream file;
    file.open(path, ios::out);
    if (file.is_open()) {
        file << "x,y,z" << endl;
        for (auto &ite : data) {
            file << ite.x << "," << ite.y << "," << ite.z << endl;
        }
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
}

/**
 * 基于map0的测试, 飞机尺寸为0.2m, 栅格地图分辨率为0.1m
 */
int wayPointsPathSmoothTestMap0() {
    float DRONE_SIZE = 0.1, GRID_SIZE = 0.1;

    glm::vec3 startPoint(0.1, 0.1, 0.1);
    glm::vec3 destination(1.0, 8.0, 9.8);

    model::threeDmodel demoModel = genDemoModelMap1();
    // test wayPointsGenerator
    cout << "wayPointsGenerator Path Smooth Test starts" << endl;
    wayPointsGenerator wpg(demoModel, GRID_SIZE, DRONE_SIZE);
    wpg.genPoints(startPoint, destination);
    cout << "way points finish generating" << endl;
    // 记录地图信息
    fstream file;
    file.open("../dataScripts/data/map0_mapStatus.csv", ios::out);
    if (file.is_open()) {
        file << wpg.printModelToString();
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    // 记录waypoint信息
    vector<glm::vec3> wayPointsResult = wpg.genPoints(startPoint, destination);
    string fileName = "../dataScripts/data/map0_wayPointTest.csv";
    printPointsFromVector(fileName, wayPointsResult);

    // 开始平滑
    vector<HGJ::vec3f> wayPoints = wpg.getGeneratedPointsHGJ();
    HGJ::pathPlanner planner;
    cout << "smoothing starts" << endl;
    auto path = planner.genCurv(wayPoints, 0.01, 0.1, 0.0, 0.0);
    cout << "smoothing finishes, log to file." << endl;
    // 开始记录平滑信息
    file.open("../dataScripts/data/map0_wayPointSmoothTest.csv", ios::out);
    if (file.is_open()) {
        file << std::setprecision(10);
        file << "x,y,z" << endl;
        for (const auto &point: path) {
            file << point << endl;
        }
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    cout << "log finished" << endl;
    return 0;
}
/**
 * 基于map1的第一组测试, 飞机尺寸为0.4m, 栅格地图分辨率为0.1m
 */
int wayPointsPathSmoothTestMap1_1() {
    float DRONE_SIZE = 0.2, GRID_SIZE = 0.1;

    glm::vec3 startPoint(0.1, 0.1, 0.1);
    glm::vec3 destination(3.5, 3.5, 3.5);

    model::threeDmodel demoModel = genDemoModelMap444();
    // test wayPointsGenerator
    cout << "wayPointsGenerator Path Smooth Test starts" << endl;
    wayPointsGenerator wpg(demoModel, GRID_SIZE, DRONE_SIZE);
    wpg.genPoints(startPoint, destination);
    cout << "way points finish generating" << endl;
    // 记录地图信息
    fstream file;
    file.open("../dataScripts/data/map1_1_mapStatus.csv", ios::out);
    if (file.is_open()) {
        file << wpg.printModelToString();
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    // 记录waypoint信息
    vector<glm::vec3> wayPointsResult = wpg.genPoints(startPoint, destination);
    string fileName = "../dataScripts/data/map1_1_wayPointTest.csv";
    printPointsFromVector(fileName, wayPointsResult);

    // 开始平滑
    vector<HGJ::vec3f> wayPoints = wpg.getGeneratedPointsHGJ();
    HGJ::pathPlanner planner;
    cout << "smoothing starts" << endl;
    auto path = planner.genCurv(wayPoints, 0.01, 0.1, 0.0, 0.0);
    cout << "smoothing finishes, log to file." << endl;
    // 开始记录平滑信息
    file.open("../dataScripts/data/map1_1_wayPointSmoothTest.csv", ios::out);
    if (file.is_open()) {
        file << std::setprecision(10);
        file << "x,y,z" << endl;
        for (const auto &point: path) {
            file << point << endl;
        }
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    cout << "log finished" << endl;
    return 0;
}

/**
 * 基于map1的第二组测试, 飞机尺寸为0.4m, 栅格地图分辨率为0.1m
 */
int wayPointsPathSmoothTestMap1_2() {
    float DRONE_SIZE = 0.2, GRID_SIZE = 0.1;

    glm::vec3 startPoint(0.3, 3.5, 0.1);
    glm::vec3 destination(2, 2, 2);

    model::threeDmodel demoModel = genDemoModelMap444();
    // test wayPointsGenerator
    cout << "wayPointsGenerator Path Smooth Test starts" << endl;
    wayPointsGenerator wpg(demoModel, GRID_SIZE, DRONE_SIZE);
    wpg.genPoints(startPoint, destination);
    cout << "way points finish generating" << endl;
    // 记录地图信息
    fstream file;
    file.open("../dataScripts/data/map1_2_mapStatus.csv", ios::out);
    if (file.is_open()) {
        file << wpg.printModelToString();
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    // 记录waypoint信息
    vector<glm::vec3> wayPointsResult = wpg.genPoints(startPoint, destination);
    string fileName = "../dataScripts/data/map1_2_wayPointTest.csv";
    printPointsFromVector(fileName, wayPointsResult);

    // 开始平滑
    vector<HGJ::vec3f> wayPoints = wpg.getGeneratedPointsHGJ();
    HGJ::pathPlanner planner;
    cout << "smoothing starts" << endl;
    auto path = planner.genCurv(wayPoints, 0.01, 0.1, 0.0, 0.0);
    cout << "smoothing finishes, log to file." << endl;
    // 开始记录平滑信息
    file.open("../dataScripts/data/map1_2_wayPointSmoothTest.csv", ios::out);
    if (file.is_open()) {
        file << std::setprecision(10);
        file << "x,y,z" << endl;
        for (const auto &point: path) {
            file << point << endl;
        }
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    cout << "log finished" << endl;
    return 0;
}

/**
 * 基于map1的第三组测试, 飞机尺寸为0.4m, 栅格地图分辨率为0.1m
 */
int wayPointsPathSmoothTestMap1_3() {
    float DRONE_SIZE = 0.2, GRID_SIZE = 0.1;

    glm::vec3 startPoint(2, 2, 2);
    glm::vec3 destination(3.5, 0.5, 2);

    model::threeDmodel demoModel = genDemoModelMap444();
    // test wayPointsGenerator
    cout << "wayPointsGenerator Path Smooth Test starts" << endl;
    wayPointsGenerator wpg(demoModel, GRID_SIZE, DRONE_SIZE);
    wpg.genPoints(startPoint, destination);
    cout << "way points finish generating" << endl;
    // 记录地图信息
    fstream file;
    file.open("../dataScripts/data/map1_3_mapStatus.csv", ios::out);
    if (file.is_open()) {
        file << wpg.printModelToString();
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    // 记录waypoint信息
    vector<glm::vec3> wayPointsResult = wpg.genPoints(startPoint, destination);
    string fileName = "../dataScripts/data/map1_3_wayPointTest.csv";
    printPointsFromVector(fileName, wayPointsResult);

    // 开始平滑
    vector<HGJ::vec3f> wayPoints = wpg.getGeneratedPointsHGJ();
    HGJ::pathPlanner planner;
    cout << "smoothing starts" << endl;
    auto path = planner.genCurv(wayPoints, 0.01, 0.1, 0.0, 0.0);
    cout << "smoothing finishes, log to file." << endl;
    // 开始记录平滑信息
    file.open("../dataScripts/data/map1_3_wayPointSmoothTest.csv", ios::out);
    if (file.is_open()) {
        file << std::setprecision(10);
        file << "x,y,z" << endl;
        for (const auto &point: path) {
            file << point << endl;
        }
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    cout << "log finished" << endl;
    return 0;
}

/**
 * 基于map2的第一组测试, 飞机尺寸为0.4m, 栅格地图分辨率为0.1m
 */
int wayPointsPathSmoothTestMap2_1() {
    float DRONE_SIZE = 0.2, GRID_SIZE = 0.1;

    glm::vec3 startPoint(0.3, 0.3, 0.3);
    glm::vec3 destination(7.5, 7.5, 3.5);

    model::threeDmodel demoModel = genDemoModelMap884();
    // test wayPointsGenerator
    cout << "wayPointsGenerator Path Smooth Test starts" << endl;
    wayPointsGenerator wpg(demoModel, GRID_SIZE, DRONE_SIZE);
    wpg.genPoints(startPoint, destination);
    cout << "way points finish generating" << endl;
    // 记录地图信息
    fstream file;
    file.open("../dataScripts/data/map2_1_mapStatus.csv", ios::out);
    if (file.is_open()) {
        file << wpg.printModelToString();
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    // 记录waypoint信息
    vector<glm::vec3> wayPointsResult = wpg.genPoints(startPoint, destination);
    string fileName = "../dataScripts/data/map2_1_wayPointTest.csv";
    printPointsFromVector(fileName, wayPointsResult);

    // 开始平滑
    vector<HGJ::vec3f> wayPoints = wpg.getGeneratedPointsHGJ();
    HGJ::pathPlanner planner;
    cout << "smoothing starts" << endl;
    auto path = planner.genCurv(wayPoints, 0.01, 0.1, 0.0, 0.0);
    cout << "smoothing finishes, log to file." << endl;
    // 开始记录平滑信息
    file.open("../dataScripts/data/map2_1_wayPointSmoothTest.csv", ios::out);
    if (file.is_open()) {
        file << std::setprecision(10);
        file << "x,y,z" << endl;
        for (const auto &point: path) {
            file << point << endl;
        }
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    cout << "log finished" << endl;
    return 0;
}

/**
 * 基于map2的第二组测试, 飞机尺寸为0.4m, 栅格地图分辨率为0.1m
 */
int wayPointsPathSmoothTestMap2_2() {
    float DRONE_SIZE = 0.2, GRID_SIZE = 0.1;

    glm::vec3 startPoint(0.3, 7.5, 3.5);
    glm::vec3 destination(7.5, 7.5, 3.5);

    model::threeDmodel demoModel = genDemoModelMap884();
    // test wayPointsGenerator
    cout << "wayPointsGenerator Path Smooth Test starts" << endl;
    wayPointsGenerator wpg(demoModel, GRID_SIZE, DRONE_SIZE);
    wpg.genPoints(startPoint, destination);
    cout << "way points finish generating" << endl;
    // 记录地图信息
    fstream file;
    file.open("../dataScripts/data/map2_2_mapStatus.csv", ios::out);
    if (file.is_open()) {
        file << wpg.printModelToString();
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    // 记录waypoint信息
    vector<glm::vec3> wayPointsResult = wpg.genPoints(startPoint, destination);
    string fileName = "../dataScripts/data/map2_2_wayPointTest.csv";
    printPointsFromVector(fileName, wayPointsResult);

    // 开始平滑
    vector<HGJ::vec3f> wayPoints = wpg.getGeneratedPointsHGJ();
    HGJ::pathPlanner planner;
    cout << "smoothing starts" << endl;
    auto path = planner.genCurv(wayPoints, 0.01, 0.1, 0.0, 0.0);
    cout << "smoothing finishes, log to file." << endl;
    // 开始记录平滑信息
    file.open("../dataScripts/data/map2_2_wayPointSmoothTest.csv", ios::out);
    if (file.is_open()) {
        file << std::setprecision(10);
        file << "x,y,z" << endl;
        for (const auto &point: path) {
            file << point << endl;
        }
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    cout << "log finished" << endl;
    return 0;
}

/**
 * 基于map2的第三组测试, 飞机尺寸为0.4m, 栅格地图分辨率为0.1m
 */
int wayPointsPathSmoothTestMap2_3() {
    float DRONE_SIZE = 0.2, GRID_SIZE = 0.1;

    glm::vec3 startPoint(4, 4, 0.3);
    glm::vec3 destination(7.5, 0.5, 3.5);

    model::threeDmodel demoModel = genDemoModelMap884();
    // test wayPointsGenerator
    cout << "wayPointsGenerator Path Smooth Test starts" << endl;
    wayPointsGenerator wpg(demoModel, GRID_SIZE, DRONE_SIZE);
    wpg.genPoints(startPoint, destination);
    cout << "way points finish generating" << endl;
    // 记录地图信息
    fstream file;
    file.open("../dataScripts/data/map2_3_mapStatus.csv", ios::out);
    if (file.is_open()) {
        file << wpg.printModelToString();
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    // 记录waypoint信息
    vector<glm::vec3> wayPointsResult = wpg.genPoints(startPoint, destination);
    string fileName = "../dataScripts/data/map2_3_wayPointTest.csv";
    printPointsFromVector(fileName, wayPointsResult);

    // 开始平滑
    vector<HGJ::vec3f> wayPoints = wpg.getGeneratedPointsHGJ();
    HGJ::pathPlanner planner;
    cout << "smoothing starts" << endl;
    auto path = planner.genCurv(wayPoints, 0.01, 0.1, 0.0, 0.0);
    cout << "smoothing finishes, log to file." << endl;
    // 开始记录平滑信息
    file.open("../dataScripts/data/map2_3_wayPointSmoothTest.csv", ios::out);
    if (file.is_open()) {
        file << std::setprecision(10);
        file << "x,y,z" << endl;
        for (const auto &point: path) {
            file << point << endl;
        }
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    cout << "log finished" << endl;
    return 0;
}

/**
 * 基于map3的第一组测试, 飞机尺寸为0.4m, 栅格地图分辨率为0.1m
 */
int wayPointsPathSmoothTestMap3_1() {
    float DRONE_SIZE = 0.2, GRID_SIZE = 0.1;

    glm::vec3 startPoint(0.3, 0.3, 0.3);
    glm::vec3 destination(6, 5, 4);

    model::threeDmodel demoModel = genDemoModelMap20208();
    // test wayPointsGenerator
    cout << "wayPointsGenerator Path Smooth Test starts" << endl;
    wayPointsGenerator wpg(demoModel, GRID_SIZE, DRONE_SIZE);
    wpg.genPoints(startPoint, destination);
    cout << "way points finish generating" << endl;
    // 记录地图信息
    fstream file;
    file.open("../dataScripts/data/map3_1_mapStatus.csv", ios::out);
    if (file.is_open()) {
        file << wpg.printModelToString();
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    // 记录waypoint信息
    vector<glm::vec3> wayPointsResult = wpg.genPoints(startPoint, destination);
    string fileName = "../dataScripts/data/map3_1_wayPointTest.csv";
    printPointsFromVector(fileName, wayPointsResult);

    // 开始平滑
    vector<HGJ::vec3f> wayPoints = wpg.getGeneratedPointsHGJ();
    HGJ::pathPlanner planner;
    cout << "smoothing starts" << endl;
    auto path = planner.genCurv(wayPoints, 0.01, 0.1, 0.0, 0.0);
    cout << "smoothing finishes, log to file." << endl;
    // 开始记录平滑信息
    file.open("../dataScripts/data/map3_1_wayPointSmoothTest.csv", ios::out);
    if (file.is_open()) {
        file << std::setprecision(10);
        file << "x,y,z" << endl;
        for (const auto &point: path) {
            file << point << endl;
        }
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    cout << "log finished" << endl;
    return 0;
}
/**
 * 基于map3的第二组测试, 飞机尺寸为0.4m, 栅格地图分辨率为0.1m
 */
int wayPointsPathSmoothTestMap3_2() {
    float DRONE_SIZE = 0.2, GRID_SIZE = 0.1;

    glm::vec3 startPoint(0.3, 0.3, 0.3);
    glm::vec3 destination(15, 5, 6);

    model::threeDmodel demoModel = genDemoModelMap20208();
    // test wayPointsGenerator
    cout << "wayPointsGenerator Path Smooth Test starts" << endl;
    wayPointsGenerator wpg(demoModel, GRID_SIZE, DRONE_SIZE);
    wpg.genPoints(startPoint, destination);
    cout << "way points finish generating" << endl;
    // 记录地图信息
    fstream file;
    file.open("../dataScripts/data/map3_2_mapStatus.csv", ios::out);
    if (file.is_open()) {
        file << wpg.printModelToString();
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    // 记录waypoint信息
    vector<glm::vec3> wayPointsResult = wpg.genPoints(startPoint, destination);
    string fileName = "../dataScripts/data/map3_2_wayPointTest.csv";
    printPointsFromVector(fileName, wayPointsResult);

    // 开始平滑
    vector<HGJ::vec3f> wayPoints = wpg.getGeneratedPointsHGJ();
    HGJ::pathPlanner planner;
    cout << "smoothing starts" << endl;
    auto path = planner.genCurv(wayPoints, 0.01, 0.1, 0.0, 0.0);
    cout << "smoothing finishes, log to file." << endl;
    // 开始记录平滑信息
    file.open("../dataScripts/data/map3_2_wayPointSmoothTest.csv", ios::out);
    if (file.is_open()) {
        file << std::setprecision(10);
        file << "x,y,z" << endl;
        for (const auto &point: path) {
            file << point << endl;
        }
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    cout << "log finished" << endl;
    return 0;
}
/**
 * 基于map3的第三组测试, 飞机尺寸为0.4m, 栅格地图分辨率为0.1m
 */
int wayPointsPathSmoothTestMap3_3() {
    float DRONE_SIZE = 0.2, GRID_SIZE = 0.1;

    glm::vec3 startPoint(0.3, 0.3, 0.3);
    glm::vec3 destination(17, 16, 7);

    model::threeDmodel demoModel = genDemoModelMap20208();
    // test wayPointsGenerator
    cout << "wayPointsGenerator Path Smooth Test starts" << endl;
    wayPointsGenerator wpg(demoModel, GRID_SIZE, DRONE_SIZE);
    wpg.genPoints(startPoint, destination);
    cout << "way points finish generating" << endl;
    // 记录地图信息
    fstream file;
    file.open("../dataScripts/data/map3_3_mapStatus.csv", ios::out);
    if (file.is_open()) {
        file << wpg.printModelToString();
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    // 记录waypoint信息
    vector<glm::vec3> wayPointsResult = wpg.genPoints(startPoint, destination);
    string fileName = "../dataScripts/data/map3_3_wayPointTest.csv";
    printPointsFromVector(fileName, wayPointsResult);

    // 开始平滑
    vector<HGJ::vec3f> wayPoints = wpg.getGeneratedPointsHGJ();
    HGJ::pathPlanner planner;
    cout << "smoothing starts" << endl;
    auto path = planner.genCurv(wayPoints, 0.01, 0.1, 0.0, 0.0);
    cout << "smoothing finishes, log to file." << endl;
    // 开始记录平滑信息
    file.open("../dataScripts/data/map3_3_wayPointSmoothTest.csv", ios::out);
    if (file.is_open()) {
        file << std::setprecision(10);
        file << "x,y,z" << endl;
        for (const auto &point: path) {
            file << point << endl;
        }
        file.close();
    } else {
        cout << "unable to open file." << endl;
    }
    cout << "log finished" << endl;
    return 0;
}

int main() {
    cout << "Test map 0" << endl;
    wayPointsPathSmoothTestMap0();
    cout << "=========================================================================" << endl;
    cout << "Test map 1_1" << endl;
    wayPointsPathSmoothTestMap1_1();
    cout << "=========================================================================" << endl;
    cout << "Test map 1_2" << endl;
    wayPointsPathSmoothTestMap1_2();
    cout << "=========================================================================" << endl;
    cout << "Test map 1_3" << endl;
    wayPointsPathSmoothTestMap1_3();
    cout << "=========================================================================" << endl;
    cout << "Test map 2_1" << endl;
    wayPointsPathSmoothTestMap2_1();
    cout << "=========================================================================" << endl;
    cout << "Test map 2_2" << endl;
    wayPointsPathSmoothTestMap2_2();
    cout << "=========================================================================" << endl;
    cout << "Test map 2_3" << endl;
    wayPointsPathSmoothTestMap2_3();
    cout << "=========================================================================" << endl;
    cout << "Test map 3_1" << endl;
    wayPointsPathSmoothTestMap3_1();
    cout << "=========================================================================" << endl;
    cout << "Test map 3_2" << endl;
    wayPointsPathSmoothTestMap3_2();
    cout << "=========================================================================" << endl;
    cout << "Test map 3_3" << endl;
    wayPointsPathSmoothTestMap3_3();
    cout << "=========================================================================" << endl;
    return 0;
}
