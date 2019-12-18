#include <iostream>

#include "model/threeDModel.h"
#include "aStar/aStar.hpp"
#include "pathSimplifier/regressionSimplifier.hpp"
#include <glm/glm.hpp>
#include <fstream>
//#include "octree2ThreeDModel/tree2grid.h"
#include <wayPointsGenerator/wayPointsGenerator.hpp>
#include <HGJPathSmooth/pathSmoother.h>
//#include <pcl/io/ply_io.h>
//#include "pcl/octree/octree_search.h"
//#include <pcl/impl/point_types.hpp>

using namespace std;


int octreeTest() {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
//    cloud->width = 100;
//    cloud->height = 100;
//    cloud->points.resize(cloud->width * cloud->height);
//    int index = 0;
//
//    for (int i = 40; i < 50; ++i) {
//        for (int j = 0; j < 20; ++j) {
//            for (int k = 0; k < 100; ++k) {
//                cloud->points[index].x = (float)i;
//                cloud->points[index].y = (float)j;
//                cloud->points[index].z = (float)k;
//                ++index;
//            }
//        }
//    }
//
//    for (int i = 0; i < 50; ++i) {
//        for (int j = 40; j < 50; ++j) {
//            for (int k = 0; k < 100; ++k) {
//                cloud->points[index].x = (float)i;
//                cloud->points[index].y = (float)j;
//                cloud->points[index].z = (float)k;
//                ++index;
//            }
//        }
//    }
//
//    for (int i = 30; i < 40; ++i) {
//        for (int j = 70; j < 100; ++j) {
//            for (int k = 0; k < 100; ++k) {
//                cloud->points[index].x = (float)i;
//                cloud->points[index].y = (float)j;
//                cloud->points[index].z = (float)k;
//                ++index;
//            }
//        }
//    }
//
//    for (int i = 60; i < 100; ++i) {
//        for (int j = 60; j < 70; ++j) {
//            for (int k = 0; k < 100; ++k) {
//                cloud->points[index].x = (float)i;
//                cloud->points[index].y = (float)j;
//                cloud->points[index].z = (float)k;
//                ++index;
//            }
//        }
//    }
//    float resolution=1.0f;
//    pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(resolution); //初始化octree
//    octree.setInputCloud(cloud);                                               //设置输入点云
//    octree.addPointsFromInputCloud();                                         //构建octree
//    tree2grid gridBuilder = tree2grid(octree);
//    gridBuilder.setBoundary(100,100,100);
//    model::threeDmodel modelFromCloud = gridBuilder.buildGrid();
//
//    modelFromCloud.printInfo();

    return 0;
}

model::threeDmodel genDemoModel() {
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

void printVec3fFromVector(const string &path, vector<glm::vec3> &data) {
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


int aStarTest() {
    float DRONE_SIZE = 1;

    glm::vec3 startPoint(1, 1, 1);
    glm::vec3 destination(10, 80, 98);

    model::threeDmodel demoModel = genDemoModel();
    // test astar
    aStar astar(demoModel);
    astar.setDroneSize(DRONE_SIZE);
    if (astar.aStarPathPlan(startPoint, destination)) {
        cout << "astar plan finished" << endl;
        cout << "writing search and map data" << endl;
        fstream file;
        file.open("../dataScripts/data/status.csv", ios::out);
        if (file.is_open()) {
            file << astar.printModelToString();
            file.close();
        } else {
            cout << "unable to open file." << endl;
        }
        cout << "search and map data logging finished" << endl;

        // print out path
        cout << "writing path data" << endl;
        vector<glm::vec3> path = astar.getPath();
        string fileName = "../dataScripts/data/cpp_astar_path.csv";
        printPointsFromVector(fileName, path);
        cout << "path data finished writing" << endl;

        // test regression simplifier
        cout << "start path simplifying" << endl;
        regressionSimplifier rs = regressionSimplifier();
        rs.setDroneSize(DRONE_SIZE);
        vector<glm::vec3> regSimplified = rs.simplify(path);
        cout << "regression simplifying finished, start logging to file" << endl;
        fileName = "../dataScripts/data/cpp_reg_simplified.csv";
        printPointsFromVector(fileName, regSimplified);
        cout << "regression logging finished." << endl;
    } else {
        cout << "path finding failed" << endl;
        return -1;
    }
    return 0;
}

int wayPointsTest() {
    float DRONE_SIZE = 1;

    glm::vec3 startPoint(1, 1, 1);
    glm::vec3 destination(10, 80, 98);

    model::threeDmodel demoModel = genDemoModel();
    // test wayPointsGenerator
    cout << "wayPointsGenerator Test starts" << endl;
    wayPointsGenerator wpg(demoModel, 1.0, DRONE_SIZE);
    vector<glm::vec3> wayPointsResult = wpg.genPoints(startPoint, destination);
    string fileName = "../dataScripts/data/cpp_wayPointGenerator.csv";
    printPointsFromVector(fileName, wayPointsResult);
    cout << "wayPointsGenerator Test finished, log to file" << endl;
    return 0;
}

int wayPointsPathSmoothTest() {
    float DRONE_SIZE = 1;

    glm::vec3 startPoint(1, 1, 1);
    glm::vec3 destination(10, 80, 98);

    model::threeDmodel demoModel = genDemoModel();
    // test wayPointsGenerator
    cout << "wayPointsGenerator Path Smooth Test starts" << endl;
    wayPointsGenerator wpg(demoModel, 1.0, DRONE_SIZE);
    wpg.genPoints(startPoint, destination);
    cout << "way points finish generating" << endl;
    vector<HGJ::vec3f> wayPoints = wpg.getGeneratedPointsHGJ();
    HGJ::pathPlanner planner;
    cout << "smoothing starts" << endl;
    auto path = planner.genCurv(wayPoints, 0.01, 10, 0.0, 0.0 );
    cout << "smoothing finishes, log to file." << endl;
    fstream file;
    file.open("../dataScripts/data/cpp_smoothCurv.csv", ios::out);
    if (file.is_open()) {
        file << "x,y,z" << endl;
        for (const auto & point: path) {
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
    return wayPointsPathSmoothTest();
}
