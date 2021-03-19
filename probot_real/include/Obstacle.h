//
//  Obstacle.h
//  障碍物，暂时只考虑球形
//  Created by 韩奔 on 2020/11/20.
//

#ifndef OBSTACLE_H
#define OBSTACLE_H

#include <eigen3/Eigen/Geometry>

struct Obstacle{    
    Eigen::Matrix<double, 3, 1> position;      // 障碍物中心的位置，维度为3×1
    double radius;         // 障碍物的半径
    
    // 构造函数
    Obstacle():
        position(Eigen::Matrix<double, 3, 1>::Zero()),
        radius(0) {};
    Obstacle(Eigen::Matrix<double, 3, 1>& p, double r):
            position(p),
            radius(r) {};
};

#endif /* OBSTACLE_H */
