#pragma once
#include <eigen3/Eigen/Dense>
#include <iostream>
#include "../factor/imu_factor.h"
#include "../utility/utility.h"
#include <ros/ros.h>
#include <map>
#include "../feature_manager.h"

using namespace Eigen;
using namespace std;

class ImageFrame
{
    public:
        ImageFrame(){};
        ImageFrame(const map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>>& _points, double _t):t{_t},is_key_frame{false}
        {
            points = _points;
            for(auto point:_points)
            {
                pids_.push_back(point.first);
                Vector2d nom(point.second[0].second(0), point.second[0].second(1));
                Vector2d pixel(point.second[0].second(3), point.second[0].second(4));

//                double fx = 461.6;
//                double fy = 460.3;
//                double cx = 376;
//                double cy = 240;
//
//                pixel(0) = nom(0)*fx+cx;
//                pixel(1) = nom(1)*fy+cy;
                points_on_nom.emplace_back(nom);
                points_on_pixel.emplace_back(pixel);
            }
        };
        map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>> > > points;
        double t;
        Matrix3d R;
        Vector3d T;
        IntegrationBase *pre_integration;
        bool is_key_frame;

        std::vector<int> pids_;
        std::vector<Eigen::Vector2d> points_on_pixel;
        std::vector<Eigen::Vector2d> points_on_nom;
};

bool VisualIMUAlignment(map<double, ImageFrame> &all_image_frame, Vector3d* Bgs, Vector3d &g, VectorXd &x);