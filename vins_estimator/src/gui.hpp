//
// Created by xin on 7/9/21.
//

#ifndef SRC_GUI_HPP
#define SRC_GUI_HPP

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include <thread>


class Gui
{
public:
    Gui();
    ~Gui() = default;

    void run();

    inline void draw_cam(const pangolin::OpenGlMatrix& Twc);
    void draw_cams( const std::vector< pangolin::OpenGlMatrix > &_Twcs );
    void draw_points( const std::vector< Eigen::Vector3d > & _points );

    void update_data( cv::Mat &_img, std::vector< Eigen::Matrix4d > &_Twcs, std::vector< Eigen::Vector3d > &_Points );


    std::mutex gui_mutex;

private:
//        std::vector< Eigen::Matrix4d > Twcs;
//    std::vector< std::vector<pangolin::OpenGlMatrix> > Twcs_dataset;
//    std::vector< std::vector< Eigen::Vector3d > > points_dataset;
//    std::vector< cv::Mat > img_dataset;

    std::unordered_map< int64_t, std::vector<pangolin::OpenGlMatrix> > Twcs_dataset;
    std::unordered_map< int64_t, std::vector< Eigen::Vector3d > > points_dataset;
    std::unordered_map< int64_t, cv::Mat > img_dataset;

    int64_t show_id;

    std::shared_ptr<std::thread> view_thread;
};

#endif //SRC_GUI_HPP
