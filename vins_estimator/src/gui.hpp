//
// Created by xin on 7/9/21.
//

#ifndef SRC_GUI_HPP
#define SRC_GUI_HPP

#include <pangolin/pangolin.h>
#include <opencv2/opencv.hpp>
#include <thread>
#include "triangle_manager.hpp"

class Gui
{
public:
    Gui();
    ~Gui() = default;

    void run();

    bool pause(){ return pause_status; }

    inline void draw_cam(const pangolin::OpenGlMatrix& Twc);
    void draw_cams( const std::vector< pangolin::OpenGlMatrix > &_Twcs );
    void draw_points( const std::unordered_map< int, Eigen::Vector3d > & id_points );
    void draw_mesh(const std::vector< std::vector<Eigen::Vector3d> >& tris_3d, const std::vector< std::pair< double, std::vector< Eigen::Vector2d >>> &tris_pixels,
                   const std::unordered_map< double, cv::Mat > &imgs);
    void draw_single_mesh(const std::vector< std::vector<Eigen::Vector3d> >& tris_3d,
                          const std::vector<  std::vector< Eigen::Vector2d >> &tris_pixels,
                   const cv::Mat &imgs);

    void update_data(
            cv::Mat &_img, std::vector< Eigen::Matrix4d > &_Twcs,
            std::unordered_map< int, Eigen::Vector3d > &id_point_datasets,
//            std::vector< Eigen::Vector3d > &_Points,
            TriManager &gui_tri,
            std::vector< double > &_times, double time
            );


    std::mutex gui_mutex;

private:
//        std::vector< Eigen::Matrix4d > Twcs;
//    std::vector< std::vector<pangolin::OpenGlMatrix> > Twcs_dataset;
//    std::vector< std::vector< Eigen::Vector3d > > points_dataset;
//    std::vector< cv::Mat > img_dataset;

//    std::unordered_map< int64_t,
//    std::tuple< pangolin::OpenGlMatrix, std::vector< Eigen::Vector3d >, cv::Mat, double > > gui_dataset;
    std::unordered_map< int64_t, std::vector<pangolin::OpenGlMatrix> > Twcs_dataset;
    std::unordered_map< int64_t, std::vector< double > > Timess_dataset;
    std::unordered_map< int64_t, std::unordered_map< int, Eigen::Vector3d > > points_dataset;
    std::unordered_map< int64_t, TriManager > tris_dataset;
    std::unordered_map< int64_t, cv::Mat > img_dataset;
    std::unordered_map< int64_t, double > time_dataset;

    std::unordered_map< double, int64_t > time2datasett;

    int64_t show_id;

    std::shared_ptr<std::thread> view_thread;


    bool pause_status;
};

#endif //SRC_GUI_HPP
