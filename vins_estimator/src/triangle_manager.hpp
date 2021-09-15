//
// Created by xin on 13/9/21.
//

#ifndef SRC_TRIANGLE_MANAGER_HPP
#define SRC_TRIANGLE_MANAGER_HPP


#include <algorithm>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <map>

class TriPerFrame
{
public:
    TriPerFrame(double _frame_timestamp,
                const Eigen::Vector2d &_pixel_a, const Eigen::Vector2d &_pixel_b, const Eigen::Vector2d &_pixel_c,
                const Eigen::Vector3d &_pt3d_a, const Eigen::Vector3d &_pt3d_b, const Eigen::Vector3d &_pt3d_c,
                int _feature_a, int _feature_b, int _feature_c)
    :frame_timestamp(_frame_timestamp),
    feature_a(_feature_a), feature_b(_feature_b), feature_c(_feature_c)
    {
        pixel_a = _pixel_a;
        pixel_b = _pixel_b;
        pixel_c = _pixel_c;

        pt3d_a = _pt3d_a;
        pt3d_b = _pt3d_b;
        pt3d_c = _pt3d_c;
    }

    double frame_timestamp;
    Eigen::Vector2d pixel_a, pixel_b, pixel_c;
    Eigen::Vector3d pt3d_a, pt3d_b, pt3d_c;
    const int feature_a, feature_b, feature_c;
};

class TriID
{
public:
    const int feature_a, feature_b, feature_c; // a < b < c

    bool operator < (const TriID &other)const
    {
        if(other.feature_a == feature_a)
        {
            if(other.feature_b == feature_b)
            {
                if(other.feature_c == feature_c) return false;
                else return feature_c < other.feature_c;
            }
            else return feature_b < other.feature_b;
        }
        else return feature_a < other.feature_a;
    }

    bool operator == (const TriID& other) const
    {
        return feature_a == other.feature_a && feature_b == other.feature_b
               && feature_c == other.feature_c;
    }

    std::map< double, TriPerFrame > tri_per_frame;

    TriID(int _feature_a, int _feature_b, int _feature_c)
            :feature_a(_feature_a),
             feature_b(_feature_b), feature_c(_feature_c)
    {

    }
};

class TriPerID
{
public:
    const int feature_a, feature_b, feature_c; // a < b < c
    TriID id;

    bool operator < (const TriPerID &other)const
    {
        if(other.feature_a == feature_a)
        {
            if(other.feature_b == feature_b)
            {
                if(other.feature_c == feature_c) return false;
                else return feature_c < other.feature_c;
            }
            else return feature_b < other.feature_b;
        }
        else return feature_a < other.feature_a;
    }

    bool operator == (const TriPerID& other) const
    {
        return feature_a == other.feature_a && feature_b == other.feature_b
        && feature_c == other.feature_c;
    }

    std::map< double, TriPerFrame > tri_per_frame;

    TriPerID(int _feature_a, int _feature_b, int _feature_c)
    :feature_a(_feature_a),
    feature_b(_feature_b), feature_c(_feature_c),
    id{ feature_a, feature_b, feature_c }
    {

    }

};

class TriManager
{
public:
    TriManager(){}

//    std::unordered_map< int, std::unordered_map<int, std::unordered_map<int, TriPerID>>> tris;
//    std::set< TriPerID > tris;

    std::map< TriID, TriPerID > tris;

#define min_ratio_between_largest_an_smallest_side 0.25
#define min_elongation_ratio 0.25
#define max_triangle_side 2.0

    double getRatioBetweenSmallestAndLargestSide(
            const double& d12,
            const double& d23,
            const double& d31) const {
        // Measure sides.
        double minSide = std::min(d12, std::min(d23, d31));
        double maxSide = std::max(d12, std::max(d23, d31));

        // Compute and return ratio.
        return minSide / maxSide;
    }

    double getRatioBetweenTangentialAndRadialDisplacement(
            const std::vector<Eigen::Vector3d>& points) {
        // Compute radial directions.
        double min_z = std::numeric_limits<double>::max();
        double max_z = 0;
        for (size_t i = 0; i < points.size(); i++) {
            double z_i = points.at(i).z();
            if (z_i < min_z) {
                min_z = z_i;
            }
            if (z_i > max_z) {
                max_z = z_i;
            }
        }

        std::vector<Eigen::Vector3d> points_rescaled;
        for (size_t i = 0; i < points.size(); i++) {
            // Point rescaled is a projection of point on a virtual img plane at
            // distance minZ.
            points_rescaled.push_back((points.at(i) / points.at(i).z()) * min_z);
        }

        double max_t = 0.0;
        double tangential_elongation_1 =
                (points_rescaled.at(0) - points_rescaled.at(1)).norm();
        max_t = std::max(max_t, tangential_elongation_1);
        double tangential_elongation_2 =
                (points_rescaled.at(1) - points_rescaled.at(2)).norm();
        max_t = std::max(max_t, tangential_elongation_2);
        double tangential_elongation_3 =
                (points_rescaled.at(0) - points_rescaled.at(2)).norm();
        max_t = std::max(max_t, tangential_elongation_3);

        // Points must be in front of the camera, and min should be less than max.
        if (min_z < 0 || max_z < 0 || min_z > max_z) {
//            VLOG(100) << "min_r = " << min_z << " max_r = " << max_z << "\n";
//            LOG(ERROR) << "Negative radial components! Points are behind camera.";
            return 0;
        }
        return max_t / (max_z - min_z);
    }

    bool isBad(
            const TriPerFrame& triPerFrame,
            const Eigen::Matrix4d &Twc
            )
    {
        double ab = ( triPerFrame.pt3d_a - triPerFrame.pt3d_b ).norm();
        double ac = ( triPerFrame.pt3d_a - triPerFrame.pt3d_c ).norm();
        double bc = ( triPerFrame.pt3d_b - triPerFrame.pt3d_c ).norm();

        double ratioSides_i = getRatioBetweenSmallestAndLargestSide(ab, ac, bc);

        Eigen::Matrix4d Tcw = Twc.inverse();
        Eigen::Vector3d p1 = Tcw.block(0,0,3,3) * triPerFrame.pt3d_a + Tcw.block(0,3,3,1);
        Eigen::Vector3d p2 = Tcw.block(0,0,3,3) * triPerFrame.pt3d_b + Tcw.block(0,3,3,1);
        Eigen::Vector3d p3 = Tcw.block(0,0,3,3) * triPerFrame.pt3d_c + Tcw.block(0,3,3,1);
        std::vector< Eigen::Vector3d > pts3d;
        pts3d.push_back(p1);
        pts3d.push_back(p2);
        pts3d.push_back(p3);

        double ratioTangentialRadial_i = getRatioBetweenTangentialAndRadialDisplacement(
                pts3d
                );


        std::array<double, 3> sidesLen;
        sidesLen.at(0) = ab;
        sidesLen.at(1) = ac;
        sidesLen.at(2) = bc;
        const auto& it = std::max_element(sidesLen.begin(), sidesLen.end());
        double maxTriangleSide_i = *it;


        if ((ratioSides_i >= min_ratio_between_largest_an_smallest_side) &&
            (ratioTangentialRadial_i >= min_elongation_ratio) &&
            (maxTriangleSide_i <= max_triangle_side)) {
            return false;
        } else {
            return true;
        }
    }

    void insert_tri( double cur_frame_time, const TriPerFrame& triPerFrame,
                     const Eigen::Matrix4d &Twc)
    {
        if(isBad( triPerFrame, Twc )) return;

        auto it = tris.find( TriID( triPerFrame.feature_a, triPerFrame.feature_b, triPerFrame.feature_c ) );
        if(it != tris.end())
        {
            it->second.tri_per_frame.insert( {cur_frame_time, triPerFrame} );
        }
        else
        {
            TriPerID triPerId( triPerFrame.feature_a, triPerFrame.feature_b, triPerFrame.feature_c );
            triPerId.tri_per_frame.insert( {cur_frame_time, triPerFrame} );

            TriID triId( triPerFrame.feature_a, triPerFrame.feature_b, triPerFrame.feature_c );

            tris.insert({triId, triPerId});
        }
    }
};


#endif //SRC_TRIANGLE_MANAGER_HPP
