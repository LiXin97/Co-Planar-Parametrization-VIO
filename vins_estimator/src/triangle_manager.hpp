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
                int _feature_a, int _feature_b, int _feature_c)
    :frame_timestamp(_frame_timestamp),
    feature_a(_feature_a), feature_b(_feature_b), feature_c(_feature_c)
    {
        pixel_a = _pixel_a;
        pixel_b = _pixel_b;
        pixel_c = _pixel_c;
    }

    double frame_timestamp;
    Eigen::Vector2d pixel_a, pixel_b, pixel_c;
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

    void insert_tri( double cur_frame_time, TriPerFrame& triPerFrame )
    {
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
