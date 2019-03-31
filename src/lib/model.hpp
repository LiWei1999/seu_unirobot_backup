#pragma once

#include <string>
#include <map>
#include <eigen3/Eigen/Dense>

struct GoalPost
{
    enum TYPE
    {
        SENSORMODEL_POST_UNKNOWN,
        SENSORMODEL_POST_L, //left post
        SENSORMODEL_POST_R, //right post
    };

    enum SIDE
    {
        SENSORMODEL_SIDE_OUR,
        SENSORMODEL_SIDE_OPP,
        SENSORMODEL_SIDE_UNKNOW,
    };

    TYPE _type = SENSORMODEL_POST_UNKNOWN;
    SIDE _side = SENSORMODEL_SIDE_UNKNOW;
    float _theta = 0.0f;
    float _distance = 1000.0f;
};

struct filed_info
{
    int field_length;
    int field_width;
    int goal_depth;
    int goal_width;
    int goal_height;
    int goal_area_length;
    int goal_area_width;
    int penalty_mark_distance;
    int center_circle_diameter;
    int border_strip_width_min;
};

struct player_info
{
    int id;
    float x, y, dir;
    float ball_x, ball_y;
    bool available = false;
    player_info(float m_x, float m_y, float m_d)
        :x(m_x), y(m_y), dir(m_d){}

    player_info(){}
};

struct ball_block
{
    Eigen::Vector2d global;
    Eigen::Vector2d self;
    Eigen::Vector2i pixel;
    float alpha, beta;
    bool sure=false;
};

struct self_block
{
    Eigen::Vector2d global;
    double dir;
    bool sure=false;
};

struct camera_info
{
    std::string name;
    int id;
    float value;
    float default_value;
    float min_value;
    float max_value;
};

struct camera_param
{
    float fx, fy;
    float cx, cy;
    float k1, k2;
    float p1, p2;
    float h_v, v_v;
};

struct object_det
{
    int id;
    float prob;
    int x, y, w, h;
    object_det(int i=0, float p=1, int x_=0, int y_=0, int w_=0, int h_=0) 
        : id(i), prob(p), x(x_), y(y_), w(w_), h(h_){}
    bool operator< (const object_det &obj)
    {
        return prob<obj.prob;
    }
};

enum { player_info_size = sizeof(player_info)};
