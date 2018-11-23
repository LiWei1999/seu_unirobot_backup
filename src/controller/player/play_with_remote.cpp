#include <list>
#include "player.hpp"
#include "plan/action_plan.hpp"
#include "plan/lookat_plan.hpp"
#include "plan/walk_plan.hpp"
#include "tcp.hpp"

using namespace robot;
using namespace std;

list<plan_ptr> player::play_with_remote()
{
    list<plan_ptr> plist;
    remote_data rdata = WM->rmt_data();

    if (rdata.type == WALK_DATA)
    {
        float x, y, d, h;
        memcpy(&x, rdata.data.c_str(), float_size);
        memcpy(&y, rdata.data.c_str() + float_size, float_size);
        memcpy(&d, rdata.data.c_str() + 2 * float_size, float_size);
        memcpy(&h, rdata.data.c_str() + 3 * float_size, float_size);
        plist.push_back(make_shared<walk_plan>(x, y, d, h));
    }
    else if (rdata.type == ACT_DATA)
    {
        int blksz = int_size + 5 * sizeof(robot_pose);
        int pos_count = rdata.size / blksz;

        int act_t;
        robot_pose temp_pose;
        map<robot_motion, robot_pose> posesinfo;
        vector< map<robot_motion, robot_pose> > poses;
        vector<int> pos_times;

        for (int i = 0; i < pos_count; i++)
        {
            memcpy(&act_t, rdata.data.c_str() + i * blksz, int_size);
            posesinfo.clear();
            int j = 0;

            for (auto nmm : name_motion_map)
            {
                if (nmm.second != MOTION_HEAD && nmm.second != MOTION_NONE)
                {
                    memcpy(&temp_pose, rdata.data.c_str() + i * blksz + int_size + j * sizeof(robot_pose), sizeof(robot_pose));
                    posesinfo[nmm.second] = temp_pose;
                    j++;
                }
            }

            pos_times.push_back(act_t);
            poses.push_back(posesinfo);
        }

        plist.push_back(make_shared<action_plan>(poses, pos_times));
    }
    else if (rdata.type == LOOKAT_DATA)
    {
        float yaw, pitch;
        memcpy(&yaw, rdata.data.c_str(), float_size);
        memcpy(&pitch, rdata.data.c_str() + float_size, float_size);
        plist.push_back(make_shared<lookat_plan>(yaw, pitch));
    }
    else if (rdata.type == JOINT_OFFSET)
    {
        robot_joint_deg jdeg;

        for (size_t i = 0; i < ROBOT->get_joint_map().size(); i++)
        {
            memcpy(&jdeg, rdata.data.c_str() + i * sizeof(robot_joint_deg), sizeof(robot_joint_deg));
            ROBOT->get_joint(jdeg.id)->offset_ = jdeg.deg;
        }
    }
    else if (rdata.type == CAMERA_SET)
    {
        camera_para info;
        memcpy(&(info.id), rdata.data.c_str(), int_size);
        memcpy(&(info.value), rdata.data.c_str() + int_size, float_size);
        VISION->set_camera_para(info);
        shared_ptr<camera> cm = dynamic_pointer_cast<camera>(get_sensor("camera"));
        cm->set_camera_para(info);
    }
    else if (rdata.type == COLOR_SAMPLE)
    {
        int c, x, y, w, h;
        memcpy(&c, rdata.data.c_str(), int_size);
        memcpy(&(x), rdata.data.c_str() + int_size, int_size);
        memcpy(&(y), rdata.data.c_str() + 2 * int_size, int_size);
        memcpy(&(w), rdata.data.c_str() + 3 * int_size, int_size);
        memcpy(&(h), rdata.data.c_str() + 4 * int_size, int_size);
    }
    else if (rdata.type == IMAGE_SEND_TYPE)
    {
        int t;
        memcpy(&t, rdata.data.c_str(), int_size);
//        VISION->set_img_send_type(static_cast<image_send_type>(t));
    }

    WM->reset_rmt_data();
    return plist;
}

