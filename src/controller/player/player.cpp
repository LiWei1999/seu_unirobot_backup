#include "player.hpp"
#include "configuration.hpp"
#include "task/action_task.hpp"
#include "task/look_task.hpp"
#include "task/walk_task.hpp"
#include "task/gcret_task.hpp"
#include "task/say_task.hpp"
#include "server/server.hpp"
#include "core/adapter.hpp"
#include "engine/IKWalk/WalkEngine.hpp"
#include "engine/scan/ScanEngine.hpp"
#include "engine/action/ActionEngine.hpp"
#include "engine/led/LedEngine.hpp"
#include "fsm/fsm_state_getup.hpp"
#include "fsm/fsm_state_goto_ball.hpp"
#include "fsm/fsm_state_kick_ball.hpp"
#include "fsm/fsm_state_ready.hpp"
#include "fsm/fsm_state_search_ball.hpp"
#include "fsm/fsm_state_sl.hpp"

using namespace std;
using namespace motion;
using namespace Eigen;

player::player(): timer(CONF->get_config_value<int>("think_period"))
{
    is_alive_ = false;
    period_count_ = 0;
    btn_count_ = 0;
    role_ = CONF->get_my_role();
    attack_range_ = CONF->get_config_vector<float>("strategy."+role_+".attack_range");
    last_search_dir_ = 0.0;
    in_search_ball_ = false;
    see_last_ = false;
    played_ = false;
    self_location_count_ = 0;
    keeper_kicked_ = false;

    fsm_ = make_shared<FSM>();
}

void player::run()
{
    if (is_alive_)
    {
        period_count_++;

        if (OPTS->use_robot())
        {
            if(WM->button_status(1)&&WM->button_status(2))
            {
                btn_count_++;
                if(btn_count_%20==0)
                    raise(SIGINT);
            }
            else
            {
                btn_count_=0;
            }
        }
        if(OPTS->use_remote())
        {
            play_with_remote();
        }
        else
        {
            list<task_ptr> tasks, tlist;
            if((period_count_*period_ms_/100)%5 == 0)
            {
                if(OPTS->use_gc())
                    tasks.push_back(make_shared<gcret_task>());
                if(OPTS->use_comm())
                    tasks.push_back(make_shared<say_task>());
            }
            if((period_count_*period_ms_/100)%20 == 0)
            {
                WM->reset_hear_info();
            }

            tlist = think();
            tasks.insert(tasks.end(), tlist.begin(), tlist.end());    
            for(auto &tsk:tasks)
            {
                if(tsk.get())
                    tsk->perform();
            }
        }
    }
}

list<task_ptr> player::think()
{
    list<task_ptr> tasks, tlists;
    if(OPTS->image_record())
    {
        //play_with_remote();
        tasks.push_back(make_shared<look_task>(HEAD_STATE_SEARCH_BALL));
        tasks.push_back(make_shared<walk_task>(0.0, 0.0, 0.0, true));
    }
    else 
    {
        if(OPTS->kick_mode()==options::KICK_NORMAL)
        {
            if(WM->self().global.x()>1.5 && self_location_count_*period_ms_%90000==0 && self_location_count_ !=0 
                && !in_search_ball_ && !AE->in_action_)
            {
                LOG(LOG_INFO)<<"localization start"<<endll;
                self_location_count_ ++;
                tlists = play_skill_localization();
            }
            else
            {
                if(!WM->in_localization_)
                {
                    if(OPTS->use_gc())
                    {
                        tlists = play_with_gc();
                    }
                    else
                    {
                        tlists = play_without_gc();
                        //tasks.push_back(make_shared<look_task>(0.0, 40.0, HEAD_STATE_LOOKAT));
                        //tasks.push_back(make_shared<walk_task>(0.02, 0.0, 0.0, true));
                        /*
                        if(WM->fall_data()!=FALL_NONE)
                        {
                            LOG(LOG_INFO)<<WM->fall_data()<<endll;
                            if(WM->fall_data()==FALL_FORWARD)
                                tasks.push_back(make_shared<action_task>("front_getup"));
                            else if(WM->fall_data()==FALL_BACKWARD)
                                tasks.push_back(make_shared<action_task>("back_getup"));
                        }
                        */
                    }
                }
            }
        }
        else if(OPTS->kick_mode()==options::KICK_PENALTY)
        {
            static bool start_penalty=false;
            static bool left=true; 
            static float init_dir = 0.0;
            tasks.push_back(make_shared<look_task>(0.0, 60.0));
            if(WM->button_status(1))
            {
                start_penalty = true;
                left = true;
                init_dir = WM->self().dir;
            }
            else if(WM->button_status(2))
            {
                start_penalty = true;
                left = false;
                init_dir = WM->self().dir;
            }
            if(start_penalty)
                tasks.push_back(play_skill_penalty_kick(left, init_dir));
        }
    }

    tasks.insert(tasks.end(), tlists.begin(), tlists.end());
    return tasks;
}

bool player::in_my_attack_range(const Eigen::Vector2d &ball)
{
    return ball.x()>=attack_range_[0]&&ball.x()<=attack_range_[1];
}

bool player::init()
{
    fsm_->Register(FSM_STATE_READY, make_shared<FSMStateReady>(fsm_));
    fsm_->Register(FSM_STATE_GETUP, make_shared<FSMStateGetup>(fsm_));
    fsm_->Register(FSM_STATE_SEARCH_BALL, make_shared<FSMStateSearchBall>(fsm_));
    fsm_->Register(FSM_STATE_GOTO_BALL, make_shared<FSMStateGotoBall>(fsm_));
    fsm_->Register(FSM_STATE_KICK_BALL, make_shared<FSMStateKickBall>(fsm_));
    fsm_->Register(FSM_STATE_SL, make_shared<FSMStateSL>(fsm_));
    fsm_->Trans(FSM_STATE_READY);

    if(OPTS->use_debug())
        SERVER->start();

    if (!regist())
    {
        return false;
    }
    is_alive_ = true;

    if (OPTS->use_robot())
    {
        while (!dynamic_pointer_cast<motor>(sensors_["motor"])->is_connected() && is_alive_)
        {
            LOG(LOG_WARN) << "waiting for motor connection, please turn on the power." << endll;
            sleep(1);
        }
        
        if (!is_alive_)
        {
            return true;
        }
    }

    MADT->start();
    WE->start();
    SE->start();
    AE->start();
    if(OPTS->use_robot())
    {
        LE->start();
    }
    action_task p("ready");
    p.perform();
    start_timer();
    return true;
}

void player::stop()
{
    is_alive_ = false;
    WE->stop();
    SE->stop();
    AE->stop();
    if(OPTS->use_robot())
    {
        LE->stop();
    }
    MADT->stop();

    if (is_alive_)
    {
        delete_timer();
    }

    sleep(1);
    unregist();
    sleep(1);
    if(OPTS->use_debug())
        SERVER->stop();
}

bool player::regist()
{
    sensors_.clear();

    if (OPTS->use_camera())
    {
        sensors_["camera"] = std::make_shared<camera>();
        sensors_["camera"]->attach(VISION);
        sensors_["camera"]->start();

        if (!VISION->start())
        {
            return false;
        }
    }

    sensors_["motor"] = std::make_shared<motor>();
    sensors_["motor"]->attach(WE);

    if (!sensors_["motor"]->start())
    {
        return false;
    }

    if (OPTS->use_robot())
    {
        sensors_["imu"] = std::make_shared<imu>();
        sensors_["imu"]->attach(WM);
        sensors_["imu"]->attach(VISION);
        sensors_["imu"]->attach(WE);
        sensors_["imu"]->start();
        
        sensors_["button"] = std::make_shared<button>();
        sensors_["button"]->attach(WM);
        sensors_["button"]->start();
    }

    if (OPTS->use_gc())
    {
        sensors_["gamectrl"] = std::make_shared<gamectrl>();
        sensors_["gamectrl"]->attach(WM);
        sensors_["gamectrl"]->start();
    }

    if (OPTS->use_comm())
    {
        sensors_["hear"] = std::make_shared<hear>();
        sensors_["hear"]->attach(WM);
        sensors_["hear"]->start();
    }

    return true;
}

void player::unregist()
{
    if (sensors_.find("button") != sensors_.end())
    {
        sensors_["button"]->detach(WM);
        sensors_["button"]->stop();
    }
    if (sensors_.find("imu") != sensors_.end())
    {
        sensors_["imu"]->detach(WM);
        sensors_["imu"]->detach(VISION);
        sensors_["imu"]->detach(WE);
        sensors_["imu"]->stop();
    }
    if (sensors_.find("motor") != sensors_.end())
    {
        sensors_["motor"]->detach(WE);
        sensors_["motor"]->stop();
    }
    if (sensors_.find("camera") != sensors_.end())
    {
        sensors_["camera"]->detach(VISION);
        sensors_["camera"]->stop();
        VISION->stop();
    }
    if (sensors_.find("gamectrl") != sensors_.end())
    {
        sensors_["gamectrl"]->detach(WM);
        sensors_["gamectrl"]->stop();
    }
    if (sensors_.find("hear") != sensors_.end())
    {
        sensors_["hear"]->detach(WM);
        sensors_["hear"]->stop();
    }
}

sensor_ptr player::get_sensor(const std::string &name)
{
    auto iter = sensors_.find(name);

    if (iter != sensors_.end())
    {
        return iter->second;
    }

    return nullptr;
}