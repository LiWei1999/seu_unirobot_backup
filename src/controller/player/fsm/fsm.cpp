#include "fsm_state_ready.hpp"
#include "fsm_state_getup.hpp"
#include "fsm_state_search_ball.hpp"
#include "fsm_state_goto_ball.hpp"
#include "fsm_state_kick_ball.hpp"
#include "fsm_state_dribble.hpp"
#include "fsm_state_sl.hpp"
#include "core/worldmodel.hpp"

using namespace std;
using namespace Eigen;
using namespace motion;
using namespace robot_math;

task_list FSMStateSearchBall::OnStateTick()
{
    task_list tasks;
    if (WM->fall_data() != FALL_NONE) // 若已摔倒
        return fsm_->Trans(FSM_STATE_GETUP);  //状态转移
    if (WM->localization_time_)
        return fsm_->Trans(FSM_STATE_SL);       //定位时间到，进入寻找球门定位状态

    ball_block ball = WM->ball();                                                       
    if (ball.can_see)
    {
        float dir = azimuth_deg(ball.self);
        //LOG(LOG_INFO)<<dir<<endll;
        if (fabs(dir) < can_goto_dir_)
            return fsm_->Trans(FSM_STATE_GOTO_BALL);
        else
        {
            tasks.push_back(make_shared<WalkTask>(0.0, 0.0, dir, true));
            return tasks;
        }
    }

    if (first_in_)                                                                          //首次进入状态，执行站立找球动作
    {
        first_in_ = false;
        tasks.push_back(std::make_shared<LookTask>(motion::HEAD_STATE_SEARCH_BALL));
        tasks.push_back(std::make_shared<WalkTask>(0.0, 0.0, 0.0, false));
    }
    else
    {
        if (motion::SE->search_ball_end_)                                                   //转圈找球
        {
            tasks.push_back(std::make_shared<LookTask>(motion::HEAD_STATE_SEARCH_BALL));
            tasks.push_back(std::make_shared<WalkTask>(0.0, 0.0, 10.0, true));
        }
    }
    return tasks;
}

task_list FSMStateGotoBall::OnStateTick()
{
    task_list tasks;
    if (WM->fall_data() != FALL_NONE)
        return fsm_->Trans(FSM_STATE_GETUP);
    ball_block ball = WM->ball();
    self_block self = WM->self();

    if (!ball.can_see)
        return fsm_->Trans(FSM_STATE_SEARCH_BALL);
    if (WM->localization_time_)
        return fsm_->Trans(FSM_STATE_SL);

    float ball_dir = azimuth_deg(ball.self);
    float ball_dis = ball.self.norm();
    double self2left_dir = azimuth_deg(WM->opp_post_left - self.global);            //在worldmodel.cpp修改
    double self2right_dir = azimuth_deg(WM->opp_post_right - self.global);
    //double self2mid_dir = azimuth_deg(WM->opp_post_mid - self.global);        //朝向中心方向
    
    //LOG(LOG_INFO) << "opp_post_left:" << WM->opp_post_left<< endl;                      
    if (ball_dis <= enter_kick_dis_ && self.dir >= self2right_dir && self.dir <= self2left_dir){
        //LOG(LOG_INFO) << "FSMStateGotoBall: if1";
        return fsm_->Trans(FSM_STATE_KICK_BALL);
    }
    else if (ball_dis > enter_kick_dis_)                                                                      //未到踢球距离时，先调整转向走向球
    {
        //LOG(LOG_INFO) << "FSMStateGotoBall: else if-2";
        //if (fabs(ball_dir) > 10.0)
          //  tasks.push_back(make_shared<WalkTask>(0.0, 0.0, ball_dir, true));                               //转向修改
        if (ball_dir > 10)
            tasks.push_back(make_shared<WalkTask>(0.0, 0.0, 8, true));
        else if (ball_dir < -10)
            tasks.push_back(make_shared<WalkTask>(0.0, 0.0, -8, true));
        else
            tasks.push_back(make_shared<WalkTask>(0.035, 0.0, 0.0, true));
    }
    else if (self.dir > self2left_dir)                                                                          //调整自身方向使其面向球门
    {
        //LOG(LOG_INFO) << "FSMStateGotoBall: else if-3";
        tasks.push_back(make_shared<WalkTask>(-0.008, 0.015, -8.0, true)); //-0.01, 0.01, -8.0  before  
    }
    else if (self.dir < self2right_dir)
    {
        //LOG(LOG_INFO) << "FSMStateGotoBall: else if-4";
        tasks.push_back(make_shared<WalkTask>(-0.008, -0.015, 8.0, true)); //-0.01, -0.01, 8.0 before
    }

    if (ball.self.x() < retreat_x_dis_ && fabs(ball.self.y()) > retreat_y_dis_)                                 //离球太近后退
    {
        //LOG(LOG_INFO) << "FSMStateGotoBall: ball.self.x() < retreat_x_dis_ ";
        tasks.clear();
        tasks.push_back(make_shared<WalkTask>(-0.02, 0.0, 0.0, true));                              
    }
    return tasks;
}

task_list FSMStateKickBall::OnStateTick()
{
    task_list tasks;
    if (WM->fall_data() != FALL_NONE)
        return fsm_->Trans(FSM_STATE_GETUP);
    ball_block ball = WM->ball();
    self_block self = WM->self();
    if (!ball.can_see) // 若看不到球则找球
        return fsm_->Trans(FSM_STATE_SEARCH_BALL);

    float ball_dir = azimuth_deg(ball.self);
    float ball_dis = ball.self.norm();

    double self2left_dir = azimuth_deg(WM->opp_post_left - self.global) + 5;                    //new
    double self2right_dir = azimuth_deg(WM->opp_post_right - self.global) - 10;
    double self2mid_dir = azimuth_deg(WM->opp_post_mid);                  //朝向中心
    if (ball_dis > exit_kick_dis_){ // 若离球太远则向球走
        return fsm_->Trans(FSM_STATE_GOTO_BALL);
        //LOG(LOG_INFO) << "FSMStateKickBall: 离球太远";
    }

    if (ball.beta > retreat_beta_ && fabs(ball.alpha) > retreat_alpha_){ // 若离球太近 
        tasks.push_back(make_shared<WalkTask>(-0.015, 0.0, 0.0, true));
        LOG(LOG_INFO) << "FSMStateKickBall: 离球太近" << endl;
        //LOG(LOG_INFO) << "FSMStateKickBall: 离球太近--if";
    }
    else                                                                                    
    {
        //LOG(LOG_INFO) << "FSMStateKickBall: 离球太近--else";
    if (self.dir > self2left_dir)
    {
        tasks.push_back(make_shared<WalkTask>(-0.008, 0.008, -10.0, true)); 
        LOG(LOG_INFO) << "FSMStateKickBall: left转向" << endl;
    }
    else if (self.dir < self2right_dir)
    {
        tasks.push_back(make_shared<WalkTask>(-0.008, -0.008, 10.0, true));
        LOG(LOG_INFO) << "FSMStateKickBall: right转向" << endl;
    }                                 
                                                                                                                                                                                                  
    else if (ball.alpha > -0.05 && self.dir < self2left_dir && self.dir > self2right_dir)   
    {                                                      
        tasks.push_back(std::make_shared<WalkTask>(0.0, -0.012, 0, true)); 
        LOG(LOG_INFO) << "FSMStateKickBall: right横移" << endl;
    }      
    else if (ball.alpha < -0.15 && self.dir < self2left_dir && self.dir > self2right_dir)  
    {                                                  
        tasks.push_back(std::make_shared<WalkTask>(0.0, 0.012, 0, true));        
        LOG(LOG_INFO) << "FSMStateKickBall: left横移" << endl;
    }
        else
        {
            if (ball.beta < 0.33)                                   //ball.beta < 0.35 BEFORE
            {                                                      
                tasks.push_back(std::make_shared<WalkTask>(0.012, 0.0, 0, true));   
                LOG(LOG_INFO) << "FSMStateKickBall: 前进" << endl; 
            } 
            else if (ball.beta > 0.45)
            {
                tasks.push_back(std::make_shared<WalkTask>(-0.01, 0.0, 0.0, true));
                LOG(LOG_INFO) << "FSMStateKickBall: 后退" << endl;
            }
            else
            {
                tasks.push_back(std::make_shared<ActionTask>("left_little_kick"));
                LOG(LOG_INFO) << "FSMStateKickBall: 踢球" << endl;
            }
        }
    }
    return tasks;
}

task_list FSMStateDribble::OnStateTick()
{
    task_list tasks;
    if (WM->fall_data() != FALL_NONE)
        return fsm_->Trans(FSM_STATE_GETUP);
    ball_block ball = WM->ball();
    if (!ball.can_see)
        return fsm_->Trans(FSM_STATE_SEARCH_BALL);
    if (WM->localization_time_)
        return fsm_->Trans(FSM_STATE_SL);

    float ball_dir = azimuth_deg(ball.self);
    float ball_dis = ball.self.norm();
    if (ball_dis > exit_kick_dis_)
        return fsm_->Trans(FSM_STATE_GOTO_BALL);

    if (ball.beta > retreat_beta_ && fabs(ball.alpha) > retreat_alpha_)
        tasks.push_back(make_shared<WalkTask>(-0.02, 0.0, 0.0, true));
    else
    {
        if (ball.alpha > 0.1)
            tasks.push_back(std::make_shared<WalkTask>(0.0, -0.01, 0.0, true));
        else if (ball.alpha < -0.1)
            tasks.push_back(std::make_shared<WalkTask>(0.0, 0.01, 0.0, true));
        else
        {
            float target_dir = azimuth_deg(target_pos_ - WM->self().global);
            float deg = normalize_deg(WM->self().dir - target_dir);
            if (fabs(deg) > 15.0)
                tasks.push_back(std::make_shared<WalkTask>(0.0, 0.0, -deg, true));
            else
                tasks.push_back(std::make_shared<WalkTask>(0.035, 0.0, 0.0, true));
        }
    }
    return tasks;
}

task_list FSMStateSL::OnStateTick()                                                             //寻找球门定位，定位时间在player.cpp
{
    task_list tasks;
    if (WM->fall_data() != FALL_NONE)
        return fsm_->Trans(FSM_STATE_GETUP);
    if (motion::SE->search_post_end_)
        return fsm_->Trans(FSM_STATE_SEARCH_BALL);

    tasks.push_back(std::make_shared<WalkTask>(0.0, 0.0, 0.0, false));
    tasks.push_back(std::make_shared<LookTask>(motion::HEAD_STATE_SEARCH_POST));                //执行寻找球门动作，该动作在scan_engine.cpp
    
    return tasks;
}

task_list FSMStateGetup::OnStateTick()
{
    task_list tasks;
    if (WM->fall_data() != FALL_NONE)
    {
        if (WM->fall_data() == FALL_FORWARD)
            tasks.push_back(std::make_shared<ActionTask>("front_getup"));
        else if (WM->fall_data() == FALL_BACKWARD)
            tasks.push_back(std::make_shared<ActionTask>("back_getup"));
        else if (WM->fall_data() == FALL_LEFT)
            tasks.push_back(std::make_shared<ActionTask>("right_arm"));
        else
            tasks.push_back(std::make_shared<ActionTask>("left_arm"));
    }
    else
    {
        tasks = fsm_->Trans(FSM_STATE_READY);
    }
    return tasks;
}

task_list FSMStateReady::OnStateTick()
{
    task_list tasks, tlist;
    if (WM->fall_data() != FALL_NONE)
        return fsm_->Trans(FSM_STATE_GETUP);
    tasks.push_back(std::make_shared<WalkTask>(0.0, 0.0, 0.0, false));
    tasks.push_back(std::make_shared<LookTask>(0.0, 40.0));
    tlist = fsm_->Trans(FSM_STATE_SEARCH_BALL);
    tasks.insert(tasks.end(), tlist.begin(), tlist.end());
    return tasks;
}
