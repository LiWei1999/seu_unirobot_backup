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
        return fsm_->Trans(FSM_STATE_GETUP);
    if (WM->localization_time_)
        return fsm_->Trans(FSM_STATE_SL);

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

    if (first_in_)
    {
        first_in_ = false;
        tasks.push_back(std::make_shared<LookTask>(motion::HEAD_STATE_SEARCH_BALL));
        tasks.push_back(std::make_shared<WalkTask>(0.0, 0.0, 0.0, false));
    }
    else
    {
        if (motion::SE->search_ball_end_)
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
    double self2left_dir = azimuth_deg(WM->opp_post_left - self.global);
    double self2right_dir = azimuth_deg(WM->opp_post_right - self.global);
    if (ball_dis <= enter_kick_dis_ && self.dir >= self2right_dir && self.dir <= self2left_dir)
        return fsm_->Trans(FSM_STATE_KICK_BALL);
    else if (ball_dis > enter_kick_dis_)
    {
        if (fabs(ball_dir) > 10.0)
            tasks.push_back(make_shared<WalkTask>(0.0, 0.0, ball_dir, true));
        else
            tasks.push_back(make_shared<WalkTask>(0.035, 0.0, 0.0, true));
    }
    else if (self.dir > self2left_dir)
    {
        tasks.push_back(make_shared<WalkTask>(-0.01, 0.01, -8.0, true));
    }
    else if (self.dir < self2right_dir)
    {
        tasks.push_back(make_shared<WalkTask>(-0.01, -0.01, 8.0, true));
    }

    if (ball.self.x() < retreat_x_dis_ && fabs(ball.self.y()) > retreat_y_dis_)
    {
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
    if (!ball.can_see) // 若看不到球则找球
        return fsm_->Trans(FSM_STATE_SEARCH_BALL);

    float ball_dir = azimuth_deg(ball.self);
    float ball_dis = ball.self.norm();
    if (ball_dis > exit_kick_dis_) // 若离球太远则向球走
        return fsm_->Trans(FSM_STATE_GOTO_BALL);

    if (ball.beta > retreat_beta_ && fabs(ball.alpha) > retreat_alpha_) // 若离球太近
        tasks.push_back(make_shared<WalkTask>(-0.015, 0.0, 0.0, true));
    else
    {
        if (ball.alpha > -0.1)
            tasks.push_back(std::make_shared<WalkTask>(-0.01, -0.01, 0.0, true));
        else if (ball.alpha < -0.2)
            tasks.push_back(std::make_shared<WalkTask>(-0.01, 0.01, 0.0, true));
        else
        {
            if (ball.beta < 0.3)
                tasks.push_back(std::make_shared<WalkTask>(0.01, 0.0, 0.0, true));
            else
                tasks.push_back(std::make_shared<ActionTask>("left_little_kick"));
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

task_list FSMStateSL::OnStateTick()
{
    task_list tasks;
    if (WM->fall_data() != FALL_NONE)
        return fsm_->Trans(FSM_STATE_GETUP);
    if (motion::SE->search_post_end_)
        return fsm_->Trans(FSM_STATE_SEARCH_BALL);

    tasks.push_back(std::make_shared<LookTask>(motion::HEAD_STATE_SEARCH_POST));
    tasks.push_back(std::make_shared<WalkTask>(0.0, 0.0, 0.0, false));
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