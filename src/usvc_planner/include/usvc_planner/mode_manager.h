#ifndef USVC_PLANNER_MODE_MANAGER_H
#define USVC_PLANNER_MODE_MANAGER_H

#include <cstdint>

namespace usvc_planner {

// 运行模式（对齐 usvc run_mode）
enum class RunMode   { ALONG_EDGE = 0, AUTO_NAV = 1 };

// 任务状态（对齐 usvc task_state，PAUSE 预留）
enum class TaskState { STOP = 0, BEGIN = 1, PAUSE = 2 };

// 状态机管理：只记录状态与切换事件，不执行动作
class ModeManager {
public:
    void setStartState(uint16_t start_run_mode, uint8_t start_task_state);
    void onCtrl(uint16_t run_mode, uint8_t task_state);

    bool consumeModeChanged();
    bool consumeStopEvent();

    RunMode   runMode()   const { return run_mode_; }
    TaskState taskState() const { return task_state_; }
    bool      running()   const { return task_state_ == TaskState::BEGIN; }

private:
    RunMode   run_mode_     = RunMode::ALONG_EDGE;
    TaskState task_state_   = TaskState::STOP;
    bool      mode_changed_ = false;
    bool      stop_event_   = false;
};

}  // namespace usvc_planner

#endif
