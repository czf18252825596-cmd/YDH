#include "usvc_planner/mode_manager.h"

namespace usvc_planner {

void ModeManager::setStartState(uint16_t start_run_mode, uint8_t start_task_state) {
    run_mode_   = static_cast<RunMode>(start_run_mode);
    task_state_ = static_cast<TaskState>(start_task_state);
}

void ModeManager::onCtrl(uint16_t run_mode, uint8_t task_state) {
    RunMode   new_mode = static_cast<RunMode>(run_mode);
    TaskState new_task = static_cast<TaskState>(task_state);

    if (new_mode != run_mode_) {
        run_mode_     = new_mode;
        mode_changed_ = true;
    }

    task_state_ = new_task;

    if (new_task == TaskState::STOP) stop_event_ = true;
}

bool ModeManager::consumeModeChanged() {
    bool v = mode_changed_;
    mode_changed_ = false;
    return v;
}

bool ModeManager::consumeStopEvent() {
    bool v = stop_event_;
    stop_event_ = false;
    return v;
}

}  // namespace usvc_planner
