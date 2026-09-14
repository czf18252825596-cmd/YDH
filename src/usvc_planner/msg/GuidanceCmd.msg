# usvc_guidance → usvc_controller/ship_node
# 期望航向/速度（对齐 USVC RunCtrl 字段命名）

float32 obj_speed      # 期望速度 (m/s, 前进为正)
float32 obj_heading    # 期望航向 (deg, 0=北, 顺时针, 0~360)
bool    is_avoiding    # APF 是否在避障
bool    reached        # 是否到达目标点（到达则 ship_node 停船）