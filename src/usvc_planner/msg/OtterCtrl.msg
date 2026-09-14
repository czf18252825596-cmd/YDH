# 控制指令（来自桥接节点，由 USVC RunCtrl 转换而来）

uint16 run_mode        # 0=沿边  1=自主导航（暂不用 1）
uint8  task_state      # 0=停止  1=开始
uint8  boundary_select # 0=自动匹配最近  1/2/3=指定（暂不用，留字段）