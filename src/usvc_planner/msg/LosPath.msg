# 沿边/路径跟踪注入消息（来自桥接节点，由上位机 task_file.json 转换而来）
# path 字段是经纬度点列（与 YDH map 系 xy 不同，但消息结构一致）

std_msgs/Header header
nav_msgs/Path path          # 路径点（Pose.position.x=lon, y=lat）
bool water_is_on_left       # 水在船左侧 true / 右侧 false