# 一次已提交观测；点云按该时刻去畸变到 odometry.child_frame_id 坐标系。
uint64 instance_id    # 哪个前端进程发的
uint64 epoch          # 这是第几代数据，用来区分重置前后
uint64 sequence       # 这一代里的第几条消息
builtin_interfaces/Time begin  # 这段点云从什么时候开始采集
nav_msgs/Odometry odometry
sensor_msgs/PointCloud2 cloud
uint32 matches       # 前端匹配到了多少个有效点
bool lidar_updated   # 这次有没有接受激光校正