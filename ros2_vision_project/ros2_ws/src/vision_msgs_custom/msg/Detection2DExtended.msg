# 扩展的2D检测消息，包含深度信息

std_msgs/Header header

# 2D边界框
vision_msgs/BoundingBox2D bbox

# 检测结果
vision_msgs/ObjectHypothesisWithPose[] results

# 深度信息（米）
float32 distance

# 置信度
float32 confidence

# 类别ID和名称
int32 class_id
string class_name

# 3D位置（相机坐标系）
geometry_msgs/Point position_3d