# Difference between the current measurement and prediction
float64 position_diff
float64 yaw_diff
# Unfiltered position and yaw
geometry_msgs/Point position
float64 yaw
# 检查云台实际角度与目标角度的误差
float64 bc_yaw
float64 gimbal_yaw_error
# 检查前哨站的索引
int32 outpost_idx
