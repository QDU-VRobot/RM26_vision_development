# 哨兵机器人当前工作姿态状态

# 姿态模式常量
uint8 POSE_ATTACK = 1       # 进攻姿态
uint8 POSE_DEFENCE = 2    # 防御姿态
uint8 POSE_PATROL = 3      # 移动/巡逻姿态

uint8 current_state        # 当前工作姿态 (1=进攻, 2=防御, 3=移动)
