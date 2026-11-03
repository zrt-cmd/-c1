# 总线舵机全量上报状态（下位机 0x08 主动推送 / 查询触发解析后发布）
# 每个字段均为等长数组，下标 = 舵机 id（0~N-1）
std_msgs/Header header
int32[] id               # 舵机 id 列表（0..N-1）
bool[]  valid            # 每颗是否读取有效（0x00=有效）
int32[] position         # 逻辑位置 PWM（已减设备 offset，与 /joint_states 一致），µs
int32[] temperature      # 温度 °C
float32[] voltage        # 电压 V（0.1V 上报换算）