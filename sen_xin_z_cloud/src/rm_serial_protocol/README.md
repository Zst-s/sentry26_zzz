## 线上布局（含0xA5和总长度字段，索引从0开始，小端）

|字段|帧内偏移|长度|
|---|---:|---:|
|独立帧头0xA5|0|1|
|整帧总字节数|1|1|
|Navigation_x|2|4|
|Navigation_y|6|4|
|Navigation_z|10|4|
|target_stance|14|1|
|nav_ready|15|1|
|super_cap_flag|16|1|
|if_guodong|17|1|
|if_stronger|18|1|
|sof1|19|1|
|data_length1|20|2|
|seq1|22|1|
|crc8_1|23|1|
|cmd_id|24|2|
|bt_status_data|26|1|
|bt_remain_time|27|2|
|bt_game_status|29|1|
|crc16_1|30|2|
|sof2|32|1|
|data_length2|33|2|
|seq2|35|1|
|crc8_2|36|1|
|cmd_id2|37|2|
|intention|39|1|
|start_position_x|40|2|
|start_position_y|42|2|
|delta_x[49]|44|49|
|delta_y[49]|93|49|
|sender_id|142|2|
|crc16_2|144|2|
|crc16_3|146|2|

共享C++头文件对所有字段offsetof和sizeof=146逐一static_assert，并显式编码小端数据。两个正式串口节点也使用同一编码器；正式导航节点继续发送Nav2速度，独立测试则明确固定零速度。

## 下位机仍需核对
用户明确要求第0字节0xA5，第1字节为数据总长度。本版将总长度定义为实际发送的整帧字节数：完整148，控制前缀19。下位机先检查USB_RX_BUF[0]==0xA5，再使用USB_RX_BUF[1]确定整帧长度，从USB_RX_BUF+2开始提取packed结构体。完整模式需累积148字节，缓冲区不能小于148。USB分段累积与实际接收回调仍未提供，尚未验证实车。独立测试所有帧Navigation_x/y/z固定0。
