1:安装px4, 确保在根目录上有文件夹~/PX4-Autopilot
2.编译：在工作空间内运行 catkin build fly_curve
3.运行：在终端内输入 roslaunch fly_curve gt_sim.launch. 待gazebo仿真环境完全启动后，在另一终端内运行roslaunch fly_curve program_flyonce.launch. 如果是第一次运行,  先在终端运行以下指令给脚本添加权限：roscd fly_curve/scripts && chmod +x *

路径点和速度约束等信息保存在 scripts/outer_control.py 主函数内。其他函数由于还需调整改进，暂不提供源代码，只保留编译后文件以供初步测试。
