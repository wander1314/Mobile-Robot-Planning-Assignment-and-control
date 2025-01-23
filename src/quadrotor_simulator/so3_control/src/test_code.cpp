#include <iostream>
#include <vector>
#include <limits>
#include <Eigen/Dense>
#include "SO3Control.cpp"

// 假设有一个控制器对象
SO3Control controller;

// 假设你有一个仿真函数或测试函数来评估控制器的性能
double runSimulationOrTest(const Eigen::Vector3d& des_pos, const Eigen::Vector3d& des_vel, 
                            const Eigen::Vector3d& des_acc, double des_yaw, double des_yaw_dot) {
    // 这里用一个简单的模拟或者测试，实际中应该根据控制器行为来评估
    // 例如，计算位置误差或其他评估指标
    Eigen::Vector3d current_pos = controller.getPosition();
    double error = (des_pos - current_pos).norm();
    return error;  // 这里我们假设误差越小越好
}

void gridSearch() {
    // 设定搜索参数范围和步长
    std::vector<double> kx_values = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10};  // kx的候选值
    std::vector<double> kv_values = {0.1, 0.5, 1, 2, 3, 4, 5};        // kv的候选值

    // 最优参数初始化
    double best_kx = 0, best_kv = 0;
    double best_performance = std::numeric_limits<double>::max();  // 最小误差初始化为正无穷

    // 假设期望的目标位置、速度、加速度等
    Eigen::Vector3d des_pos(10.0, -4.0, 0.0);  // 期望位置
    Eigen::Vector3d des_vel(0.0, 0.0, 0.0);    // 期望速度
    Eigen::Vector3d des_acc(0.0, 0.0, 0.0);    // 期望加速度
    double des_yaw = 0.0;   // 期望偏航角
    double des_yaw_dot = 0.0;  // 期望偏航角速率

    // 遍历所有参数组合
    for (double kx : kx_values) {
        for (double kv : kv_values) {
            // 设置控制器参数
            controller.setKx(Eigen::Vector3d(kx, kx, kx));  // 假设kx在XYZ方向相同
            controller.setKv(Eigen::Vector3d(kv, kv, kv));  // 假设kv在XYZ方向相同

            // 运行仿真或实际测试
            double performance = runSimulationOrTest(des_pos, des_vel, des_acc, des_yaw, des_yaw_dot);

            // 如果当前性能优于最优性能，更新最优参数
            if (performance < best_performance) {
                best_performance = performance;
                best_kx = kx;
                best_kv = kv;
            }
        }
    }

    // 输出最优参数
    std::cout << "Best kx: " << best_kx << ", Best kv: " << best_kv << std::endl;
}

int main() {
    // 执行网格搜索
    gridSearch();
    return 0;
}
