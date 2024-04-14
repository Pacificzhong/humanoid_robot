//以下电机位置的值均为角度制，且在电机自身坐标系下
//本规划适用于荆楚一号机在膝关节定死的状态下为应对检查来设计的小步态
//髋关节前后摆为hip_pitch
//髋关节侧摆为hip_roll
//踝关节前后摆为knee_pitch
#include <iostream>
#include <cmath>

// 三角函数插值
double triangle_Interpolation(double start, double end, double t) 
{
    // 将t限制在[0, 1]的范围内
    t = std::fmin(1.0, std::fmax(0.0, t));//返回参数中的最小值和最大值

    // 对t进行三角函数插值
    double interpolated_Value = start + (end - start) * (0.5 - 0.5 * std::cos(t * M_PI));//M_PI经宏定义为3.14

    return interpolated_Value;
}

int main() 
{
    double startValue = 0.0; // 起始值
    double endValue = 100.0; // 终点值
    double t = 0; // 插值参数，范围在[0, 1]之间
    int i=0;//关节位置数组元素索引
    double hip_Pitch[100];//定义三个关节前后摆位置数组
    double hip_roll[100];
    double knee_pitch[100];
    for (i=0;i<=100;i++)
    {
    double hip_Pitch[i] = triangle_Interpolation(startValue, endValue, t);
    std::cout << "Interpolated value: " << hip_Pitch[i] << std::endl;
    }
    



    return 0;
}

