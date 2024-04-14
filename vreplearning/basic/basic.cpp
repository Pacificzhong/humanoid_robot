//以下电机位置的值均为角度制，且在电机自身坐标系下。
//变量左右命名方式均为从机器人背后看向机器人时的左右。
//从电机背面看向输出轴，顺时针转角度为正，逆时针转角度为负，直立状态时角度为0。
//本规划适用于荆楚一号机在膝关节定死的状态下为应对检查来设计的小步态
//髋关节前后摆为hip_pitch，左腿为hip_pitch_l，右腿为hip_pitch_r，设定为控制两大腿内侧电机，在本实例中，左右髋关节是同步的。
//髋关节侧摆为hip_roll，左腿为hip_roll_l，右腿为hip_roll_r,在本实例中，左右的髋关节永远是同步运动的。
//踝关节前后摆为ankle_pitch，左腿左侧电机为ankle_pitch_ll，左腿右侧电机为ankle_pitch_lr；右腿左侧电机为ankle_pitch_rl，右腿右侧电机为ankle_pitch_rr。
//在本实例中，ankle_pitch_ll=ankle_pitch_rr=ankle_pitch_outside;
//ankle_pitch_lr=ankle_pitch_rl=ankle_pitch_inside。

#include <iostream>
#include <cmath>
#define hip_pitch_target 100
#define hip_roll_target 100
#define ankle_pitch_target 100

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
    int full_montion_time=8000;//在髋关节侧摆从速度0到下一个速度0的过程中，所有的关节都在动的时间长度，以数组长度来表示。注，宏定义不能用来表示数组长度
    int hip_roll_only_montion_time=2000;//只有髋关节侧摆在动，其他关节都不动的时间，以数组长度来表示。
    //定义三个关节在单个运动过程中前后摆位置数组
    //髋关节侧摆（左右同步）
    double hip_roll_begin[full_montion_time+hip_roll_only_montion_time];
    double hip_roll_t11_t12[full_montion_time+hip_roll_only_montion_time];
    double hip_roll_t21_t22[full_montion_time+hip_roll_only_montion_time];
    double hip_roll_t31_t32[full_montion_time+hip_roll_only_montion_time];
    double hip_roll_t41_t42[full_montion_time+hip_roll_only_montion_time];
    double hip_roll[4*(full_montion_time+hip_roll_only_montion_time)];
    double hip_roll_end[full_montion_time+hip_roll_only_montion_time];

    //髋关节前后摆(左右同步)
    //左腿
    double hip_pitch_t11[full_montion_time];
    double hip_pitch_t22[full_montion_time];
    double hip_pitch_t31[full_montion_time];
    double hip_pitch_t42[full_montion_time];
    double hip_pitch[4*(full_montion_time+hip_roll_only_montion_time)];

    //踝关节前后摆
    //小腿外侧电机
    double ankle_pitch_outside_t11[full_montion_time];
    double ankle_pitch_outside_t22[full_montion_time];
    double ankle_pitch_outside_t31[full_montion_time];
    double ankle_pitch_outside_t42[full_montion_time];
    double ankle_pitch_outside[4*(full_montion_time+hip_roll_only_montion_time)];
    //小腿内侧电机
    double ankle_pitch_inside_t11[full_montion_time];
    double ankle_pitch_inside_t22[full_montion_time];
    double ankle_pitch_inside_t31[full_montion_time];
    double ankle_pitch_inside_t42[full_montion_time];
    double ankle_pitch_inside[4*(full_montion_time+hip_roll_only_montion_time)];

    //计算髋关节侧摆hip_roll的T11-T12段hip_roll_t11_t12
    double startValue=hip_roll_target;
    double endValue = 0.0;// 终点值
    double t=0;//插值参数，范围在[0，1]之间
    double delta=1.0/(full_montion_time+hip_roll_only_montion_time);
 for (int i=0;i<(full_montion_time+hip_roll_only_montion_time);i++)
    {
    hip_roll_t11_t12[i] = triangle_Interpolation(startValue, endValue, t);
    t=t+delta;
    }

    //计算髋关节侧摆hip_roll的T21-T22段hip_roll_t21_t22
 for (int i=0;i<(full_montion_time+hip_roll_only_montion_time);i++)
    {
    hip_roll_t21_t22[i] = hip_roll_t11_t12[i]-hip_roll_target;
    }

    //计算髋关节侧摆hip_roll的T31-T32段hip_roll_t31_t32
 for (int i=0;i<(full_montion_time+hip_roll_only_montion_time);i++)
    {
    hip_roll_t31_t32[i] = -1.0*hip_roll_t11_t12[i];
    }

    //计算髋关节侧摆hip_roll的T41-T42段hip_roll_t41_t42
for (int i=0;i<(full_montion_time+hip_roll_only_montion_time);i++)
    {
    hip_roll_t41_t42[i] = hip_roll_t31_t32[i]+hip_roll_target;
    }

    //计算髋关节前后摆hip_pitch的T11段hip_pitch_t11
    startValue=0.0;
    endValue = hip_pitch_target;// 终点值
    t=0;//插值参数，范围在[0，1]之间
    delta=1.0/(full_montion_time);
 for (int i=0;i<(full_montion_time);i++)
    {
    hip_pitch_t11[i] = triangle_Interpolation(startValue, endValue, t);
    t=t+delta;
    }

    //计算髋关节前后摆hip_pitch的T22段hip_pitch_t22
    startValue=hip_pitch_target;
    endValue = 0.0;// 终点值
    t=0;//插值参数，范围在[0，1]之间
    delta=1.0/(full_montion_time);
 for (int i=0;i<(full_montion_time);i++)
    {
    hip_pitch_t22[i] = triangle_Interpolation(startValue, endValue, t);
    t=t+delta;
    }

    //计算髋关节前后摆hip_pitch的T31段hip_pitch_t31
 for (int i=0;i<(full_montion_time);i++)
    {
    hip_pitch_t31[i] = hip_pitch_t22[i]-hip_pitch_target;
    t=t+delta;
    }

    //计算髋关节前后摆hip_pitch的T42段hip_pitch_t42
 for (int i=0;i<(full_montion_time);i++)
    {
    hip_pitch_t42[i] = hip_pitch_t11[i]-hip_pitch_target;
    t=t+delta;
    }

//计算踝关节前后摆外侧电机ankle_pitch_outside的T11段ankle_pitch_outside_t11_t12
    startValue=0.0;
    endValue = ankle_pitch_target;// 终点值
    t=0;//插值参数，范围在[0，1]之间
    delta=1.0/(full_montion_time);
 for (int i=0;i<(full_montion_time);i++)
    {
    ankle_pitch_outside_t11[i] = triangle_Interpolation(startValue, endValue, t);
    t=t+delta;
    }

//计算踝关节前后摆外侧电机ankle_pitch_outside的T22段ankle_pitch_outside_t22
    startValue=ankle_pitch_target;
    endValue = 0.0;// 终点值
    t=0;//插值参数，范围在[0，1]之间
    delta=1.0/(full_montion_time);
 for (int i=0;i<(full_montion_time);i++)
    {
    ankle_pitch_outside_t22[i] = triangle_Interpolation(startValue, endValue, t);
    t=t+delta;
    }

//计算踝关节前后摆外侧电机ankle_pitch_outside的T31段ankle_pitch_outside_t31

 for (int i=0;i<(full_montion_time);i++)
    {
    ankle_pitch_outside_t31[i] = ankle_pitch_outside_t22[i]-ankle_pitch_target;
    t=t+delta;
    }

//计算踝关节前后摆外侧电机ankle_pitch_outside的T42段ankle_pitch_outside_t42

 for (int i=0;i<(full_montion_time);i++)
    {
    ankle_pitch_outside_t42[i] = ankle_pitch_outside_t11[i]-ankle_pitch_target;
    t=t+delta;
    }

//计算踝关节前后摆内侧电机ankle_pitch_inside的T11段ankle_pitch_inside_t11
 for (int i=0;i<(full_montion_time);i++)
    {
    ankle_pitch_inside_t11[i] = -1.0*ankle_pitch_outside_t11[i];
    t=t+delta;
    }


//计算踝关节前后摆内侧电机ankle_pitch_inside的T22段ankle_pitch_inside_t22
 for (int i=0;i<(full_montion_time);i++)
    {
    ankle_pitch_inside_t22[i] = -1.0*ankle_pitch_outside_t22[i];
    t=t+delta;
    }


//计算踝关节前后摆内侧电机ankle_pitch_inside的T31段ankle_pitch_inside_t31
 for (int i=0;i<(full_montion_time);i++)
    {
    ankle_pitch_inside_t31[i] = -1.0*ankle_pitch_outside_t31[i];
    t=t+delta;
    }

//计算踝关节前后摆内侧电机ankle_pitch_inside的T42段ankle_pitch_inside_t42
 for (int i=0;i<(full_montion_time);i++)
    {
    ankle_pitch_inside_t42[i] = -1.0*ankle_pitch_outside_t42[i];
    t=t+delta;
    }

//计算髋关节侧摆hip_roll的起始段hip_roll_begin
    startValue = 0.0; // 起始值
    endValue = hip_roll_target;// 终点绝对值
    t=0;//插值参数，范围在[0，1]之间
    delta=1.0/(full_montion_time+hip_roll_only_montion_time);
 for (int i=0;i<(full_montion_time+hip_roll_only_montion_time);i++)
    {
    hip_roll_begin[i] = triangle_Interpolation(startValue, endValue, t);
    //std::cout << "Interpolated value: "<< i<<"=" << hip_roll_start_l[i]<<std::endl;
    //std::cout << "t="<<t<<std::endl;
    //std::cout << "delta="<<delta<<std::endl;
    t=t+delta;
    }

//构建髋关节侧摆在一个循环中的位置数组
for(int i=0;i<(full_montion_time+hip_roll_only_montion_time);i++)
{
    hip_roll[i]=hip_roll_t11_t12[i];
}
for(int i=0;i<(full_montion_time+hip_roll_only_montion_time);i++)
{
    hip_roll[i+(full_montion_time+hip_roll_only_montion_time)]=hip_roll_t21_t22[i];
}
for(int i=0;i<(full_montion_time+hip_roll_only_montion_time);i++)
{
    hip_roll[i+2*(full_montion_time+hip_roll_only_montion_time)]=hip_roll_t31_t32[i];
}
for(int i=0;i<(full_montion_time+hip_roll_only_montion_time);i++)
{
    hip_roll[i+3*(full_montion_time+hip_roll_only_montion_time)]=hip_roll_t41_t42[i];
}

//构建髋关节前后摆在一个循环中的位置数组
for(int i=0;i<(full_montion_time);i++)
{
    hip_pitch[i]=hip_pitch_t11[i];
}
for(int i=0;i<(2*hip_roll_only_montion_time);i++)
{
    hip_pitch[i+full_montion_time]=hip_pitch_t11[full_montion_time-1];
}
for(int i=0;i<(full_montion_time);i++)
{
    hip_pitch[i+full_montion_time+2*hip_roll_only_montion_time]=hip_pitch_t22[i];
}
for(int i=0;i<(full_montion_time);i++)
{
    hip_pitch[i+2*full_montion_time+2*hip_roll_only_montion_time]=hip_pitch_t31[i];
}
for(int i=0;i<(2*hip_roll_only_montion_time);i++)
{
    hip_pitch[i+3*full_montion_time+2*hip_roll_only_montion_time]=hip_pitch_t31[full_montion_time-1];
}
for(int i=0;i<(full_montion_time);i++)
{
    hip_pitch[i+3*full_montion_time+4*hip_roll_only_montion_time]=hip_pitch_t42[i];
}

//构建踝关节外侧电机前后摆在一个循环中的位置数组ankle_pitch_outside
for(int i=0;i<(full_montion_time);i++)
{
    ankle_pitch_outside[i]=ankle_pitch_outside_t11[i];
}
for(int i=0;i<(2*hip_roll_only_montion_time);i++)
{
    ankle_pitch_outside[i+full_montion_time]=ankle_pitch_outside_t11[full_montion_time-1];
}
for(int i=0;i<(full_montion_time);i++)
{
    ankle_pitch_outside[i+full_montion_time+2*hip_roll_only_montion_time]=ankle_pitch_outside_t22[i];
}
for(int i=0;i<(full_montion_time);i++)
{
    ankle_pitch_outside[i+2*full_montion_time+2*hip_roll_only_montion_time]=ankle_pitch_outside_t31[i];
}
for(int i=0;i<(2*hip_roll_only_montion_time);i++)
{
    ankle_pitch_outside[i+3*full_montion_time+2*hip_roll_only_montion_time]=ankle_pitch_outside_t31[full_montion_time-1];
}
for(int i=0;i<(full_montion_time);i++)
{
    ankle_pitch_outside[i+3*full_montion_time+4*hip_roll_only_montion_time]=ankle_pitch_outside_t42[i];
}

//构建踝关节内侧电机前后摆在一个循环中的位置数组ankle_pitch_inside
for(int i=0;i<(full_montion_time);i++)
{
    ankle_pitch_inside[i]=ankle_pitch_inside_t11[i];
}
for(int i=0;i<(2*hip_roll_only_montion_time);i++)
{
    ankle_pitch_inside[i+full_montion_time]=ankle_pitch_inside_t11[full_montion_time-1];
}
for(int i=0;i<(full_montion_time);i++)
{
    ankle_pitch_inside[i+full_montion_time+2*hip_roll_only_montion_time]=ankle_pitch_inside_t22[i];
}
for(int i=0;i<(full_montion_time);i++)
{
    ankle_pitch_inside[i+2*full_montion_time+2*hip_roll_only_montion_time]=ankle_pitch_inside_t31[i];
}
for(int i=0;i<(2*hip_roll_only_montion_time);i++)
{
    ankle_pitch_inside[i+3*full_montion_time+2*hip_roll_only_montion_time]=ankle_pitch_inside_t31[full_montion_time-1];
}
for(int i=0;i<(full_montion_time);i++)
{
    ankle_pitch_inside[i+3*full_montion_time+4*hip_roll_only_montion_time]=ankle_pitch_inside_t42[i];
}

//计算髋关节侧摆hip_roll的结尾段hip_roll_end
    startValue = hip_roll_target; // 起始值
    endValue = 0.0;// 终点绝对值
    t=0;//插值参数，范围在[0，1]之间
    delta=1.0/(full_montion_time+hip_roll_only_montion_time);
 for (int i=0;i<(full_montion_time+hip_roll_only_montion_time);i++)
    {
    hip_roll_end[i] = triangle_Interpolation(startValue, endValue, t);
    t=t+delta;
    }
    return 0;
}


/*这里头的代码传参有问题
#include <iostream>
#include <cmath>
#define hip_pitch_target 100
#define hip_roll_target 100
#define ankle_pitch_target 100
#define full_montion_time 800//在髋关节侧摆从速度0到下一个速度0的过程中，所有的关节都在动的时间长度，以数组长度来表示。
#define hip_roll_only_montion_time 200//只有髋关节侧摆在动，其他关节都不动的时间，以数组长度来表示。
// 三角函数插值
double triangle_Interpolation(double start, double end, double t) 
{
    // 将t限制在[0, 1]的范围内
    t = std::fmin(1.0, std::fmax(0.0, t));//返回参数中的最小值和最大值

    // 对t进行三角函数插值
    double interpolated_Value = start + (end - start) * (0.5 - 0.5 * std::cos(t * M_PI));//M_PI经宏定义为3.14

    return interpolated_Value;
}

double* motion_planning(int n,double startValue,double endValue)//n为数组长度，髋关节侧摆与前后摆的插值长度不同
{
    double t=0;//插值参数，范围在[0，1]之间
    double temp[n];
    double delta=1.0/n;
 for (int i=0;i<n;i++)
    {
    temp[i] = triangle_Interpolation(startValue, endValue, t);
    std::cout << "Interpolated value: "<< i<<"=" << temp[i]<<std::endl;
    std::cout << "t="<<t<<std::endl;
    std::cout << "delta="<<delta<<std::endl;
    t=t+delta;
    }

    return temp;
}

int main() 
{
    double hip_Pitch[full_montion_time];//定义三个关节前后摆位置数组

    //double *hip_roll_start[full_montion_time+hip_roll_only_montion_time];
    double* hip_roll_start;
    double hip_roll_start_op[full_montion_time+hip_roll_only_montion_time];
    double hip_roll_l[4*(full_montion_time+hip_roll_only_montion_time)];
    

    double ankle_pitch[full_montion_time];
    double startValue = 0.0; // 起始值
    //计算hip_roll_l的起始段
    double endValue = hip_pitch_target;// 终点值
    hip_roll_start=motion_planning(full_montion_time+hip_roll_only_montion_time,startValue,endValue);
    for(int i=0;i<full_montion_time+hip_roll_only_montion_time;i++)
    {
        hip_roll_start_op[i]=*(hip_roll_start+i);
        std::cout << "赋值后的数组值为"<<hip_roll_start_op[i]<<std::endl;
    }


    //计算hip_roll_l从t11到t12的值
    double t = 0; // 插值参数，范围在[0, 1]之间
    int i=0;//关节位置数组元素索引

   delete[] hip_roll_start;//释放动态分配的内存
    return 0;
}
*/


