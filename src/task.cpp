

#include<iostream>
#include<Eigen/Core>
#include<Eigen/Geometry>
using namespace Eigen;

int main(int argc, char const *argv[])
{
    /**************************************************QUESTION***********************************************/
    // 下面的题目是b站中博主给的题目，我觉得写的很好，我把这个题目先做了一下
    //有两个右手系1和2,其中2系的x轴与1系的y轴方向相同，2系的y轴与1系z轴方向相反，2系的z轴与1系的x轴相反,两个坐标系原点重合
    //求R12，求1系中(1,1,1)在2系中的坐标。请自己编写一个c++程序实现它，并用Cmake编译，得到能输出答案的可执行文件
    //@ author: 全日制学生混(bilibili)
    //@ homePage: https://space.bilibili.com/336103007
    //@ github: https://github.com/cckaixin/Practical_Homework_for_slambook14
    /*********************************************************************************************************/
    // 将1系固定，然后固定轴旋转，先绕Z转90度，再绕Y转-90度，最后得到的矩阵是R12,也就是R_1_2
    // t_1_2是1坐标系的原点在2坐标系下的坐标值

    // 第一步：在1坐标系下围绕Z轴旋转90度  yaw是航向角-Z轴
    const double angle_z = M_PI_2; // 90度转换为弧度
    AngleAxisd rotation_z(angle_z, Vector3d::UnitZ());

    // 第二步：在1坐标系(也就是原始坐标系)下围绕Y轴旋转-90度  pitch是俯仰角-Y轴
    const double angle_y = -M_PI_2; // -90度转换为弧度
    AngleAxisd rotation_y(angle_y, Vector3d::UnitY());

    // x轴不需要旋转-roll翻滚角

    // 组合两个旋转
    Matrix3d final_rotation = rotation_y.toRotationMatrix() * rotation_z.toRotationMatrix();
    std::cout << "Final rotation matrix:\n" << final_rotation << std::endl;
    // Final rotation matrix:
    //   3.7494e-33 -6.12323e-17           -1
    //            1  6.12323e-17            0
    //  6.12323e-17           -1  6.12323e-17

    Vector3d t_1_2(0, 0, 0); // 1系原点在2系下的坐标

    // 测试点 1
    Vector3d pointA(1, 1, 1); // 1坐标系下的点
    Vector3d pointA_transformed = final_rotation.transpose() * pointA + t_1_2;
    std::cout << "Point A in Frame2: " << pointA_transformed.transpose() << std::endl;

    // 测试点 2
    Vector3d pointB(0, 2, 0); // 1坐标系下的点
    Vector3d pointB_transformed = final_rotation.transpose() * pointB + t_1_2;
    std::cout << "Point B in Frame2: " << pointB_transformed.transpose() << std::endl;


    // -----------------------------------------------------
    // -----------分隔符 下面是liosam中两个旋转矩阵的求解--------
    // -----------------------------------------------------

    // LIO-SAM中的extrinsicRot的计算过程
    // 将imu坐标系旋转到雷达坐标系
    // 具体的旋转是固定imu坐标系，将imu坐标系旋转到雷达坐标系，绕imu系的y轴旋转180度，得到和雷达坐标系一样的坐标系
    const double angle1 = M_PI; 
    AngleAxisd angle1_lio_y(angle1, Vector3d::UnitY());
    Matrix3d extrinsicRot = angle1_lio_y.toRotationMatrix(); 
    std::cout << "extrinsicRot rotation matrix:\n" << extrinsicRot << std::endl;
    // extrinsicRot rotation matrix:
    //           -1            0  1.22465e-16
    //            0            1            0
    // -1.22465e-16            0           -1

    // -----------------------------------------------------

    // LIO-SAM中的extQRPY的计算过程，也就是R_mag_lidar
    // 将磁力计坐标系旋转到lidar坐标系
    // 固定磁力计坐标系，绕磁力计坐标系的z轴旋转-90度，得到和lidar坐标系一样的坐标系
    const double angel2 = -M_PI_2; 
    AngleAxisd angle2_lio_z(angel2, Vector3d::UnitZ());
    Matrix3d extQRPY = angle2_lio_z.toRotationMatrix(); 
    std::cout << "extQRPY rotation matrix:\n" << extQRPY << std::endl;
    // extQRPY rotation matrix:
    // 6.12323e-17           1           0
    //          -1 6.12323e-17           0
    //           0           0           1

    return 0;
}
