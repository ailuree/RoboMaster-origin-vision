/*
 * @Description: 姿态解算
 * @Version: 1.0
 * @Author: Barimu
 * @Date: 2022-10-30 00:10:23
 * @LastEditors: Barimu
 * @LastEditTime: 2023-03-06 22:28:06
 */
#ifndef RM2022_SOLVEPNP_CPP
#define RM2022_SOLVEPNP_CPP

#include "../include/SolvePnP.h"
#include <cmath>
#include <vector>
#include "../include/serial.h"

using namespace std;
using namespace cv;

/**
 * @description: 调用pnp
 * @param {armors} finalarmor
 */
void SOLVEPNP ::caculate(vgd_stl::armors &finalarmor) {


        static float final_distance;
        float tmp;

        picture_points.push_back(finalarmor.corner[1]);
        picture_points.push_back(finalarmor.corner[4]);
        picture_points.push_back(finalarmor.corner[2]);
        picture_points.push_back(finalarmor.corner[3]);

        //判断是大装甲板还是小装甲板
        if (finalarmor.number==0) {
            //大装甲板            
            xishu = (22.8 / finalarmor.boardw + 4.85 / finalarmor.boardh) / 2;
                //世界坐标
            tmp = PNP(finalarmor,1);
            //if(tmp > 10) 
            final_distance = tmp;

        } else {
            //小装甲板
            xishu = (13.25 / finalarmor.boardw + 4.85 / finalarmor.boardh) / 2;
            tmp = PNP(finalarmor,0);
            ///if(tmp > 10) 
            final_distance = tmp;
            //cout << "final_distance  " << tmp<<endl;
        }
        //calAngle(camera_matrix,dist_coeffs,finalarmor.center.x,finalarmor.center.y);
        
        double distance_to_midboard_x, distance_to_midboard_y;
        distance_to_midboard_x = xishu * (finalarmor.center.x - midx);
        distance_to_midboard_y = xishu * (finalarmor.center.y - midy);


	finalarmor.dtm = sqrt((finalarmor.center.x - midx)*(finalarmor.center.x - midx)+(finalarmor.center.y - midy)*(finalarmor.center.y - midy));

        double angle_x = atan2(distance_to_midboard_x, final_distance);
        double angle_y = atan2(distance_to_midboard_y, final_distance)+4.134;

        double final_angle_x = angle_x / P * 180;
        double final_angle_y = angle_y / P * 180;

        finalarmor.position[0]=distance_to_midboard_x/100 ;
        finalarmor.position[1]=distance_to_midboard_y/100 ;
        finalarmor.position[2]=final_distance/100 ;
//         cout << "final_distance  " << tmp<<endl;
//         cout<<"yaw="<<final_angle_x<<endl;
//         cout<<"pitch="<<final_angle_y<<endl<<endl;
        
//#ifdef NX
        //if(tmp > 10)
            //uart.sSendData(final_angle_x, final_angle_y,final_distance,1);

//#endif
}

/**
 * @description: 进行pnp解算
 * @param {int} flag 通过flag的值判断大小装甲板
 * TODO:通过数字分类来选真实坐标
 */
float SOLVEPNP::PNP(vgd_stl::armors &finalarmor,int flag)
{
    //写入真实值
    if(flag==0)
    {
            model_points.push_back(Point3d(-66.75f, -24.25f, 0));
            model_points.push_back(Point3d(+66.75f, -24.25f, 0));
            model_points.push_back(Point3d(-66.75f, +24.25f, 0));
            model_points.push_back(Point3d(+66.75f, +24.25f, 0));
    }
    if(flag==1)
    {
            model_points.push_back(Point3d(-114.0f, -24.25f, 0));
            model_points.push_back(Point3d(+114.0f, -24.25f, 0));
            model_points.push_back(Point3d(-114.0f, +24.25f, 0));
            model_points.push_back(Point3d(+114.0f, +24.25f, 0));
    }

    solvePnP(model_points, picture_points, camera_matrix, dist_coeffs,
        rotation_vector, translation_vector, 0, cv::SOLVEPNP_ITERATIVE);
    // 默认ITERATIVE方法，可尝试修改为EPNP（CV_EPNP）,P3P（CV_P3P）

    Mat Rvec;
    Mat Tvec;
    rotation_vector.convertTo(Rvec, CV_32F);  // 旋转向量转换格式
    translation_vector.convertTo(Tvec, CV_32F); // 平移向量转换格式 

    // finalarmor.position[0]=translation_vector.at<double>(0) ;        //后边系数可调，旋转向量并未转换成旋转矩阵
    // finalarmor.position[1]=translation_vector.at<double>(1) ;
    // finalarmor.position[2]=translation_vector.at<double>(2) ;
    //finalarmor.position[0]=Tvec.at<float>(0,0) ;        
    //finalarmor.position[1]=Tvec.at<float>(1,0) ;
    //finalarmor.position[2]=Tvec.at<float>(2,0) ;
    //cout<<finalarmor.position[0]<<finalarmor.position[1]<<endl;
    
    //cout<<Tvec<<endl;

    //double current_yaw = std::atan2(finalarmor.position[0], finalarmor.position[2])/CV_PI*180;
    //cout<<current_yaw<<endl;
	
	//double rm[9];
	//Mat rotMat(3,3,CV_64FC1,rm);
    Mat_<float> rotMat(3, 3);
    //Mat_<float> traMat(3, 3);
    Rodrigues(Rvec, rotMat);
	//yaw=atan2(rotMat(2,1),rotMat(2,2))*57.2958;
	//pitch=atan2(-rotMat(2,0),sqrt(rotMat(2,0)*rotMat(2,0)+rotMat(2,2)*rotMat(2,2)))*57.298;

    // 旋转向量转成旋转矩阵

    Mat P_oc;
    P_oc = -rotMat.inv() * Tvec;
    // 求解相机的世界坐标，得出p_oc的第三个元素即相机到物体的距离即深度信息，单位是mm

    //迭代器版
    // MatIterator_<float> it = P_oc.begin<float>();
    // MatIterator_<float> it_end = P_oc.end<float>();
    // for(int i = 1;it != it_end;it++,i++)
    // {
    //     if(i == 3)
    //     {
    //          distance = (float)(*it);
    //          break;
    //     }

    // }
    //at版
    distance = Tvec.at<float>(2,0);
    distance /= 10; 
//	cout<<P_oc<<endl;
//cout<<Tvec<<endl;
    //yaw=atan(Tvec.at<float>(0, 0)/Tvec.at<float>(2, 0))/CV_PI*180;
    //pitch=atan(Tvec.at<float>(1, 0)/Tvec.at<float>(2, 0))/CV_PI*180;
	//cout<<Tvec<<endl;
    //cout<<yaw<<endl;

     //cout<<yaw<<" "<<pitch<<" "<<endl;
    //cout << "P_oc " << endl << P_oc << endl;
     //cout << "distance " << abs(distance)<< endl;
    //cout<<flag<<endl;
    return abs(distance);
}

void SOLVEPNP::calAngle(Mat cam,Mat dis,int x,int y)
{
    double fx=cam.at<double>(0,0);
    double fy=cam.at<double>(1,1);
    double cx=cam.at<double>(0,2);
    double cy=cam.at<double>(1,2);
    Point2f pnt;
    vector<cv::Point2f> in;
    vector<cv::Point2f> out;
    in.push_back(Point2f(x,y));
    //对像素点去畸变
    undistortPoints(in,out,cam,dis,noArray(),cam);
    pnt=out.front();
    //没有去畸变时的比值
    double rx=(x-cx)/fx;
    double ry=(y-cy)/fy;
    //去畸变后的比值
     rxNew=(pnt.x-cx)/fx;
     ryNew=(pnt.y-cy)/fy;

    //输出未去畸变时测得的角度和去畸变后测得的角度
    //ecout<< " angleNew:"<<atan(rxNew)/CV_PI*180<<" ";
    //cout<< " angleNew:"<<atan(ryNew)/CV_PI*180<<endl;
    // cout<<pitch<<" "<<yaw<<endl;
}
#endif
