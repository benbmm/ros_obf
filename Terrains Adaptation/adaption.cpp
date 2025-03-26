#include <math.h>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>

// ROS2
#include "interfaces/srv/command_adaption.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

void load_cpg(void);
void turn(int);
void cal_out_3_walk(int);


#define _Maxstep 40000
#define _count 1000
#define NUM_SERVOS 18

const char *SERVOS[NUM_SERVOS] = {"R00", "R01", "R02", "R10", "R11", "R12",
                                  "R20", "R21", "R22", "L00", "L01", "L02",
                                  "L10", "L11", "L12", "L20", "L21", "L22"};
int k = 0;
int kk = 0, kkk[7] = {0}, aaa[7] = {0}, judge[7] = {0, 2, 2, 2, 2, 2, 2},
    cc[7] = {0};
double pi[7] = {0}, ro[7] = {0},
       ladder[7] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
double thres = 0.05;
double roll, pitch, yaw;
double e[7];  // 存roll, pitch在六個方向的分量
double temp = 0, number[7] = {0};
double kp[7] = {0, -14, -16.25, -14, 14, 16.25, 14};
double kd[7] = {0, -0.2, -0.2, -0.2, 0.2, 0.2, 0.2};
double h[7] = {0};
//當輸出的膝關節控制量發生變化，則change=1
int change=0;
int reduce_by_min_135=0;
int reduce_by_min_246=0;
double min_abs_val;



using namespace std;

class OSC {
  friend class CPG;

 public:
  double dUe[_count + _Maxstep], dUf[_count + _Maxstep], dVe[_count + _Maxstep],
      dVf[_count + _Maxstep];
  double Ue[_count + _Maxstep], Uf[_count + _Maxstep], Ve[_count + _Maxstep],
      Vf[_count + _Maxstep], Ye[_count + _Maxstep], Yf[_count + _Maxstep],
      Y[_count + _Maxstep];
};

class CPG {
  friend class OSC;

 public:
  OSC osc[4 + 1];
  double deep[_Maxstep + 1];
  double out[_Maxstep + 1];
  double lpara, lpara2, height[_Maxstep + 1], ttemp = 0;
  int clad = 1, no = 0, ww = 1;
};

CPG leg[6 + 1];

class adaption_node : public rclcpp::Node {
 public:
  adaption_node() : Node("adaption_node") {
    sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data", 1,
        std::bind(&adaption_node::callback, this, std::placeholders::_1));
    service_ = this->create_service<interfaces::srv::CommandAdaption>(
        "commandadaption",
        std::bind(&adaption_node::Service_callback, this, std::placeholders::_1,
                  std::placeholders::_2));

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "adaption_node ok");
  }
  void callback(const sensor_msgs::msg::Imu::SharedPtr imu_data) {
    // 四元数
    tf2::Quaternion q(imu_data->orientation.x, imu_data->orientation.y,
                      imu_data->orientation.z, imu_data->orientation.w);
    // 將四元數轉換為旋轉矩陣
    tf2::Matrix3x3 m(q);
    // 转欧拉角
    m.getRPY(pitch, roll, yaw);

    FILE *pitch_data = fopen("/home/user/ros2_obf_ws/src/pitch_data.txt", "a");
    FILE *roll_data = fopen("/home/user/ros2_obf_ws/src/roll_data.txt", "a");
    fprintf(pitch_data, "%f\n", roll);
    fprintf(roll_data, "%f\n", pitch);
    fclose(pitch_data);
    fclose(roll_data);

    roll=-roll;
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Roll: %f, Pitch: %f, Yaw: %f",roll, pitch, yaw);
    
    if (fabs(pitch) < thres) {
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pitch: %f, Roll<%f",pitch, thres);
      pitch = 0;
      
    }
    if (fabs(roll) < thres) {
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Roll: %f, Roll<%f",roll, thres);
      roll = 0;
      
    }
    
  }
  void Service_callback(
      const std::shared_ptr<interfaces::srv::CommandAdaption::Request> request,
      std::shared_ptr<interfaces::srv::CommandAdaption::Response> response) {
    int step = request->step;

    initialization();
    deep(step);
    cal_out_3_walk(step);
    /*
    if (reduce_by_min_135){
      reduce_by_min_if_nonzero(1);
    }
    if(reduce_by_min_246){
      reduce_by_min_if_nonzero(2);
    }
    */
    response->h1 = h[1];
    response->h2 = h[2];
    response->h3 = h[3];
    response->h4 = h[4];
    response->h5 = h[5];
    response->h6 = h[6];
    response->change = change;


    // RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "adapting{%d}", step);
    //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"h1: %f\th2: %f\th3: %f\th4: %f\th5: %f\th6: %f\nchange:%d\n", h[1], h[2],h[3], h[4], h[5], h[6],change);
    double ctrl_val;
    if (leg[1].osc[2].Y[step]>=0 || leg[6].osc[2].Y[step]<=0){
      if(leg[1].osc[2].Y[step]==0){
        ctrl_val=abs(h[1]);
      }else{
        ctrl_val=abs(h[6]);
      }
    }else{
      ctrl_val=999;
    }
      
	RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"pitch:%f\troll:%f\tchange:%d\nctrl_val:%f\n", pitch,roll,change,ctrl_val);
  }
  void reduce_by_min_if_nonzero(int set){
    if(set==1){
      min_abs_val = std::min({std::abs(h[1]), std::abs(h[3]), std::abs(h[5])});
      if (min_abs_val != 0){
        if(std::abs(h[1]) == min_abs_val){
          if(h[1]>=0){
            h[1]+=h[1];
            h[3]+=h[1];
            h[5]-=h[1];
          }else{
            h[1]-=h[1];
            h[3]-=h[1];
            h[5]+=h[1];
          }
        }
        if(std::abs(h[3]) == min_abs_val){
          if(h[3]>=0){
            h[1]+=h[3];
            h[3]+=h[3];
            h[5]-=h[3];
          }else{
            h[1]-=h[3];
            h[3]-=h[3];
            h[5]+=h[3];
          }
        }
        if(std::abs(h[5]) == min_abs_val){
          if(h[5]>=0){
            h[1]+=h[5];
            h[3]+=h[5];
            h[5]-=h[5];
          }else{
            h[1]-=h[5];
            h[3]-=h[5];
            h[5]+=h[5];
          }
        }
      }
      h[1] = std::clamp(h[1], -0.6, 0.6);
      h[3] = std::clamp(h[3], -0.6, 0.6);
      h[5] = std::clamp(h[5], -0.6, 0.6);
    }
    if(set==2){
      min_abs_val = std::min({std::abs(h[2]), std::abs(h[4]), std::abs(h[6])});
      if (min_abs_val != 0){
        if(std::abs(h[2]) == min_abs_val){
          if(h[2]>=0){
            h[2]+=h[2];
            h[4]-=h[2];
            h[6]-=h[2];
          }else{
            h[2]-=h[2];
            h[4]+=h[2];
            h[6]+=h[2];
          }
        }
        if(std::abs(h[4]) == min_abs_val){
          if(h[4]>=0){
            h[2]+=h[4];
            h[4]-=h[4];
            h[6]-=h[4];
          }else{
            h[2]-=h[4];
            h[4]+=h[4];
            h[6]+=h[4];
          }
        }
        if(std::abs(h[6]) == min_abs_val){
          if(h[6]>=0){
            h[2]+=h[6];
            h[4]-=h[6];
            h[6]-=h[6];
          }else{
            h[2]-=h[6];
            h[4]+=h[6];
            h[6]+=h[6];
          }
        }
      }
      h[2] = std::clamp(h[2], -0.6, 0.6);
      h[4] = std::clamp(h[4], -0.6, 0.6);
      h[6] = std::clamp(h[6], -0.6, 0.6);
    }
    
  }
  void deep(int num_count) {
    e[1] = (pitch + roll) * 0.707;
    e[2] = roll;
    e[3] = (-pitch + roll) * 0.707;
    e[4] = (-pitch - roll) * 0.707;
    e[5] = -roll;
    e[6] = (pitch - roll) * 0.707;

    for (int i = 1; i < 7; i++) {
      leg[i].deep[num_count] = e[i];
      // RCLCPP_INFO(rclcpp::get_logger("rclcpp"),
      // "leg[%d].deep[%d]=%f\n",i,num_count,e[i]);
    }
  }
  void initialization() {
    for (int i = 1; i <= 6; i++) {
      leg[i].lpara = kp[i];
      leg[i].lpara2 = kd[i] / 0.2;
    }
  }
  void cal_out_3_walk(int num_count) {
    /* for (int i = 1; i < 7; i++) {
      printf("deep[%d]=%f\t",i,leg[i].deep[num_count]);
    }
    printf("\n"); */
    reduce_by_min_135=0;
    reduce_by_min_246=0;
    change=0;
    for (int j = 1; j <= 6; j++) {
      leg[j].height[num_count] = leg[j].height[num_count - 1];
      //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"leg[%d].height[%d]=%f",j,num_count,leg[j].height[num_count]);
      number[j] = fabs(leg[j].height[num_count]);
    }
    for (int a = 1; a <= 5; a = a + 2) {
      for (int b = a; b <= 5; b = b + 2) {
        if (number[b] > number[a]) {
          temp = number[b];
          number[b] = number[a];
          number[a] = temp;
        }
      }
    }

    for (int a = 2; a <= 6; a = a + 2) {
      for (int b = a; b <= 6; b = b + 2) {
        if (number[b] > number[a]) {
          temp = number[b];
          number[b] = number[a];
          number[a] = temp;
        }
      }
    }
    /*for(int i=1; i<=6; i++){
      printf("number[%d] = %lf\t", i, number[i]);
    }*/
    //-------------------------------------------------------
    for (int i = 0; i < NUM_SERVOS; i = i + 3) {
      ///*
      if (i == 0 || i == 3 || i == 6) {
        int j = 0;
        if (i == 0) {
          j = 1;
        } else if (i == 3) {
          j = 2;
        } else {
          j = 3;
        }
        //------------------------------------------------------------------------------------

        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "leg[%d].osc[2].Y[%d]=%f,173",j, num_count, leg[j].osc[2].Y[num_count]);
        if (leg[j].osc[2].Y[num_count] < 0) {
          //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ww=%d", leg[j].ww);
          if (leg[j].ww ==
              1) {  // when switch in this condition(>0),use number of "no" to
                    // judge if it's success in adjust or not

            if (leg[j].no >= 10) {
              //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "no=%d,adjust success",leg[j].no);
              judge[j] = 1;  // adjust success
              leg[j].no = 0;
              leg[j].ww = 2;
            } else {
              //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "no=%d,adjust failed",leg[j].no);
              judge[j] = 2;  // adjust failed
              leg[j].ww = 2;
              leg[j].no = 0;
              leg[j].height[num_count - 1] = 0;  // reset the height
            }
          }

          //--------inorder to make the robot smoothly
          leg[j].height[num_count] = leg[j].height[num_count - 1];
          kkk[j]++;  // kkk++ then kkk==1
          if (kkk[j] !=
              2) {  // consider when it adjust success(height > 0),we
                    // dont want the leg go back to 0 then go to height
            if (leg[j].osc[2].Y[num_count] >
                leg[j].height[num_count]) {  // if the wave < height, then stop
                                             // at height till wave > height
              // printf("First-->height-upupupup=%lf\n",leg[1].height[num_count]);
              h[j] = leg[j].height[num_count];
              kkk[j] = 0;
            } else {  // now is the moment that wave > height
              h[j] = leg[j].osc[2].Y[num_count];
              // printf("First-->out-upupupup=%lf\n",leg[1].osc[2].Y[num_count]);
              kkk[j] = 1;
            }
          }
          //--------inorder to make the robot smoothly
          else {
            if (leg[j].osc[2].Y[num_count] < leg[j].height[num_count]) {
              h[j] = leg[j].osc[2].Y[num_count];
              // printf("Second-->out-upupupup=%lf\n",leg[1].osc[2].Y[num_count]);
            } else {  // when the wave go back lower, we dont want the leg
                      // follow the wave while wave < height(stays at height)
              h[j] = leg[j].height[num_count];
              // printf("Second-->height-upupupup=%lf\n",leg[1].height[num_count]);
            }
            kkk[j] = 1;
          }
          leg[j].clad = 1;  // count of ladder

        }
        //------------------------------------------------------------------------------------
        else if (leg[j].osc[2].Y[num_count] >= 0) {
          leg[j].height[num_count] = leg[j].height[num_count - 1];

          //*************************************condition
          // 1*************************************
          if (judge[j] == 1) {  // no need to adjust, use last time's height
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"no need to adjust\t231\n");
            if (j == 1 || j == 3) {
              if (number[5] >= 0) {
                h[j] = leg[j].height[num_count] + number[5];
                printf("out[%d]=%lf - NNNNNNNNNNNNNN\t", j,
                       leg[j].height[num_count] + number[5]);
              } else {
                h[j] = leg[j].height[num_count] - number[5];
                printf("out[%d]=%lf - NNNNNNNNNNNNNN\t", j,
                       leg[j].height[num_count] - number[5]);
              }
            } else {
              if (number[6] >= 0) {
                h[j] = leg[j].height[num_count] + number[6];
                printf("out[%d]=%lf - NNNNNNNNNNNNNN\t", j,
                       leg[j].height[num_count] + number[6]);
              } else {
                h[j] = leg[j].height[num_count] - number[6];
                printf("out[%d]=%lf - NNNNNNNNNNNNNN\t", j,
                       leg[j].height[num_count] - number[6]);
              }
            }

            ////wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]);
            // printf(">>out[%d] = %lf
            // NONONONONONO\t",j,leg[j].height[num_count]);
          }
          //*************************************condition
          // 1*************************************
          //*************************************condition
          // 2*************************************
          else if (judge[j] == 2) {  // need to adjust again
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "need to adjust\t263\n");
            if (leg[j].deep[num_count] * leg[j].lpara +
                    (leg[j].deep[num_count] - leg[j].deep[num_count - 1]) *
                        leg[j].lpara2 <=
                leg[j].out[num_count - 1]) {
              leg[j].out[num_count] =
                  leg[j].deep[num_count] * leg[j].lpara +
                  (leg[j].deep[num_count] - leg[j].deep[num_count - 1]) *
                      leg[j].lpara2;
            } else {
              leg[j].out[num_count] = leg[j].out[num_count - 1];
            }
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "leg[%d].out[%d]=%f",j,num_count,leg[j].out[num_count]);

            // if(pitch==0 && roll==0){
            if (fabs(pitch) <= thres && fabs(roll) <= thres) {
              leg[j].out[num_count - 1] = leg[j].ttemp;
              leg[j].out[num_count] = leg[j].out[num_count - 1];

            } else if (leg[j].out[num_count] <= -0.6) {
              leg[j].out[num_count] = -0.6;
            }
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "fin.leg[%d].out[%d]=%f",j,num_count,leg[j].out[num_count]);
            leg[j].height[num_count] = leg[j].out[num_count];

            /*
            wb_motor_set_position(R_servo[i+1], leg[j].out[num_count]);
            printf("out[%d]=%lf - yyyyyyyyyyyyyy\t", j,leg[j].out[num_count]);
            */

            //*
            if (leg[j].clad == 1) {
              if (fabs(leg[j].height[num_count]) > ladder[2]) {
                h[j] = -ladder[2];
                leg[j].ttemp = -ladder[2];
                leg[j].clad = 2;
                change=1;
                printf("1realout[%d]=%lf\t", j, -ladder[2]);
              } else {
                h[j] = leg[j].height[num_count];
                leg[j].ttemp = leg[j].height[num_count];
                leg[j].clad = 1;
                change=1;
                printf("1realout[%d]=%lf\t", j, leg[j].height[num_count]);
              }
            }

            else if (leg[j].clad == 2) {
              if (fabs(leg[j].height[num_count]) > ladder[3]) {
                h[j] = -ladder[3];
                leg[j].ttemp = -ladder[3];
                leg[j].clad = 3;
                change=1;
                printf("2realout[%d]=%lf\t", j, -ladder[3]);
                //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ffffffffffffffffffffffffffffffffff");
              } else {
                h[j] = leg[j].height[num_count];
                leg[j].ttemp = leg[j].height[num_count];
                leg[j].clad = 2;
                change=1;
                printf("2realout[%d]=%lf\t", j, leg[j].height[num_count]);
              }
            }

            else if (leg[j].clad == 3) {
              if (fabs(leg[j].height[num_count]) > ladder[4]) {
                h[j] = -ladder[4];
                leg[j].ttemp = -ladder[4];
                leg[j].clad = 4;
                change=1;
                printf("3realout[%d]=%lf\t", j, -ladder[4]);
              } else {
                h[j] = leg[j].height[num_count];
                leg[j].ttemp = leg[j].height[num_count];
                leg[j].clad = 3;
                change=1;
                printf("3realout[%d]=%lf\t", j, leg[j].height[num_count]);
              }
            }

            else if (leg[j].clad == 4) {
              if (fabs(leg[j].height[num_count]) > ladder[5]) {
                h[j] = -ladder[5];
                leg[j].ttemp = -ladder[5];
                leg[j].clad = 5;
                change=1;
                printf("4realout[%d]=%lf\t", j, -ladder[5]);
              } else {
                h[j] = leg[j].height[num_count];
                leg[j].ttemp = leg[j].height[num_count];
                leg[j].clad = 4;
                change=1;
                printf("4realout[%d]=%lf\t", j, leg[j].height[num_count]);
              }
            }

            else if (leg[j].clad == 5) {
              if (fabs(leg[j].height[num_count]) > ladder[6]) {
                h[j] = -ladder[6];
                leg[j].ttemp = -ladder[6];
                leg[j].clad = 5;
                change=1;
                printf("5realout[%d]=%lf\t", j, -ladder[6]);
              } else {
                h[j] = leg[j].height[num_count];
                leg[j].ttemp = leg[j].height[num_count];
                leg[j].clad = 5;
                change=1;
                printf("5realout[%d]=%lf\t", j, leg[j].height[num_count]);
              }
            }
            if(j==1 or j==3){
              reduce_by_min_135=1;
            }else{
              reduce_by_min_246=1;
            }
            //*/
            kkk[j] = 0;
          }
          //*************************************condition
          // 2*************************************
          //-------------------------------------
          pi[j] = pitch;
          ro[j] = roll;
          //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pitch=%f,roll=%f,373",pitch, roll);
          if (fabs(pi[j]) < thres &&
              fabs(ro[j]) < thres) {  // no need to adjust
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "no++");
            leg[j].no++;
          }
          // printf("judge[0]=%d\n",judge[0]);
          //-------------------------------------
          leg[j].ww = 1;
          kkk[j] = 0;
        }
        //------------------------------------------------------------------------------------
        // wb_motor_set_position(R_servo[i+2], -leg[j].osc[3].Y[num_count]);
      }
      //*/
      ///*
      if (i == 9 || i == 12 || i == 15) {
        int j = 0;
        if (i == 9) {
          j = 6;
        } else if (i == 12) {
          j = 5;
        } else {
          j = 4;
        }

        //------------------------------------------------------------------------------------
        //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "leg[%d].osc[2].Y[%d]=%f,400",j, num_count, leg[j].osc[2].Y[num_count]);
        if (leg[j].osc[2].Y[num_count] > 0) {
          //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "ww=%d", leg[j].ww);
          if (leg[j].ww == 1) {
            if (leg[j].no >= 10) {
              //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "no=%d,adjust success",leg[j].no);
              judge[j] = 1;
              leg[j].no = 0;
              leg[j].ww = 2;
            } else {
              //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "no=%d,adjust failed",leg[j].no);
              judge[j] = 2;
              leg[j].ww = 2;
              leg[j].no = 0;
              leg[j].height[num_count - 1] = 0;
            }
          }

          leg[j].height[num_count] = leg[j].height[num_count - 1];
          kkk[j]++;
          if (kkk[j] != 2) {
            if (leg[j].osc[2].Y[num_count] < leg[j].height[num_count]) {
              // printf("First-->height-upupupup=%lf\n",leg[1].height[num_count]);
              h[j] = leg[j].height[num_count];
              kkk[j] = 0;
              // printf("out[%d] = %lf\t",j,leg[j].height[num_count]);
            } else {
              h[j] = leg[j].osc[2].Y[num_count];
              // printf("First-->out-upupupup=%lf\n",leg[1].osc[2].Y[num_count]);
              kkk[j] = 1;
              // printf("out[%d] = %lf\t",j,leg[j].osc[2].Y[num_count]);
            }

          }
          //--------inorder to make the robot smoothly

          else {
            // printf("height!!!!!!!!=%lf\n",leg[1].height[num_count]);

            if (leg[j].osc[2].Y[num_count] > leg[j].height[num_count]) {
              h[j] = leg[j].osc[2].Y[num_count];
              // printf("out[%d] = %lf\t",j,leg[j].osc[2].Y[num_count]);
              // printf("Second-->out-upupupup=%lf\n",leg[1].osc[2].Y[num_count]);
            } else {
              h[j] = leg[j].height[num_count];
              // printf("out[%d] = %lf\t",j,leg[j].height[num_count]);
              // printf("Second-->height-upupupup=%lf\n",leg[1].height[num_count]);
            }

            kkk[j] = 1;
          }
          leg[j].clad = 1;
        }
        //------------------------------------------------------------------------------------
        else if (leg[j].osc[2].Y[num_count] <= 0) {
          leg[j].height[num_count] = leg[j].height[num_count - 1];

          //*************************************condition
          // 1*************************************
          if (judge[j] == 1) {  // no need to adjust
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"),"no need to adjust\t452\n");
            ///*
            if (j == 5) {
              if (number[5] >= 0) {
                h[j] = leg[j].height[num_count] - number[5];
                printf("out[%d]=%lf - NNNNNNNNNNNNNN\t", j,
                       leg[j].height[num_count] - number[5]);
              } else {
                h[j] = leg[j].height[num_count] + number[5];
                printf("out[%d]=%lf - NNNNNNNNNNNNNN\t", j,
                       leg[j].height[num_count] + number[5]);
              }
            } else {
              if (number[6] >= 0) {
                h[j] = leg[j].height[num_count] - number[6];
                printf("out[%d]=%lf - NNNNNNNNNNNNNN\t", j,
                       leg[j].height[num_count] - number[6]);
              } else {
                h[j] = leg[j].height[num_count] + number[6];
                printf("out[%d]=%lf - NNNNNNNNNNNNNN\t", j,
                       leg[j].height[num_count] + number[6]);
              }
            }

            //*/
            ////wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]);
            // printf(">>out[%d] = %lf
            // NONONONONONO\t",j,leg[j].height[num_count]);
          }
          //*************************************condition
          // 1*************************************

          //*************************************condition
          // 2*************************************
          else if (judge[j] == 2) {  // start to adjust
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "need to adjust\t487\n");
            if (leg[j].deep[num_count] * leg[j].lpara +
                    (leg[j].deep[num_count] - leg[j].deep[num_count - 1]) *
                        leg[j].lpara2 >=
                leg[j].out[num_count - 1]) {  // compare with last time's out
              leg[j].out[num_count] =
                  leg[j].deep[num_count] * leg[j].lpara +
                  (leg[j].deep[num_count] - leg[j].deep[num_count - 1]) *
                      leg[j].lpara2;
              // printf("\ncccccccccccccccccc\nleg[j].out[num_count]=%f\n",leg[j].out[num_count]);
            } else {
              leg[j].out[num_count] = leg[j].out[num_count - 1];
            }
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "leg[%d].out[%d]=%f",j,num_count,leg[j].out[num_count]);
            // if(pitch==0 && roll==0){
            if (fabs(pitch) <= thres && fabs(roll) <= thres) {
              leg[j].out[num_count - 1] = leg[j].ttemp;
              leg[j].out[num_count] = leg[j].out[num_count - 1];
            } else if (leg[j].out[num_count] >= 0.6) {
              leg[j].out[num_count] = 0.6;
            }
            //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "fin,leg[%d].out[%d]=%f",j,num_count,leg[j].out[num_count]);
            leg[j].height[num_count] = leg[j].out[num_count];

            /*
            wb_motor_set_position(R_servo[i+1], leg[j].out[num_count]);
            printf("out[%d]=%lf - yyyyyyyyyyyyyy\t", j,leg[j].out[num_count]);
            */

            //*
            if (leg[j].clad == 1) {
              if ((leg[j].height[num_count]) > ladder[2]) {
                h[j] = ladder[2];
                leg[j].ttemp = ladder[2];
                leg[j].clad = 2;
                change=1;
                printf("1realout[%d]=%lf\t", j, ladder[2]);
              } else {
                h[j] = leg[j].height[num_count];
                leg[j].ttemp = leg[j].height[num_count];
                leg[j].clad = 1;
                change=1;
                printf("1realout[%d]=%lf\t", j, leg[j].height[num_count]);
              }
            }

            else if (leg[j].clad == 2) {
              if ((leg[j].height[num_count]) > ladder[3]) {
                h[j] = ladder[3];
                leg[j].ttemp = ladder[3];
                leg[j].clad = 3;
                change=1;
                printf("2realout[%d]=%lf\t", j, ladder[3]);
              } else {
                h[j] = leg[j].height[num_count];
                leg[j].ttemp = leg[j].height[num_count];
                leg[j].clad = 2;
                change=1;
                printf("2realout[%d]=%lf\t", j, leg[j].height[num_count]);
              }
            }

            else if (leg[j].clad == 3) {
              if ((leg[j].height[num_count]) > ladder[4]) {
                h[j] = ladder[4];
                leg[j].ttemp = ladder[4];
                leg[j].clad = 4;
                change=1;
                printf("3realout[%d]=%lf\t", j, ladder[4]);
              } else {
                h[j] = leg[j].height[num_count];
                leg[j].ttemp = leg[j].height[num_count];
                leg[j].clad = 3;
                change=1;
                printf("3realout[%d]=%lf\t", j, leg[j].height[num_count]);
              }
            }

            else if (leg[j].clad == 4) {
              if ((leg[j].height[num_count]) > ladder[5]) {
                h[j] = ladder[5];
                leg[j].ttemp = ladder[5];
                leg[j].clad = 5;
                change=1;
                printf("4realout[%d]=%lf\t", j, ladder[5]);
              } else {
                h[j] = leg[j].height[num_count];
                leg[j].ttemp = leg[j].height[num_count];
                leg[j].clad = 4;
                change=1;
                printf("4realout[%d]=%lf\t", j, leg[j].height[num_count]);
              }
            }

            else if (leg[j].clad == 5) {
              if ((leg[j].height[num_count]) > ladder[6]) {
                h[j] = ladder[6];
                leg[j].ttemp = ladder[6];
                leg[j].clad = 5;
                change=1;
                printf("5realout[%d]=%lf\t", j, ladder[6]);
              } else {
                h[j] = leg[j].height[num_count];
                leg[j].ttemp = leg[j].height[num_count];
                leg[j].clad = 5;
                change=1;
                printf("5realout[%d]=%lf\t", j, leg[j].height[num_count]);
              }
            }
            if(j==5){
              reduce_by_min_135=1;
            }else{
              reduce_by_min_246=1;
            }
            //*/
            kkk[j] = 0;
          }
          //*************************************condition
          // 2*************************************
          //-------------------------------------
          pi[j] = pitch;
          ro[j] = roll;
          //RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "pitch=%f,roll=%f,602",pitch, roll);

          if (fabs(pi[j]) < thres &&
              fabs(ro[j]) < thres) {  // no need to adjust
            leg[j].no++;
          }
          // printf("judge[0]=%d\n",judge[0]);
          //-------------------------------------
          leg[j].ww = 1;
          kkk[j] = 0;
        }
        // printf("leg[j].deep[%d] = %lf\tdeep[%d]*kp = %lf\tdelta->deep[%d]*kd
        // = %lf\tleg[j].out[%d] =
        // %lf\n",num_count,leg[j].deep[num_count],num_count,leg[j].deep[num_count]*leg[j].lpara,num_count,leg[j].deep[num_count]-leg[j].deep[num_count-1]*leg[j].lpara2,num_count,leg[j].out[num_count]);
        //------------------------------------------------------------------------------------
        // wb_motor_set_position(R_servo[i+2], -leg[j].osc[3].Y[num_count]);
      }
      //*/
    }
  }

 private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_;
  rclcpp::Service<interfaces::srv::CommandAdaption>::SharedPtr service_;
};

int main(int argc, char **argv) {
  load_cpg();
  rclcpp::init(argc, argv);
  adaption_node adap;
  auto node = std::make_shared<adaption_node>();

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

void load_cpg(void) {
  FILE *YYout11 =
      fopen("/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout11.txt", "r");
  FILE *YYout21 =
      fopen("/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout21.txt", "r");
  FILE *YYout31 =
      fopen("/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout31.txt", "r");
  FILE *YYout41 =
      fopen("/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout41.txt", "r");
  FILE *YYout51 =
      fopen("/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout51.txt", "r");
  FILE *YYout61 =
      fopen("/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout61.txt", "r");

  FILE *YYout12 =
      fopen("/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout12.txt", "r");
  FILE *YYout22 =
      fopen("/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout22.txt", "r");
  FILE *YYout32 =
      fopen("/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout32.txt", "r");
  FILE *YYout42 =
      fopen("/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout42.txt", "r");
  FILE *YYout52 =
      fopen("/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout52.txt", "r");
  FILE *YYout62 =
      fopen("/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout62.txt", "r");

  FILE *YYout13 =
      fopen("/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout13.txt", "r");
  FILE *YYout23 =
      fopen("/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout23.txt", "r");
  FILE *YYout33 =
      fopen("/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout33.txt", "r");
  FILE *YYout43 =
      fopen("/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout43.txt", "r");
  FILE *YYout53 =
      fopen("/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout53.txt", "r");
  FILE *YYout63 =
      fopen("/home/user/ros2_obf_ws/src/cpg/fixed_cpg/YYout63.txt", "r");

  for (int count = 1; count <= _Maxstep; ++count) {
    fscanf(YYout11, "%lf\n", &(leg[1].osc[1].Y[count]));
    fscanf(YYout12, "%lf\n", &(leg[1].osc[2].Y[count]));
    fscanf(YYout13, "%lf\n", &(leg[1].osc[3].Y[count]));

    fscanf(YYout21, "%lf\n", &(leg[2].osc[1].Y[count]));
    fscanf(YYout22, "%lf\n", &(leg[2].osc[2].Y[count]));
    fscanf(YYout23, "%lf\n", &(leg[2].osc[3].Y[count]));

    fscanf(YYout31, "%lf\n", &(leg[3].osc[1].Y[count]));
    fscanf(YYout32, "%lf\n", &(leg[3].osc[2].Y[count]));
    fscanf(YYout33, "%lf\n", &(leg[3].osc[3].Y[count]));

    fscanf(YYout41, "%lf\n", &(leg[4].osc[1].Y[count]));
    fscanf(YYout42, "%lf\n", &(leg[4].osc[2].Y[count]));
    fscanf(YYout43, "%lf\n", &(leg[4].osc[3].Y[count]));

    fscanf(YYout51, "%lf\n", &(leg[5].osc[1].Y[count]));
    fscanf(YYout52, "%lf\n", &(leg[5].osc[2].Y[count]));
    fscanf(YYout53, "%lf\n", &(leg[5].osc[3].Y[count]));

    fscanf(YYout61, "%lf\n", &(leg[6].osc[1].Y[count]));
    fscanf(YYout62, "%lf\n", &(leg[6].osc[2].Y[count]));
    fscanf(YYout63, "%lf\n", &(leg[6].osc[3].Y[count]));
  }

  fclose(YYout11);
  fclose(YYout21);
  fclose(YYout31);
  fclose(YYout41);
  fclose(YYout51);
  fclose(YYout61);

  fclose(YYout12);
  fclose(YYout22);
  fclose(YYout32);
  fclose(YYout42);
  fclose(YYout52);
  fclose(YYout62);

  fclose(YYout13);
  fclose(YYout23);
  fclose(YYout33);
  fclose(YYout43);
  fclose(YYout53);
  fclose(YYout63);

  turn(_Maxstep);
}

void turn(int num_count) {
  for (int i = 0; i <= num_count; ++i) {
    if (leg[2].osc[3].Y[i] < 0) {
      leg[2].osc[3].Y[i] = 0;
    }
    if (leg[5].osc[3].Y[i] < 0) {
      leg[5].osc[3].Y[i] = 0;
    }
    for (int j = 1; j <= 6; ++j) {
      if (leg[j].osc[2].Y[i] < 0) {
        leg[j].osc[2].Y[i] = 0;
      }
      if (j <= 3) {
        leg[j].osc[2].Y[i] = -leg[j].osc[2].Y[i];
        leg[j].osc[3].Y[i] = -leg[j].osc[3].Y[i];
      } else {
        leg[j].osc[1].Y[i] = -leg[j].osc[1].Y[i];
      }
    }
    leg[1].osc[3].Y[i] = -leg[1].osc[3].Y[i];
    leg[6].osc[3].Y[i] = -leg[6].osc[3].Y[i];
  }
}
