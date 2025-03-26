//////////////////////////////  include  ///////////////////////////////////////////
#include <sstream>
#include <iostream>
#include <algorithm>
#include <fstream>
#include <math.h>
#include <vector>
#include <string>

//ROS2
#include "rclcpp/rclcpp.hpp"
#include "interfaces/msg/output.hpp" 
#include "sensor_msgs/msg/laser_scan.hpp"
#include "interfaces/srv/command.hpp" 

//////////////////////////////  include  ///////////////////////////////////////////


//////////////////////////////  define區  ////////////////////////////////////////
#define MIN(a,b) (a<b?a:b)
#define _max_rule 30
int _in_clu;	//紀錄rule數量的變數
#define _in_varl 4
#define _out_varl 2  //輸出個數 
#define max_step 40000 //最大步數
#define _rule_delta 10
#define _input_scale 1.0   //20200416   
#define PI 3.14159    
#define r 0.05
#define tran r*2*PI/60
#define left_wheel_speed 4  //如果控制器的後件部s有正規化則要*最高速
#define max_speed 50.0
//////////////////////////////  define區  /////////////////////////////////////////

#define load_data_clu "/home/user/ros2_obf_ws/src/controller_server/rule_s.txt"	//紀錄rule數量的文件

#define load_data_FC "/home/user/ros2_obf_ws/src/controller_server/s.txt" 	//紀錄FC參數的文件

//#define save_ave_vel "controller/nsga_v1/save_ave_speed.txt"
//#define save_ave_error "controller/nsga_v1/save_ave_dis_error.txt" 
//#define save_data_path1  "save/along_wall.txt" 

//////////////////////////////  變數宣告區  ///////////////////////////////////////{
double in[_in_varl+1] ;

double out_y[_out_varl+1] ; //輸出項暫存變數//左右輪輸出
double robot_vel[max_step+1] ;
double  sick_1[max_step+1], 
		sick_2[max_step+1], 
		sick_3[max_step+1], 
		sick_4[max_step+1]; 

double  sick_all[max_step+1], 
		sick_wall[max_step+1] ;

float far=12;
//const float PI=3.14;
const float x=9;

const float dis= 0.57; // wei change width

double ave_speed=0.0;
float ave_distance_error=0;

const int _mem_length = 2*_in_varl+ _out_varl ;
long double fuzzy_real[_mem_length*_max_rule+1] ;
double min_m[_in_varl+1],max_m[_in_varl+1];//最小中心點 & 最大中心點
int ber=1 ; //代表第幾個解，此為一個替代的暫存變數
double deviation_whirl = 0.; //輪速差

int steps,status;


int counts=0;
int angle_left_min,angle_right_min ;
double left_min;
double right_min;
double stright_min;
//double error_z;
double back_left,back_right;
float laser_temp[897];
float laser_temp_scan[897];
double read_1;
double read_2;
double read_3;
double read_4;
double read_5;
double read_6;
double amcl_orientation_x,amcl_orientation_y,amcl_orientation_z,amcl_position_x,amcl_position_y,amcl_position_z;
double v1;
double v2;
double position_x1,position_y1,orientation_z1;

double roll_s,pitch_s,yaw_s;

int decision_left =0,decision_right =0;
int counts_left=0 ,counts_right=0;

double distances_sum;

using namespace std;
inline float phi(float x,float m,float v)
{ return(  exp( - (x-m)*(x-m)/(v*v) )) ; }

class C_Rule
{
public :

	double  in_mu ;//輸入的激發量
	double  con[_out_varl+1] ;//每個rule的後鑑部
	void    In_mem_fir(double *,int) ;//計算前鑑部的激發量
	friend  void Fir_str(double *, int,int) ;
	friend  void fuzzy(double *,int,int);//計算最終輸出

} ;

C_Rule  _Rule[_max_rule+1] ;


void C_Rule::In_mem_fir(double *in,int _rule) 
{
  
	int i ;
	in_mu =  1. ;
	
		for(i=1;i<= _in_varl;i++)
		{
/* 			if(in[i] < min_m[i] || in[i] > max_m[i])
			 {
			 	in_mu = in_mu * 1;
			 }
			 else */
			 //{
			 	in_mu = in_mu * phi(in[i] , fuzzy_real[(_rule-1)*_mem_length+i*2-1] , fuzzy_real[(_rule-1)*_mem_length+i*2] ) ;
				                //( 輸入  ，中心點                                   ,寬 )
				
			// }
			//in_mu = in_mu * phi(in[i] , fuzzy_real[(_rule-1)*_mem_length+i*2-1] , fuzzy_real[(_rule-1)*_mem_length+i*2] ) ;//20211215

		}
		//printf("rule[%d]_in_mu=%f\n",_rule,in_mu);
	//}
}




class controller : public rclcpp::Node{
	public:
////////////////////////////////////////////  Constructor  //////////////////////////////////////////////	
		controller() : Node("controller")
		{
			sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>("/scan", 1,std::bind(&controller::callback, this, std::placeholders::_1));
			pub_ = this->create_publisher<interfaces::msg::Output>("output",10);
            service_ = this->create_service<interfaces::srv::Command>("command", std::bind(&controller::Service_callback, this, std::placeholders::_1, std::placeholders::_2));
		}
		double minimun(int i, int j , int &k  )
		{  
			double laser_min=100;
			for (k=i ;k<=j;k++){
				
				if (laser_min>laser_temp[k])
					{ 
					laser_min=laser_temp[k];
					
					}
			}
			return laser_min;
		}
		void callback(const sensor_msgs::msg::LaserScan::SharedPtr scan) 
		{
			RCLCPP_INFO(this->get_logger(), "Received LaserScan message: %zu ranges", scan->ranges.size());
			int k;
			for(int i=1;i<=scan->ranges.size();i++) //ranges有720筆資料，每0.5度一筆
			{
				laser_temp_scan[i]=scan->ranges[i];
/* 				if (i==1 || i==45*2 || i==181 || i==135*2 || i==361 || i==225*2 || i==270*2 || i==631 ){
					printf("laser_temp_scan[%d]=%f\n",i,laser_temp_scan[i]);
				} */
			}
			for (int i = 0; i <= scan->ranges.size(); i++)
			{
				//double angle_rad = (scan->angle_min + i * scan->angle_increment) + M_PI;
				//double angle_deg = angle_rad*180/M_PI;
				
				//因為每0.5度一筆資料，所以laser_temp[i]中，i/2=角度
				//例如:laser_temp[10]是五度的距離
				if (!std::isinf(laser_temp_scan[i]))
				{
					laser_temp[i] = laser_temp_scan[i];
				}
				else{
					laser_temp[i] = 999;
				}		
			}
			/* 
 			for(int i=536;i<545;i+=2){
				printf("laser_temp[%d]=%f\n",i,laser_temp[i]);
			}
             */
			//laser_temp[195*2]為195度的距離
			
			left_min=minimun(195*2,269*2,k);
			right_min=minimun(90*2,165*2,k);
			stright_min=minimun(165*2,195*2,k);
		}
        void Service_callback(const std::shared_ptr<interfaces::srv::Command::Request> request, std::shared_ptr<interfaces::srv::Command::Response> response)
        {
            printf("sssssssssssssssssssssssssssssssssssssssssss");
            counts++;
            RCLCPP_INFO(this->get_logger(), "Incoming request");
            test_open(); //to get max and min
            decision_(counts);
            get_sensor(counts) ;
            sick_limit(counts) ;
            fuzzy_in(counts) ;					//輸入是雷測數值，只是調整fuzzy的input比例			
            Fir_str(in , _in_clu , ber) ;	//讀取後件部計算機發量
            fuzzy(_in_clu , ber);  		//解模糊產生out_y

            if (decision_left == 1) {
                response->a = out_y[1];
                response->b = out_y[2];
            } else if (decision_right == 1) {
                response->a = out_y[2];
                response->b = out_y[1];
            }
            printf("left=%d\t right=%d\n",decision_left,decision_right);
            printf("a=%lf \t b=%lf \n",out_y[1],out_y[2]);
            
        }

		void decision_(int j){
			//目前都用右沿牆
			decision_left = 0;
			decision_right= 1;
/* 			if (left_min<=right_min){
				decision_left = 1;
				decision_right= 0;
			}
			else if (left_min>right_min ){
				decision_left = 0;
				decision_right= 1;
			} */
			
			if(decision_left == 1 )
			{	
				std::cout << "LLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL" << std::endl;
				printf("left_min:  %lf\n",left_min);
				printf("right_min:  %lf\n",right_min);
				//找出每區域的最小值
				read_1 = far ;
				read_2 = far ;
				read_3 = far ;
				read_4 = far ;
				read_5 = far ;
				read_6 = far ;
				//179.5~201.5
				for(size_t i = 359; i <403; i++)
				{	if(read_4>laser_temp[i])
					read_4=std::min(laser_temp[i],far);
				}
				//printf("%d\n",1);
				//201.5~224
				for(size_t i = 403; i < 224*2; i++)
				{	if(read_3>laser_temp[i])
					read_3=std::min(laser_temp[i],far);
				}
				//printf("%d\n",2);
				//224~246
				for(size_t i = 224*2; i < 246*2; i++)
				{	if(read_2>laser_temp[i])
					read_2=std::min(laser_temp[i],far);
				}
				//printf("%d\n",3);
				//246~268.5
				for(size_t i = 246*2; i < 537; i++)
				{	if(read_1>laser_temp[i])
					read_1=std::min(laser_temp[i],far);
				}
				//printf("%d\n",4);
				//90~270
				for(size_t i = 90*2; i <= 270*2; i++)
				{	if(read_5>laser_temp[i])
					read_5=std::min(laser_temp[i],far);
				}			
				//printf("%d\n",5);
				//90~180
				for(size_t i = 90*2; i <= 180*2; i++)
				{	if(read_6>laser_temp[i])
					read_6=std::min(laser_temp[i],far);
				}
				printf("left_along_wall\n");
			}
			else if(decision_right == 1){
				std::cout << "RRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRRR" << std::endl;
				read_1 = far ;
				read_2 = far ;
				read_3 = far ;
				read_4 = far ;
				read_5 = far ;
				read_6 = far ;
				//158.5~180.5
				for(size_t i = 317; i <361; i++)
				{	if(read_4>laser_temp[i])
					read_4=std::min(laser_temp[i],far);
				}
				//printf("%d\n",1);
				//136~158.5
				for(size_t i = 136*2; i < 317; i++)
				{	if(read_3>laser_temp[i])
					read_3=std::min(laser_temp[i],far);
				}
				//printf("%d\n",2);
				//114~136
				for(size_t i = 114*2; i < 136*2; i++)
				{	if(read_2>laser_temp[i])
					{
					read_2=std::min(laser_temp[i],far);
					}	
				}
				//printf("%d\n",3);
				//91.5~114
				for(size_t i = 183; i < 228; i++)
				{	if(read_1>laser_temp[i]){
					
					read_1=std::min(laser_temp[i],far);
					}
				}
				//printf("%d\n",4);
				//90~270
				for(size_t i = 90*2; i <=270*2; i++)
				{	if(read_5>laser_temp[i])
					read_5=std::min(laser_temp[i],far);
				}			
				//printf("%d\n",5);
				//90~180
				for(size_t i = 90*2; i <= 180*2; i++)
				{	if(read_6>laser_temp[i])
					read_6=std::min(laser_temp[i],far);
				}
				printf("right_along_wall\n");	

			}
		}
				
///////////////////////////////  get distance value from different angles from sensor  ///////////////////////

		
	

////////////////////////////////////  Output cmd_velocity  ////////////////////////////////////
		void test_run(int j){
			auto msg = interfaces::msg::Output();
			msg.step = j;
			if(decision_left == 1 ){
				msg.a = out_y[1];
				msg.b = out_y[2];
			}
			if(decision_right == 1 ){
				msg.a = out_y[2];
				msg.b = out_y[1];
			}
			pub_->publish(msg);
            
		
			printf("left=%d\t right=%d\n",decision_left,decision_right);
            printf("a=%lf \t b=%lf \n",out_y[1],out_y[2]);		
		}


		void test_open(){

			for(int ssss=1; ssss<=_in_varl; ssss++)
			{
				min_m[ssss]=1.0e10;  //為了找最小值，所以初始的最小值令為1，即為"最大"的最小值。
				max_m[ssss]=1.0e-10;  //為了找最大值，所以初始的最大值令為0，即為"最小"的最大值。		
			////-1~1之間的中心點範圍 
			//max_m[ber][ssss]=0.0;  //為了找最大值，所以初始的最大值令為-1，即為"最小"的最大值。        
					
			}
			 
			FILE *fnoise1,*fnoise0;  
			printf("read file\n");		
			if((fnoise0=fopen(load_data_clu,"r"))==NULL)
			{
				printf("Nothing1\n");
				exit(1);
			}
			fscanf(fnoise0,"%d", &_in_clu);
			fclose(fnoise0);
			
			if((fnoise1=fopen(load_data_FC,"r"))==NULL)
			{
				printf("Nothing2\n");
				exit(1);
			}
			for(int i=1;i<=_mem_length*_in_clu;i++)
			{
				fscanf(fnoise1,"%Lf \n", &fuzzy_real[i]);
				//printf("check%Lf\t",fuzzy_real[i]);
			}  

			fclose(fnoise1);  

			for(int jj=1; jj<=_in_clu; jj++)
			{
				for(int jjj=1; jjj<=_in_varl; jjj++)
					{   
					if(fuzzy_real[(jj-1)*_mem_length+jjj*2-1] < min_m[jjj])
					{   
						min_m[jjj]=fuzzy_real[(jj-1)*_mem_length+jjj*2-1];
					}
					if(fuzzy_real[(jj-1)*_mem_length+jjj*2-1] > max_m[jjj])
					{
						max_m[jjj]=fuzzy_real[(jj-1)*_mem_length+jjj*2-1];
					}
				}

			}
			
		}
///////////////////////////////////  Open the file from leaning ///////////////////////////////

///////////////////////////////////  Save sensor data to another vector & print out  /////////////////////////
		void get_sensor(int jj)
		{
			sick_1[jj] = read_1;
			sick_2[jj] = read_2; 
			sick_3[jj] = read_3;
			sick_4[jj] = read_4;	
			sick_all[jj] = read_5;
			sick_wall[jj] = read_6;
			if(jj>=5){
	
				ave_distance_error = ave_distance_error + fabs(read_6-0.35);
				printf("distance_error == %f\n",fabs(read_6-0.35));
			}
/* 			robot_vel[jj] =out_y[1]+out_y[2]; //功能:左輪+右輪的輪速
			deviation_whirl=out_y[1]-out_y[2]; */
		}
		void sick_limit(int jj) 
		{ 
			double _min = 0.2;
			double _max = 1.5;  

			if (sick_1[jj] >= _max)
				sick_1[jj] = _max ;
			else if (sick_1[jj] < _min)
				sick_1[jj] = _min ;
			
			if (sick_2[jj] >= _max)
				sick_2[jj] = _max ;
			else if (sick_2[jj] < _min)
				sick_2[jj] = _min ;
			
			if (sick_3[jj] >= _max)
				sick_3[jj] = _max ;
			else if (sick_3[jj] < _min)
				sick_3[jj] = _min ;

			if (sick_4[jj] >= _max)
				sick_4[jj] = _max ;	
			else if (sick_4[jj] < _min)
				sick_4[jj] = _min ;

			if (sick_all[jj] >= _max)
				sick_all[jj] = _max ;
			else if (sick_all[jj] < _min)
				sick_all[jj] = _min ;
					
			if (sick_wall[jj] >= _max)
				sick_wall[jj] = _max ;
			else if (sick_wall[jj] < _min)
				sick_wall[jj] = _min ; 


			printf(" ~~~~~~~~~~~~~~~~~~~~~~~~~~ \n" ) ;
			
			printf("Sick 1 === %f\n" , 		sick_1[counts] );  
			printf("Sick 2 === %f\n" , 		sick_2[counts] );  
			printf("Sick 3 === %f\n" , 		sick_3[counts] );
			printf("Sick 4 === %f\n" , 		sick_4[counts] );
			printf("Sick wall === %f\n" ,sick_wall[counts] ) ;
			printf("~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~");
			printf("\n");
			printf("read_1 === %f\n" ,read_1 ) ;
			printf("read_2 === %f\n" ,read_2 ) ;
			printf("read_3 === %f\n" ,read_3 ) ;
			printf("read_4 === %f\n" ,read_4 ) ;
			printf("read_5 === %f\n" ,read_5 ) ;
			

			printf("\nleft_min===%f\n",left_min);
			printf("right_min===%f\n",right_min);
			printf("stright_min=%f\n",stright_min);

			FILE *read,*wall_distance;
			//存雷達的四個最小值
			read=fopen ("/home/user/ros2_obf_ws/src/read.txt","a");
			//存與牆距離
			wall_distance=fopen ("/home/user/ros2_obf_ws/src/wall_distance.txt","a");
			fprintf(read,"%f\n",read_1);
			fprintf(read,"%f\n",read_2);
			fprintf(read,"%f\n",read_3);
			fprintf(read,"%f\n",read_4);
			double min_read = read_1;
			if (min_read > read_2){
				min_read=read_2;
			}
			else if (min_read > read_3){
				min_read=read_3;
			}
			else if(min_read > read_4){
				min_read=read_4;
			}
			fprintf(wall_distance,"%f\n",min_read);
			distances_sum+=min_read;
			fclose(read);
			fclose(wall_distance);
		}
///////////////////////////////////  Save sensor data to another vector & print out  ///////////////////////////


//////////////////////////////////  Rescale sensor data for fuzzy  /////////////////////////////////////////
		void fuzzy_in(int jj)
		{ 

			in[1] = sick_1[jj] / 1.3 ; 
			in[2] = sick_2[jj] / 1.3 ;  
			in[3] = sick_3[jj] / 1.3 ;  
			in[4] = sick_4[jj] / 1.3 ;

			printf(" ~~~~~~~~~~~~~~~~~~~~~~~~~~ \n" ) ;
			
			printf("in[1] === %f\n" , 		in[1] );  
			printf("in[2] === %f\n" , 		in[2] );  
			printf("in[3] === %f\n" , 		in[3] );
			printf("in[4] === %f\n" , 		in[4] );
			FILE *input;
			input=fopen ("/home/user/ros2_obf_ws/src/input.txt","a");
			fprintf(input,"%f\n",in[1]);
			fprintf(input,"%f\n",in[2]);
			fprintf(input,"%f\n",in[3]);
			fprintf(input,"%f\n",in[4]);
			fclose(input);


	//		in[5] = sick_wall[jj] * _input_scale ;//平移再normalize//2017_8_2
			
			//for(int i=1;i<=5;i++)
				//printf("in[%d]= %lf\n",i,in[i]);		
		}
//////////////////////////////////  Rescale sensor data for fuzzy  /////////////////////////////////////////



///////////////////////////////// Calculate firing strengh  ////////////////////////////////////////////
		void Fir_str(double *in, int _rule, int ber)     
		{
			//這裡的ber，等於main函式內的ber，又ber的值為popsize的值。?
			//這裡的 _rule，等於設定變數的_in_clu的值，代表rule的數量
			//這裡的in，代表儲存輸入的陣列

			int j,k ;

			for(k=1; k<=_rule; k++)
			{
				for (j=1; j<=_out_varl; j++)
				{
					_Rule[k].con[j] = fuzzy_real[k*_mem_length-_out_varl+j];
				} // fuzzy_real為模糊控制器參數，這裡在讀取後鑑部的部分
				_Rule[k].In_mem_fir(in,k) ; // 計算前鑑部激發量
				
			}
		
		}
///////////////////////////////// Calculate firing strengh  ////////////////////////////////////////////

////////////////////////////////////// speed limit    //////////////////////////////////////
		void speed_limit()
		{
			if(out_y[1] >= max_speed)
				out_y[1] = max_speed;
			if(out_y[2] >= max_speed)
				out_y[2] = max_speed;
			if(out_y[1] <= -(max_speed))
				out_y[1] = -max_speed;
			if(out_y[2] <= -(max_speed))
				out_y[2] = -max_speed;
		}
////////////////////////////////////// speed limit    //////////////////////////////////////

///////////////////////////////////////  Defuzzifier  ///////////////////////////////////////
		void fuzzy(int _rule , int ber)  // 權重式平均法
		{
			int i , j;
			double den[_out_varl+1] , num[_out_varl+1] ;

			
			for (j=1; j<=_out_varl; j++)
			{
				den[j] = 0. ;
				num[j] = 0. ;
				
				for (i=1; i<=_rule; i++)
				{
					//if ( fuzzy_xrule[ber][i] == _true )
				//	{
						num[j] = num[j] + _Rule[i].in_mu * _Rule[i].con[j];
						den[j] = den[j] + _Rule[i].in_mu ;
				//	}
				}


				if ( fabs(den[j]) < 1e-8 )  //20211215
					out_y[j] = 0 ;
				else
					out_y[j] = num[j]/den[j] ;
				//printf("num[%d]=%f\tden[%d]=%f\n",j,num[j],j,den[j]);
			}
			
			printf("out_y1=%lf \t out_y2=%lf\n ",out_y[1],out_y[2]);

		}
///////////////////////////////////////  Defuzzifier  ///////////////////////////////////////u


//////////////////////////////////  Stop the car  ///////////////////////////////////////
		void stop(){
			if(counts==max_step)
				{
					auto msg = interfaces::msg::Output();
					msg.a = 0;
					msg.b = 0;

					pub_->publish(msg);
					
					//exit(1);
				}
		}
//////////////////////////////////  Stop the car  ///////////////////////////////////////

		void save_Average_distance(){
			FILE *Average_distance;
			Average_distance=fopen("/home/user/ros2_obf_ws/src/Average_distance.txt","a");//"controller/20210701/save_ave_dis_error.txt"
			if(Average_distance==NULL){
				printf("Fail to open file");
				exit (1);
			}
			fprintf(Average_distance,"%f\n",(distances_sum/counts));
			fclose(Average_distance);
		}
	

	private: 
		
		
		rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
		rclcpp::Publisher<interfaces::msg::Output>::SharedPtr pub_;	
        rclcpp::Service<interfaces::srv::Command>::SharedPtr service_;	

};

int main(int argc,char ** argv)
{
	rclcpp::init(argc, argv);
	controller ctrl;
	auto node = std::make_shared<controller>();
    rclcpp::spin(node);
	node->save_Average_distance();

	node->stop();
	rclcpp::shutdown();

    return 0;
}
