#include <windows.h>
#include <webots/robot.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <math.h>
#include <fstream>
#include <webots/robot.h>
#include <webots/lidar.h>

#include <C:/Program Files/Webots/include/controller/c/webots/motor.h>
#include <C:/Program Files/Webots/include/controller/c/webots/robot.h>
#include <C:/Program Files/Webots/include/controller/c/webots/gps.h>
#include <C:/Program Files/Webots/include/controller/c/webots/receiver.h>
#include <C:/Program Files/Webots/include/controller/c/webots/emitter.h>
#include <C:/Program Files/Webots/include/controller/c/webots/gyro.h>
#include <C:/Program Files/Webots/include/controller/c/webots/camera.h>
#include <C:/Program Files/Webots/include/controller/c/webots/distance_sensor.h>
#include <C:/Program Files/Webots/include/controller/c/webots/accelerometer.h>
#include <C:/Program Files/Webots/include/controller/c/webots/supervisor.h>
#include <C:/Program Files/Webots/include/controller/c/webots/inertial_unit.h>

// #define FC_file  "..\\..\\..\\27\\Final_first.dat"
// #define clu_file "..\\..\\..\\27\\Final_clu.dat"
#define FC_file  "..\\..\\..\\30-copy0906\\controllers\\MOFCACO_train\\s.txt"
#define clu_file "..\\..\\..\\30-copy0906\\controllers\\MOFCACO_train\\rule_s.txt"

//#define FC_file  "..\\..\\..\\1\\controllers\\MOFCACO_train\\oldp.dat"
//#define clu_file "..\\..\\..\\1\\controllers\\MOFCACO_train\\orule.dat"


#define TIME_STEP 50
#define NUM_SERVOS 18
#define _Maxstep 60000 
#define _count 1000
//-------------------------
void init_devices (void );
double ch_max (double );
//-------------------------
void cal_out_3(int );
void cal_out_3_sec(int );
void cal_out_3_walk(int);
void ch_wave3(int);
void save_3(int);
void create_cpg(void);
void turn(int);
void load_cpg(void);
//-------------------------
void initial(void);
double PID(double ,double);
//-------------------------
WbDeviceTag R_servo[NUM_SERVOS],gyro,accelerometer,inertialunit,R_Urg,SSensor;
void get_gyro(void);
void get_acce(void);
double get_iner_pitch(void);
double get_iner_roll(void);

const char *SERVOS[NUM_SERVOS]={
    "R00", "R01", "R02", "R10", "R11", "R12", "R20", "R21", "R22",
    "L00", "L01", "L02", "L10", "L11", "L12", "L20", "L21", "L22"   
};

int k=0,q=0;
int kk=0, kkk[7]={0}, aaa[7]={0}, judge[7]={0,2,2,2,2,2,2}, cc[7]={0};
double pi[7]={0}, ro[7]={0}, ladder[7]={0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
double roll, pitch,thres=0.03;
double temp=0,number[7]={0};
//double kp[7] = {0,-14,-16,-14,14,16,14};
//double kp[7] = {0,-12,-14,-12,12,14,12};
//double kp[7] = {0,-10,-12,-10,10,12,10};
//double kp[7] = {0,-15.5,-18,-15.5,15.5,18,15.5};
double kp[7] = {0,-14,-16.25,-14,14,16.25,14};
double kd[7] = {0,-0.2,-0.2,-0.2,0.2,0.2,0.2};

//********************************************for along the wall************************************
/// Constants
#define RANDMAX 32767
#define INFMAX  1.0e10
#define INFMIN  1.0e-10
#define INF 1.0e14
#define _pi  3.1415926
//#define random(x)   rand()%x

// For normalize
#define _uscale 1


/// For fuzzy controller
#define _in_varl  4         // 4 inputs
#define _out_varl   2     // 5 outputs
int _rule;        // 10 rules  //20220323
#define _max_rule 30
#define _gap    (2*_in_varl+_out_varl)  // gap of rule = (2*4+4)=12  //*2 cuz center & width

// Limit of FC parameters

// 1. Antecedent (using Gaussian fuzzy set that needs a center and a width)
// center = 0.1~1.0
#define _center_max 1.0
#define _center_min 0.0
// width = 0.05~0.3
#define _range_max  0.3
#define _range_min  0.05 

// 2. Consequence (Real number)
// consequence = -2~2
#define _conq_max   1.0
#define _conq_min   -1.0


/// For learning
#define _lpara  ((2*_in_varl+_out_varl)*_max_rule)  // 13*10 = 130

#define _nobj   1                   // the number of objects is 2 //20220323
#define _popsize    40         // a swarm contains 40 particles(粒子)
#define _maxgen 600        // 100 iteration //20220323
#define MAX_STEP    20200  // command amount
#define MAX_D   15              // least walking distance
#define kersin 0.85

// robot condition
#define COLLIDE -1
#define AWAY 1
#define FAIR 0

//For GA
#define  _pc  0.5
#define  _pm  0.1
// Ant
void update_ant(int);
double GaussDeviate(double, double);
 
// use the suggested value of thesis
#define   _eta_c    20      // crossover index  在crossover會用到
#define   _eta_m    20    // mutation index 在mutation會用到

// For binary, not used here
#define   _lbin     0
#define   _pc_bin   0    // binary crossover probability
#define   _pm_bin   0  // binary mutation probability

/// For webots
#define COMMUNICATION_CHANNEL 1     // communication channel for supervisor and controller
#define TIME_STEP 50                                        // control time step
#define NUM_SERVO 18                                   // the number of servos
//#define trans[3]
// divide URG area into 8 sections
// -90~90 -> -90, -67.5, -45, -22.5, 0, 22.5, 45, 67.5, 90
#define NUM_DS 8
//--------------------------------
#define _mem_length  2*_in_varl+ _out_varl    //**13
#define new_row 20  //*************


double DS=0;
int sw;
//非主導排序相關
int *temp1=NULL ;  //用在非主導排序的暫存變數
int *temp_sort ; // 用在非主導排序的暫存變數
int front_sort[2*_popsize+1] ;  // 用在非主導排序的暫存變數
int sorted_crowding[2*_popsize+1] ;// 用在非主導排序的暫存變數
int front_size ; // 用在非主導排序，排除front1後，統計該回合產生多少個Front的轉存變數

//多樣性相關
double temp_crowd[2*_popsize+1][_nobj+2] ;  //排列解的多樣性的暫存變數
int mixpop_sorted[2*_popsize+1] ; //排列子代解的多樣性的暫存變數


void update_newparent();

///////////////////////  排序相關  ////////////////////////{

void non_domination(int,int) ;  //非主導排序:排列解的好壞
void sort_on_obj(int,int) ;  //按照 cost value 將解排序
void sort_on_crowd(int , int) ; //排列解的多樣性
void sort_on_rank_crowd(int) ; //做排列子代解的多樣性

///////////////////////  排序相關  ////////////////////////}

int selection() ; //競爭法
void crossover(int , int , int) ; //交配
void mutation() ;    // 突變
void update_pop() ;   //解的更新

int elapsed=0;
double Urg[NUM_DS];
int load_gen=0;

int save_gen(int);
void load_gen_data();

void walk(int,double,double);
void robot_get_distance(double*);
void robot_get_position(double *);
int robot_state_judge(double *);
double randn(double , double );
double min_variable(int );
double max_variable(int );
double Gaussian(double, double, double);
int check = 0; //20220324
//********************************************for along the wall************************************ 


const double step = 0.2;
const double Wfe = -1.5; //Flexor&Extensor weight connection
const double t1 = 0.5;  //orig = 1
const double t2 = 7.5; //orig = 15
//const double U0 = 1.1;//3??????
const double U0 = 1.2;//4??????
const double b = 3;
const double Wij = -1;

using namespace std;

class OSC{
   
   friend class CPG;
   
   public:
          double dUe[_count+_Maxstep],dUf[_count+_Maxstep],dVe[_count+_Maxstep],dVf[_count+_Maxstep];
          double Ue[_count+_Maxstep],Uf[_count+_Maxstep],Ve[_count+_Maxstep],Vf[_count+_Maxstep],Ye[_count+_Maxstep],Yf[_count+_Maxstep],Y[_count+_Maxstep];
};

class CPG{

    friend class OSC;

    public:
            OSC osc[4+1];
            double deep[_Maxstep+1];
            double out[_Maxstep+1];
            double lpara,lpara2,height[_Maxstep+1], ttemp=0;
            int clad=1, no=0, ww=1;
};

CPG leg[6+1];

class individual{  //就是個體(一個控制器)，共有40個個體，給個個體都有以下這些參數
	friend class Front;
public:
    // a solution in PSO has:
    // 1. parameter
    // 2. fitness value/cost value (scored by object functions)
	double p[_lpara+1], fitness[_nobj+1], CT[3]; //_lpara=110 ,_nobj=2(目標個數) //20220323
    // 3. the rank and the crowding distance
           int frnt; //frnt就是他屬於哪個front
           double cd_len; //這是crowding distance
           
           int sp[2*_popsize+1] ;
           int nsp ;
           int sorted_front ;
           double dist[_nobj+2] ;
           int sorted_crowd ;
           int nnp ;
           int np[2*_popsize+1] ;
           int bin[_lbin];	
	void clear(void);
};
int select(individual* );
void individual :: clear(void){
   // Clear:
   // clear the rank and crowding distance
    cd_len=0.0;
    frnt=0;
    // set the cost value of particle as a large number
   for(int i=1; i<=_nobj; i++){ 
	   fitness[i]=INF;
   }
}

class Front{
	friend class individual;
public:
  int member[2*_popsize+1] ;
  int number ;
};

individual oldpop[_popsize+new_row+1], newpop[_popsize+new_row+1], mixpop[2*_popsize+1],temp_pop[2*_popsize+1];
Front  _front[2*_popsize+1] ;
// Fuzzy controller
class FCR{  //4個input有4*2=8個"中心點"+"寬"，output有3個後件部(左擺+右擺+基本速度)，加起來11個
            //因10條rule所以共110個 
	friend class individual;  //其中input為紅外線由90度~-180度切成4等份
        	                          //每等份的最小值當4個input
public:
    // 1. has 10 rules(110 parameters)
    // 2. each rule generates a firing strength
	double FCRule[_lpara+1], FCS[_max_rule+1];  //FCRule在存110個參數，FCS在存10條rule的激發量

	void getRule(double* );                    // get parameter
          	void Controller(double*, double* );        // main FC
	void Fuzzifier(double* );	        // fuzzifier
	void Defuzzifier(double* );	        // defuzzifier
};

void FCR :: getRule(double *p){  //把每隻鳥(控制器)的110個參數傳給模糊控制內
    // read parameters from the input array
	for(int i=1; i<=_gap*_rule; i++){
	  FCRule[i]=p[i];
	}
}

void FCR :: Controller(double* x, double* y){
    // Fuzzy Controller : fuzzifier and defuzzifier
    //controller同時執行兩個方程式(Fuzzifier和Defuzzifier)的意思
	Fuzzifier(x);
	Defuzzifier(y);
}

void FCR :: Fuzzifier(double *input){
    double k, center_max[_in_varl+1], center_min[_in_varl+1];

    // each input has a fuzzy set in each rule
    for(int i=1; i<=_in_varl; i++){ 
        center_max[i]=INFMIN;   //先傳超小的值給她去比大小，找出最大的中心點在哪
        //比如說有4個input，其中第1個input因為有10條rule，所以會有10個中心點
        //我去找出最大最小中心點在哪，在這個範圍外我就把她激發量給1
        center_min[i]=INFMAX;   //先傳超大值給她去比大小，找出最小的中心點在哪
        //同上觀念
        // find the maximum and the minimum centers of each input
        for(int j=1; j<=_rule; j++){  //_rule=10
            center_max[i]=(center_max[i]<FCRule[((_gap)*(j-1))+(2*(i-1))+1])?FCRule[((_gap)*(j-1))+(2*(i-1))+1]:center_max[i];
            center_min[i]=(center_min[i]>FCRule[((_gap)*(j-1))+(2*(i-1))+1])?FCRule[((_gap)*(j-1))+(2*(i-1))+1]:center_min[i];
        } //例如:FCRule[0],FCRule[11],FCRule[22],FCRule[33],FCRule[44],....FCRule[99]共10個       
    }     //再把這10個比大小，找出第一個輸入的10個fuzzyset最大和最小的中心點，下一步再把她拉平


    // generate firing strength (for each rule)
    for(int i=1; i<=_rule; i++){ //這邊開始是每條rule個別去看了，因為要去算mu值和激發量，(做10次)
            k=1.0;  //因為我是用product求激發量，所以一開始K給1

            // for each input
    	        for(int j=1; j<=_in_varl; j++){
    	        //這邊是算4個input進去得出的mu值連乘4次得到激發量
                    // if center = cmin or cmax, set u(x) = 1.0
             	     /*if((input[j]>=center_max[j])||(input[j]<=center_min[j])){
                            k*=1;}  
                            //當我的輸入比最大的中心點大，比最小的中心點還小，那我的mu值就設為1
                   // else: u(x) = gaussian membership function
                            else{
                                 k*=Gaussian(input[j], FCRule[((_gap)*(i-1))+(2*(j-1))+1], FCRule[((_gap)*(i-1))+(2*(j-1))+2]);}    */ //20220323  
                    k*=Gaussian(input[j], FCRule[((_gap)*(i-1))+(2*(j-1))+1], FCRule[((_gap)*(i-1))+(2*(j-1))+2]);    //20220323  
				}    //上面分別為(位置   ，  中心點                 ，寬)  (input是紅外線掃到的最小值)
                   FCS[i]=k;   // 儲存4個mu值連乘後得到的激發量(共10個)
    }
}

void FCR :: Defuzzifier(double* output){
    double den[_out_varl+1], num[_out_varl+1];

    // a consequence generates a real output
	for(int i=1; i<=_out_varl; i++){ //在求後件部，所以迴圈3次即可
		den[i]=0.0;  //每條rule每次要計算之前先歸0而以
  		num[i]=0.0;  //每條rule每次要計算之前先歸0而以

		// for each rule
		for(int j=1; j<=_rule; j++){ //共10條rule所以迴圈要10次
                                  // sum of firing strength
			den[i]+=FCS[j];  //總共10個firing strength加起來當分母

			// 激發量*後件部再相加
			num[i]+=(FCS[j]*FCRule[((_gap)*(j-1))+(2*_in_varl)+i]); 
  		} //例如FCRule[8](Rule1的1號後件部值)*激發量+FCRule[19](Rule2的1號後件部值)*激發量+FCRule[30](Rule3的1號後件部值)*激發量...
  		
    	       //output[i]=(num[i]/den[i]>_out_max)?_out_max:(num[i]/den[i]<_out_min)?_out_min:num[i]/den[i];
		output[i]=(fabs(den[i]) == 0)?0:(num[i]/den[i]); //20220323
	}          //3個後件部書出值output被算出來，如果分母太小輸出就強迫=0，如果正常就用Zmom解模糊
	//printf("%f %f \n",output[0],output[1]);
}

/// Data
class  Data{
    public  :
    void    input(int,double* ) ;
    void    output(int, double* ) ;
} ;


//Data這邊的input在取得雷射值當輸入
//在fuzzifier和Defuzzifier用到input的就是他們(雷射掃到的4個最小值)
void  Data :: input(int num_count,double* in){
    double urgg[4] = {0};
    printf("Urg[0]=%lf\n",Urg[0]);
    for (int i=0; i<4; i++){
      if (Urg[i]<0.2) {urgg[i]=0.2;}
      else if (Urg[i]>1.5){urgg[i]=1.5;}
      else {(urgg[i]=Urg[i]);}  
      urgg[i]=urgg[i]/1.3;
    }
    
    in[1]=urgg[0];
    in[2]=urgg[1];
    in[3]=urgg[2];
    in[4]=urgg[3];
}

void  Data :: output(int num_count,double *out){ //output就是解模糊後的3個值(左擺幅，右擺幅，基本速度)
    double l_adj=0, r_adj=0;
    //double *outt[6];
    
    l_adj = out[1];
    r_adj = out[2];
    
    //printf("\n %f %f %f \n", out[0], out[1], out[2]);

    // send output to robot
    walk(num_count, l_adj, r_adj);
    //cal_out_3_walk
}

void initialization(void );//產生初始隨機值給前件部(中心點+寬)和後件部
void generation(int );     //在算PSO的那些公式
void Eva_Fitness(individual* );
void punish(individual*, int );
void checkpop(double* );  //單純再確認一次p式在上下界範圍內而以
void gen_old_pop(Front*, individual* );
void mix_population(void );


int main(int argc, char **argv)
{
  wb_robot_init();
  srand(time(NULL));  
  init_devices();
  // load_gen_data();  
  
  // if (load_gen<=_maxgen){
  
     // if (load_gen==1)
     // {
       // create_cpg();
       // initialization();
       // generation(load_gen) ;
       // load_gen = save_gen(load_gen);
       // wb_supervisor_world_reload();
     // }
     // else
     // {
       // create_cpg();
       // generation(load_gen) ;
       // load_gen = save_gen(load_gen);
       // fflush(stdout);
       // wb_robot_step(TIME_STEP);
       // wb_supervisor_world_reload();
     // }
     
  // }
   // create_cpg();
    load_cpg(); //20230401
    generation(3) ;
       
  wb_robot_cleanup();
  return 0;
}

//********************************************************
double ch_max (double a){

  if (a>=0){ return a; }
  
  else {return 0;}
 
}
//********************************************************

void create_cpg(void){
  initial();  
  ch_wave3(_count);
  save_3(_count);
  cal_out_3_sec(_count);
  turn(_Maxstep);
}

void load_cpg(void){

	for (int i=0; i<NUM_SERVOS; i++){
		wb_motor_set_position(R_servo[i], 0);
	}
	wb_robot_step(500);

	FILE *YYout11, *YYout21, *YYout31, *YYout41, *YYout51, *YYout61; 
	FILE *YYout12, *YYout22, *YYout32, *YYout42, *YYout52, *YYout62;
	FILE *YYout13, *YYout23, *YYout33, *YYout43, *YYout53, *YYout63;
	FILE *YYout14, *YYout24, *YYout34, *YYout44, *YYout54, *YYout64;

	YYout11 = fopen ("fixed_cpg\\YYout11.txt","r");
	YYout21 = fopen ("fixed_cpg\\YYout21.txt","r");
	YYout31 = fopen ("fixed_cpg\\YYout31.txt","r");
	YYout41 = fopen ("fixed_cpg\\YYout41.txt","r");
	YYout51 = fopen ("fixed_cpg\\YYout51.txt","r");
	YYout61 = fopen ("fixed_cpg\\YYout61.txt","r");

	YYout12 = fopen ("fixed_cpg\\YYout12.txt","r");
	YYout22 = fopen ("fixed_cpg\\YYout22.txt","r");
	YYout32 = fopen ("fixed_cpg\\YYout32.txt","r");
	YYout42 = fopen ("fixed_cpg\\YYout42.txt","r");
	YYout52 = fopen ("fixed_cpg\\YYout52.txt","r");
	YYout62 = fopen ("fixed_cpg\\YYout62.txt","r");

	YYout13 = fopen ("fixed_cpg\\YYout13.txt","r");
	YYout23 = fopen ("fixed_cpg\\YYout23.txt","r");
	YYout33 = fopen ("fixed_cpg\\YYout33.txt","r");
	YYout43 = fopen ("fixed_cpg\\YYout43.txt","r");
	YYout53 = fopen ("fixed_cpg\\YYout53.txt","r");
	YYout63 = fopen ("fixed_cpg\\YYout63.txt","r");
	
	YYout14 = fopen ("fixed_cpg\\YYout14.txt","r");
	YYout24 = fopen ("fixed_cpg\\YYout24.txt","r");
	YYout34 = fopen ("fixed_cpg\\YYout34.txt","r");
	YYout44 = fopen ("fixed_cpg\\YYout44.txt","r");
	YYout54 = fopen ("fixed_cpg\\YYout54.txt","r");
	YYout64 = fopen ("fixed_cpg\\YYout64.txt","r");	

	for (int count=1; count<=_Maxstep; count++){
		for (int i=1; i<=6; i++){
			for (int j=1; j<=4; j++){
				if( j==1 || j==2 || j==3){
					if(i==1){
						if(j==1){
							fscanf(YYout11,"%lf\n",&(leg[i].osc[j].Y[count]));
                        }
                        if(j==2){
							fscanf(YYout12,"%lf\n",&(leg[i].osc[j].Y[count]));
                        }
                        if(j==3){
							fscanf(YYout13,"%lf\n",&(leg[i].osc[j].Y[count]));
                        }
					}
					else if(i==2){
						if(j==1){
							fscanf(YYout21,"%lf\n",&(leg[i].osc[j].Y[count]));
                        }
                        if(j==2){
							fscanf(YYout22,"%lf\n",&(leg[i].osc[j].Y[count]));
                        }
                        if(j==3){
							fscanf(YYout23,"%lf\n",&(leg[i].osc[j].Y[count]));
                        }
					}
					else if(i==3){
						if(j==1){
							fscanf(YYout31,"%lf\n",&(leg[i].osc[j].Y[count]));
                        }
                        if(j==2){
							fscanf(YYout32,"%lf\n",&(leg[i].osc[j].Y[count]));
                        }
                        if(j==3){
							fscanf(YYout33,"%lf\n",&(leg[i].osc[j].Y[count]));
                        }
					}
					else if(i==4){
						if(j==1){
							fscanf(YYout41,"%lf\n",&(leg[i].osc[j].Y[count]));
                        }
                        if(j==2){
							fscanf(YYout42,"%lf\n",&(leg[i].osc[j].Y[count]));
                        }
                        if(j==3){
							fscanf(YYout43,"%lf\n",&(leg[i].osc[j].Y[count]));
                        }
					}
					else if(i==5){
						if(j==1){
							fscanf(YYout51,"%lf\n",&(leg[i].osc[j].Y[count]));
                        }
                        if(j==2){
							fscanf(YYout52,"%lf\n",&(leg[i].osc[j].Y[count]));
                        }
                        if(j==3){
							fscanf(YYout53,"%lf\n",&(leg[i].osc[j].Y[count]));
                        }
					}
					else if(i==6){
						if(j==1){
							fscanf(YYout61,"%lf\n",&(leg[i].osc[j].Y[count]));
                        }
                        if(j==2){
							fscanf(YYout62,"%lf\n",&(leg[i].osc[j].Y[count]));
                        }
                        if(j==3){
							fscanf(YYout63,"%lf\n",&(leg[i].osc[j].Y[count]));
                        }
					}
				}
				else
				{
					if(i==1){
						fscanf(YYout14,"%lf\n",&(leg[i].osc[j].Y[count]));
					}
					else if(i==2){
						fscanf(YYout24,"%lf\n",&(leg[i].osc[j].Y[count]));
					}
					else if(i==3){
						fscanf(YYout34,"%lf\n",&(leg[i].osc[j].Y[count]));
					}
					else if(i==4){
						fscanf(YYout44,"%lf\n",&(leg[i].osc[j].Y[count]));
					}
					else if(i==5){
						fscanf(YYout54,"%lf\n",&(leg[i].osc[j].Y[count]));
					}
					else if(i==6){
						fscanf(YYout64,"%lf\n",&(leg[i].osc[j].Y[count]));
					}
				}
			}
		}
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
	
	fclose(YYout14);
	fclose(YYout24);
	fclose(YYout34);
	fclose(YYout44);
	fclose(YYout54);
	fclose(YYout64);

	turn(_Maxstep);
}
void turn(int num_count){

  for(int kk=0; kk<=num_count; kk++){
    //-------------------------------inorder to control the knee permanently positive------------------------ 
    for (int i=1; i<=6; i++){
      if(leg[i].osc[2].Y[kk]<=0){
         leg[i].osc[2].Y[kk] = 0;
      }   
      
      if(leg[i].osc[4].Y[kk]<=0){
         leg[i].osc[4].Y[kk] = 0;      
      }
    }
   //-------------------------------inorder to control the knee permanently positive------------------------ 
   }
   for(int kk=0; kk<=num_count; kk++){
   //--------------------------------inorder to force realism the same amid webots--------------------------------
     for (int ii=1; ii<=6; ii++){
     
      if(ii==1 || ii==2 || ii==3){
        leg[ii].osc[3].Y[kk] = -leg[ii].osc[3].Y[kk];
      }
                    
       else if(ii==4 || ii==5 || ii==6){
         for(int jj=1; jj<=4; jj++){
          if(jj==1||jj==2||jj==4){
            //printf("before=%lf\t",leg[ii].osc[jj].Y[num_count]);
            leg[ii].osc[jj].Y[kk] = -leg[ii].osc[jj].Y[kk];
           //printf("after=%lf\n",leg[ii].osc[jj].Y[num_count]);
            //fflush(stdout);
          }
         }
       }
    }
  }    
   //--------------------------------inorder to force realism the same amid webots--------------------------------  
  
}


/////////////////////////////////////////////////////////////////////////////-------------Tripoied gait-------------/////////////////////////////////////////////////////////////////////////////
void cal_out_3(int num_count){


	int count = 0;
	
//-------------------------------------------------------------initialize----------------------------------------------
	for(int i=1; i<=6; i++){
		for(int j=1; j<=3;j++){
			for(int k=0; k<=_count; k++){
				leg[i].osc[j].dUe[k]=0; leg[i].osc[j].Ue[k]=0;
				leg[i].osc[j].dUf[k]=0;	leg[i].osc[j].Uf[k]=0;			
				leg[i].osc[j].dVe[k]=0; leg[i].osc[j].Ve[k]=0;
				leg[i].osc[j].dVf[k]=0; leg[i].osc[j].Vf[k]=0;
				leg[i].osc[j].Ye[k]=0;  leg[i].osc[j].Yf[k]=0, leg[i].osc[j].Y[k]=0;
			}
		}
	}
//-------------------------------------------------------------initialize----------------------------------------------

//-------------------------------------------------------------random--------------------------------------------------	
	srand(time(NULL));

	for(int i=1; i<=6; i++){
		for (int j=1; j<=3; j++){
			leg[i].osc[j].Ue[1] = 0;
			leg[i].osc[j].Ve[1] = 0;
			do{
			leg[i].osc[j].Uf[1] =  (double(rand()%10)/10+0.1);
			leg[i].osc[j].Vf[1] =  (double(rand()%10)/10+0.1);			
			}while(leg[i].osc[j].Uf[1] == leg[i].osc[j].Vf[1]);
		}
	}
//-------------------------------------------------------------random--------------------------------------------------    
	
	for (count=1; count<=num_count; count++){
			 
		for (int i=1; i<=6; i++){

			int a=(i+1)%6;		if(a==0){a=6;}
			int aa=(i+2)%6;		if(aa==0){aa=6;}
			int aaa=(i+3)%6;		if(aaa==0);{aaa=6;}
			int aaaa=(i+4)%6;		if(aaaa==0){aaaa=6;}
			int aaaaa=(i+5)%6;		if(aaaaa==0){aaaaa=6;}
			//ex. i==1?,a=2,aa=3,aaa=5,aaaa=6
			//printf("a=%d\taa=%d\taaa=%d\taaaa=%d\n",a,aa,aaa,aaaa);
			for (int j=1; j<=4; j++){
			
                                        if(j==1 || j==2 || j==3){
				int k=(j+1)%3;  if(k==0){k=3;}
				int kk=(j+2)%3; if(kk==0){kk=3;}
				//ex. j==2?,k=3,kk=1
					
				//**********************************Extensor neuron******************************************
				leg[i].osc[j].dUe[count] = (-leg[i].osc[j].Ue[count] + (Wfe * leg[i].osc[j].Yf[count]) - (b * leg[i].osc[j].Ve[count]) + U0 + (Wij * (ch_max(leg[i].osc[k].Ye[count]) + ch_max(leg[i].osc[kk].Ye[count]) + 
				ch_max(leg[a].osc[j].Ye[count]) + ch_max(leg[aaaaa].osc[j].Ye[count])))) / t1;//??????? - ch_max(leg[aa].osc[j].Ye[count]) - ch_max(leg[aaaa].osc[j].Ye[count])??paper?excite,??????3?,????2?
				leg[i].osc[j].Ue[count + 1] = leg[i].osc[j].Ue[count] + (step * leg[i].osc[j].dUe[count]);
				leg[i].osc[j].Ye[count + 1] = max(0.00, leg[i].osc[j].Ue[count + 1]);
	
				leg[i].osc[j].dVe[count] = (-leg[i].osc[j].Ve[count] + leg[i].osc[j].Ye[count + 1]) / t2;
				leg[i].osc[j].Ve[count + 1] = leg[i].osc[j].Ve[count] + (step * leg[i].osc[j].dVe[count]);
				//**********************************Extensor neuron******************************************
	
				//**********************************Flexor neuron******************************************** 
				leg[i].osc[j].dUf[count] = (-leg[i].osc[j].Uf[count] + (Wfe * leg[i].osc[j].Ye[count]) - (b * leg[i].osc[j].Vf[count]) + U0 + (Wij * (ch_max(leg[i].osc[k].Yf[count]) + ch_max(leg[i].osc[kk].Yf[count]) + 
				ch_max(leg[a].osc[j].Yf[count]) + ch_max(leg[aaaaa].osc[j].Yf[count])))) / t1;//??????? - ch_max(leg[aa].osc[j].Ye[count]) - ch_max(leg[aaaa].osc[j].Ye[count])??paper?excite,??????3?,????2?
				leg[i].osc[j].Uf[count + 1] = leg[i].osc[j].Uf[count] + (step * leg[i].osc[j].dUf[count]);
				leg[i].osc[j].Yf[count + 1] = max(0.00, leg[i].osc[j].Uf[count + 1]);
	
				leg[i].osc[j].dVf[count] = (-leg[i].osc[j].Vf[count] + leg[i].osc[j].Yf[count + 1]) / t2;
				leg[i].osc[j].Vf[count + 1] = leg[i].osc[j].Vf[count] + (step * leg[i].osc[j].dVf[count]);
				//**********************************Flexor neuron********************************************
				
				leg[i].osc[j].Y[count] = leg[i].osc[j].Yf[count]-leg[i].osc[j].Ye[count];
				
				if(j==2){
					if(leg[i].osc[j].Y[count]<=0){leg[i].osc[j].Y[count]=0;}
				}
                                        }
                                        
                                        else{
				int k=1;
				int kk=3;
				//ex. j==2?,k=3,kk=1
					
				//**********************************Extensor neuron******************************************
				leg[i].osc[j].dUe[count] = (-leg[i].osc[j].Ue[count] + (Wfe * leg[i].osc[j].Yf[count]) - (b * leg[i].osc[j].Ve[count]) + U0 + (Wij * (ch_max(leg[i].osc[k].Ye[count]) + ch_max(leg[i].osc[kk].Ye[count]) + 
				ch_max(leg[a].osc[j].Ye[count]) + ch_max(leg[aaaaa].osc[j].Ye[count])))) / t1;//??????? - ch_max(leg[aa].osc[j].Ye[count]) - ch_max(leg[aaaa].osc[j].Ye[count])??paper?excite,??????3?,????2?
				leg[i].osc[j].Ue[count + 1] = leg[i].osc[j].Ue[count] + (step * leg[i].osc[j].dUe[count]);
				leg[i].osc[j].Ye[count + 1] = max(0.00, leg[i].osc[j].Ue[count + 1]);

				leg[i].osc[j].dVe[count] = (-leg[i].osc[j].Ve[count] + leg[i].osc[j].Ye[count + 1]) / t2;
				leg[i].osc[j].Ve[count + 1] = leg[i].osc[j].Ve[count] + (step * leg[i].osc[j].dVe[count]);
				//**********************************Extensor neuron******************************************
	
				//**********************************Flexor neuron******************************************** 
				leg[i].osc[j].dUf[count] = (-leg[i].osc[j].Uf[count] + (Wfe * leg[i].osc[j].Ye[count]) - (b * leg[i].osc[j].Vf[count]) + U0 + (Wij * (ch_max(leg[i].osc[k].Yf[count]) + ch_max(leg[i].osc[kk].Yf[count]) + 
				ch_max(leg[a].osc[j].Yf[count]) + ch_max(leg[aaaaa].osc[j].Yf[count])))) / t1;//??????? - ch_max(leg[aa].osc[j].Ye[count]) - ch_max(leg[aaaa].osc[j].Ye[count])??paper?excite,??????3?,????2?
				leg[i].osc[j].Uf[count + 1] = leg[i].osc[j].Uf[count] + (step * leg[i].osc[j].dUf[count]);
				leg[i].osc[j].Yf[count + 1] = max(0.00, leg[i].osc[j].Uf[count + 1]);
	
				leg[i].osc[j].dVf[count] = (-leg[i].osc[j].Vf[count] + leg[i].osc[j].Yf[count + 1]) / t2;
				leg[i].osc[j].Vf[count + 1] = leg[i].osc[j].Vf[count] + (step * leg[i].osc[j].dVf[count]);
				//**********************************Flexor neuron********************************************
				
				leg[i].osc[j].Y[count] = leg[i].osc[j].Yf[count]-leg[i].osc[j].Ye[count];
				
				if(leg[i].osc[j].Y[count]<=0){leg[i].osc[j].Y[count]=0;}                                       
                                        }
			}
                             leg[i].deep[count] = leg[i].osc[4].Y[count] - leg[i].osc[2].Y[count];		
		}
	}
		
}
/////////////////////////////////////////////////////////////////////////////-------------Tripoied gait-------------/////////////////////////////////////////////////////////////////////////////

void cal_out_3_sec(int num_count){

  	FILE *YYout11, *YYout21, *YYout31, *YYout41, *YYout51, *YYout61; 
	FILE *YYout12, *YYout22, *YYout32, *YYout42, *YYout52, *YYout62;
	FILE *YYout13, *YYout23, *YYout33, *YYout43, *YYout53, *YYout63;
	FILE *YYout14, *YYout24, *YYout34, *YYout44, *YYout54, *YYout64;	
	FILE *Deep1, *Deep2, *Deep3, *Deep4, *Deep5, *Deep6; 
	
	YYout11 = fopen ("YYout11.txt","a");
	YYout21 = fopen ("YYout21.txt","a");
	YYout31 = fopen ("YYout31.txt","a");
	YYout41 = fopen ("YYout41.txt","a");
	YYout51 = fopen ("YYout51.txt","a");
	YYout61 = fopen ("YYout61.txt","a");

	YYout12 = fopen ("YYout12.txt","a");
	YYout22 = fopen ("YYout22.txt","a");
	YYout32 = fopen ("YYout32.txt","a");
	YYout42 = fopen ("YYout42.txt","a");
	YYout52 = fopen ("YYout52.txt","a");
	YYout62 = fopen ("YYout62.txt","a");

	YYout13 = fopen ("YYout13.txt","a");
	YYout23 = fopen ("YYout23.txt","a");
	YYout33 = fopen ("YYout33.txt","a");
	YYout43 = fopen ("YYout43.txt","a");
	YYout53 = fopen ("YYout53.txt","a");
	YYout63 = fopen ("YYout63.txt","a");
	
	YYout14 = fopen ("YYout14.txt","a");
	YYout24 = fopen ("YYout24.txt","a");
	YYout34 = fopen ("YYout34.txt","a");
	YYout44 = fopen ("YYout44.txt","a");
	YYout54 = fopen ("YYout54.txt","a");
	YYout64 = fopen ("YYout64.txt","a");	
	
	Deep1 = fopen ("Deep1.txt","a");
	Deep2 = fopen ("Deep2.txt","a");
	Deep3 = fopen ("Deep3.txt","a");
	Deep4 = fopen ("Deep4.txt","a");
	Deep5 = fopen ("Deep5.txt","a");
	Deep6 = fopen ("Deep6.txt","a");
	
	for (int count=num_count; count<=_Maxstep; count++){
                   //printf("------------>count=%d\n",count);

		pitch = get_iner_pitch();
                       roll = get_iner_roll();
                       if (fabs(pitch)<0.03){pitch = 0;}
                       if (fabs(roll)<0.03){roll = 0;}
                       //printf("pitch=%lf\t roll=%lf\n",pitch,roll);
                       
		for (int i=1; i<=6; i++){
                                                                   
			int a=(i+1)%6;		if(a==0){a=6;}
			int aa=(i+2)%6;		if(aa==0){aa=6;}
			int aaa=(i+3)%6;		if(aaa==0);{aaa=6;}
			int aaaa=(i+4)%6;		if(aaaa==0){aaaa=6;}
			int aaaaa=(i+5)%6;		if(aaaaa==0){aaaaa=6;}

			//----------------------------------------calculate feed--------------------------------
			double feed=0;
  
                                  if(i==1){
                                    feed = (pitch + roll) * 0.707;
                                  }
                                  
                                  else if(i==2){
                                    feed = roll;
                                  }
                                  
                                  else if(i==3){
                                    feed = (-pitch + roll) * 0.707;
                                  }
                                  
                                  else if(i==4){
                                    feed = (-pitch - roll) * 0.707;
                                  }
                                                                    
                                  else if(i==5){
                                    feed = -roll;
                                  }
                                  
                                  else if(i==6){
                                    feed = (pitch - roll) * 0.707;
                                  }
                                  
                                  if(count<=1050){feed = 0;} //to avoid -nan

                                  //feed = 0;
                                  //printf("i = %d\tfeed = %lf\n",i,feed);
                                  //----------------------------------------calculate feed--------------------------------
                                                            			
			for (int j=1; j<=4; j++){
    			      
                                        if( j==1 || j==2 || j==3){
                                        
				int k=(j+1)%3;  if(k==0){k=3;}
				int kk=(j+2)%3; if(kk==0){kk=3;}
				
				//**********************************Extensor neuron******************************************
				leg[i].osc[j].dUe[count] = (-leg[i].osc[j].Ue[count] + (Wfe * leg[i].osc[j].Yf[count]) - (b * leg[i].osc[j].Ve[count]) + U0 + (Wij * (ch_max(leg[i].osc[k].Ye[count]) + ch_max(leg[i].osc[kk].Ye[count]) + 
				ch_max(leg[a].osc[j].Ye[count]) + ch_max(leg[aaaaa].osc[j].Ye[count])))) / t1;				
				leg[i].osc[j].Ue[count + 1] = leg[i].osc[j].Ue[count] + (step * leg[i].osc[j].dUe[count]);
				leg[i].osc[j].Ye[count + 1] = max(0.00, leg[i].osc[j].Ue[count + 1]);
	
				leg[i].osc[j].dVe[count] = (-leg[i].osc[j].Ve[count] + leg[i].osc[j].Ye[count + 1]) / t2;
				leg[i].osc[j].Ve[count + 1] = leg[i].osc[j].Ve[count] + (step * leg[i].osc[j].dVe[count]);
				//**********************************Extensor neuron******************************************
	
				//**********************************Flexor neuron******************************************** 
				leg[i].osc[j].dUf[count] = (-leg[i].osc[j].Uf[count] + (Wfe * leg[i].osc[j].Ye[count]) - (b * leg[i].osc[j].Vf[count]) + U0 + (Wij * (ch_max(leg[i].osc[k].Yf[count]) + ch_max(leg[i].osc[kk].Yf[count]) + 
				ch_max(leg[a].osc[j].Yf[count]) + ch_max(leg[aaaaa].osc[j].Yf[count])))) / t1;
				leg[i].osc[j].Uf[count + 1] = leg[i].osc[j].Uf[count] + (step * leg[i].osc[j].dUf[count]);
				leg[i].osc[j].Yf[count + 1] = max(0.00, leg[i].osc[j].Uf[count + 1]);
	
				leg[i].osc[j].dVf[count] = (-leg[i].osc[j].Vf[count] + leg[i].osc[j].Yf[count + 1]) / t2;
				leg[i].osc[j].Vf[count + 1] = leg[i].osc[j].Vf[count] + (step * leg[i].osc[j].dVf[count]);
				//**********************************Flexor neuron********************************************
				
				leg[i].osc[j].Y[count] = leg[i].osc[j].Yf[count]-leg[i].osc[j].Ye[count];	
				
                                             //-----------------save the individual 6 feet's 1 & 2 & 3leg--------------------------{
                                             if(i==1){
                                             
                                                  if(j==1){
					fprintf(YYout11,"%lf\n",leg[i].osc[j].Y[count]);
                                                  }
                                                  if(j==2){
					fprintf(YYout12,"%lf\n",leg[i].osc[j].Y[count]);
                                                  }
                                                  if(j==3){
					fprintf(YYout13,"%lf\n",leg[i].osc[j].Y[count]);
                                                  }
                                             }
		  
                                             else if(i==2){
                                             
                                                    if(j==1){
					fprintf(YYout21,"%lf\n",leg[i].osc[j].Y[count]);
                                                    }
                                                    if(j==2){
					fprintf(YYout22,"%lf\n",leg[i].osc[j].Y[count]);
                                                    }
                                                    if(j==3){
					fprintf(YYout23,"%lf\n",leg[i].osc[j].Y[count]);
                                                    }
                                             }
		  
                                             else if(i==3){
                                             
                                                    if(j==1){
					fprintf(YYout31,"%lf\n",leg[i].osc[j].Y[count]);
                                                    }
                                                    if(j==2){
					fprintf(YYout32,"%lf\n",leg[i].osc[j].Y[count]);
                                                    }
                                                    if(j==3){
    					fprintf(YYout33,"%lf\n",leg[i].osc[j].Y[count]);
                                                    }
                                             }
                  
                                             else if(i==4){
                                             
                                                    if(j==1){
					fprintf(YYout41,"%lf\n",leg[i].osc[j].Y[count]);
                                                    }
                                                    if(j==2){
					fprintf(YYout42,"%lf\n",leg[i].osc[j].Y[count]);
                                                    }
                                                    if(j==3){
					fprintf(YYout43,"%lf\n",leg[i].osc[j].Y[count]);
                                                    }
                                             }
                                             
                                             else if(i==5){
                                             
                                                    if(j==1){
					fprintf(YYout51,"%lf\n",leg[i].osc[j].Y[count]);
                                                    }
                                                    if(j==2){
					fprintf(YYout52,"%lf\n",leg[i].osc[j].Y[count]);
                                                    }
                                                    if(j==3){
					fprintf(YYout53,"%lf\n",leg[i].osc[j].Y[count]);
                                                    }
                                             }                                             
                                             
                                             
                                             else if(i==6){
                                             
                                                    if(j==1){
					fprintf(YYout61,"%lf\n",leg[i].osc[j].Y[count]);
                                                    }
                                                    if(j==2){
					fprintf(YYout62,"%lf\n",leg[i].osc[j].Y[count]);
                                                    }
                                                    if(j==3){
					fprintf(YYout63,"%lf\n",leg[i].osc[j].Y[count]);
                                                    }
                                             }
                                             ////-----------------save the individual 6 feet's 1 & 2 & 3leg--------------------------}
                                        }
                                        
                                        else{
                                              
				int k=1;  
				int kk=3;

				//**********************************Extensor neuron******************************************
				leg[i].osc[j].dUe[count] = (feed-leg[i].osc[j].Ue[count] + (Wfe * leg[i].osc[j].Yf[count]) - (b * leg[i].osc[j].Ve[count]) + U0 + (Wij * (ch_max(leg[i].osc[k].Ye[count]) + ch_max(leg[i].osc[kk].Ye[count]) + 
				ch_max(leg[a].osc[j].Ye[count]) + ch_max(leg[aaaaa].osc[j].Ye[count])))) / t1;				
				leg[i].osc[j].Ue[count + 1] = leg[i].osc[j].Ue[count] + (step * leg[i].osc[j].dUe[count]);
				leg[i].osc[j].Ye[count + 1] = max(0.00, leg[i].osc[j].Ue[count + 1]);
	
				leg[i].osc[j].dVe[count] = (-leg[i].osc[j].Ve[count] + leg[i].osc[j].Ye[count + 1]) / t2;
				leg[i].osc[j].Ve[count + 1] = leg[i].osc[j].Ve[count] + (step * leg[i].osc[j].dVe[count]);
				//**********************************Extensor neuron******************************************
	
				//**********************************Flexor neuron******************************************** 
				leg[i].osc[j].dUf[count] = (feed-leg[i].osc[j].Uf[count] + (Wfe * leg[i].osc[j].Ye[count]) - (b * leg[i].osc[j].Vf[count]) + U0 + (Wij * (ch_max(leg[i].osc[k].Yf[count]) + ch_max(leg[i].osc[kk].Yf[count]) + 
				ch_max(leg[a].osc[j].Yf[count]) + ch_max(leg[aaaaa].osc[j].Yf[count])))) / t1;
				leg[i].osc[j].Uf[count + 1] = leg[i].osc[j].Uf[count] + (step * leg[i].osc[j].dUf[count]);
				leg[i].osc[j].Yf[count + 1] = max(0.00, leg[i].osc[j].Uf[count + 1]);
	
				leg[i].osc[j].dVf[count] = (-leg[i].osc[j].Vf[count] + leg[i].osc[j].Yf[count + 1]) / t2;
				leg[i].osc[j].Vf[count + 1] = leg[i].osc[j].Vf[count] + (step * leg[i].osc[j].dVf[count]);
				//**********************************Flexor neuron********************************************
				
				leg[i].osc[j].Y[count] = leg[i].osc[j].Yf[count]-leg[i].osc[j].Ye[count];
				

                                            //-----------------save the individual 6 feet's 4th leg--------------------------
                                             if(i==1){
					fprintf(YYout14,"%lf\n",leg[i].osc[j].Y[count]);
                                             }
		  
                                             else if(i==2){
					fprintf(YYout24,"%lf\n",leg[i].osc[j].Y[count]);
                                             }
		  
                                             else if(i==3){                                           
					fprintf(YYout34,"%lf\n",leg[i].osc[j].Y[count]);
                                             }
                  
                                             else if(i==4){                                             
					fprintf(YYout44,"%lf\n",leg[i].osc[j].Y[count]);
                                             }
                                             
                                             else if(i==5){                                             
					fprintf(YYout54,"%lf\n",leg[i].osc[j].Y[count]);
                                             }                                       
                                                                                          
                                             else if(i==6){                                            
					fprintf(YYout64,"%lf\n",leg[i].osc[j].Y[count]);
                                             }
                                             //-----------------save the individual 6 feet's 4th leg--------------------------
                                             /*if(i==1){
                                               printf("leg[%d].osc[%d].Y[%d] = %lf\n",i,2,count,leg[i].osc[2].Y[count]);
                                               printf("leg[%d].osc[%d].Y[%d] = %lf\n",i,4,count,leg[i].osc[4].Y[count]);
                                             }*/
                                        }
                        			
			}
			
                            leg[i].deep[count] = (leg[i].osc[4].Y[count] - leg[i].osc[2].Y[count]);
                            //printf("deep--->leg[%d].deep[%d] = %lf\n",i,count,leg[i].deep[count]);
                            
                           /*if(i==1){
                              printf("leg[%d].deep[%d] = %lf\n",i,count, leg[i].deep[count]);         
                            }      */                       
                                      
                            if(i==1){fprintf(Deep1,"%lf\n",leg[i].deep[count]);}
                            else if(i==2){fprintf(Deep2,"%lf\n",leg[i].deep[count]);}
                            else if(i==3){fprintf(Deep3,"%lf\n",leg[i].deep[count]);}
                            else if(i==4){fprintf(Deep4,"%lf\n",leg[i].deep[count]);}
                            else if(i==5){fprintf(Deep5,"%lf\n",leg[i].deep[count]);}
                            else{fprintf(Deep6,"%lf\n",leg[i].deep[count]);}                            
		}

                       //cal_out_3_walk(count);   
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
	
	fclose(YYout14);
	fclose(YYout24);
	fclose(YYout34);
	fclose(YYout44);
	fclose(YYout54);
	fclose(YYout64);
	
	fclose(Deep1);
	fclose(Deep2);
	fclose(Deep3);
	fclose(Deep4);
	fclose(Deep5);
	fclose(Deep6);	
		
}
 //********************************************************3gait********************************************   
 
void cal_out_3_walk(int num_count){
    
    //-------------------------------inorder to control the knee permanently positive------------------------ 
    for (int i=1; i<=6; i++){
      if(leg[i].osc[2].Y[num_count]<=0){
         leg[i].osc[2].Y[num_count] = 0;
      }   
      
      if(leg[i].osc[4].Y[num_count]<=0){
         leg[i].osc[4].Y[num_count] = 0;      
      }
    }
   //-------------------------------inorder to control the knee permanently positive------------------------ 
   
   //--------------------------------inorder to force realism the same amid webots--------------------------------
     for (int ii=1; ii<=6; ii++){
     
      if(ii==1 || ii==2 || ii==3){
        leg[ii].osc[3].Y[num_count] = -leg[ii].osc[3].Y[num_count];
      }
                    
       else if(ii==4 || ii==5 || ii==6){
         for(int jj=1; jj<=4; jj++){
          if(jj==1||jj==2||jj==4){
            leg[ii].osc[jj].Y[num_count] = -leg[ii].osc[jj].Y[num_count];
          }
         }
       }
    }
   //--------------------------------inorder to force realism the same amid webots--------------------------------  
        //--------------------------------------------------------
        for(int j=1; j<=6; j++){
          leg[j].height[num_count]=leg[j].height[num_count-1];
          number[j]=fabs(leg[j].height[num_count]);
        }
        for(int a=1; a<=5; a=a+2){
          for(int b=a; b<=5; b=b+2){                  
            if( number[b] > number[a] ) {
              temp = number[b];
              number[b] = number[a];
              number[a] = temp;
            }
          }
        }
              
        for(int a=2; a<=6; a=a+2){
          for(int b=a; b<=6; b=b+2){                  
            if( number[b] > number[a] ) {
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
        for (int i=0; i<NUM_SERVOS; i=i+3){
        ///*
        if (i==0||i==3||i==6){
          int j=0;
          if(i==0){j=1;}
          else if(i==3){j=2;}
          else{j=3;}
          
          wb_motor_set_position(R_servo[i], leg[j].osc[1].Y[num_count]);
//------------------------------------------------------------------------------------
          if(leg[j].osc[2].Y[num_count]>0){
          
            if(leg[j].ww==1){
              if(leg[j].no>=10){
                judge[j]=1;
                leg[j].no=0;
                leg[j].ww=2;
              }
              else{
                judge[j]=2;
                leg[j].ww=2;
                leg[j].no=0;
                leg[j].height[num_count-1]=0;
              }
            }
            
            //--------inorder to make the robot smoothly   
            leg[j].height[num_count] = leg[j].height[num_count-1];                   
            kkk[j]++;            
            if(kkk[j]!=2){                
              if(leg[j].osc[2].Y[num_count]<leg[j].height[num_count]){
                //printf("First-->height-upupupup=%lf\n",leg[1].height[num_count]);              
                wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]);
                kkk[j] = 0;
              }           
              else {
                wb_motor_set_position(R_servo[i+1], leg[j].osc[2].Y[num_count]);
                //printf("First-->out-upupupup=%lf\n",leg[1].osc[2].Y[num_count]);
                kkk[j] = 1;
              }              
            }
            //--------inorder to make the robot smoothly             
            else{
              if(leg[j].osc[2].Y[num_count]>leg[j].height[num_count]){
                wb_motor_set_position(R_servo[i+1], leg[j].osc[2].Y[num_count]);
                //printf("Second-->out-upupupup=%lf\n",leg[1].osc[2].Y[num_count]);
              }
              else{
                wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]);
                //printf("Second-->height-upupupup=%lf\n",leg[1].height[num_count]);
              }            
              kkk[j]=1;
            }
            leg[j].clad=1;  
            
          }
//------------------------------------------------------------------------------------
          else if(leg[j].osc[2].Y[num_count]<=0){
                
            leg[j].height[num_count]=leg[j].height[num_count-1];

            //*************************************condition 1*************************************              
            if(judge[j]==1){//no need to adjust

              if(j==1 || j==3){
                if(number[5]>=0){
                  wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]-number[5]);
                  printf("out[%d]=%lf - NNNNNNNNNNNNNN\t",j,leg[j].height[num_count]-number[5]);
                }
                else{
                  wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]+number[5]);
                  printf("out[%d]=%lf - NNNNNNNNNNNNNN\t",j,leg[j].height[num_count]+number[5]);
                }
              }
              else{
                if(number[6]>=0){
                  wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]-number[6]);
                  printf("out[%d]=%lf - NNNNNNNNNNNNNN\t",j,leg[j].height[num_count]-number[6]);
                }
                else{
                  wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]+number[6]);
                  printf("out[%d]=%lf - NNNNNNNNNNNNNN\t",j,leg[j].height[num_count]+number[6]);
                }
                
              }
              
              ////wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]);            
              //printf(">>out[%d] = %lf NONONONONONO\t",j,leg[j].height[num_count]);
            }
            //*************************************condition 1************************************* 
            //*************************************condition 2************************************* 
             else if(judge[j]==2){
             
             
                if(leg[j].deep[num_count]*leg[j].lpara + (leg[j].deep[num_count]-leg[j].deep[num_count-1])*leg[j].lpara2 >= leg[j].out[num_count-1]){
                    leg[j].out[num_count] = leg[j].deep[num_count]*leg[j].lpara + (leg[j].deep[num_count]-leg[j].deep[num_count-1])*leg[j].lpara2;                 
                }
                else{
                  leg[j].out[num_count] = leg[j].out[num_count-1];
                }

                //if(pitch==0 && roll==0){
                if(fabs(pitch)<=thres && fabs(roll)<=thres){
                  leg[j].out[num_count-1] = leg[j].ttemp;
                  leg[j].out[num_count] = leg[j].out[num_count-1];
                }                               
                else if(leg[j].out[num_count]>=0.6){leg[j].out[num_count] = 0.6;}
                leg[j].height[num_count] = leg[j].out[num_count];
                
                /*
                wb_motor_set_position(R_servo[i+1], leg[j].out[num_count]);
                printf("out[%d]=%lf - yyyyyyyyyyyyyy\t", j,leg[j].out[num_count]);
                */

                //*
                if(leg[j].clad==1){
                  if(leg[j].height[num_count]>ladder[2]){
                    wb_motor_set_position(R_servo[i+1], ladder[2]);
                    leg[j].ttemp = ladder[2];
                    leg[j].clad=2;
                    printf("1realout[%d]=%lf\t",j,ladder[2]);
                  }
                  else{
                    wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]);
                    leg[j].ttemp = leg[j].height[num_count];
                    leg[j].clad=1;
                    printf("1realout[%d]=%lf\t",j,leg[j].height[num_count]);
                  }                  
                }
                
                else if(leg[j].clad==2){
                  if(leg[j].height[num_count]>ladder[3]){
                    wb_motor_set_position(R_servo[i+1], ladder[3]);
                    leg[j].ttemp = ladder[3];
                    leg[j].clad=3;
                    printf("2realout[%d]=%lf\t",j,ladder[3]);
                  }
                  else{
                    wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]);
                    leg[j].ttemp = leg[j].height[num_count];
                    leg[j].clad=2;  
                    printf("2realout[%d]=%lf\t",j,leg[j].height[num_count]);
                  }
                }             
                
                else if(leg[j].clad==3){
                  if(leg[j].height[num_count]>ladder[4]){
                    wb_motor_set_position(R_servo[i+1], ladder[4]);
                    leg[j].ttemp = ladder[4];
                    leg[j].clad=4;
                    printf("3realout[%d]=%lf\t",j,ladder[4]);
                  }
                  else{
                    wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]);
                    leg[j].ttemp = leg[j].height[num_count];
                    leg[j].clad=3;
                    printf("3realout[%d]=%lf\t",j,leg[j].height[num_count]);
                  }
                }      
                
                else if(leg[j].clad==4){
                  if(leg[j].height[num_count]>ladder[5]){
                    wb_motor_set_position(R_servo[i+1], ladder[5]);
                    leg[j].ttemp = ladder[5];
                    leg[j].clad=5;
                    printf("4realout[%d]=%lf\t",j,ladder[5]);
                  }
                  else{
                    wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]);
                    leg[j].ttemp = leg[j].height[num_count];
                    leg[j].clad=4;
                    printf("4realout[%d]=%lf\t",j,leg[j].height[num_count]);
                  }
                }                                   

                else if(leg[j].clad==5){
                  if(leg[j].height[num_count]>ladder[6]){
                    wb_motor_set_position(R_servo[i+1], ladder[6]);
                    leg[j].ttemp = ladder[6];
                    leg[j].clad=5;
                    printf("5realout[%d]=%lf\t",j, ladder[6]);
                  }
                  else{
                    wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]);
                    leg[j].ttemp = leg[j].height[num_count];
                    leg[j].clad=5;
                    printf("5realout[%d]=%lf\t",j,leg[j].height[num_count]);
                  }
                }
                //*/                
                kkk[j]=0;  
             }
             //*************************************condition 2************************************* 
            //-------------------------------------                       
            pi[j] = get_iner_pitch();
            ro[j] = get_iner_roll();

            if(fabs(pi[j])<thres && fabs(ro[j])<thres){//no need to adjust
              leg[j].no++;             
            }
            //printf("judge[0]=%d\n",judge[0]);
            //-------------------------------------
            leg[j].ww=1;
            kkk[j]=0;
          }
//------------------------------------------------------------------------------------
          //wb_motor_set_position(R_servo[i+2], -leg[j].osc[3].Y[num_count]);
        }
        //*/
        ///*                                                 
         if (i==9||i==12||i==15){
          int j=0;
          if(i==9){j=6;}
          else if(i==12){j=5;}
          else{j=4;}
          
          wb_motor_set_position(R_servo[i], leg[j].osc[1].Y[num_count]);
//------------------------------------------------------------------------------------
          if(leg[j].osc[2].Y[num_count]<0){

            if(leg[j].ww==1){
              if(leg[j].no>=10){
                judge[j]=1;
                leg[j].no=0;
                leg[j].ww=2;
              }
              else{
                judge[j]=2;
                leg[j].ww=2;
                leg[j].no=0;
                leg[j].height[num_count-1]=0;
              }
            }
            
            leg[j].height[num_count] = leg[j].height[num_count-1];                      
            kkk[j]++;            
            if(kkk[j]!=2){                                
                   
              if(leg[j].osc[2].Y[num_count]>leg[j].height[num_count]){
                //printf("First-->height-upupupup=%lf\n",leg[1].height[num_count]);              
                wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]);
                kkk[j] = 0;
                //printf("out[%d] = %lf\t",j,leg[j].height[num_count]);
              }           
              else{
                wb_motor_set_position(R_servo[i+1], leg[j].osc[2].Y[num_count]);
                //printf("First-->out-upupupup=%lf\n",leg[1].osc[2].Y[num_count]);
                kkk[j] = 1;
                //printf("out[%d] = %lf\t",j,leg[j].osc[2].Y[num_count]);
              }
              
            }
            //--------inorder to make the robot smoothly 
            
            else{

              //printf("height!!!!!!!!=%lf\n",leg[1].height[num_count]);
                         
              if(leg[j].osc[2].Y[num_count]<leg[j].height[num_count]){
                wb_motor_set_position(R_servo[i+1], leg[j].osc[2].Y[num_count]);
                //printf("out[%d] = %lf\t",j,leg[j].osc[2].Y[num_count]);
                //printf("Second-->out-upupupup=%lf\n",leg[1].osc[2].Y[num_count]);
              }
              else{
                wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]);
                //printf("out[%d] = %lf\t",j,leg[j].height[num_count]);
                //printf("Second-->height-upupupup=%lf\n",leg[1].height[num_count]);
              }
              
              kkk[j]=1;
            }      
            leg[j].clad=1;      
          }
//------------------------------------------------------------------------------------
          else if(leg[j].osc[2].Y[num_count]>=0){

               
            leg[j].height[num_count]=leg[j].height[num_count-1];

            //*************************************condition 1*************************************              
            if(judge[j]==1){//no need to adjust
              

              ///*
              if(j==5){
                if(number[5]>=0){
                  wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]+number[5]);
                  printf("out[%d]=%lf - NNNNNNNNNNNNNN\t",j,leg[j].height[num_count]+number[5]);
                }
                else{
                  wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]-number[5]);
                  printf("out[%d]=%lf - NNNNNNNNNNNNNN\t",j,leg[j].height[num_count]-number[5]);
                }
              }
              else{
                if(number[6]>=0){
                  wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]+number[6]);
                  printf("out[%d]=%lf - NNNNNNNNNNNNNN\t",j,leg[j].height[num_count]+number[6]);
                }
                else{
                  wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]-number[6]);
                  printf("out[%d]=%lf - NNNNNNNNNNNNNN\t",j,leg[j].height[num_count]-number[6]);
                }              
              }
              
              //*/
              ////wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]);
              //printf(">>out[%d] = %lf NONONONONONO\t",j,leg[j].height[num_count]);
            }
            //*************************************condition 1************************************* 
             
            //*************************************condition 2*************************************     
            else if(judge[j]==2){//start to adjust            
                if(leg[j].deep[num_count]*leg[j].lpara + (leg[j].deep[num_count]-leg[j].deep[num_count-1])*leg[j].lpara2 <= leg[j].out[num_count-1]){
                    leg[j].out[num_count] = leg[j].deep[num_count]*leg[j].lpara + (leg[j].deep[num_count]-leg[j].deep[num_count-1])*leg[j].lpara2;
                }
                else{
                  leg[j].out[num_count] = leg[j].out[num_count-1];
                }

                //if(pitch==0 && roll==0){
                if(fabs(pitch)<=thres && fabs(roll)<=thres){
                  leg[j].out[num_count-1] = leg[j].ttemp;
                  leg[j].out[num_count] = leg[j].out[num_count-1];
                }
                else if(leg[j].out[num_count]<=-0.6){leg[j].out[num_count] = -0.6;}
                leg[j].height[num_count] = leg[j].out[num_count];
                
                /*
                wb_motor_set_position(R_servo[i+1], leg[j].out[num_count]);
                printf("out[%d]=%lf - yyyyyyyyyyyyyy\t", j,leg[j].out[num_count]);
                */

                //*
                if(leg[j].clad==1){
                  if(fabs(leg[j].height[num_count])>ladder[2]){
                    wb_motor_set_position(R_servo[i+1], -ladder[2]);
                    leg[j].ttemp = -ladder[2];
                    leg[j].clad=2;
                    printf("1realout[%d]=%lf\t",j,-ladder[2]);
                  }
                  else{
                    wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]);
                    leg[j].ttemp = leg[j].height[num_count];
                    leg[j].clad=1;
                    printf("1realout[%d]=%lf\t",j,leg[j].height[num_count]);
                  }                  
                }
                
                else if(leg[j].clad==2){
                  if(fabs(leg[j].height[num_count])>ladder[3]){
                    wb_motor_set_position(R_servo[i+1], -ladder[3]);
                    leg[j].ttemp = -ladder[3];
                    leg[j].clad=3;
                    printf("2realout[%d]=%lf\t",j,-ladder[3]);
                  }
                  else{
                    wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]);
                    leg[j].ttemp = leg[j].height[num_count];
                    leg[j].clad=2;
                    printf("2realout[%d]=%lf\t",j,leg[j].height[num_count]);
                  }
                }
                
                else if(leg[j].clad==3){
                  if(fabs(leg[j].height[num_count])>ladder[4]){
                    wb_motor_set_position(R_servo[i+1], -ladder[4]);
                    leg[j].ttemp = -ladder[4];
                    leg[j].clad=4;
                    printf("3realout[%d]=%lf\t",j,-ladder[4]);
                  }
                  else{
                    wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]);
                    leg[j].ttemp = leg[j].height[num_count];
                    leg[j].clad=3;
                    printf("3realout[%d]=%lf\t",j,leg[j].height[num_count]);
                  }
                }
                
                else if(leg[j].clad==4){
                  if(fabs(leg[j].height[num_count])>ladder[5]){
                    wb_motor_set_position(R_servo[i+1], -ladder[5]);
                    leg[j].ttemp = -ladder[5];
                    leg[j].clad=5;
                    printf("4realout[%d]=%lf\t",j,-ladder[5]);
                  }
                  else{
                    wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]);
                    leg[j].ttemp = leg[j].height[num_count];
                    leg[j].clad=4;
                    printf("4realout[%d]=%lf\t",j,leg[j].height[num_count]);
                  }
                }

                else if(leg[j].clad==5){
                  if(fabs(leg[j].height[num_count])>ladder[6]){
                    wb_motor_set_position(R_servo[i+1], -ladder[6]);
                    leg[j].ttemp = -ladder[6];
                    leg[j].clad=5;
                    printf("5realout[%d]=%lf\t",j,-ladder[6]);
                  }
                  else{
                    wb_motor_set_position(R_servo[i+1], leg[j].height[num_count]);
                    leg[j].ttemp = leg[j].height[num_count];
                    leg[j].clad=5;
                    printf("5realout[%d]=%lf\t",j,leg[j].height[num_count]);
                  }
                }
                //*/ 
                kkk[j]=0;           
          }
          //*************************************condition 2*************************************
            //-------------------------------------                       
            pi[j] = get_iner_pitch();
            ro[j] = get_iner_roll();

            if(fabs(pi[j])<thres && fabs(ro[j])<thres){//no need to adjust
              leg[j].no++;             
            }
            //printf("judge[0]=%d\n",judge[0]);
            //-------------------------------------
            leg[j].ww=1;
            kkk[j]=0;  
          }        
          //printf("leg[j].deep[%d] = %lf\tdeep[%d]*kp = %lf\tdelta->deep[%d]*kd = %lf\tleg[j].out[%d] = %lf\n",num_count,leg[j].deep[num_count],num_count,leg[j].deep[num_count]*leg[j].lpara,num_count,leg[j].deep[num_count]-leg[j].deep[num_count-1]*leg[j].lpara2,num_count,leg[j].out[num_count]);
//------------------------------------------------------------------------------------
          //wb_motor_set_position(R_servo[i+2], -leg[j].osc[3].Y[num_count]);
        }
        //*/
    }
    wb_robot_step(100);
  
}
 //********************************************************3gait********************************************************

void ch_wave3(int num_count){

    int a=1;
    
      do{
        if(a==1){
          cal_out_3(num_count);
        }

      //*********************************************************************************************** 
        for (int i=400; i<=num_count; i++){
        
          //??----------?????>??-------?????<??---------??
          if(leg[1].osc[1].Y[i-1]!=0 && leg[1].osc[2].Y[i-1]!=0 && leg[1].osc[1].Y[i-1]>leg[1].osc[2].Y[i-1]&& leg[1].osc[1].Y[i]<leg[1].osc[2].Y[i]){
                a=1; 
                /*printf("Error\n");
                wb_robot_step(10);*/
                break;
          }
          else if(fabs(leg[1].osc[1].Y[i])>0.7 || fabs(leg[2].osc[1].Y[i])>0.7 || fabs(leg[3].osc[1].Y[i])>0.7 || fabs(leg[4].osc[1].Y[i])>0.7 || fabs(leg[5].osc[1].Y[i])>0.7 || fabs(leg[6].osc[1].Y[i])>0.7){
                a=1;
                break;
          }
          else if(fabs(leg[1].osc[2].Y[i])>0.7 || fabs(leg[2].osc[2].Y[i])>0.7 || fabs(leg[3].osc[2].Y[i])>0.7 || fabs(leg[4].osc[2].Y[i])>0.7 || fabs(leg[5].osc[2].Y[i])>0.7 || fabs(leg[6].osc[2].Y[i])>0.7){
                a=1;
                break;
          }
          else if(fabs(leg[1].osc[3].Y[i])>0.7 || fabs(leg[2].osc[3].Y[i])>0.7 || fabs(leg[3].osc[3].Y[i])>0.7 || fabs(leg[4].osc[3].Y[i])>0.7 || fabs(leg[5].osc[3].Y[i])>0.7 || fabs(leg[6].osc[3].Y[i])>0.7){
                a=1;
                break;
          }
          //??----------?????<??-------?????>??---------??
          else if(leg[1].osc[1].Y[i-1]!=0 && leg[1].osc[2].Y[i-1]!=0 && leg[1].osc[1].Y[i-1]<leg[1].osc[2].Y[i-1]&& leg[1].osc[1].Y[i]>leg[1].osc[2].Y[i]){
                a=0; 
                /*printf("Correct\n");
                wb_robot_step(10);   */             
                break;   
          }
        }
      //***********************************************************************************************                     
      }while (a==1); 
    	
}

void save_3(int num_count){

	FILE *YYout11, *YYout21, *YYout31, *YYout41, *YYout51, *YYout61; 
	FILE *YYout12, *YYout22, *YYout32, *YYout42, *YYout52, *YYout62;
	FILE *YYout13, *YYout23, *YYout33, *YYout43, *YYout53, *YYout63;
	FILE *YYout14, *YYout24, *YYout34, *YYout44, *YYout54, *YYout64;		
	FILE *Deep1, *Deep2, *Deep3, *Deep4, *Deep5, *Deep6; 	

	YYout11 = fopen ("YYout11.txt","w");
	YYout21 = fopen ("YYout21.txt","w");
	YYout31 = fopen ("YYout31.txt","w");
	YYout41 = fopen ("YYout41.txt","w");
	YYout51 = fopen ("YYout51.txt","w");
	YYout61 = fopen ("YYout61.txt","w");

	YYout12 = fopen ("YYout12.txt","w");
	YYout22 = fopen ("YYout22.txt","w");
	YYout32 = fopen ("YYout32.txt","w");
	YYout42 = fopen ("YYout42.txt","w");
	YYout52 = fopen ("YYout52.txt","w");
	YYout62 = fopen ("YYout62.txt","w");

	YYout13 = fopen ("YYout13.txt","w");
	YYout23 = fopen ("YYout23.txt","w");
	YYout33 = fopen ("YYout33.txt","w");
	YYout43 = fopen ("YYout43.txt","w");
	YYout53 = fopen ("YYout53.txt","w");
	YYout63 = fopen ("YYout63.txt","w");

	YYout14 = fopen ("YYout14.txt","w");
	YYout24 = fopen ("YYout24.txt","w");
	YYout34 = fopen ("YYout34.txt","w");
	YYout44 = fopen ("YYout44.txt","w");
	YYout54 = fopen ("YYout54.txt","w");
	YYout64 = fopen ("YYout64.txt","w");

	Deep1 = fopen ("Deep1.txt","w");
	Deep2 = fopen ("Deep2.txt","w");
	Deep3 = fopen ("Deep3.txt","w");
	Deep4 = fopen ("Deep4.txt","w");
	Deep5 = fopen ("Deep5.txt","w");
	Deep6 = fopen ("Deep6.txt","w");
	
	for (int i=1; i<num_count; i++){
                  for (int j=1; j<=6; j++){
		  if(j==1){
			for(int k=1; k<=4; k++){
				if(k==1){
					fprintf(YYout11,"%lf\n",leg[j].osc[k].Y[i]);
				}
				if(k==2){
					fprintf(YYout12,"%lf\n",leg[j].osc[k].Y[i]);
				}
				if(k==3){
					fprintf(YYout13,"%lf\n",leg[j].osc[k].Y[i]);
				}
				if(k==4){
					fprintf(YYout14,"%lf\n",leg[j].osc[k].Y[i]);
				}								
			}
		  }
		  else if(j==2){
			for(int k=1; k<=4; k++){
				if(k==1){
					fprintf(YYout21,"%lf\n",leg[j].osc[k].Y[i]);
				}
				if(k==2){
					fprintf(YYout22,"%lf\n",leg[j].osc[k].Y[i]);
				}
				if(k==3){
					fprintf(YYout23,"%lf\n",leg[j].osc[k].Y[i]);
				}
				if(k==4){
					fprintf(YYout24,"%lf\n",leg[j].osc[k].Y[i]);
				}				
			}		
		  }
		  else if(j==3){
			for(int k=1; k<=4; k++){
				if(k==1){
					fprintf(YYout31,"%lf\n",leg[j].osc[k].Y[i]);
				}
				if(k==2){
					fprintf(YYout32,"%lf\n",leg[j].osc[k].Y[i]);
				}
				if(k==3){
					fprintf(YYout33,"%lf\n",leg[j].osc[k].Y[i]);
				}
				if(k==4){
					fprintf(YYout34,"%lf\n",leg[j].osc[k].Y[i]);
				}
			}		
		  }
		  else if(j==4){
			for(int k=1; k<=4; k++){
				if(k==1){
					fprintf(YYout41,"%lf\n",leg[j].osc[k].Y[i]);
				}
				if(k==2){
					fprintf(YYout42,"%lf\n",leg[j].osc[k].Y[i]);
				}
				if(k==3){
					fprintf(YYout43,"%lf\n",leg[j].osc[k].Y[i]);
				}
				if(k==4){
					fprintf(YYout44,"%lf\n",leg[j].osc[k].Y[i]);
				}
			}		
		  }
		  else if(j==5){
			for(int k=1; k<=4; k++){
				if(k==1){
					fprintf(YYout51,"%lf\n",leg[j].osc[k].Y[i]);
				}
				if(k==2){
					fprintf(YYout52,"%lf\n",leg[j].osc[k].Y[i]);
				}
				if(k==3){
					fprintf(YYout53,"%lf\n",leg[j].osc[k].Y[i]);
				}
				if(k==4){
					fprintf(YYout54,"%lf\n",leg[j].osc[k].Y[i]);
				}	
			}		
		  }
		  else if(j==6){
			for(int k=1; k<=4; k++){
				if(k==1){
					fprintf(YYout61,"%lf\n",leg[j].osc[k].Y[i]);
				}
				if(k==2){
					fprintf(YYout62,"%lf\n",leg[j].osc[k].Y[i]);
				}
				if(k==3){
					fprintf(YYout63,"%lf\n",leg[j].osc[k].Y[i]);
				}
				if(k==4){
					fprintf(YYout64,"%lf\n",leg[j].osc[k].Y[i]);
				}
			}		
		  }
                  }
                  fprintf(Deep1,"%lf\n",leg[1].deep[i]);
                  fprintf(Deep2,"%lf\n",leg[2].deep[i]);
                  fprintf(Deep3,"%lf\n",leg[3].deep[i]);
                  fprintf(Deep4,"%lf\n",leg[4].deep[i]);
                  fprintf(Deep5,"%lf\n",leg[5].deep[i]);
                  fprintf(Deep6,"%lf\n",leg[6].deep[i]);                  
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
	
	fclose(YYout14);
	fclose(YYout24);
	fclose(YYout34);
	fclose(YYout44);
	fclose(YYout54);
	fclose(YYout64);	
		
	fclose(Deep1);
	fclose(Deep2);
	fclose(Deep3);
	fclose(Deep4);
	fclose(Deep5);
	fclose(Deep6);
	
}

////////////////////////////////////////////////////////////////////////////
void walk(int x, double a ,double b){
    /*
    a=1;
    b=1;
    //*/
    int c=1,d=0;//if i wanna let ankle move, then d=1 
    printf("a=%lf\tb=%lf\n",a,b);
    /*if(a<=0 && b<=0){
      a=0;
      b=0;
      c=0;
      printf("wrong gait ,force it not moving\n");
    }*/

    if(x<=200){
      for (int i=0; i<NUM_SERVO; i=i+1){
         wb_motor_set_position(R_servo[i],0);
      }
    }

    else{
      for (int i=0; i<NUM_SERVO; i=i+1){
        
          if (i==0){
            wb_motor_set_position(R_servo[i],   leg[1].osc[1].Y[x]*a);
            wb_motor_set_position(R_servo[i+1], leg[1].osc[2].Y[x]*c);
            wb_motor_set_position(R_servo[i+2],-leg[1].osc[3].Y[x]*d);//-
          }

          if (i==3){
            wb_motor_set_position(R_servo[i],   leg[2].osc[1].Y[x]*a);
            wb_motor_set_position(R_servo[i+1], leg[2].osc[2].Y[x]*c);
            wb_motor_set_position(R_servo[i+2],-leg[2].osc[3].Y[x]*d);
          }
      
          if (i==6){
            wb_motor_set_position(R_servo[i],   leg[3].osc[1].Y[x]*a);
            wb_motor_set_position(R_servo[i+1], leg[3].osc[2].Y[x]*c);
            wb_motor_set_position(R_servo[i+2], leg[3].osc[3].Y[x]*d);
          }
        
          if (i==9){
            wb_motor_set_position(R_servo[i],   leg[6].osc[1].Y[x]*b);
            wb_motor_set_position(R_servo[i+1], leg[6].osc[2].Y[x]*c);
            wb_motor_set_position(R_servo[i+2],-leg[6].osc[3].Y[x]*d);//-
          }
        
          if (i==12){
            wb_motor_set_position(R_servo[i],   leg[5].osc[1].Y[x]*b);
            wb_motor_set_position(R_servo[i+1], leg[5].osc[2].Y[x]*c);
            wb_motor_set_position(R_servo[i+2],-leg[5].osc[3].Y[x]*d);
          }
          
          if (i==15){
            wb_motor_set_position(R_servo[i],   leg[4].osc[1].Y[x]*b);
            wb_motor_set_position(R_servo[i+1], leg[4].osc[2].Y[x]*c);
            wb_motor_set_position(R_servo[i+2], leg[4].osc[3].Y[x]*d);
          }      
          
      }
      wb_robot_step(36);  //20220323
    }      
  
}

// Generate a random value between nmin and nmax
double randn(double nmin, double nmax){
	double rinv;
	rinv=nmax-nmin;
	return ( (double)(rand()%(RAND_MAX)) / (double)(RAND_MAX-1) )*rinv+nmin;
}      

// Lower bound
double min_variable(int i){  //為了用來比較用，寫一個函式先把最小值傳給每個參數(中心+寬+後件)
   double ge;

    // input : parameter index = 0~109
    // (x%11)=0~10
    int j=((i-1)%(_gap));  //_gap=12  i是1~_lpara

    if(j>=(2*_in_varl)){    // 8~10 = consequence(後件部)，即j比8大的就是後件部
        ge=_conq_min;  //ge=-2
    }
    else{       // 0~7 = antecedent(前件部，包含中心點+寬)
        if(j%2==0){ // even(0,2,4,6) = center
            ge=_center_min;
        }
        else{               // odd(1,3,5,7) = width
            ge=_range_min;
        }
    }
    return ge;  // return the corresponding minimum
}

// Upper bound
double max_variable(int i){  //在限制前件部的中心點，寬以及後件部的最大值(上界)
    double ge;

     // input : parameter index = 0~109
    // (x%11)=0~10
    int j=((i-1)%(_gap));  //i是0~_lpara-1

    if(j>=(2*_in_varl)){   // 8~10 = consequence
        ge=_conq_max;
    }
    else{       // 0~7 = antecedent
        if(j%2==0){ // even(0,2,4,6) = center
            ge=_center_max;
        }
        else{              // odd(1,3,5,7) = width
            ge=_range_max;
        }
    }
    return ge;  // return the corresponding maximum
}

// Gaussian membership function
double Gaussian(double x, double m, double w){
    return(  exp( - (x-m)*(x-m)/(w*w) )) ;
}

/////////////////////////////////////////////////////////////////////////////
// Population initialization
void initialization(void){//隨機產生中心點，寬，後件部，每代只用一次而已(第一次)
    printf("Start to initialize...\n");
  
    // generate parameters randomly
    for(int i=1; i<=_popsize; i++){  //共40個母代，每個母代有110個參數
                                    //40個母代都給110個參數隨機的值
        for(int j=1; j<=_rule; j++){    
            for(int k=1; k<=_in_varl; k++){    

                // fuzzy sets:
                // center = random(-1~1)
                oldpop[i].p[(_gap)*(j-1)+(2*k)-1] = ((rand()%201)-100)/100.;  //產生40個隨機中心點，且要40組(因有40隻鳥)
                //p[(_gap)*j+2*k]=p[0],p[2],p[4],p[6],p[11],p[13],p[15].p[17]...p[105]就是中心點
                // width = random(0.05~0.75)
                oldpop[i].p[(_gap)*(j-1)+(2*k)] =  0.05 + ((rand()%251)/1000.0);//產生40個隨機寬，且要40組(因有40隻鳥)
            }   //p[(_gap)*j+2*k]=p[1],p[3],p[5],p[7],p[12],p[14],p[16],p[18]...p[106]就是寬

            for(int k=1; k<=_out_varl; k++){   // for each output
                // consequence = random(-1~1)
                oldpop[i].p[(_gap)*(j-1)+2*(_in_varl)+k] = _uscale*((rand()%2001)-1000)/1000.;
            } //產生30個隨機後件部，且要40組
        }  //p[8],p[9],p[10],p[19],p[20],p[21]...p[107],p[108],p[109]都試後件部  其中_uscale=1
        checkpop(oldpop[i].p);   // limits all parameters
        oldpop[i].clear(); //把兩個fitness重新變很大的值
    }
    

    for(int i=1; i<=_popsize; i++){//計算這初始化的40組控制器的fitness，共40個的fitness
        printf("popsize=%d\n",i);
        fflush(stdout);
        // 1. build a fuzzy controller
        // 2. scored by cost functions
        Eva_Fitness(&oldpop[i]);//因Eva_Fitness輸入型別關係，所以要加&
		if(check){  //20220324
		   FILE *fy2 ;
		   if( (fy2=fopen("..\\..\\c_matrix.dat","a"))==NULL) exit(1) ;//存取最後成功的次數
		   fprintf(fy2,"%d\n", i);
		   fclose(fy2) ;
		   break;
		}
    }
    punish(oldpop, _popsize);  // penalty function
 	if(check){load_gen=_maxgen+1;} //20220324  
    //已經產生的40組解，進去做排序，看誰是front1，front2，front3...    
    //n_front=fast_nd_sort(oldpop, front, _popsize);
    
    sw = 1 ; // 使親代進入非主導排序的條件 
    non_domination(sw , load_gen) ; //非主導排序
    sort_on_crowd(sw , load_gen) ; //排列解的多樣性
        
    printf("Initialize finished.\n");
    fflush(stdout);
}
///////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////
void generation(int iter){
    //int prt1, prt2;
    //individual parent[2];
    FILE *oldp,*all_oldp,*ofitness,*all_ofitness;
    FILE *ocd_len,*ofrnt, *input, *clu_first;
    int clu;
    //FILE *foldp,*ffitness,*fcd_len,*ffrnt,*fflag,*fm,*fn;
    
    if (iter>=2){
      //********************讀oldpop用*********************
      wb_robot_step(100);
      // oldp = fopen("oldp.dat","r");
      // ocd_len = fopen("ocd_len.dat","r");
      // ofrnt = fopen("ofrnt.dat","r");
      // ofitness = fopen("ofitness.dat","r");
      ///input = fopen("..\\..\\..\\3\\controllers\\MOFCACO_train\\oldp.dat", "r");
      input = fopen(FC_file, "r");
      clu_first = fopen(clu_file, "r");
      wb_robot_step(100);
      fscanf(clu_first,"%d",&clu);
      _rule = clu;
      // cout << _mem_length << endl;
            // cout << clu << endl;
                   // cout << clu*10 << endl;
                   // cout << "1231321aaa" << endl;
      //exit(-1);
      for (int a=1; a<=1; a++){
        
        for (int b=1; b<=10*clu; b++){
          fscanf(input,"%lf",&oldpop[a].p[b]);
                     cout << oldpop[a].p[b] << endl;
        }
        // for (int k=1; k<=_nobj; k++){
          // fscanf(ofitness,"%lf",&oldpop[a].fitness[k]);
        // }
          // fscanf(ocd_len,"%lf",&oldpop[a].cd_len);
          // fscanf(ofrnt,"%d",&oldpop[a].frnt);
         // fscanf(oflag,"%d",&oldpop[a].flag);
      }
     // exit(-1);
      wb_robot_step(100);
      // fclose(oldp);
      // fclose(ocd_len);
      // fclose(ofrnt);
      // fclose(ofitness);
      fclose(input);
      fclose(clu_first);
      wb_robot_step(100);
      //********************讀oldpop用*********************
    }
    
   // int t_loop = int(_popsize*0.25) ; //L = N*(現在的疊代次數/總疊代次數)，L為新產生的解之個數
    //printf("t_loop=%d\n",t_loop);
   // update_ant(t_loop);  //ACO解的更新
   
   // update_pop() ; // 解的更新
   
                   // for (int b=1; b<=10*clu; b++){
            // cout << oldpop[1].p[b] << endl;}
    for(unsigned int i=1; i<=1; i++){  //20220323
        printf("現在在跑第%d代的第%d個popsize\n",iter,i);
        fflush(stdout);
        checkpop(oldpop[i].p);       // limit the parameter range
        Eva_Fitness(&oldpop[i]);  // cost functions
        printf("跑完第%d個解且計算完fitness\n",i);
        wb_robot_step(500);
        printf("*************************************\n");
        fflush(stdout);
        if(check){
                for (int b=1; b<=10*clu; b++){
            cout << oldpop[i].p[b] << endl;
        }
        exit(-1);}
		// if(check){  //20220324
		   // FILE *fy2 ;
		   // if( (fy2=fopen("..\\..\\c_matrix.dat","a"))==NULL) exit(1) ;//存取最後成功的次數
		   // fprintf(fy2,"%d\n", (iter-1)*new_row+i+40);
		   // fclose(fy2) ;
		   
		   // FILE *fy3 ;
		   // if( (fy3=fopen("..\\..\\Final_first.dat","w"))==NULL) exit(1) ;//存取最後成功的次數
                         // for (int b=1; b<=_lpara; b++){
                            // fprintf(fy3,"%lf",newpop[i].p[b]);
                          // }
		   // fclose(fy2) ;
		   // break;
		// } 
        
    }
exit(-1);
    punish(newpop, new_row);   // penalty functions //20220323
	if(check){load_gen=_maxgen+1;} //20220324
    
    /*for (int i=1 ; i<=new_row ; i++){ //20220324
    printf("newpop[%d]的CT[0]=%lf\t,fitness[1]=%lf  ,fitness[2]=%lf\n",i,newpop[i].CT[0],newpop[i].fitness[1],newpop[i].fitness[2]);
    }
    printf("******************************************************\n");*/
    fflush(stdout);
    wb_robot_step(1000);
    
    mix_population();
    
    sw = 3 ;// 使子代進入非主導排序的條件 
    non_domination(sw , load_gen) ; //非主導排序
    sort_on_crowd(sw , load_gen) ;  //排列解的多樣性
    update_newparent() ; //使用參數歸零
    
    /*for (int i=1 ; i<=_popsize ; i++){
    printf("oldpop[%d]的front為%d\t,CT[0]=%lf\t,fitness[1]=%lf  ,fitness[2]=%lf\n",i,oldpop[i].frnt,oldpop[i].CT[0],oldpop[i].fitness[1],oldpop[i].fitness[2]);
    }*/
    fflush(stdout);
    wb_robot_step(1000);

    //********************存oldpop相關參數值**************
    wb_robot_step(100);
    oldp = fopen("oldp.dat","w");
    all_oldp = fopen("all_oldp.dat","a");
    all_ofitness = fopen("all_ofitness.dat","a");
    ocd_len = fopen("ocd_len.dat","w");
    ofrnt = fopen("ofrnt.dat","w");
    ofitness = fopen("ofitness.dat","w");
    fprintf(all_oldp,"*************以下是第%d代的參數值*********\n",iter);
    fprintf(all_ofitness,"*************以下是第%d代的參數值*********\n",iter);

    for (int a=1; a <= _popsize; a++){
    
      for (int b=1; b <= _gap*_rule; b++){
        fprintf(oldp,"%lf\n",oldpop[a].p[b]);
        fprintf(all_oldp,"%lf\t",oldpop[a].p[b]);
      }//以上在存40組控制器的110個參數
              
        fprintf(ocd_len,"%lf\n",oldpop[a].cd_len);
        fprintf(ofrnt,"%d\n",oldpop[a].frnt);
      
      for (int k=1; k<=_nobj; k++){
        fprintf(ofitness,"%lf\n",oldpop[a].fitness[k]);
        fprintf(all_ofitness,"%lf\t",oldpop[a].fitness[k]);
      }
        
    }
    wb_robot_step(100);
    fprintf(all_oldp,"\n");
    fprintf(all_ofitness,"\n");
    fclose(oldp);
    fclose(all_oldp);
    fclose(ocd_len);
    fclose(ofrnt);
    fclose(ofitness);
    fclose(all_ofitness);
    wb_robot_step(100);
    //********************存oldpop相關參數值**************
}

////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
//在計算控制器的fitness
//引數型別是individual*，所以輸入的型別也要相同
void Eva_Fitness(individual* pop){
    int step=200, robot_state=0, DD=0;
    double c1, c2, c3;
    double ct1=0, ct2=0, ct3=0, ct4=0, dw=0.2;//20220323
    double in[_in_varl+1]={0}, out[_out_varl+1]{0};
    double pos[2]={0}, xx[_Maxstep], zz[_Maxstep];

    WbNodeRef robot_node = wb_supervisor_node_get_from_def("Hexa");
    WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
    WbFieldRef  rot_field  =  wb_supervisor_node_get_field(robot_node,"rotation");
    FCR fc;  //整場只用到一次FCR型別宣告，所以沒很複雜
    Data t ; //整場只用到一次Data型別宣告，所以沒很複雜

    robot_get_position(pos);
    xx[step]=pos[0];
    zz[step]=pos[1];
    
    /*printf("初始的X位置=%lf Y位置=%lf\n",xx[step],zz[step]);
    fflush(stdout);
    wb_robot_step(400);*/
     
    fc.getRule(pop->p); // 把p[110個]參數傳給FCRule而已(build an FC by solution)

    do{
        step++;
        cout << "-----------step = "<<step<<"------------\n";
        //-----------------------------------------------------------
        DS = wb_distance_sensor_get_value(SSensor);
        /*printf("DS = %lf\n",DS);
        fflush(stdout);
        wb_robot_step(100);*/       
        if (DS>=400) {
          DD = 1;
          printf("********DS = %lf , wrong way********\n",DS);
          fflush(stdout);
          wb_robot_step(10); 
        }
        //-----------------------------------------------------------
                
//********************************************testing*****************************************************  
        
        robot_get_distance(Urg);        // 更新DS值
              
        if(leg[1].osc[1].Y[step-1]<=0 && leg[1].osc[1].Y[step]>0){
          t.input(step,in) ;                   // 把DS值當作輸入
          fc.Controller(in, out);         // 做模糊化和解模糊的後件部輸出值
        }
        else if(leg[1].osc[1].Y[step-1]>=0 && leg[1].osc[1].Y[step]<0){
          t.input(step,in) ;                   // 把DS值當作輸入
          fc.Controller(in, out);         // 做模糊化和解模糊的後件部輸出值        
        }
        
        t.output(step,out) ;                 // use output to drive the robot
        
//******************************************testing*******************************************************     
        /*
        robot_get_distance(Urg);        // 更新DS值              
        t.input(step,in) ;                   // 把DS值當作輸入
        fc.Controller(in, out);         // 做模糊化和解模糊的後件部輸出值          
        t.output(step,out) ;                 // use output to drive the robot        
        */
        
        //得到3個後件部輸出值後，在裡面output又執行了robot_set_motion讓機器人動
        robot_get_distance(Urg);       // 動了之後再算一次DS值
        robot_state=robot_state_judge(Urg);
        //雷射距離>1則robot_state=1，//雷射距離<0.3則robot_state=-1
        robot_get_position(pos);    // store the coordinates 存座標
        
        xx[step]=pos[0];
        zz[step]=pos[1];

        // ct1: sum( (dis(t)/0.5) - 1)
        //c1= fabs(((Urg[0]-0.25)/dw)-1);
        c1= fabs(((Urg[0]-0.2)/dw)-1); //20220323
		//控制沿牆距離
        // ct2: sum(absolute walking distance)
        c2= fabs(sqrt(pow(((xx[step])-(xx[step-1])),2)+pow(((zz[step])-(zz[step-1])),2)));
         //sqrt:求平方根
        // ct3: sum(walking distance)
        c3= sqrt(pow(((xx[step])-(xx[step-1])),2)+pow(((zz[step])-(zz[step-1])),2));
        //pow(a,b)是a的b次方，就是在算位移距離(x變化量平方+z變化量平方相加再開根號)
        ct1+=c1;
        ct2+=c2;
        ct3+=c3;

        // ct3=0~least walking distance
        if (ct3 >= MAX_D){  //MAX_D=20 走的最小步數
            ct3=MAX_D;
        }

        if (step%200==0){    // 每50步，計算前50步位置與現在位置的距離，如果小於20公分則robot_state=-1，跳出迴圈
            ct4=sqrt(pow(((xx[step])-(xx[step-200])),2)+pow(((zz[step])-(zz[step-200])),2));

            //if (ct4<0.1){   // moves less than 20cm
            if (ct4<0.05){ //20220323
				robot_state=COLLIDE;    // reset
                printf("The moving distance is not enough.\n");
            }
        }
    }while(robot_state==FAIR && DD==0 && step<_Maxstep);  //FAIR=0，_Maxstep=1500
    //因為初始robot_state=0，只有在前50步走不到20公分(robot_state=-1)以及step++到1500時才會跳出迴圈
    printf("rescent step is %d\n",step);
	FILE *fy11 ;
	if( (fy11=fopen("..\\..\\step.dat","a"))==NULL) exit(1) ;//存取每一代步數
	fprintf(fy11,"%d\n",step-200);
	fclose(fy11);
	if(step >= _Maxstep)
		check = 1;
    //printf("*************************************\n");
    fflush(stdout);
    robot_get_position(pos); 
    wb_robot_step(TIME_STEP);

    // penalty function:
    // 1. walked step < _Maxstep (reset)
    // 2. walking distance < least walking distance
    //我存CT[0~3]是為了punish函式計算懲罰值用的
    pop->CT[0] = ((_Maxstep-step)+10000*(MAX_D-ct3));
    //如果走不到1500步就跳出迴圈，則懲罰[(1500-跳出步數)+10000*(20-一棟距離)]
    
    // F1: stay an 0.5m average robot-wall distance
    pop->CT[1] = (ct1/step);
    //
    // F2: ( 1 / average speed)
    pop->CT[2] = (step/(ct2*100));
    //[1500/(走的距離*100)]，當我走的距離不夠遠，分母不夠大懲罰會較大，當我走的距離很遠，分母夠大懲罰會較小
    //wb_supervisor_world_reload();
    
        const double INITIAL[3] = { -0.970139, 0.0917459, -2.04816 };
        const double ROTATE [4] = {2.7195e-05, 1, 1.99653e-05, -0.521178};
        wb_supervisor_field_set_sf_rotation(rot_field, ROTATE);
        wb_robot_step(1000);
        wb_supervisor_field_set_sf_vec3f(trans_field, INITIAL);
        wb_robot_step(1000);
        for (int i=0; i<18; i++){
           wb_motor_set_position(R_servo[i], 0);
        }
        wb_robot_step(400);
    
}
///////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////
//count基本上都是40，每組控制器都會計算
void punish(individual* eva, int count){
    for (int i=1;i<=count;i++){
        //eva[i].fitness[1]=eva[i].CT[0]+eva[i].CT[1];
        //eva[i].fitness[2]=eva[i].CT[0]+eva[i].CT[2];
		eva[i].fitness[1] = (eva[i].CT[0]+eva[i].CT[1])*0.5 + (eva[i].CT[0]+eva[i].CT[2])*0.5; //20220323
		FILE *fr;
		if( (fr=fopen("..\\..\\avg_reward.dat","a"))==NULL) exit(1) ;
		fprintf(fr,"%lf\t%lf\t%lf\t", (eva[i].CT[0]+eva[i].CT[1]), (eva[i].CT[0]+eva[i].CT[2]),eva[i].fitness[1]); 
		fprintf(fr,"\n");
		fclose(fr) ;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//-------------------------------------------------------------------

void update_pop(){
int _tour1 , _tour2 , jj  ;

  for (jj=1; jj<=_popsize; jj=jj+2)
  {	
    do
    {
    _tour1 = selection() ;  // 使用競爭法
    _tour2 = selection() ;  // 使用競爭法
    }
    while (_tour1==_tour2);   
  
  printf("jj=%d時\t _tour1=%d\t _tour2=%d\n",jj,_tour1,_tour2);
  fflush(stdout);
  wb_robot_step(100);
  
  crossover(jj , _tour1, _tour2) ; //交配
  }	

mutation();  //突變		
}


//////////////////////////////  突變   //////////////////////////////////{
//用在解的更新  update_pop

void mutation()
{
  int i,j;
  double rnd,delta,indi,deltaq;
  double y,y_min,y_max,val,xy;
  
  for (i=1; i<=_popsize; i++)
  {
    for(j=1; j<=_gap*_rule; j++)
    {
    rnd=randn(0,1);
     if(rnd<= _pm) // for each variable find whether to do mutation or not
     {
      y=newpop[i].p[j];
      y_max=max_variable(j);
      y_min=min_variable(j);
                          
       if (y>y_min)
       {
         if((y-y_min) < (y_max-y))
          delta = (y - y_min)/(y_max - y_min);
                                
         else
         delta = (y_max - y)/(y_max - y_min);
                                
         rnd=randn(0,1);
         indi=1.0/(_eta_m+1.0);
                                
         if (rnd<=0.5)
         {
          xy = 1.0-delta;
          val = 2*rnd+(1-2*rnd)*(pow(xy,(_eta_m+1)));
          deltaq =  pow(val,indi) - 1.0;
         }
         else
         {
          xy = 1.0-delta;
          val = 2.0*(1.0-rnd)+2.0*(rnd-0.5)*(pow(xy,(_eta_m+1)));
          deltaq = 1.0 - (pow(val,indi));
         }
          
          y = y + deltaq * (y_max-y_min);
                                 
          if (y < y_min) y=y_min;
          if (y > y_max) y=y_max;
                                 
          newpop[i].p[j]=y;
        }
        else
        {
         xy=randn(0,1);
         newpop[i].p[j]=xy*(y_max-y_min)+y_min;
        }
      }
    }
  }
}
//////////////////////////////  突變   //////////////////////////////////}


/////////////////////////////     交配    ///////////////////////////////{
// Function to cross two individuals //
//此函式用在"交配與突變"內，即  update_pop 。

void crossover (int kk , int _tour1 , int _tour2)
{
int i;
double rnd,par1,par2,chld1,chld2,betaq,beta,alpha;
double y1,y2,y_max,y_min,expp;
    
rnd=randn(0,1);
    
  if(rnd<=_pc)
  {
    for(i=1; i<=_gap*_rule; i++)
    {
      par1=oldpop[_tour1].p[i];
      par2=oldpop[_tour2].p[i];
      y_max=max_variable(i);
      y_min=min_variable(i);
      
      rnd=randn(0,1);
      
      if(rnd<=0.5)
      {
        if(fabs(par1 - par2) > 0.000001) 
        {
          if(par2 > par1)
          {
          y2 = par2;
          y1 = par1;
          }
          else
          {
          y2 = par1;
          y1 = par2;
          }
                                           
          if((y1 - y_min) > (y_max - y2))     /*Find beta value*/
          {
          beta = 1 + (2*(y_max - y2)/(y2 - y1));
          }
          else
          {
          beta = 1 + (2*(y1-y_min)/(y2-y1));
          }
                                           
          expp = _eta_c + 1.0; 
          beta = 1.0/beta; 
          alpha = 2.0 - pow(beta,expp);
                                           
          if (alpha < 0.0) 
          {
          alpha=0.000001;
     //     printf("ERRRROR %f %f %f\n",alpha,par1,par2);
          }
                                           
          rnd=randn(0,1);
                                            
          if (rnd <= 1.0/alpha)
          {
           alpha = alpha*rnd;
           expp = 1.0/(_eta_c+1.0);
           betaq = pow(alpha,expp);
          }
          else
          {
          alpha = alpha*rnd;
          alpha = 1.0/(2.0-alpha);
          expp = 1.0/(_eta_c+1.0);
         
            if (alpha < 0.0) 
            alpha=0.000001;
         
            if (alpha < 0.0) 
            {
    //         printf("ERRRORRR \n");
             exit(-1);
            }
                                                  
          betaq = pow(alpha,expp);                                          
          }                                           
       chld1 = 0.5*((y1+y2) - betaq*(y2-y1));
       chld2 = 0.5*((y1+y2) + betaq*(y2-y1));
       }
       else
       {                                  
        betaq = 1.0;
        y1 = par1; y2 = par2;
                                        
         /*Generation two children*/
         chld1 = 0.5*((y1+y2) - betaq*(y2-y1));
         chld2 =  0.5*((y1+y2) + betaq*(y2-y1));
       }
      if (chld1 < y_min) chld1 = y_min;
      if (chld1 > y_max) chld1 = y_max;
      if (chld2 < y_min) chld2 = y_min;
      if (chld2 > y_max) chld2 = y_max;                           
    }
    else
    {
    chld1 = par1;
    chld2 = par2;
    }
    
   newpop[kk].p[i]=chld1;
   newpop[kk+1].p[i]=chld2;
  }		
 }
}
/////////////////////////////     交配    ///////////////////////////////}


int selection() 
{
int *tour_cand , *cand_rank;
double *cand_dis ;
int tour_member , j ;
int tour_size = 2 ;
		
tour_cand = new int[tour_size+1] ;
cand_rank = new int[_popsize+1] ;
cand_dis = new double[_popsize+1] ;
  
  srand(time(NULL));
  
  for (j=1; j<=tour_size; j++)
  {
  tour_cand[j] = (rand()%40) + 1 ;
  cand_rank[j] = oldpop[ tour_cand[j] ].frnt ;
  cand_dis[j] = oldpop[ tour_cand[j] ].cd_len ;
  }
  
  if (cand_rank[1] == cand_rank[2])
  {
    if (cand_dis[1] >= cand_dis[2])
    tour_member = tour_cand[1] ;
    else
    tour_member = tour_cand[2] ;
  }
  else
  {
    if (cand_rank[1] < cand_rank[2])
    tour_member = tour_cand[1];
    else
    tour_member = tour_cand[2];
  }

delete [] tour_cand;
delete [] cand_rank;
delete [] cand_dis ;
tour_cand = 0 ;
cand_rank = 0 ;
cand_dis = 0 ;

return tour_member;
}


void update_newparent()
{
	int i,j ;

	for(i=1;i<=_popsize;i++)
		oldpop[i] = mixpop[ mixpop_sorted[i] ] ;
	//newpop initial
	for(i=1; i<=_popsize+new_row; i++)
	{
		newpop[i].frnt = 0 ;
		newpop[i].cd_len = 0 ;
				
		for (j=1; j<=_gap*_rule; j++)
			newpop[i].p[j] = 0 ;

		for (j=1; j<=_nobj; j++)
			newpop[i].fitness[j] = 0 ;
	}

	//temp_child initial
	for(i=1; i<=_popsize+new_row; i++)
	{
		temp_pop[i].frnt = 0 ;
		temp_pop[i].cd_len = 0 ;
		
		for (j=1; j<=_gap*_rule; j++)
			temp_pop[i].p[j] = 0 ;

		for (j=1; j<=_nobj; j++)
			temp_pop[i].fitness[j] = 0 ;
	}

	
	//mixpop initial
	for(i=1; i<=_popsize+new_row; i++)
	{
		mixpop[i].frnt = 0 ;
		mixpop[i].cd_len = 0 ;
		
		for (j=1; j<=_gap*_rule; j++)
			mixpop[i].p[j] = 0 ;

		for (j=1; j<=_nobj; j++)
			mixpop[i].fitness[j] = 0 ;
	}	
}

////////////////////////////  非主導排序  ////////////////////////////////////{
// 求每個個體(solution)所對應的Front number and Crowding distance
//功能:排序解的"好壞"

void non_domination(int sw , int gg)
{

//////////  定義變數  ///////////{
int i,j,k,w,m ;
double _fmin=0. , _fmax=0., next_obj=0. , pre_obj=0. ;
double _dist=0.;
int less, equal, more ;
int frnt ; 
int pop_size=0 ;
//////////  定義變數  ///////////}

//////////////////  親代的轉存  ////////////////////{
  if (sw == 1)
  {
   pop_size = _popsize ; //轉存解的數量
   
    for(i=1; i<=pop_size; i++)
    {
      for (j=1; j<=_nobj; j++)
      {
       temp_pop[i].fitness[j] = oldpop[i].fitness[j] ; //cost value的轉存
      }
    }
  }
//////////////////  親代的轉存  ////////////////////}
	
//////////////////  子代的轉存  ////////////////////{	
  else if (sw == 3)
  {
   pop_size = _popsize + new_row ; //轉存解的數量
      
    for(i=1; i<=pop_size; i++)
    {
      for (j=1; j<=_nobj; j++)
      {
       temp_pop[i].fitness[j] = mixpop[i].fitness[j] ; //cost value的轉存
      }
    }
  }
//////////////////  子代的轉存  ////////////////////}

//////////////  初始化、歸零  /////////////////{
  for (i=1; i<=pop_size; i++)
    for (j=1; j<=pop_size; j++)
      _front[i].member[j] = 0 ;

frnt = 1 ;
w = 0 ;

  for (i=1; i<=pop_size; i++)
 {
 temp_pop[i].nnp = 0 ;
 temp_pop[i].nsp = 0 ;
 temp_pop[i].frnt = 0 ;
 temp_pop[i].cd_len = 0 ;
 _front[i].number = 0 ;
 }
 //////////////  初始化、歸零  /////////////////}

///////////////////////////  第一個 front 的處理  //////////////////////////{
  for (i=1; i<=pop_size; i++)
  {
    for (j=1; j<=pop_size; j++)
    {
    less=0 ;
    equal=0 ; 
    more=0 ;
  
    ///////////////////  情況分類:大於、等於、小於  ///////////////{
    for (k=1; k<=_nobj; k++) 
    {
      if (temp_pop[i].fitness[k] < temp_pop[j].fitness[k])
      {
      less ++ ;
      }
      else if (temp_pop[i].fitness[k] == temp_pop[j].fitness[k])
      {
      equal ++ ;
      }
      else
      {
      more ++ ;
      }
    }	
      ///////////////////  情況分類:大於、等於、小於  ///////////////} 
////////////////////  含有等號的處理  //////////////////////{    	
     if( less == 0 && equal != _nobj) // p被支配的解
     {
     // 含有一個等號配上一個大於的情況
     temp_pop[i].nnp ++ ;  //計數器加1
     temp_pop[i].np[ temp_pop[i].nnp ] = j ;  
     }
     else if (more == 0 && equal != _nobj) // p支配的解
     {
     //含有一個等號配上一個小於的情況
     temp_pop[i].nsp ++ ;  //計數器加1
     temp_pop[i].sp[ temp_pop[i].nsp ] = j ;  
     }	
////////////////////  含有等號的處理  //////////////////////}      
    }

//////////////////////  記錄 front 1 //////////////////////{    
    if ( temp_pop[i].nnp == 0 ) 
    {
    //如果沒有"等號配上一個大於"的情況，那保證必定是小於的情況
    w ++ ; //計數器加1 
    temp_pop[i].frnt = 1 ;  //  說明第i個解的排名是1 
    
    _front[1].member[w] = i ; // 記錄在第幾個 i 的時候，是第 w 個 front 1，因為 front 1 可以同時存在很多個 
    }
    
  _front[1].number = w ; //記下目前有幾個 front 1 的個數 
//////////////////////  記錄 front 1 //////////////////////}  
  }
///////////////////////////  第一個 front 的處理  //////////////////////////}
	
///////////////////// 找出第二個Front以後，含有等號的相對應的rank  /////////////////////{ 
	
  while ( _front[frnt].member[1] != 0 )
  //這裡的0不能寫NULL，NULL是給指標用的.NULL在指標裡代表.0,無物,無值.
  //而在指標外就無意義了，不可NULL當作數字來用.
  //在 _front[frnt].member[1] 的 frnt值於此副程式的開頭定義成 1，
  //代表一定會找到一個 front 1 有"值"之後，才會進到這個迴圈。 
  //總和之意: 當 front 1 的個數不為零時，進入while迴圈          
  {
  temp1 = new int[2*_popsize+1] ;
  w= 0 ;
  
    for (i=1; i<=_front[frnt].number; i++)
    //frnt值於此副程式的開頭定義成 1，而_front[frnt].number表示記下目前有幾個 front 1 的個數
    {
      if ( temp_pop[ _front[frnt].member[i] ].sp[1] != 0 )
      //這裡的0不能寫NULL，NULL是給指標用的.NULL在指標裡代表.0,無物,無值.
      //在指標外就無意義了，不可NULL當作數字來用.
      //frnt值於此副程式的開頭定義成 1，所以 _front[frnt].member[i] 是記錄在第幾個 i 的時候，是 front 1
      //sp[1]的意思是，含有一個等號配上一個小於的情況，所以計數器加 1 
      //所以總和之意:當第 i 個解為front 1 ，且在i的編號之前出現"含有一個等號配上一個小於的情況 "，其數值恰不為零，則進入迴圈。
      {
        for (j=1; j<=temp_pop[ _front[frnt].member[i] ].nsp; j++)
        //"當第 i 個解為front 1 ，且在i的編號之前出現含有一個等號配上一個小於的情況 "的計數器數值拿來用
        {
          temp_pop[ temp_pop[ _front[frnt].member[i] ].sp[j] ].nnp = temp_pop[ temp_pop[ _front[frnt].member[i] ].sp[j] ].nnp - 1 ; 
          //??     
          if ( temp_pop[ temp_pop[ _front[frnt].member[i] ].sp[j] ].nnp == 0 ) // 若該解沒有被任何解支配, 則分配Front rank
          {
            temp_pop[ temp_pop[ _front[frnt].member[i] ].sp[j] ].frnt = frnt + 1 ;
            w ++ ; // 計數每個Front成員的個數
            temp1[w] = temp_pop[ _front[frnt].member[i] ].sp[j] ; 
          }
        }
       }
     }
  
  frnt = frnt + 1 ; // frnt
  
  for (j=1; j<= w; j++)
  _front[frnt].member[j] = temp1[j] ;

  _front[frnt].number = w ; 

////////  釋放 temp1 的記憶體  ///////{
  delete [] temp1 ;
  temp1 = 0 ;
////////  釋放 temp1 的記憶體  ///////}
  } 
///////////////////// 找出第二個Front以後，含有等號的相對應的rank  /////////////////////}

front_size = frnt-1 ; // 排除front1後，統計該回合產生多少個Front的轉存變數


//////////// 將solutions按照Front rank排序前的轉存 ////////////////{

k=0 ;
  for (i=1; i<=pop_size; i++)
  front_sort[i] = 0 ; //歸零

  for (i=1; i<=front_size; i++)
  {
    for (j=1; j<=_front[i].number; j++)
    {
      k ++ ;
      front_sort[k] = _front[i].member[j] ;
    }
  }
  for (i=1; i<=pop_size; i++)
  temp_pop[ front_sort[i] ].sorted_front = i ;
  
//////////// 將solutions按照Front rank排序前的轉存 ////////////////}

/////////////////////////// crowding distance的計算  /////////////////////////////{

  for (frnt =1 ; frnt<=front_size; frnt++)
  {
    for (m=1; m<=_nobj; m++)
    {
      temp_sort = new int[2*_popsize+1] ;
      sort_on_obj(frnt , m);  //按照 cost value 排序
      
      _fmin = temp_pop[ temp_sort[1] ].fitness[m] ;  //cost value 的最小值為 front 1 
      _fmax = temp_pop[ temp_sort[ _front[frnt].number ] ].fitness[m] ; //cost value 的最大值為最後一名的 front  	
      
      temp_pop[ temp_sort[1] ].dist[m] = INF ;  // front 1 的 crowding distance 定為無窮大
      temp_pop[ temp_sort[ _front[frnt].number ] ].dist[m] = INF ; // 最後一名的 front 的 crowding distance 定為無窮大

      for (j=2; j<=(_front[ frnt].number-1); j++)
      {
      next_obj = temp_pop[ temp_sort[j+1] ].fitness[m] ;
      pre_obj = temp_pop[ temp_sort[j-1] ].fitness[m] ;
      
        if ( fabs(_fmax-_fmin) <= 1E-8 )
        temp_pop[ temp_sort[j] ].dist[m] = INF ;
        else
        temp_pop[ temp_sort[j] ].dist[m] = (next_obj-pre_obj)/(_fmax-_fmin) ;  //??
      }   
      ////////  釋放 temp1 的記憶體  ///////{
      delete [] temp_sort ;
      temp_sort = 0 ;
      ////////  釋放 temp1 的記憶體  ///////}
    }  
/////////  累加一組解的所有cost value 的 crowding distance ////////{   
    for (i=1; i<= _front[frnt].number; i++)
    {
    _dist = 0;
    
    for (m=1; m<=_nobj; m++)
    _dist = _dist + temp_pop[ _front[frnt].member[i] ].dist[m] ; //累加一組解的所有cost value 的 crowding distance
    
    temp_pop[ _front[frnt].member[i] ].cd_len = _dist ; //轉存累加後的值
    }
//////////  累加一組解的所有cost value 的 crowding distance //////////}
  } 	
/////////////////////////// crowding distance的計算  /////////////////////////////}

//////////////////  親代的轉存  ////////////////////{
  if ( sw == 1)
  {
    for (i=1; i<=pop_size; i++)
    {
    oldpop[i].nnp = temp_pop[i].nnp ; // 含有一個等號配上一個大於的計數器的轉存
    oldpop[i].nsp = temp_pop[i].nsp ; // 含有一個等號配上一個小於的計數器的轉存
    oldpop[i].sorted_front = temp_pop[i].sorted_front ; //將solutions按照Front rank排序前的轉存
    oldpop[i].frnt = temp_pop[i].frnt ; //轉存 rank
    oldpop[i].cd_len = temp_pop[i].cd_len ; //轉存 crowding distance ，用在sort_on_crowd的函式
    
      for (j=1; j<=_nobj; j++)
      {
      oldpop[i].dist[j] = temp_pop[i].dist[j]; //轉存累加的 crowding distance
      }
      
      for (j=1; j<= temp_pop[i].nsp; j++)
      {
      oldpop[i].sp[j] = temp_pop[i].sp[j] ;  // 轉存含有一個等號配上一個小於的情況的數量解??
      }
    }
  }
//////////////////  親代的轉存  ////////////////////} 
 
//////////////////  子代的轉存  ////////////////////{ 
  else if (sw == 3)
  {
    for (i=1; i<=pop_size; i++)
    {
    mixpop[i].nnp = temp_pop[i].nnp ;// 含有一個等號配上一個大於的計數器的轉存
    mixpop[i].nsp = temp_pop[i].nsp ; // 含有一個等號配上一個小於的計數器的轉存
    mixpop[i].sorted_front = temp_pop[i].sorted_front ;//將solutions按照Front rank排序前的轉存
    mixpop[i].frnt = temp_pop[i].frnt ;//轉存 rank
    mixpop[i].cd_len = temp_pop[i].cd_len ;//轉存 crowding distance ，用在sort_on_crowd的函式
    
      for (j=1; j<=_nobj; j++)
      {
       mixpop[i].dist[j] = temp_pop[i].dist[j] ;//轉存累加的 crowding distance
      }
    
      for (j=1; j<= temp_pop[i].nsp; j++)
      {
       mixpop[i].sp[j] = temp_pop[i].sp[j] ;// 轉存含有一個等號配上一個小於的情況的數量解??
      }
    }
  }
//////////////////  子代的轉存  ////////////////////}   
}

////////////////////////////  非主導排序  ////////////////////////////////////}

///////////////////////// 按照 cost value 排序  //////////////////////////{
void sort_on_obj(int frnt , int m) 
{
//這裡的 m 是 _nobj 。

int pass,i,j;
double hold;
double temp_obj[2*_popsize+1][_nobj+2] ;

  for (i=1 ; i<= _front[frnt].number; i++) 
  {
   temp_obj[i][1] = _front[frnt].member[i] ; // 轉存第幾個解
   temp_obj[i][2] = temp_pop[ _front[frnt].member[i] ].fitness[m] ; //轉存cost value		
  }
  
 
///////////////////////  排序，順序正確即保存  /////////////////////{      
  for (pass=1 ; pass<= _front[ frnt].number-1; pass++)
  {
    for( i=1; i<= _front[ frnt].number-1; i++)
    {
      if (temp_obj[i][2] > temp_obj[i+1][2] )
      {
        for (j=1; j<=2; j++)
        {
        //這裡使用變數的交換儲存技巧
        hold = temp_obj[i][j] ;
        temp_obj[i][j] = temp_obj[i+1][j] ;
        temp_obj[i+1][j] = hold;
        }
      }
    }
  }
///////////////////////  排序，順序正確即保存  /////////////////////}

///////////////////////  將排好的順序重新轉存  ///////////////////{
  for (i=1 ; i<= _front[frnt].number; i++)
  {
  temp_sort[i] = temp_obj[i][1] ;
  }
///////////////////////  將排好的順序重新轉存  ///////////////////}
}
///////////////////////// 按照 cost value 排序  //////////////////////////}

//////////////////////////// //排列解的多樣性  ///////////////////////////////{
// 按照 crodwing distance 排序 (大至小)
void sort_on_crowd(int sw , int gg) 
{

int pass,i,j,k,s,h=0;
double hold;
int pop_size=0 ;

//////////////////  親代的轉存  ////////////////////{
  if (sw == 1)
  {
  pop_size = _popsize ;

    for (i=1; i<=pop_size; i++)
    temp_pop[i].cd_len = oldpop[i].cd_len ;
  }
//////////////////  親代的轉存  ////////////////////}	

//////////////////  子代的轉存  ////////////////////{	
  else if (sw == 3)
  {
  pop_size = _popsize + new_row ;
  
  for (i=1; i<=pop_size; i++)
  temp_pop[i].cd_len = mixpop[i].cd_len ;
  }
//////////////////  子代的轉存  ////////////////////}

  for (i=1; i<=pop_size; i++)
  sorted_crowding[i] = 0 ;  //暫存變數的歸零
  
  for (k=1; k<=front_size; k++)
  {
    for (j=1; j<=_popsize + new_row; j++)  //暫存變數的歸零
    {
    temp_crowd[j][1] = 0. ;
    temp_crowd[j][2] = 0. ;
    }
    
    for (j=1; j<=_front[k].number; j++)
    {
    temp_crowd[j][1] = _front[k].member[j] ;  //轉存解的個數
    temp_crowd[j][2] = temp_pop[ _front[k].member[j] ].cd_len ; //轉存crodwing distance
    }
    ///////////////////////  排序，順序正確即保存  /////////////////////{ 
    for (pass=1; pass<= _front[k].number-1; pass++)
    {
      for (i=1; i<= _front[k].number-1; i++)
      {
        if (temp_crowd[i][2] < temp_crowd[i+1][2])
        {
          for (j=1; j<=2; j++)
          {
           //這裡使用變數的交換儲存技巧
	hold = temp_crowd[i][j] ;
	temp_crowd[i][j] = temp_crowd[i+1][j] ;
	temp_crowd[i+1][j] = hold ;
          }
        }
      }
    }
    ///////////////////////  排序，順序正確即保存  /////////////////////}

    for (s=1; s<=_front[k].number; s++)
    {
     h ++ ;
     sorted_crowding[h] = temp_crowd[s][1] ; //將排序好的結果轉存  (尚未對應到_popsize)
    }	
  }
  
   for (i=1; i<=pop_size; i++)  
   temp_pop[ sorted_crowding[i] ].sorted_crowd =  i ; //設定的數量以符合接下來的轉存，因為親代的數量與子代不同。
   
//////////////////  親代的轉存  ////////////////////{   
   if (sw == 1)
   {
   for (i=1; i<=pop_size; i++)
   oldpop[i].sorted_crowd = temp_pop[i].sorted_crowd ;
   }
//////////////////  親代的轉存  ////////////////////}   

//////////////////  子代的轉存  ////////////////////{   
   else if (sw == 3)
   {
     for (i=1; i<=pop_size; i++)
     {
     mixpop[i].sorted_crowd = temp_pop[i].sorted_crowd ;
     }
    sort_on_rank_crowd(pop_size) ; //做排列子代解的多樣性 
   }
//////////////////  子代的轉存  ////////////////////}
}
//////////////////////////// //排列解的多樣性  ///////////////////////////////}

///////////////////////////   排列子代解的多樣性   ////////////////////////////{
void sort_on_rank_crowd(int pop_size) 
{

int pass,i,j;
double hold;
double temp_obj[2*_popsize+1][_nobj+2] ; //因為子代解至少會比親代多，所以創造2倍的空間

  for (i=1 ; i<= pop_size; i++) 
  {
  temp_obj[i][1] = i ; //轉存解的個數
  temp_obj[i][2] = mixpop[i].sorted_crowd ;//轉存crodwing distance	
  }
 ///////////////////////  排序，順序正確即保存  /////////////////////{   
  for (pass=1 ; pass<= pop_size-1; pass++)
  {
    for( i=1; i<= pop_size-1; i++)
    {
      if (temp_obj[i][2] > temp_obj[i+1][2] )
      {
        for (j=1; j<=2; j++)
        {
         hold = temp_obj[i][j] ;
         temp_obj[i][j] = temp_obj[i+1][j] ;
         temp_obj[i+1][j] = hold;
        }
      }
    }
  }
 ///////////////////////  排序，順序正確即保存  /////////////////////}
 
  for (i=1 ; i<= pop_size; i++)
  {
  mixpop_sorted[i] = temp_obj[i][1] ; //排序完畢，轉存 
  }

}
///////////////////////////   排列子代解的多樣性   ////////////////////////////}

// combine the old and the new population
//就把母代和子代共80個放在一起而已
void mix_population(void ){ 
  int i, w;	
  for (i=1; i<=_popsize; i++)
    mixpop[i] = oldpop[i] ; 
	
  for (i=1, w=(_popsize+1); i<=new_row; i++, w++)
    mixpop[w] = newpop[i] ;
}


/// For GA
// parent chosen: tournament selection
// only for the two-objective problem

//--------------------------------------------------------------------

// Parameter limitation
void checkpop(double* p){  //單純再確認一次p式在上下界範圍內而以
    double pmax, pmin;

    // for each parameter
    for(int i=1; i<=_gap*_rule; i++){
        pmin=min_variable(i);  //在限制前件部的中心點，寬以及後件部的最小值(下界)
        pmax=max_variable(i);  //在限制前件部的中心點，寬以及後件部的最大值(上界)

        // if parameter<lower bound, parameter=lower bound
        if(p[i]<pmin){
            p[i]=pmin;
        }

        // if parameter>upper bound, parameter=upper bound
        if(p[i]>pmax){
            p[i]=pmax;
        }
    }
}

int robot_state_judge(double *dis){
    int hexapod_state=0;

    // 1. too close
    // any Ds return a value<0.2
    //if( (dis[0]<0.27) || (dis[1]<0.27) || (dis[2]<0.27) || (dis[3]<0.27)){
	if( (dis[0]<0.25) || (dis[1]<0.3) || (dis[2]<0.3) || (dis[3]<0.3)){ //20220323
        hexapod_state=COLLIDE;  //COLLIDE=-1

        printf("Colliding.\n");//碰撞
    }

    // 2. away from the wall
    // the left sensor return a value=0.8
    //else if( dis[0]>0.7){
	else if( dis[0]>1.0){ //20220323
        hexapod_state=AWAY;  //AWAY=1
        printf("Out of the route.\n");
        printf("%lf\n",dis[0]);
    }

    return hexapod_state;
}

///////////////////////////////////////////////////////////////////////

void initial(){
  
  for (int i=0; i<NUM_SERVOS; i++){
    wb_motor_set_position(R_servo[i], 0);
  }
  wb_robot_step(500);

  for(int i=1; i<=6; i++){
    leg[i].lpara = kp[i];
    leg[i].lpara2 = kd[i]/0.2;    
  }
  
}


void get_gyro(){

  const double *gyro_values = wb_gyro_get_values(gyro);
  double x[3]={0};  
  
  for (int i=0; i<3; i++){
    x[i] = gyro_values[i];     
    printf("gyro_values[%d] = %lf\t",i,x[i]);
    wb_robot_step(1);
  }
  printf("\n");

}

void get_acce(){
  
  const double *acce_values = wb_accelerometer_get_values(accelerometer);
  double x[3]={0};
  
  for (int i=0; i<3; i++){
    x[i] = acce_values[i];
    printf("acce_values[%d] = %lf\t",i,x[i]);
    wb_robot_step(1);
  }
  printf("\n");
  
}

double get_iner_pitch(){
  
  const double *iner_values = wb_inertial_unit_get_roll_pitch_yaw(inertialunit);
  double x[3]={0};
  
  for (int i=0; i<3; i++){
    x[i] = iner_values[i];
  }

  return x[0];
}

double get_iner_roll(){
  
  const double *iner_values = wb_inertial_unit_get_roll_pitch_yaw(inertialunit);
  double x[3]={0};
  
  for (int i=0; i<3; i++){
    x[i] = iner_values[i];
  }
  
  return x[1];
}

///////////////////////////////////////////////////////////////////////
void robot_get_distance(double *urg){
    const float *urg04lx_values = wb_lidar_get_range_image(R_Urg);
    double minD=255.0;

    for(int i=0; i<NUM_DS; i++){
         urg[i]=255.0;
    }

    for(int i=0; i<NUM_DS; i++){
        if(urg[i]==255.0){
            minD=255.0;
        }

        // 180 degrees = 490 points, about 2.72 points/degree
        switch(i){
            case 0:
                // -90~-67.5 degrees = 88~149
                for(int j=88; j<150; j++){
                    minD=(minD>urg04lx_values[j])?urg04lx_values[j]:minD;
                    
                }
                break;
            case 1:
                // -67.5~-45 degrees = 150~211
                for(int j=150; j<212; j++){
                    minD=(minD>urg04lx_values[j])?urg04lx_values[j]:minD;
                }
                break;
            case 2:
                // -45~-22.5 degrees = 212~273
                for(int j=212; j<274; j++){
                    minD=(minD>urg04lx_values[j])?urg04lx_values[j]:minD;
                }
                break;
            case 3:
                // -22.5~0 degrees = 274~335
                for(int j=274; j<336; j++){
                    minD=(minD>urg04lx_values[j])?urg04lx_values[j]:minD;
                }
                break;
            case 4:
                // 0~22.5 degrees = 336~397
                for(int j=336; j<398; j++){
                    minD=(minD>urg04lx_values[j])?urg04lx_values[j]:minD;
                }
                break;
            case 5:
                // 22.5~45 degrees = 398~459
                for(int j=398; j<460; j++){
                    minD=(minD>urg04lx_values[j])?urg04lx_values[j]:minD;
                }
                break;
            case 6:
                // 45~67.5 degrees = 460~521
                for(int j=460; j<522; j++){
                    minD=(minD>urg04lx_values[j])?urg04lx_values[j]:minD;
                }
                break;
            case 7:
                // 67.5~90 degrees = 522~583
                for(int j=522; j<583; j++){
                    minD=(minD>urg04lx_values[j])?urg04lx_values[j]:minD;
                }
                break;
        }
        urg[i]=minD;
        /*printf("urg[%d]=%f\n",i,minD);   
        fflush(stdout); */ 
          
        /*if (minD<0.2) {minD=0.2;}
        else if (minD>1.5){minD=1.5;}  
        urg[i]=minD/1.3;*/        
    }
        //printf("urg[0]=%f\n",urg[0]);
        fflush(stdout);    
}
///////////////////////////////////////////////////////////////////////
// get the robot position by GPS
void robot_get_position(double *a){
    WbNodeRef robot_node = wb_supervisor_node_get_from_def("Hexa");
    WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
    const double *trans = wb_supervisor_field_get_sf_vec3f(trans_field);
    a[0]=trans[0];
    a[1]=trans[2];
}
///////////////////////////////////////////////////////////////////////

void init_devices(void){

     //Sensor
       SSensor = wb_robot_get_device("sensor_body1");
       wb_distance_sensor_enable(SSensor,TIME_STEP/5);
     
     //Gyro
       gyro = wb_robot_get_device("Gyro1");
       wb_gyro_enable(gyro,TIME_STEP);
       
     //Accelerometer
       accelerometer = wb_robot_get_device("accelerometer1");
       wb_accelerometer_enable(accelerometer,TIME_STEP);
     
     //InertialUnit
       inertialunit = wb_robot_get_device("inertialunit1");
       wb_inertial_unit_enable(inertialunit,TIME_STEP);
       
     // URG
       R_Urg=wb_robot_get_device("URG-04LX");
       wb_lidar_enable(R_Urg, TIME_STEP);       
       
     //Servo
     for(int i=0; i<NUM_SERVOS; i++){
       R_servo[i]=wb_robot_get_device (SERVOS[i]);  
     }
     
}
////////////////////////////////////////////////////////////////////////

int save_gen(int load_gen){

load_gen=load_gen+1;

FILE *generate;  

  generate = fopen("gen_record.txt","w");
   	
  fprintf(generate,"%d\n", load_gen) ;  
  fclose(generate);

printf("save_gen=%d\n",load_gen);

fflush(stdout);

return load_gen;
}


void load_gen_data(){
 
FILE *generate;  

  generate = fopen("gen_record.txt","r");
   
  fscanf(generate,"%d\n",&load_gen);	
  
  fclose(fileerate);

printf("load_in=%d\n",load_gen);

fflush(stdout);
}

void update_ant(int t_loop){

  double aa , mu , sigma ,C1=0;
  int ww , jj,_tour1 ,l_best; 	
  int front_one_counter=0; //計算 front one 的個數的計數器
  
  C1 = load_gen/_maxgen;
  
  ////////////////  計算front1數量  //////////////////{
  for (int i=1; i<=_popsize; i++){
    if(oldpop[i].frnt==1){
        front_one_counter = front_one_counter+1;
    }
  }
  ////////////////  計算front1數量  //////////////////}

  /////////////////////  前半的取樣(1~L)  //////////////////{
  for (int kk=1; kk<=t_loop; kk++){//t_loop為L個
  	
    ////////////  phase1:菁英解(1~L)  ///////////{		
    _tour1 = kk ;   
    ////////////  phase1:菁英解(1~L)  ///////////}
  
    for (ww=1; ww<=_gap*_rule; ww++){
      aa = 0. ;
      mu = oldpop[ _tour1 ].p[ww] ;
      ////////////  phase2:計算標準差  ///////////{
      for(jj=1;jj<=_popsize;jj++){		
        aa += fabs(oldpop[jj].p[ww] - mu);						
      }			
      sigma = kersin * aa/(_popsize -1);
      ////////////  phase2:計算標準差  ///////////}
      newpop[kk].p[ww] = GaussDeviate(mu , sigma) ;  //高斯重新取樣									
    }	
  					
    l_best =1+(int)((front_one_counter-1)*(double)rand()/RAND_MAX);//取local best
  		
    ////////////////////////  向最佳解趨近  //////////////////////{
    for(ww=1; ww<=_gap*_rule; ww++){
      newpop[kk].p[ww] = newpop[kk].p[ww] + C1 * ((double)rand()/RAND_MAX)*(oldpop[l_best].p[ww]-oldpop[kk].p[ww]);			
    }
    //高斯重新取樣的意思是對base(parent_data)做展開，成為新的"群"。
    //向最佳解趨近的過程是，最佳解的位置與群的距離差乘上一個0~1之間的值，加上原本群的位置，即為最佳解的位置。			
    ////////////////////////  向最佳解趨近  //////////////////////}
  
  }
  /////////////////////  前半的取樣(1~L)  //////////////////}
  
  /////////////////////  後半的取樣(L+1~2L)  //////////////////{
  for (int kk=t_loop+1; kk<=new_row; kk++){

  
  	for (ww=1; ww<=_gap*_rule; ww++){						
  		aa = 0. ;		
  		
              	////////////  phase1:競爭解(L+1~2L)  ///////////{		
              	_tour1 = select(oldpop) ; 
              	////////////  phase1:競爭解(L+1~2L)  ///////////}    
              						
  		mu = oldpop[ _tour1 ].p[ww] ;
              	
  		////////////  phase2:計算標準差  ///////////{				
  		for(jj=1;jj<=_popsize;jj++){			
  			aa += fabs(oldpop[jj].p[ww] - mu);			
  		}
  
  		sigma = kersin * aa/(_popsize -1);
  		////////////  phase2:計算標準差  ///////////}
  			
  		newpop[kk].p[ww] = GaussDeviate(mu , sigma) ;  //高斯重新取樣					
  	}
  		
  	///////////////  取local best  //////////////{
  			
  	l_best =1+(int)((front_one_counter-1)*(double)rand()/RAND_MAX);
  
  	///////////////  取local best  //////////////}
  		
  	////////////////////////  向最佳解趨近  //////////////////////{
  	for(ww=1; ww<=_gap*_rule; ww++){
              newpop[kk].p[ww] = newpop[kk].p[ww] + C1 * ((double)rand()/RAND_MAX)*(oldpop[l_best].p[ww]-oldpop[kk].p[ww]);			
  	}
  	
  	//高斯重新取樣的意思是對base(parent_data)做展開，成為新的"群"。
  	//向最佳解趨近的過程是，最佳解的位置與群的距離差乘上一個0~1之間的值，加上原本群的位置，即為最佳解的位置。
  			
  	////////////////////////  向最佳解趨近  //////////////////////}
  
  }
  /////////////////////  後半的取樣(L+1~2L)  //////////////////}
}
///////////////////////////////////////  ACO解的更新  //////////////////////////////////////}

double GaussDeviate(double mu, double sigma)
{
	double fac,r,v1,v2,x,new_var ;
	static int gaussSaved = 0; 
	static double gaussSave;    
	
	if (gaussSaved) 
	{
		x = gaussSave; 
		gaussSaved = 0 ;
	}
	else 
	{
		do
		{
			v1 = 2.0*(double)rand()/RAND_MAX - 1.0 ;
			v2 = 2.0*(double)rand()/RAND_MAX - 1.0 ;
			r  = v1*v1 + v2*v2 ;
		}
		while (r>=1.0) ;
		
		fac = sqrt(-2.0*log(r)/r) ;
		gaussSaved = 1 ;
		gaussSave = v1*fac ;
		x = v2*fac ;
	}
	
	new_var = x * sigma + mu ;
	
	if (new_var > 1.0)
		new_var = 1.0 ;
	
	if (new_var < -1.0)
		new_var = -1.0 ;


	return new_var;
	
}

////////////////////////////////////////////////////////////////////////
int select(individual* pop){
    int winner;
	int cand[2+1], cand_rank[2+1];
	float cand_dist[2+1];
            //floor是返還不大於該值的最大整數值(向下取整數)
	for(int i=1; i<=2; i++){//這邊就是隨機選兩個人出來看誰有交配權
		cand[i]=floor(randn(0,_popsize-1));
		cand_rank[i]=pop[cand[i]].frnt;
		cand_dist[i]=pop[cand[i]].cd_len;
	}

	if(cand_rank[1]>cand_rank[2]){
		winner=cand[2];
	}//先比誰的front小，誰就勝利
	else if(cand_rank[1]<cand_rank[2]){
		winner=cand[1];
	}//當兩個人在同個front，去比誰的crowding distance大就獲勝(越分散)
	else{
		if(cand_dist[1]>cand_dist[2]){
			winner=cand[1];
		}
		else{
			winner=cand[2];
		}
	}

	return winner;
}

////////////////////////////////////////////////////////////////////////