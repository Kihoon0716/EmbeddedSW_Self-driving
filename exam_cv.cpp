#include <iostream>
#include <stdio.h>
//#include <sys/time.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include <opencv2/gpu/device/utility.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include "car_lib.h"
#include <string.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <termio.h>
#include <stdlib.h>
#include <unistd.h>  
#include <sys/time.h> 
#include <time.h>
#include <sys/ioctl.h> 
#include <sys/types.h>
#include <fstream>
#include <math.h>
#include <queue>
#define PI 3.1415926

#define LOCALHOST "10.10.70.1"
#define PORT 7200
#define FRAME_WIDTH         320
#define FRAME_HEIGHT        180

#define RACING_MODE true

#define MODE_STOP 0
#define MODE_LANE_TRACING 1
#define MODE_INTERSECTION_WAITING 2
#define MODE_INTERSECTION_RUN 3
#define MODE_TRAFFIC_LIGHT_WAITING 4
#define MODE_LEFT_LANE_TRACING 5
#define MODE_RIGHT_LANE_TRACING 6
#define MODE_EMERGENCY_STOP 7
#define MODE_PARKING_1 8
#define MODE_PARKING_2 9
#define MODE_OVERKATE 10
#define MODE_FINISH_LINE 11
#define MODE_INTERSECTION_STOP 12
#define MODE_TRAFFIC_LIGHT_LEFT 13
#define MODE_TRAFFIC_LIGHT_RIGHT 14
#define MODE_GO_STRAIGHT 15
#define MODE_FINDING_YELLO_LINE 16
#define MODE_END 17
#define MODE_LEFT_TURN 18

#define NO_LANE_TURN_ANGLE 50
#define NO_LANE_TURN_ANGLE_AFTER 100

#define SPEED_SLOW 150
#define SPEED_FAST 150


using namespace std;
using namespace cv;

extern "C" {


void error(const char *msg)
{
    perror(msg);
    exit(0);
}
bool isIntersectionMissionEnd = false;
int sockfd, portno, n, imgSize, IM_HEIGHT, IM_WIDTH;
struct sockaddr_in serv_addr;
struct hostent *server;
char buffer[256];
Mat cameraFeed;


int wheelAngle;
bool leftLaneExist;
bool rightLaneExist;
float leftLaneAngle;
float rightLaneAngle;
int runningSpeed = SPEED_FAST;


void openStreaming()
{
    cout << "socket init" << endl;
    portno = PORT;
    sockfd = socket(AF_INET, SOCK_STREAM, 0);

    if (sockfd < 0)
        error("ERROR opening socket");

    server = gethostbyname(LOCALHOST);

    if (server == NULL)
    {
        fprintf(stderr, "ERROR, no such host\n");
        exit(0);
    }

    bzero((char *)&serv_addr, sizeof(serv_addr));
    serv_addr.sin_family = AF_INET;
    bcopy((char *)server->h_addr,
          (char *)&serv_addr.sin_addr.s_addr,
          server->h_length);
    serv_addr.sin_port = htons(portno);

    if (connect(sockfd, (struct sockaddr *)&serv_addr, sizeof(serv_addr)) < 0)
        error("ERROR connecting");
    cout << "socket init end" << endl;
}

// 소캣 닫기
void closeStreaming(){
    close(sockfd);
}

// 이미지 전송
void streamingClient(Mat *cameraFeed)
{
    static int c = 1;
    if (c == 1)
        openStreaming();
    c++;
    static int count = 0;
    count++;
    if (count % 3 != 0)
        return;
    imgSize = (*cameraFeed).total() * (*cameraFeed).elemSize();
    n = send(sockfd, (*cameraFeed).data, imgSize, 0);
    if (n < 0)
        error("ERROR writing to socket");
}


class LightControll{
private:
    bool front_on;
    bool back_on;
    int num;
    void turn(){
        if (back_on && front_on){
            CarLight_Write(3);
        }
        else if(!back_on && front_on){
            CarLight_Write(1);
        }
        else if(back_on && !front_on){
            CarLight_Write(2);
        }
        else{
            CarLight_Write(0);
        }
    }
public:
    void frontToggle(){
        front_on = !front_on;
        this->turn();
    }
    void frontOn(){
        front_on = true;
        this->turn();
    }
    void frontOff(){

    }
    void backToggle(){
        back_on = !back_on;
        this->turn();
    }
};
LightControll lightControll;


void CarStop(){
    DesireSpeed_Write(0); // -500 ~ 500
    SteeringServoControl_Write(1500); //1000~2000
}

void CarRun(int angle){
    DesireSpeed_Write(runningSpeed); // -500 ~ 500
    SteeringServoControl_Write(angle); //1000~2000
}

int getch(void)  
{  
  int ch;  
  struct termios buf;  
  struct termios save;  
  
   tcgetattr(0, &save);  
   buf = save;  
   buf.c_lflag &= ~(ICANON|ECHO);  
   buf.c_cc[VMIN] = 1;  
   buf.c_cc[VTIME] = 0;  
   tcsetattr(0, TCSAFLUSH, &buf);  
   ch = getchar();  
   tcsetattr(0, TCSAFLUSH, &save);  
   return ch;  
} 

string getkey(void){
	int temp = getch();
	// direction key
	if(temp == 27){
		getch();
		temp = getch();
		if (temp == 65) return "^";
		else if (temp == 66) return "v";
		else if(temp == 67) return ">";
		else return "<";
	}
	else if(temp == 127) return "back";
	else if(temp == 10) return "enter";
	else if(temp == 32) return "space";
	else {
		string out;
		out += (char)temp;
		return out;
	}
}

typedef struct Mode{
    int run;  // 0 : 정지, 1 : 0 차선주행, 2 : logicalMove, 3 : 외쪽차선 주행, 4 : 오른쪽 차선 주행, 5 : 터널 주행
    int view;
    int cameraAngle;
    int minPixelCount;
    int maxVariance;
}Mode;

struct Points{
public:
    int row;
    int col;
    Points(){}
    Points (int row, int col){
        this->row = row;
        this->col = col;
    }
};

struct Hsv {
public:
    int H_max;
    int H_min;
    int S_max;
    int S_min;
    int V_max;
    int V_min;
};

struct Homography {
public:
    Points upperPoint;
    Points lowerPoint;
};

struct LineDetection {
public:
    Homography homography;
    Hsv binarization;
};

struct TrafficLight {
public:    
	Hsv red;
    Hsv green;
    Hsv orange;
    Hsv arrow;
    int sizeMax;
    int sizeMin;
    float circleConvexityMin;
    float arrowConvexityMax;
    float circleCircularityMin;
    float arrowCircularityMax;
};

struct StopSign {
public:
    Hsv red;
    int sizeMax;
    int sizeMin;
    float convexityMax;
    float convexityMin;
    float circularityMax;
    float circularityMin;
};

struct Params {
public:
    LineDetection lineDetection;
    TrafficLight trafficLight;
    StopSign stopsign;
    Mode mode;
    int streaming; // -1 : 닫기 수행(close),  0 : 닫혀있는 상황, 1 : 열기 수행(open, streamingClient), 2 : 열려있는 상황(streamingClient) 
};

Params * p = new Params();

void moveCamera(){ // 카메라 움직이기
    CameraYServoControl_Write(p->mode.cameraAngle);
}




// 주차장, 추월차로
class logicalMove{
    //SpeedControlOnOff_Write(1);
    //PositionControlOnOff_Write(UNCONTROL);
    //SteeringServoControl_Write() 1000~ 2000
    //DesireEncoderCount_Read
    //DesireEncoderCount_Write
private:
    class logic{
    private:
        int angle;
        int distance;
        int state;
        int wall_distance;
        bool detectWall;
        bool buzzer;
    public:
        logic(){}
        logic(int angle, int distance, bool detectWall, bool buzzer){
            this->angle = angle;
            this->distance = distance;
            this->state = 0; // 0 : ready, 1 : setAngle, 2 : move
            this->detectWall = detectWall;
            this->buzzer = buzzer;
        }
        void move(){
            if(this->state == 0){
                this->state = 1;
            }
            else if(this->state == 1){
                DesireSpeed_Write(0);
                SteeringServoControl_Write(this->angle);

                if(SteeringServoControl_Read() == this->angle){
                    state = 2;
                    DesireEncoderCount_Write(this->distance);
                    //cout << "set : " << DesireEncoderCount_Read() << endl;
                    //cout << "dist" << this->distance << endl;
                }
            }
            else if(this->state == 2){
                if(this->distance > 0) DesireSpeed_Write(100);
                else DesireSpeed_Write(-100);
                //cout << "DesireEncoderCount_Read() : " << DesireEncoderCount_Read() << endl;
                if(abs(DesireEncoderCount_Read()) < 120 || (this->detectWall && LineSensor_Read() == 0)){
                    //cout << " end " << endl;
                    if(buzzer){
                        Alarm_Write(255);
                        sleep(1);
                        Alarm_Write(0);
                    }
                    DesireSpeed_Write(0);
                    state = 0;
                    
                }
            }
        }
        bool isReady(){
            if(this->state == 0) return true;
            else return false;
        }
    };
    int logic_count;
    logic logic_arr[20];
    int state;
    int logic_now;
public:
    logicalMove(){
        logic_count = 0;  
        state = 0;
        logic_now = 0;
    }
    logicalMove * addLogic(int angle, int distance, bool detectWall, bool buzzer){
        logic_arr[logic_count] = logic(angle, distance, detectWall, buzzer);
        logic_count++;
        return this;
    }
    void run(){
        //cout << "logic now" << this->logic_now  << "DesireEncoderCount_Read() : " << DesireEncoderCount_Read() << endl;
        if(this->state == 0){
            SpeedControlOnOff_Write(1);
            PositionControlOnOff_Write(1);
            this->state = 1;
        }
        if(this->state == 1){
            logic_arr[logic_now].move();
            if(logic_arr[logic_now].isReady()){
                this->logic_now++;
            }
            if(this->logic_now == this->logic_count){
                SpeedControlOnOff_Write(0);
                PositionControlOnOff_Write(0);
                this->logic_now = 0;
                this->state = 2;
                
                if(p->mode.run == MODE_OVERKATE){
                        p->mode.run = MODE_TRAFFIC_LIGHT_WAITING;
                        CameraYServoControl_Write(1600);
                        CarStop();
                }
                else if(p->mode.run == MODE_TRAFFIC_LIGHT_LEFT || p->mode.run == MODE_TRAFFIC_LIGHT_RIGHT){
                        cout << "racing end" << endl;
                        p->mode.run = MODE_END;
                        CarStop();
                
                }
                else if(p->mode.run == MODE_LEFT_TURN){
                    p->mode.run = MODE_TRAFFIC_LIGHT_WAITING;
                    CameraYServoControl_Write(1600);
                    lightControll.frontOff();
                    CarStop();
                }
                else{
                    p->mode.run = MODE_LEFT_LANE_TRACING;
                    runningSpeed = SPEED_FAST;
                }
                
            }
        }
    }
};

logicalMove parking1;
logicalMove parking2;
logicalMove overtake;
logicalMove trafficLightLeft;
logicalMove trafficLightRight;
logicalMove leftTurn;
void initParams(){
    // 불 켜기
    lightControll.frontOn();
    // 파일 스트림 열기
    ifstream ifs("params.txt", ios::binary);
    // 읽어오기
    ifs.read((char *)p, sizeof(*p)); 
    ifs.close();
    // 초기 셋팅 설정
    p->mode.run = 0; // 시작과 동시에 출발하려면 1로 변경
    p->streaming = 0;
    p->mode.view = 1;
    p->mode.cameraAngle = 1710;

    // p->lineDetection.homography.upperPoint.row = 18;
    // p->lineDetection.homography.upperPoint.col = 102;

    // p->lineDetection.binarization.H_max = 103;
    // p->lineDetection.binarization.H_min = 60;
    // p->lineDetection.binarization.S_max = 139;
    // p->lineDetection.binarization.S_min = 68;
    // p->lineDetection.binarization.V_max = 255;
    // p->lineDetection.binarization.V_min = 208;




    parking2.addLogic(1500, 700, false, false)
        ->addLogic(1000, -1400, false, false)
        ->addLogic(1500, -500, false, true) // 주차
        ->addLogic(1500, 500, false, false)
        ->addLogic(1000, 1400, false, false);// 복귀

    // parking1.addLogic(1500, 500, false, false)
    //     ->addLogic(1000, -1400, false, false)
    //     ->addLogic(1500, -300, true, false)
    //     ->addLogic(1000, 800, false, false)
    //     ->addLogic(2000, -900, false, true) // 추차
    //     ->addLogic(2000, 900, false, false)
    //     ->addLogic(1000, -800, false, false)
    //     ->addLogic(1500, 300, false, false)
    //     ->addLogic(1000, 1400, false, false);// 복귀
    parking1.addLogic(1500, 650, false, false)
        ->addLogic(1000, - 900, false, false)
        ->addLogic(1400, -300, false, false)
        ->addLogic(2000, -800, false, true)
        ->addLogic(2000, 800, false, false)
        ->addLogic(1400, 300, false, false)
        ->addLogic(1000, 800, false, false);
        //->addLogic(1500, 400, false, false);

 
    trafficLightLeft.addLogic(1500, 800, false, false)
        ->addLogic(2000, 1250, false, false)
        ->addLogic(1500, 800, false, false);

    trafficLightRight.addLogic(1500,800, false, false)
        ->addLogic(1000, 1250, false, false)
        ->addLogic(1500, 800, false, false);

    overtake.addLogic(1500, 1000, true, false)
        ->addLogic(1500, -700, false, false)
        ->addLogic(2000, 350, false, false)
        ->addLogic(1000, 350, false, false)
        ->addLogic(1500, 500, true, true);

    leftTurn.addLogic(1500, 500, false, false)
        ->addLogic(1900, 1800, false, false) //좌회전
        ->addLogic(1500, 200, false, false) // 직진
        ->addLogic(1000, 1000, false, false) // 오른쪽 차선으로 바꾸기
        ->addLogic(2000, 1000, false, false)
        ->addLogic(1500, 500, false, false) //직진
        ->addLogic(2000, 1000, false, false) // 왼쪽 차선으로 바꾸기
        ->addLogic(1000, 1000, false, false)
        ->addLogic(1500, 200, false, true); //직진



    moveCamera();
}

void saveParams(){ 
    // output 파일 스트림을 열고
    ofstream ofs("params.txt", ios::binary);   
    // 그 스트림에 Student 객체를 밀어 넣는다.
    ofs.write((char *)p, sizeof(*p));
    cout << "params saved" << endl;
    ofs.close();
}


class Controller {
private:

public:
    Controller * child[10];
    int childCnt;
    string menuName;
    string paramName;
    string paramType; // Hsv, Point
    int * intParam;
    Hsv * hsvParam;
    Points * pointParam;
    float * floatParam;
    
    bool isMenu;
    bool isMain;
    int selectedMenu;
    // 생성자
    Controller() {

    }
    Controller(string name, bool isMenu) {
        childCnt = 0;
        this->menuName = menuName;
        if (isMenu) {
            this->menuName = name;
        }
        else {
            this->paramName = name;
        }
        this->isMenu = isMenu;
        this->isMain = false;
    }

    void setParam(Points * pointParam) {
        this->pointParam = pointParam;
        this->paramType = "Point";
    }

    void setParam(Hsv * hsvParam) {
        this->hsvParam = hsvParam;
        this->paramType = "Hsv";
    }

    void setParam(int * intParam) {
        this->intParam = intParam;
        this->paramType = "int";
    }

    void setParam(float * floatParam) {
        this->floatParam = floatParam;
        this->paramType = "float";
    }

    void changeParam(string key) {
        if (this->paramType == "Point") {
            changeParamPoint(this->pointParam, key);
        }
        else if(this->paramType == "Hsv"){
            changeParamHsv(this->hsvParam, key);
        }
        else if(this->paramType == "int"){
            changeParamint(this->intParam, key);
        }
        else if(this->paramType == "float"){
            changeParamfloat(this->floatParam, key);
        }
    }

    void changeParamPoint(Points * p, string key) {
        if (key == "<") {
            p->col--;
        }
        else if (key == ">") {
            p->col++;
        }
        else if (key == "^") {
            p->row--;
        }
        else if (key == "v") {
            p->row++;
        }
    }

    void changeParamHsv(Hsv * h, string key){
        if (key == "q"){
            this->hsvParam->H_max += 2;
        }
        else if(key == "a"){
            this->hsvParam->H_max -= 2;
        }
        else if(key == "w"){
            this->hsvParam->H_min += 2;
        }
        else if(key == "s"){
            this->hsvParam->H_min -= 2;
        }
        else if(key == "e"){
            this->hsvParam->S_max += 2;
        }
        else if(key == "d"){
            this->hsvParam->S_max -= 2;
        }
        else if(key == "r"){
            this->hsvParam->S_min += 2;
        }
        else if(key == "f"){
            this->hsvParam->S_min -= 2;
        }
        else if(key == "t"){
            this->hsvParam->V_max += 2;
        }
        else if(key == "g"){
            this->hsvParam->V_max -= 2;
        }
        else if(key == "y"){
            this->hsvParam->V_min += 2;
        }
        else if(key == "h"){
            this->hsvParam->V_min -= 2;
        }
    }

    void changeParamint(int * i, string key){
        if(key == "^"){
            (*i) += 5;
        }
        else if(key == "v"){
            (*i) -= 5;
        }
        if(this->paramName == "Camera Angle") moveCamera();
    }

    void changeParamfloat(float * f, string key){
        if(key == "^"){
            (*f) += 0.01;
        }
        else if(key == "v"){
            (*f) -= 0.01;
        }
    }

    void setIsMain() {
        this->isMain = true;
    }

    void setMenuName(string name) {
        this->menuName = name;
    }

    void printMenu() {
        cout << "*** " << menuName << " ***" << endl << endl;
        for (int i = 0; i < childCnt; i++) {

            if(this->child[i]->isMenu) {
				cout << child[i]->menuName;
				if(this->selectedMenu == i){
					cout << " O" << endl;
				}
				else cout << endl;
			}
            else {
				cout << child[i]->paramName;
				if(this->selectedMenu == i){
					cout << " O" << endl;
				}
				else cout << endl;
			}

        }
    }

    void printParam() {
        cout << "*********** change param ***********" << endl;
        cout << "*** " << this->paramName << " ***" << endl << endl;
		if(this->paramType == "Point"){
			cout << "row : " << this->pointParam->row << endl;
			cout << "col : " << this->pointParam->col << endl;
		}
        else if(this->paramType == "Hsv"){
            cout << "H Max : " << this->hsvParam->H_max << endl;
            cout << "H min : " << this->hsvParam->H_min << endl;
            cout << "S Max : " << this->hsvParam->S_max << endl;
            cout << "S min : " << this->hsvParam->S_min << endl;
            cout << "V Max : " << this->hsvParam->V_max << endl;
            cout << "V min : " << this->hsvParam->V_min << endl;
        }
        else if(this->paramType == "int"){
            cout << "value : " << *this->intParam << endl;
        }
        else if(this->paramType == "float"){
            cout << "value : " << *this->floatParam << endl;
        }
    }

    Controller * addChild(Controller * child) {
        this->child[childCnt] = child;
        childCnt++;
        return this;
    }

    void start() {
        string key;
        selectedMenu = 0;
        while (1) {
            system("clear");
            if(isMenu) printMenu();
            if (!isMenu) printParam();

            // 키 입력
            
            key = getkey();
        

            // 메뉴일 때
            if (this->isMenu) {
                if (key == "^" && selectedMenu > 0) {
                    selectedMenu--;
                }
                else if (key == "v" && selectedMenu < childCnt - 1) {
                    selectedMenu++;
                }
                // 엔터 눌렀을 때
                else if (key == "enter") {
                    child[selectedMenu]->start();
                }
                // 뒤로가기 눌렀을 때
                else if (key == "back" && !this->isMain) {
                    break;
                }
                else if(key == "1"){
                    p->mode.view = 1;
                }
                else if(key == "2"){
                    p->mode.view = 2;
                }
                else if(key == "3"){
                    p->mode.view = 3;
                }
                else if(key == "4"){
                    p->mode.view = 4;
                }
                else if(key == "5"){
                    p->mode.view = 5;
                }
                else if(key == "6"){
                    p->mode.view = 6;
                }
                else if(key == "7"){
                    p->mode.view = 7;
                }
                else if(key == "8"){
                    p->mode.view = 8;
                }
                else if(key == "9"){
                    p->mode.view = 9;
                }
                else if(key == "m"){
                    if(p->streaming == 0) p->streaming = 1;
                    else if(p->streaming == 2) p->streaming = -1;
                }
                else if(key == "space"){
                    if(p->mode.run == 0) p->mode.run = MODE_LEFT_LANE_TRACING;
                    else p->mode.run = 0;
                }
                else if(key == "s"){ // 파라미터 저장
                    saveParams();
                }
                else if(key == "l"){ // 파라미터 불러오기
                    lightControll.frontToggle();
                }
                else if(key == "z"){ // 주차장 1번
                    p->mode.run = MODE_PARKING_1;
                }
                else if(key == "x"){ // 주차장 2번
                    p->mode.run = MODE_PARKING_2;
                }
                else if(key == "c"){ // 추월 미션 테스트
                    isIntersectionMissionEnd = true;
                    p->mode.run = MODE_RIGHT_LANE_TRACING;
                }
                else if(key == "b"){ // 신호등 오른쪽
                    p->mode.run = MODE_TRAFFIC_LIGHT_RIGHT;
                }
                else if(key == "n"){ // 신호등 테스트
                    p->mode.run = MODE_TRAFFIC_LIGHT_WAITING;
                    CameraYServoControl_Write(1500);
                }
            }
            // 파라미터일때
            else {
                // 엔터 눌렀을 때
                if (key == "enter") {
                    break;
                }
                // 뒤로가기 눌렀을 때
                else if (key == "back") {
                    break;
                }
                else if(key == "1"){  // 기본
                    p->mode.view = 1;
                }
                else if(key == "2"){  // homography 상단 점
                    p->mode.view = 2;
                } 
                else if(key == "3"){  // homography rgb
                    p->mode.view = 3;
                }
                else if(key == "4"){  // homography 이진화
                    p->mode.view = 4;
                 }
                else if(key == "5"){  // 돌발표지
                    p->mode.view = 5;
                }
                else if(key == "6"){  // 신호등 빨간색
                    p->mode.view = 6;
                }
                else if(key == "7"){  // 신호등 주황색
                    p->mode.view = 7;
                }
                else if(key == "8"){  // 신호등 초록색
                    p->mode.view = 8;
                }
                else if(key == "9"){  // 신호등 화살표
                    p->mode.view = 9;
                }
                else if(key == "m"){
                    if(p->streaming == 0) p->streaming = 1;
                    else if(p->streaming == 2) p->streaming = -1;
                }
                else if(key == "space"){
                    if(p->mode.run == 0) p->mode.run = MODE_LEFT_LANE_TRACING;
                    else p->mode.run = 0;
                }
                else if(key == "l"){ // 파라미터 불러오기
                    lightControll.frontToggle();
                }
                else {
                    changeParam(key);
                }
            }
        }
    }
};

void keyboardController(Params* params) {
    p = params;
    //main
    Controller * main = new Controller("Main", true);
    
        // mian -> line detection
        Controller * lineDetection = new Controller("Line Detection", true);
            // mian -> line detection -> homography
            Controller * homography = new Controller("Homography", true);
				Controller * upperPoint = new Controller("Upper Point", false);// 파라미터
				upperPoint->setParam(&params->lineDetection.homography.upperPoint);
            // mian -> line detection -> binarization
            Controller * binarization = new Controller("Binarization", false);
            binarization->setParam(&params->lineDetection.binarization);

    
        // main -> traffic light
        Controller * trafficLight = new Controller("Traffic Light", true);
            //main -> traffic light -> traffic light red
            Controller * trafficLightRed = new Controller("traffic Light Red", false);
            trafficLightRed->setParam(&params->trafficLight.red);
            Controller * trafficLightGreen = new Controller("traffic Light Green", false);
            trafficLightGreen->setParam(&params->trafficLight.green);
            Controller * trafficLightorange = new Controller("traffic Light Orange", false);
            trafficLightorange->setParam(&params->trafficLight.orange);
            Controller * trafficLightArrow = new Controller("traffic Light Arrow", false);
            trafficLightArrow->setParam(&params->trafficLight.arrow);
            Controller * trafficLightMaxSize = new Controller("traffic Light Max Size", false);
            trafficLightMaxSize->setParam(&params->trafficLight.sizeMax);
            Controller * trafficLightMinSize = new Controller("traffic Light Min Size", false);
            trafficLightMinSize->setParam(&params->trafficLight.sizeMin);
            Controller * trafficLightCircleConvexityMin = new Controller("traffic Light Circle Convexity Min", false);
            trafficLightCircleConvexityMin->setParam(&params->trafficLight.circleConvexityMin);    
            Controller * trafficLightArrowConvexityMax = new Controller("traffic Light Arrow Convexity Max", false);
            trafficLightArrowConvexityMax->setParam(&params->trafficLight.arrowConvexityMax);
            Controller * trafficLightCircleCircularityMin = new Controller("traffic Light Circle Circularity Min", false);
            trafficLightCircleCircularityMin->setParam(&params->trafficLight.circleCircularityMin);    
            Controller * trafficLightArrowCircularityMax = new Controller("traffic Light arrow Circularity Max", false);
            trafficLightArrowCircularityMax->setParam(&params->trafficLight.arrowCircularityMax);
        // main -> stop sign
        Controller * stopSign = new Controller("StopSign", true);
            // main -> stop sign -> Hsv
            Controller * stopSignHsv = new Controller("Stop Sign Hsv", false);
            stopSignHsv->setParam(&params->stopsign.red);
            Controller * stopSignMaxSize = new Controller("Stop Sign Size Max", false);
            stopSignMaxSize->setParam(&params->stopsign.sizeMax);
            Controller * stopSignMinSize = new Controller("Stop Sign Size Min", false);
            stopSignMinSize->setParam(&params->stopsign.sizeMin);
            Controller * stopSignMaxConvexity = new Controller("Stop Sign Max Convexity", false);
            stopSignMaxConvexity->setParam(&params->stopsign.convexityMax);
            Controller * stopSignMinConvexity = new Controller("Stop Sign Min Convexity", false);
            stopSignMinConvexity->setParam(&params->stopsign.convexityMin);
            Controller * stopSignMaxCircularity = new Controller("Stop Sign Max Circularity", false);
            stopSignMaxCircularity->setParam(&params->stopsign.circularityMax);
            Controller * stopSignMinCircularity = new Controller("Stop Sign Min Circularity", false);
            stopSignMinCircularity->setParam(&params->stopsign.circularityMin);
        
        // main-> mode
        Controller * mode = new Controller("Mode", true);
            //main -> mode -> cameraAngle
            Controller * cameraAngle = new Controller("Camera Angle", false);
            cameraAngle->setParam(&params->mode.cameraAngle);
            Controller * minPixelCount = new Controller("Min Pixel Count", false);
            minPixelCount->setParam(&params->mode.minPixelCount);
            Controller * maxPixelVariance = new Controller("Max Pixel Variance", false);
            maxPixelVariance->setParam(&params->mode.maxVariance);
    main->addChild(lineDetection)
        ->addChild(trafficLight)
        ->addChild(stopSign)
        ->addChild(mode)
        ->setIsMain();
		lineDetection->addChild(homography)
			->addChild(binarization);
		    homography->addChild(upperPoint);
        trafficLight->addChild(trafficLightRed)
            ->addChild(trafficLightGreen)
            ->addChild(trafficLightorange)
            ->addChild(trafficLightArrow)
            ->addChild(trafficLightMaxSize)
            ->addChild(trafficLightMinSize)
            ->addChild(trafficLightCircleCircularityMin)
            ->addChild(trafficLightArrowCircularityMax)
            ->addChild(trafficLightCircleConvexityMin)
            ->addChild(trafficLightArrowConvexityMax);
        stopSign->addChild(stopSignHsv)
            ->addChild(stopSignMaxSize)
            ->addChild(stopSignMinSize)
            ->addChild(stopSignMaxConvexity)
            ->addChild(stopSignMinConvexity)
            ->addChild(stopSignMaxCircularity)
            ->addChild(stopSignMinCircularity);
        mode->addChild(cameraAngle)
            ->addChild(minPixelCount)
            ->addChild(maxPixelVariance);
    main->start();
}



class point{
public:
    int x;
    int y;
};

#define SCENARIO_STRAIGHT_LANE_1 0
#define SCENARIO_PARKINT_1 1
#define SCENARIO_STRAIGHT_LANE_2 2
#define SCENARIO_CURVE_1_RIGHT 3
#define SCENARIO_STRAIGHT_LANE_3 4
#define SCENARIO_PARKINT_2 5
#define SCENARIO_STRAIGHT_LANE_4 6
#define SCENARIO_CURVE_2_RIGHT 7
#define SCENARIO_INTERSECTION 8
#define SCENARIO_STRAIGHT_LANE_5 9
#define SCENARIO_CURVE_3_LEFT 10
#define SCENARIO_HILL 11
#define SCENARIO_CURVE_4_LEFT 12
#define SCENARIO_STRAIGHT_LANE_6 13
#define SCENARIO_TUNNEL 14
#define SCENARIO_STRAIGHT_LANE_7 15
#define SCENARIO_CURVE_5_LEFT 16
#define SCENARIO_STRAIGHT_LANE_8 17
#define SCENARIO_CHANGE_LANE 18
#define SCENARIO_GO_TO_TRAFFIC_LIGHT 19
#define SCENARIO_WAITING_TRAFFIC_LIGHT 20
#define SCENARIO_GO_TO_GOAL 21
#define SCENARIO_END 22

class Scenario{
private:
    int now_;
    string scenarioList[100];
    int stop;
    int go_straight;
public:
    int state = 0;
    Scenario(){
        
        now_ = 0;
        stop = 0;
        go_straight = 0;
        scenarioList[0] = "straight lane 1";
        scenarioList[1] = "parking 1";
        scenarioList[2] = "straight lane 2";
        scenarioList[3] = "curve 1 (right)";
        scenarioList[4] = "straight lane 3";
        scenarioList[5] = "parking 2";
        scenarioList[6] = "straight lane 4";
        scenarioList[7] = "curve 2 (right)";
        scenarioList[8] = "intersection";
        scenarioList[9] = "straight lane 5";
        scenarioList[10] = "curve 3 (left)";
        scenarioList[11] = "hill";
        scenarioList[12] = "curve 4 (left)";
        scenarioList[13] = "straight lane 6";
        scenarioList[14] = "tunnel";
        scenarioList[15] = "straight lane 7";
        scenarioList[16] = "curve 5 (left)";
        scenarioList[17] = "straight lane 8";
        scenarioList[18] = "change lane";
        scenarioList[19] = "go to traffic light";
        scenarioList[20] = "waiting traffic light";
        scenarioList[21] = "go to goal";
        scenarioList[22] = "end scenario";
    };
    void next(){
        now_++;
        stop = 0;
        go_straight = 0;
    }
    int now(){
        return now_;
    }
    void watchLane(){ // -1 왼쪽 커브, 0 직진, 1 오른쪽 커브 // 허프에서 에서 동작
        
        static int left = 0;
        static int straight = 0;
        static int right = 0;
        
        
        if(abs(wheelAngle - 1500) < 50){
            straight++;
        }
        else straight = 0;
        
        if(wheelAngle >= 1600){ // 좌회전
            left++;
        }
        else left = 0;

        if(wheelAngle <= 1400){ // 우회전
            right++;
        }
        else right = 0;

        if(straight >= 3 && state != 0){
            state = 0;
            straight = 0;
            Winker_Write(0);
        }
        else if(left >= 3 && state != -1){
            state = -1;
            left = 0;
            Winker_Write(2);
        }
        else if(right >=3 && state != 1){
            state = 1;
            right = 0;
            Winker_Write(1);
        }

    }

    void run(){ //thread2에서 동작
        static int count = 0;

        if(now_ == SCENARIO_STRAIGHT_LANE_1){
            if(stop == 0){
                stop = 1;
                EncoderCounter_Write(0);
            }
            if(EncoderCounter_Read() > 30000 && stop == 1){
                stop = 2;
                p->mode.run = MODE_EMERGENCY_STOP;
                
                
            }
            if(stop == 2){
                count++;
            }
            if(count == 45){
                p->mode.run = MODE_LEFT_LANE_TRACING;
            }
        }

    }
};

Scenario scenario;

// thread2에서 돌리는 함수들
// 주자장 구간 감지
// 주차장 알고리즘을 시작하는 지점을 좀 더 자동차가 앞으로 간 후로 설정하고 싶으면 IR센서를 2번이 아닌 3번을 활용
void detectParkingLot(){
    const int DETECTING_DISTANCE = 600;//600
    static int parking_lot_now = 1;
    static int wall_detect_count = 0;
    static int state = 1;
    if(parking_lot_now == -1 || !(p->mode.run == MODE_LEFT_LANE_TRACING || p->mode.run == MODE_RIGHT_LANE_TRACING))return;

    if(parking_lot_now == 1){
        if(state == 1){ // 일반 주행 하는중
            if(DistanceSensor(2) > DETECTING_DISTANCE) wall_detect_count++;
            else wall_detect_count = 0;
            if(wall_detect_count > 3)
            {
                state = 2;
                lightControll.backToggle();
                wall_detect_count = 0;
            }

        }
        else if(state == 2){
            if(DistanceSensor(2) < DETECTING_DISTANCE) wall_detect_count++;
            else wall_detect_count = 0;
            if(wall_detect_count > 3){
                state = 3;
                wall_detect_count = 0;
                runningSpeed = SPEED_SLOW;
                lightControll.backToggle();
            }
        }
        else if(state == 3){
            if(DistanceSensor(2) > DETECTING_DISTANCE) wall_detect_count++;
            else wall_detect_count = 0;
            if(wall_detect_count > 3){
                state = 1;
                wall_detect_count = 0;
                p->mode.run = MODE_PARKING_1;
                parking_lot_now = 2;
                lightControll.backToggle();
            }
        }
    }

    else if(parking_lot_now == 2){
        if(state == 1){ // 일반 주행 하는중
            if(DistanceSensor(2) > DETECTING_DISTANCE) wall_detect_count++;
            else wall_detect_count = 0;
            if(wall_detect_count > 3){
                state = 2;
                wall_detect_count = 0;
                lightControll.backToggle();
            }
        }
        else if(state == 2){
            if(DistanceSensor(2) < DETECTING_DISTANCE) wall_detect_count++;
            else wall_detect_count = 0;
            if(wall_detect_count > 3){
                state = 3;
                runningSpeed = SPEED_SLOW;
                lightControll.backToggle();
            }
        }
        else if(state == 3){
            if(DistanceSensor(2) > DETECTING_DISTANCE) wall_detect_count++;
            else wall_detect_count = 0;
            if(wall_detect_count > 3){
                state = 1;
                p->mode.run = MODE_PARKING_2;
                parking_lot_now = -1;
                lightControll.backToggle();
            }
        }
    }
}

void toBinary(int num, int * output){
    for(int i = 0; i < 8; i++){
        output[i] = num%2;
        num = num>>1;
    }
}

// 바닥 흰색 선 감지 -> 교차로, 신호등 구간
void detectWhiteLine(){
    int line = LineSensor_Read();
    //cout << line << endl;
    //if(line == 0) cout << "find zero" << endl;
    static int mission = 1; // 1 : 교차로, 2 : 신호등
    static int line_count = 0;
    int scan[8];
    toBinary(line, scan);
    static int count = 0;
    count++;

    // if (line != 127)
    // {
    //     cout << "line : ";
    //     for (int i = 0; i < 8; i++)
    //     {
    //         cout << scan[i] << " ";
    //     }
    //     cout << "     " << line;
    //     cout << endl;
    // }
    if(mission == -1) return;
    if(p->mode.run == MODE_LEFT_LANE_TRACING || p->mode.run == MODE_RIGHT_LANE_TRACING){
        if(mission == 1){
            if(line == 0) line_count++;
            else line_count = 0;
            if(line_count >= 1){
                //cout << "stop" << endl;
                p->mode.run = MODE_INTERSECTION_WAITING;
                mission = -1;
                //mission = 2;
                line_count = 0;
            }
        } 
        else if(mission == 2){
            if(LineSensor_Read() == 0) line_count++;
            else line_count = 0;
            if(line_count >= 3){
                p->mode.run = MODE_TRAFFIC_LIGHT_WAITING;
                mission = -1;
                line_count = 0;
            }
        }
    }
}



// 교차로에서 앞의 차량과 간격 유지 
void followOtherCar(){
    const int closeDistance = 200; // 앞의 차가 가까이 왔음을 인지하는 기준거리
    const int followDistance = 200; // 앞의 차와의 일정 거리를 유지하는 기준거리
    static int close_count = 0;
    const int runDistance = 70000;

    //cout << p->mode.run << endl;
    if(p->mode.run == MODE_INTERSECTION_WAITING){
        //cout << "count : " << close_count << endl;
        if(DistanceSensor(1) > closeDistance) close_count++;
        //cout << close_count << endl;
        if(close_count > 10) {
            p->mode.run = MODE_INTERSECTION_RUN; // 교차로에서 움직이기 시작
            EncoderCounter_Write(0);
        }
    }
    else if(p->mode.run == MODE_INTERSECTION_RUN || p->mode.run == MODE_INTERSECTION_STOP){
        // 거리가 일정 수치 이상 -> 움직이기
        if(DistanceSensor(1) < followDistance)  p->mode.run = MODE_INTERSECTION_RUN;
        else {
            p->mode.run = MODE_INTERSECTION_STOP;
            CarStop();
        }
        if(EncoderCounter_Read() > runDistance){ // 정해진 거리 이상 움직이면 일반모드로 변경
        cout << "end" << endl;
        isIntersectionMissionEnd = true;
            p->mode.run = MODE_RIGHT_LANE_TRACING;
        }
    }
    else return;
}

// 적외선 센서로 긴급정지바 확인
void stopSignDetectionByDistance(){
    cout << "distance : " << DistanceSensor(1) << endl;
    static int count = 0;
    if(p->mode.run == MODE_LEFT_LANE_TRACING || p->mode.run == MODE_RIGHT_LANE_TRACING){
        if(DistanceSensor(1) > 1000){
            count++; 
        }
        else count = 0;
        if(count > 3) {
            p->mode.run = MODE_EMERGENCY_STOP;
            count = 0;
        }
    }
    if(p->mode.run == MODE_EMERGENCY_STOP){
        if(DistanceSensor(1) < 300){
            count++;
        }
        else count = 0;
        if(count > 3){
            p->mode.run = MODE_LEFT_LANE_TRACING;  // 긴급정지 끝난이후 따라갈 차선 선택
        }
    }
}

//thread2

void thread2(){ 
    
    //stopSignDetectionByDistance();
    // /scenario.run();
    
   
}


// 색상 인식1
Mat lightDetector(Mat *originRGB, Mat *outputRGB, Hsv hsv, int maxSize, int minSize, float maxCircularity, float minCircularity, float maxConvexity, float minConvexity, int *detectCount)
{
    // blob detection으로 구현 필요
    Mat HsvImg, HsvBinary, HsvBinaryNot;
    cvtColor(*originRGB, HsvImg, COLOR_BGR2HSV);
    // Detect the object based on HSV Range Values

    inRange(HsvImg, Scalar(hsv.H_min, hsv.S_min, hsv.V_min), Scalar(hsv.H_max, hsv.S_max, hsv.V_max), HsvBinary);
    bitwise_not(HsvBinary, HsvBinaryNot);

    // Setup SimpleBlobDetector parameters.
    SimpleBlobDetector::Params params;
    std::vector<KeyPoint> keypoints;
    // Change thresholds
    //params.minThreshold = 10;
    //params.maxThreshold = 200;

    // Filter by Area.
    params.filterByArea = true;
    params.minArea = minSize;
    params.maxArea = maxSize;

    // Filter by Circularity
    params.filterByCircularity = true;
    params.minCircularity = minCircularity;
    params.maxCircularity = maxCircularity;

    // Filter by Convexity
    params.filterByConvexity = true;
    params.maxConvexity = maxConvexity;
    params.minConvexity = minConvexity;

    // Filter by Inertia
    params.filterByInertia = false;

    // Set up detector with params
    SimpleBlobDetector detector(params);

    // You can use the detector this way
    // detector.detect( im, keypoints);

    // Detect blobs.
    detector.detect(HsvBinaryNot, keypoints);

    // Draw detected blobs as red circles.
    // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
    Mat im_with_keypoints;
    *detectCount = keypoints.size();
    drawKeypoints(*outputRGB, keypoints, *outputRGB, Scalar(255, 255, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

    // return 이진화 이미지로 필터링된 칼라이미지
    Mat bitwiseRGB;
    //bitwise_and(*originRGB, HsvBinary, bitwiseRGB);
    (*originRGB).copyTo(bitwiseRGB, HsvBinary);
    drawKeypoints(bitwiseRGB, keypoints, bitwiseRGB, Scalar(255, 255, 255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    return bitwiseRGB;
}

// 신호등
Mat trafficLightDetection(Mat * originRGB, Mat * outputRGB){
    static int trafficLightType = 0;
    // originRGB 활용해서 신호등 검출하고 outputRGB에 표시
    // 검출된 신호에 따라 run 값을 변경
    // 빨간색, 주황색 0 (정지)
    // 초록색 2 (화살표가 향한 방향의 반대 차선으로만 추행)
    // 화살표 3 (화살표가 향한 방향의 차선으로만 주행)
    if(p->mode.view == 6) trafficLightType = 0;
    else if(p->mode.view == 7) trafficLightType = 1;
    else if(p->mode.view == 8) trafficLightType = 2; // 초록
    else if(p->mode.view == 9) trafficLightType = 3; // 화살표
    int detectedCount = 0;
    Mat bitwiseRGB;
    if(trafficLightType == 0){ //빨간색
        bitwiseRGB = lightDetector(originRGB, outputRGB, p->trafficLight.red, p->trafficLight.sizeMax, p->trafficLight.sizeMin, 1, p->trafficLight.circleCircularityMin, 1, p->trafficLight.circleConvexityMin, &detectedCount);
    }
    else if(trafficLightType == 1){ //주황색
        bitwiseRGB = lightDetector(originRGB, outputRGB, p->trafficLight.orange, p->trafficLight.sizeMax, p->trafficLight.sizeMin, 1, p->trafficLight.circleCircularityMin, 1, p->trafficLight.circleConvexityMin, &detectedCount);
    }
    else if(trafficLightType == 2){ //초록색
        //bitwiseRGB = lightDetector(originRGB, outputRGB, p->trafficLight.green, p->trafficLight.sizeMax, p->trafficLight.sizeMin, 1, p->trafficLight.circleCircularityMin, 1, p->trafficLight.circleConvexityMin, &detectedCount);
        bitwiseRGB = lightDetector(originRGB, outputRGB, p->trafficLight.green, p->trafficLight.sizeMax, p->trafficLight.sizeMin, 1.0, 0.8, 1.0, 0.87, &detectedCount);
    }
    else if(trafficLightType == 3){ //초록 화살표
        
        //bitwiseRGB = lightDetector(originRGB, outputRGB, p->trafficLight.arrow, p->trafficLight.sizeMax, p->trafficLight.sizeMin, p->trafficLight.arrowCircularityMax, 0, p->trafficLight.arrowConvexityMax, 0, &detectedCount);
        bitwiseRGB = lightDetector(originRGB, outputRGB, p->trafficLight.arrow, p->trafficLight.sizeMax, p->trafficLight.sizeMin, 0.8, 0, 1.0, 0, &detectedCount);
    }




    if(p->mode.run == MODE_TRAFFIC_LIGHT_WAITING){
        if(trafficLightType == 3 && detectedCount > 0){
            cout << "arrow detected" << endl;
            p->mode.run = MODE_TRAFFIC_LIGHT_LEFT;
        }
        else if(trafficLightType == 2 && detectedCount > 0){
            cout << "green detected" << endl;
            p->mode.run = MODE_TRAFFIC_LIGHT_RIGHT;
        }
    }
    

    (++trafficLightType) %= 4;
    return bitwiseRGB;
}

// 돌발표지
Mat stopSignDetection(Mat * originRGB, Mat * outputRGB){
    Mat bitwiseRGB;
    int detectedCount = 0;
    //bitwiseRGB = lightDetector(originRGB, outputRGB, p->stopsign.red, p->stopsign.sizeMax, p->stopsign.sizeMin, p->stopsign.circularityMax, p->stopsign.circularityMin, p->stopsign.convexityMax, p->stopsign.convexityMin, &detectedCount);
    bitwiseRGB = lightDetector(originRGB, outputRGB, p->stopsign.red, p->stopsign.sizeMax, p->stopsign.sizeMin, 1, 0.8 , 1, 0.8, &detectedCount);
    static int findCount = 0;
    static int missionEnd = false;
    if ((p->mode.run == MODE_LEFT_LANE_TRACING || p->mode.run == MODE_RIGHT_LANE_TRACING) && !missionEnd){
        if (detectedCount > 0){
            findCount++;
        }
        else findCount = 0;
        if (findCount == 1){
            p->mode.run = MODE_EMERGENCY_STOP;
            findCount = 0;
        }
    }
    if(p->mode.run == MODE_EMERGENCY_STOP){
        if(detectedCount == 0){
            findCount++;
        }
        else findCount = 0;
        if(findCount > 10){
            p->mode.run = MODE_LEFT_LANE_TRACING;
            missionEnd = true;
        }
    }
    return bitwiseRGB;
}

//커브 카운팅
void countCurve(){
    static int count_curve = 0;
    static bool isReady = true;
    if((wheelAngle > 1650|| wheelAngle < 1350) && isReady){
        count_curve++;
        isReady = false;
        EncoderCounter_Write(0);

        // 특정 커브를 만나는 순간 특정 행동을 하도록
        if(count_curve == 10){
            p->mode.run = MODE_LEFT_TURN;
        }
    }
    if(isReady == false){
        if(EncoderCounter_Read() > 20000){
            isReady = true;
        }
    }
    if(EncoderCounter_Read() > 50000 && count_curve == 4){
        // 특정 커브를 지나고 특정 거리만큼 이동 했을 때 특정 미션을 수행 ex) S자 커브, 내리막길
    }
}


inline int readBinaryRGB(Mat * homographyBitwiseRGB, int row, int col){
    if((*homographyBitwiseRGB).at<Vec3b>(row,col)[0] == 0) return 0;
    else return 1;
}

// 180 x 320
inline int laneScanner(Mat * homographyBitwiseRGB, int side, int scanningRow){// side : 0이면 왼쪽 1이면 오른쪽, 스캔되는 점이 없으면 -1 있으면 평균값 반환

    int colStart = 0;
    if(side == 1){
        colStart += 100;
    }
    // 값 읽는 방법 readBinaryRGB(homographyBitwiseRGB, scanningRow, col)
    // 평균 구하기
    int col_sum = 0;
    int col_mean = 0;
    int col_count = 0;
    for(int i = 0; i < 220; i++){
        col_sum += readBinaryRGB(homographyBitwiseRGB, scanningRow, colStart + i) * (colStart + i);
        col_count += readBinaryRGB(homographyBitwiseRGB, scanningRow, colStart + i);
    }
    //cout << "count : " << col_count << endl;
    if(col_count == 0) return -1;
    col_mean = col_sum / col_count;

    // 감지된 픽셀이 최소 갯수를 못넘으면 -1 리턴
    if(col_count < p->mode.minPixelCount) return -1;

    // 분산 구하기 : 제곱의 평균
    int col_mean_square = 0;
    int col_variance = 0;
    for(int i = 0; i < 220; i++){
        col_mean_square += (col_mean - (colStart +i)) * (col_mean - (colStart +i)) * readBinaryRGB(homographyBitwiseRGB, scanningRow, colStart + i);
    }
    col_variance = col_mean_square / col_count;
    // 픽셀들의 분산이 최댓값을 넘기면 -1 리턴
    if(col_variance > p->mode.maxVariance) return -1;

    return col_mean;
}

inline void laneDetector(Mat * homographyBitwiseRGB, Mat * outputRGB, Points * left_start, Points * left_end, Points * right_start, Points * right_end){
    // 왼쪽 차선 // 320 x 180
    left_start->row = 170;
    while(1){
        left_start->col = laneScanner(homographyBitwiseRGB, 0, left_start->row);
        if(left_start->col > 0) break;
        if(left_start->row <= 50) break;
        left_start->row -= 10;
    }

    if (left_start->col > 0){ // 시작점을 찾은 경우
        left_end->row = left_start->row - 30;
        while (1){
            left_end->col = laneScanner(homographyBitwiseRGB, 0, left_end->row);
            if (left_end->col > 0) break;
            if (left_end->row <= 10)break;
            left_end->row -= 10;
        }
    }

    // 오른쪽 차선
    right_start->row = 170;
    while(1){
        right_start->col = laneScanner(homographyBitwiseRGB, 1, right_start->row);
        if(right_start->col > 0) break;
        if(right_start->row <= 50) break;
        right_start->row -= 10;
    }

    if (right_start->col > 0){ // 시작점을 찾은 경우
        right_end->row = right_start->row - 30;
        while (1){
            right_end->col = laneScanner(homographyBitwiseRGB, 1, right_end->row);
            if (right_end->col > 0) break;
            if (right_end->row <= 10)break;
            right_end->row -= 10;
        }
    }
}

int getSign(float num){
    if(num > 0) return 1;
    else return -1;
}

// int find_angle(float x1, float y1, float x2, float y2, string lane_select){
//     const float gain = 1.0;
//     if (x1 == x2)
//     {
//         x1 += 1;
//     }
//     float a = (y2 - y1) / (x2 - x1);
//     float b = -1;
//     float c = -a * x1 + y1;
//     // 자동차의 위치 (180 + 90) : 90을 수정
//     float distance = abs((a * (160) + b * (-10) + c) / sqrt(a * a + b * b)); // distance 가 + 이면 차선이 차체의 오른쪽에 있고 - 이면 차선이 왼쪽에 있다.
//     int angle = 1500 + (distance - 120) * gain;
//     //cout << "angle : " << angle << endl;
//     if(getSign(a) == 1){
//         angle -= (PI/2 - atan(a)) * 180 / PI * 7.5;
//         // cout << "atan(a) : " << atan(a) << endl;
//         // cout << "(PI/2 - atan(a)) * 180 / PI * 4 : " << (PI/2 - atan(a)) * 180 / PI * 4 << endl;
//     }
//     else {
//         angle += (PI/2 + atan(a)) * 180 / PI * 7.5;
//     }
//     // cout << "x1 : " << x1 << "   y1 : " << y1 << endl;
//     // cout << "x2 : " << x2 << "   y2 : " << y2 << endl;
//     //cout << "distance : " << distance << endl;
//     // cout << "a: " << a << endl;

//     //cout << "angle : " << angle << endl;
//     //cout << endl;
//     return angle;
// }

int find_angle(Mat * homography, Mat * homographyBitwiseRGB, Points left_start, Points left_end, Points right_start, Points right_end){
    float x1;
    float y1;
    float x2;
    float y2;
    if (p->mode.run == MODE_LEFT_LANE_TRACING){
        x1 = (float)left_start.col;
        y1 = (float)180 - left_start.row;
        x2 = (float)left_end.col;
        y2 = (float)180 - left_end.row;
        line(*homography, Point(left_start.col, left_start.row), Point(left_end.col, left_end.row), Scalar(0, 0, 255), 2); // Point(col, row)
        line(*homographyBitwiseRGB, Point(left_start.col, left_start.row), Point(left_end.col, left_end.row), Scalar(0, 0, 255), 2); // Point(col, row)
    }
    else if(p->mode.run == MODE_RIGHT_LANE_TRACING){
        x1 = (float)right_start.col;
        y1 = (float)180 - right_start.row;
        x2 = (float)right_end.col;
        y2 = (float)180 - right_end.row;
        line(*homography, Point(right_start.col, right_start.row), Point(right_end.col, right_end.row), Scalar(255, 0, 0), 2); // Point(col, row)
        line(*homographyBitwiseRGB, Point(right_start.col, right_start.row), Point(right_end.col, right_end.row), Scalar(255, 0, 0), 2); // Point(col, row)
    }

    const float gain = 1.0;
    if (x1 == x2){
        x1 += 1;
    }
    float a = (y2 - y1) / (x2 - x1);
    float b = -1;
    float c = -a * x1 + y1;
    // 자동차의 위치 (180 + 90) : 90을 수정
    float distance = abs((a * (160) + b * (-10) + c) / sqrt(a * a + b * b)); // distance 가 + 이면 차선이 차체의 오른쪽에 있고 - 이면 차선이 왼쪽에 있다.
    
    int angle = 1500;
    if(p->mode.run == MODE_LEFT_LANE_TRACING){
        angle = 1500 + (distance - 120) * gain;
    }
    else if(p->mode.run == MODE_RIGHT_LANE_TRACING){
        angle = 1500 - (distance - 150) * gain; // 차선과의 간격 조정 수치를 크게할수록 차선과 멀어짐
    }
    
    //cout << "angle : " << angle << endl;
    if (getSign(a) == 1){
        angle -= (PI / 2 - atan(a)) * 180 / PI * 7.5; //7.5
        // cout << "atan(a) : " << atan(a) << endl;
        // cout << "(PI/2 - atan(a)) * 180 / PI * 4 : " << (PI/2 - atan(a)) * 180 / PI * 4 << endl;
    }
    else{
        angle += (PI / 2 + atan(a)) * 180 / PI * 7.5; //7.5
    }
    return angle;
}

void findLaneAngle(Points left_start, Points left_end, Points right_start, Points right_end, float* leftAngle, float* rightAngle){
    float left_x1 = (float)left_start.col;
    float left_y1 = (float)180 - left_start.row;
    float left_x2 = (float)left_end.col;
    float left_y2 = (float)180 - left_end.row;
    *leftAngle = (left_y2 - left_y1) / (left_x2 - left_x1);

    float right_x1 = (float)right_start.col;
    float right_y1 = (float)180 - right_start.row;
    float right_x2 = (float)right_end.col;
    float right_y2 = (float)180 - right_end.row;
    *rightAngle = (right_y2 - right_y1) / (right_x2 - right_x1);
}

Mat laneFollow(Mat * originRGB, Mat * outputRGB){// 320 x 180
    static int crossOverMissionCount = 0;
    static bool crossOverMissionStart = false;

    if(crossOverMissionStart){
        cout << crossOverMissionCount << endl;
        crossOverMissionCount++;
        if(crossOverMissionCount > 40){
            cout << "crossOverMisson start" << endl;
            crossOverMissionStart = false;
            p->mode.run == MODE_OVERKATE;
            lightControll.backToggle();
        }
    }
    // 1. 호모그래피
    Mat homography, homographyGray, homographyBinary, homographyHsv;
    Mat frontViewWithSquareRGB = (*originRGB).clone();
    std::vector<Point2f> points1, points2;

    wheelAngle = 1500;
    int col1 = 160 - p->lineDetection.homography.upperPoint.col;
    int row1 = 90 + p->lineDetection.homography.upperPoint.row;
    int col2 = 160 + p->lineDetection.homography.upperPoint.col;
    int row2 =  90 + p->lineDetection.homography.upperPoint.row;
    int col3 = 0;
    int row3 = 179;  
    int col4 = 319;
    int row4 = 179;
    points1.push_back(Point2f(col1, row1));  // col, row
    points1.push_back(Point2f(col2, row2));
    //points1.push_back(Point2f(160 - p->lineDetection.homography.lowerPoint.col, 90 - p->lineDetection.homography.lowerPoint.row));
    //points1.push_back(Point2f(160 + p->lineDetection.homography.lowerPoint.col, 90 - p->lineDetection.homography.lowerPoint.row));
    points1.push_back(Point2f(col3, row3)); // 좌하단
    points1.push_back(Point2f(col4, row4)); //우하단

    points2.push_back(Point2f(80, 90));  // 좌상단
    points2.push_back(Point2f(240, 90)); // 우상단
    points2.push_back(Point2f(80, 179)); // 좌하단
    points2.push_back(Point2f(240, 179)); //우하단
    // frontViewWithSquareRGB 에 점이랑 직선 그려주기
    if(p->mode.view == 2){
        CarStop();
        circle(frontViewWithSquareRGB, cvPoint(col1, row1), 5, CV_RGB(255, 0, 0));
        circle(frontViewWithSquareRGB, cvPoint(col2, row2), 5, CV_RGB(255, 0, 0));
        circle(frontViewWithSquareRGB, cvPoint(col3, row3), 5, CV_RGB(255, 0, 0));
        circle(frontViewWithSquareRGB, cvPoint(col4, row4), 5, CV_RGB(255, 0, 0));
        return frontViewWithSquareRGB;
    }

    // Find homography
    Mat h = findHomography( points1, points2, RANSAC );
        
    // Use homography to warp image
    warpPerspective(*originRGB, homography, h, originRGB->size());

    // 흑백
    cvtColor( homography, homographyHsv, CV_RGB2HSV );
    // 흑백 이진화
    inRange(homographyHsv, Scalar(p->lineDetection.binarization.H_min, p->lineDetection.binarization.S_min, p->lineDetection.binarization.V_min), Scalar(p->lineDetection.binarization.H_max, p->lineDetection.binarization.S_max, p->lineDetection.binarization.V_max), homographyBinary);
    Mat homographyBitwiseRGB;
    Mat whiteImg(180, 320, CV_8UC3, Scalar(255,255, 255));
    whiteImg.copyTo(homographyBitwiseRGB, homographyBinary);    
    // 2. 차선 인식
    Points left_start(-1,-1);
    Points left_end(-1,-1);
    Points right_start(-1,-1);
    Points right_end(-1,-1);
    laneDetector(&homographyBitwiseRGB, outputRGB, &left_start, &left_end, &right_start, &right_end);
    leftLaneExist = (left_end.col > 0);
    rightLaneExist = (right_end.col > 0);

    findLaneAngle(left_start, left_end, right_start, right_end, &leftLaneAngle, &rightLaneAngle);


    // 차선 갈아타기 버전 2    
    if(p->mode.run == MODE_LEFT_LANE_TRACING){
        if(rightLaneAngle < 0 && rightLaneAngle > -2){
            p->mode.run = MODE_RIGHT_LANE_TRACING;
        }
    }
    else if(p->mode.run == MODE_RIGHT_LANE_TRACING){
        if(leftLaneAngle > 0 && leftLaneAngle < 2){
            p->mode.run = MODE_LEFT_LANE_TRACING;
        }
    }

    if(p->mode.run == MODE_LEFT_LANE_TRACING){
        if(leftLaneExist){
           wheelAngle = find_angle(&homography, &homographyBitwiseRGB, left_start, left_end, right_start, right_end); 
        }
        else{
            wheelAngle = 1550;
            if(isIntersectionMissionEnd){
                wheelAngle = 1600;
            }
        }
    }
    
    else if(p->mode.run == MODE_RIGHT_LANE_TRACING || p->mode.run == MODE_INTERSECTION_RUN){
        if(rightLaneExist){
            wheelAngle = find_angle(&homography, &homographyBitwiseRGB, left_start, left_end, right_start, right_end); 
            crossOverMissionCount = 0;
        }
        else{
            wheelAngle = 1450;
            if(isIntersectionMissionEnd){
                crossOverMissionCount++;
                wheelAngle = 1400;
                if(crossOverMissionCount > 5){
                    crossOverMissionStart = true;
                    crossOverMissionCount = 0;
                    lightControll.backToggle();
                }
            }
        }
    }

    // 각도가 특정수치 이상 넘으면 코너로 인식
    if(wheelAngle > 1650){
        //p->mode.run = MODE_LEFT_TURN;
    }

    if(p->mode.run == MODE_LEFT_LANE_TRACING || p->mode.run == MODE_RIGHT_LANE_TRACING || p->mode.run == MODE_LANE_TRACING){
        CarRun(wheelAngle);
        //SteeringServoControl_Write(angle);
    }
    else if(p->mode.run == MODE_GO_STRAIGHT){
        CarRun(1500);
    }
    else if(p->mode.run == MODE_STOP){
        CarStop();
    }
    else if(p->mode.run ==MODE_INTERSECTION_RUN){
        cout << "set angle" << endl;
        SteeringServoControl_Write(wheelAngle);
    }
    else if(p->mode.run == MODE_TRAFFIC_LIGHT_WAITING){
        CarStop();
    }
    if(p->mode.view == 4) {
        
        //cout << "white image size" << whiteImg.size() << endl;
        //cout << "homography image size" << homography.size() << endl;

        //cout << "homographyBitwiseRGB image size" << homographyBitwiseRGB.size() << endl;
        return homographyBitwiseRGB;
        //return homography;
    }
    else return homography;
}


void OpenCV_hough_transform(unsigned char *srcBuf, int iw, int ih, unsigned char *outBuf, int nw, int nh){
    //cout << "MODE_RUN : " << p->mode.run << endl;
    static int count = 0;
    if(count == 0){
        count++;
        EncoderCounter_Write(0);
    }
    cout <<"encoder : " << EncoderCounter_Read() << endl;
    cout << "running speed : " << runningSpeed << endl;
    if(p->mode.run == MODE_EMERGENCY_STOP || p->mode.run == MODE_INTERSECTION_STOP || p->mode.run == MODE_INTERSECTION_WAITING){
        DesireSpeed_Write(0);
    }
    if(p->mode.run == MODE_INTERSECTION_RUN){
        //cout << "run" << endl;
        DesireSpeed_Write(SPEED_SLOW);
    }
    if(p->mode.run == MODE_PARKING_1){
        parking1.run();
    }
    if(p->mode.run == MODE_PARKING_2){
        parking2.run();
    }
    if(p->mode.run == MODE_OVERKATE){
        overtake.run();
    }
    if(p->mode.run == MODE_TRAFFIC_LIGHT_LEFT){
        trafficLightLeft.run();
    }
    if(p->mode.run == MODE_TRAFFIC_LIGHT_RIGHT){
        trafficLightRight.run();
    }
    if(p->mode.run == MODE_END){
        CarStop();
        return;
    }
    if(p->mode.run == MODE_LEFT_TURN){
        leftTurn.run();
    }
    detectWhiteLine();
    followOtherCar();
    detectParkingLot();
    

    // 손을 왼쪽 상단 적외선 센서에 대면 출발
    static int startCount = 0;
    if(startCount == 0 && DistanceSensor(6) > 2000 && RACING_MODE){
        p->mode.run = MODE_LEFT_LANE_TRACING;
        startCount = 1;
    }

    //scenario.run();
    //scenario.watchLane();
    // if(scenario.state == -1){
    //     cout << "left" << endl;
    // }
    // else if(scenario.state == 0){
    //     cout << "straight" << endl;
    // }
    // else if(scenario.state == 1){
    //     cout << "right" << endl;
    // }
    // 앤코더

    //  static int count = 0;
    //  if(count == 0){
    //      count++;
    //     SpeedControlOnOff_Write(0);
    //     PositionControlOnOff_Write(0);
    //  }
    
    

    //cout << LineSensor_Read() << endl;
    //cout << DistanceSensor(1) << endl;    
    // if(p->mode.run == MODE_LEFT_LANE_TRACING){
    //     cout << "left" << endl;
    // }
    // else if(p->mode.run == MODE_RIGHT_LANE_TRACING){
    //     cout << "right" << endl;
    // }
    /// 초음파센서 : 전방부터 시계방향으로 1~6번
    // cout << "channel1 : " << DistanceSensor(1) << endl;
    // cout << "channel2 : " << DistanceSensor(2) << endl;
    // cout << "channel3 : " << DistanceSensor(3) << endl;
    // cout << "channel4 : " << DistanceSensor(4) << endl;
    // cout << "channel5 : " << DistanceSensor(5) << endl;
    // cout << "channel6 : " << DistanceSensor(6) << endl << endl << endl << endl;

    // static int count = 0;
    // count++;
    // Winker_Write((count/100)%4);
    // CarLight_Write((count/100)%4);
    //cout << p->mode.run << endl;
    
    static int initParamsCount = 0;
    initParamsCount++;
    if(initParamsCount == 1) initParams();
    // 변수 설정
    Mat srcRGB(ih, iw, CV_8UC3, srcBuf); // 320 x 180
    //Mat imgStream;
    Mat *imgStream;
    Mat output = srcRGB.clone();
    Mat view;
    Mat frontOrTopView; // mode.view 가 2면 front(호모그래피 파라미터 확인용) , 3이면 top
    Mat dstRGB(nh, nw, CV_8UC3, outBuf);
    frontOrTopView = laneFollow(&srcRGB, &output);
    Mat trafficLightDetectView = trafficLightDetection(&srcRGB, &output);
    Mat stopSignDetectView = stopSignDetection(&srcRGB, &output);
    //2는 Top뷰 3은 호모그래피 점 셋팅 중일 때 front뷰
    if(p->mode.view == 2 || p->mode.view == 3 || p->mode.view == 4){
         imgStream = &frontOrTopView;
         //imgStream = frontOrTopView.clone();
    }  
    else if(p->mode.view == 1){
         imgStream = &output;
         //imgStream = output.clone();
    }
    else if (p->mode.view == 6 || p->mode.view == 7 || p->mode.view == 8 || p->mode.view == 9){
        imgStream = &trafficLightDetectView;
    }else if(p->mode.view == 5){
        imgStream = &stopSignDetectView;
    }

    // LCD 출력
    cv::resize(*imgStream, dstRGB, cv::Size(nw, nh), 0, 0, CV_INTER_LINEAR);
    // 소켓통신용 이미지
    Mat imgStreaming = (*imgStream).clone();
    // 소켓통신
    if(p->streaming == -1){
        //closeStreaming();
        p->streaming = 0;
    }
    else if(p->streaming == 1){
        p->streaming = 2;
        streamingClient(&imgStreaming);
    }
    else if(p->streaming == 2){
        streamingClient(&imgStreaming);
    }
    if(p->mode.run == 2){ // 주차장 또는 추월

    }
}


}

// scp camera_opencv_disp root@10.10.70.4:/home/root/ 
// ssh root@10.10.70.4

// scp .profile root@10.10.70.4:/home/root/