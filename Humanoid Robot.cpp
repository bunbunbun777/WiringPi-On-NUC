#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.h"
#include <iostream>
#include <stdlib.h>
#include <sstream>
#include <string>
#include <math.h>
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>
#include <unistd.h>
#include <wiringPi.h>
#include <wiringSerial.h>


using namespace std;
using namespace cv;

//declare 
Mat cameraFeed, HSV, HSL, LAB, threshold, hsv, hsl, lab;
Mat camFeed, hasil_canny, merge, total;


//define untuk komunikasi data
// CM --> komunikasi data dari Raspberry Pi ke CM530
int CM[] = {29, 22, 23, 25, 30, 21};
#define CMi 24

//mode --> komunikasi data dari Arduino Nano ke Raspberry Pi
int mode[] = {8, 9, 7, 0, 2, 3, 12, 13, 14};

#define mode_trig 1

//sm --> untuk switch mode
int sm[] = {4, 5, 6, 31, 26};
int led[] = { 27, 28};

//digunakan untuk reset program arduino nano
#define reset 11

//digunakan untuk mengetahui flow program arduino nano
#define ff 10

//define nomor motion peritah gerakan
#define berdiri 1
#define jalanTempat 2
#define maju 3
#define majuKiri 4
#define majuKanan 5
#define putarKiri 7
#define putarKanan 6
#define sampingKiri 8
#define sampingKanan 9
#define frontKiri 10
#define frontKanan 11
#define tendangKiri 12
#define tendangKanan 13
#define bangunDepan 14
#define bangunBelakang 15
#define kiperReady 16
#define kiperKiri 17
#define kiperKanan 18
#define kiperDepan 19
#define jongkok 20
#define bangunJongkok 21

//define nilai sesuai kesepakatan komunkasi antara raspi - arduino
#define jatuhDepan 400
#define jatuhBelakang 401
#define jatuhKiri 403
#define jatuhKanan 404

#define cariGaris 0
//putih (HSV)
int H_MIN = 0; //87 -->gelap, 0 --> terang
int H_MAX = 255;
int S_MIN = 0; //46 -->gelap, 0
int S_MAX = 255;
int V_MIN = 201; //166 -->gelap, 207
int V_MAX = 255;

//Putih (HSL)
int H_MINL = 0; //50 -->gelap, 0
int H_MAXL = 255;
int S_MINL = 233; //170 buat tempat terang 233 buat tempat gelap
int S_MAXL = 255;
int L_MIN = 0; //35 -->gelap, 0 -->terang
int L_MAX = 255;

//(LAB)
int L_MINL = 162; //178-198 jika diruangan terang, 162 gelap
int L_MAXL = 255;
int A_MIN = 77;
int A_MAX = 255;
int B_MIN = 84;
int B_MAX = 255;

int fd;
int motion = 0;
int mulai = 1;
unsigned short obj_target = cariGaris;
	
string intToString(int number)
{
    stringstream ss; //std::stringstream ss; 
    ss << number;
    return ss.str();
}

const string nameHSL = "Trackbars Putih HSL";
const string nameHSV = "Trackbars Putih HSV";
const string nameLAB = "Trackbars Putih LAB";

void on_trackbar( int, void* )
{	//This function gets called whenever a
    // trackbar position is changed
}

void createTrackBarsHSL()
{
	namedWindow(nameHSL, 1);
		
	char TrackbarName[50];
    
	createTrackbar("H_MIN", nameHSL, &H_MINL, 255, on_trackbar);
    createTrackbar("H_MAX", nameHSL, &H_MAXL, 255, on_trackbar);
    createTrackbar("S_MIN", nameHSL, &S_MINL, 255, on_trackbar);
    createTrackbar("S_MAX", nameHSL, &S_MAXL, 255, on_trackbar);
    createTrackbar("L_MIN", nameHSL, &L_MIN, 255, on_trackbar);
    createTrackbar("L_MAX", nameHSL, &L_MAX, 255, on_trackbar);
    imshow(nameHSL, hsl);
}

void createTrackBarsHSV()
{
	namedWindow(nameHSV, 1);
		
	char TrackbarName[50];
    
	createTrackbar("H_MIN", nameHSV, &H_MIN, 255, on_trackbar);
    createTrackbar("H_MAX", nameHSV, &H_MAX, 255, on_trackbar);
    createTrackbar("S_MIN", nameHSV, &S_MIN, 255, on_trackbar);
    createTrackbar("S_MAX", nameHSV, &S_MAX, 255, on_trackbar);
    createTrackbar("V_MIN", nameHSV, &V_MIN, 255, on_trackbar);
    createTrackbar("V_MAX", nameHSV, &V_MAX, 255, on_trackbar);
    imshow(nameHSV, hsv);
}

void createTrackBarsLAB()
{
	namedWindow(nameLAB, 1);
		
	char TrackbarName[50];
    
	createTrackbar("H_MIN", nameLAB, &L_MINL, 255, on_trackbar);
    createTrackbar("H_MAX", nameLAB, &L_MAXL, 255, on_trackbar);
    createTrackbar("S_MIN", nameLAB, &A_MIN, 255, on_trackbar);
    createTrackbar("S_MAX", nameLAB, &A_MAX, 255, on_trackbar);
    createTrackbar("V_MIN", nameLAB, &B_MIN, 255, on_trackbar);
    createTrackbar("V_MAX", nameLAB, &B_MAX, 255, on_trackbar);
    imshow(nameLAB, lab);
}



void morphOps(Mat &thresh)
{
	Mat erodeElement = getStructuringElement(MORPH_RECT, Size(3,3));
	Mat dilateElement = getStructuringElement(MORPH_RECT, Size(3,3));
	
	erode(thresh, thresh, erodeElement);
	erode(thresh, thresh, erodeElement);
	dilate(thresh, thresh, dilateElement);
	dilate(thresh, thresh, dilateElement);
}

void servoGerak(int a, int b)
{
    a = 150 - a;
    b = 195 - b; //135 untuk hampir lurus

    string as = intToString(a);
    if(a<100) {
        if(a<10) {
            as = "00"+as;
        }
        else {
            as = "0"+as;
        }
    }

    string bs = intToString(b);
    if(b<100) {
        if(b<10) {
            bs = "00"+bs;
        }
        else {
            bs = "0"+bs;
        }
    }
    string wow = "#"+as+bs+"\r";
    const char *cat = wow.c_str();
    serialPuts (fd, cat) ;
    //cout <<"gerak  " <<a <<"   " <<b  <<"     ";
    //delay(2000);
}

int pow2(int a) 
{
    if(a==0)
        return 1;
    else if(a==1)
        return 2;
    else
        return 2* pow2(a-1);
}

int cekSensor()
{
    int i[10], sum;
    sum = 0;

    for(int i=0 ; i<=8 ; i++) {
        sum = sum + (digitalRead(mode[i]) * pow2(i));
    }
    return sum;
}

void send2CM (int a) 
{
    for(int i=5; i>=0 ; i--) {
        int pangkat = pow2(i);
        if(a>=pangkat) {
            digitalWrite(CM[i], HIGH);
            a -= pangkat;
			cout <<"1";
        }
        else {
            digitalWrite(CM[i], LOW);
            cout <<"0";
		}
        //if (a==0 && i>0)
        //    break;
    }
}

void myInterrupt(void) 
{
    cout <<"interrupt  ";
    
    if(digitalRead(CMi) == 1) {
        motion = motion; 
    }
    else {
        motion = 63; //63 --> dummy number
    }
    
    cout <<motion <<endl;
    send2CM(motion);
    motion = 1;
    //motion = a+1;
}

void sendCM(int a)
{   
    send2CM(a);
}

void ledState(int a) {
    if(a>=2) { a-=2; digitalWrite(led[1], HIGH); } else digitalWrite(led[1], LOW);
    if(a>=1) { a-=1; digitalWrite(led[0], HIGH); } else digitalWrite(led[0], LOW);
}

void initKomunikasi()
{
    for(int i=0; i<=8 ; i++) pinMode(mode[i], INPUT);
    for(int i=0; i<=5 ; i++) pinMode(CM[i], OUTPUT);
    for(int i=0; i<=5 ; i++) pinMode(sm[i], INPUT);
    for(int i=0; i<=1 ; i++) pinMode(led[i], OUTPUT);

    pinMode(reset, OUTPUT);
    pinMode(mode_trig, OUTPUT);
    //digitalWrite(reset,LOW);
    //delay(500);
    digitalWrite(reset,HIGH);
    digitalWrite(mode_trig, HIGH);
    delay(500);
}

int main(int argc, char* argv[])
{
	//Mat cameraFeed, hsv, hsl, lab, threshold, white, white2, white3, etc, cmf, merge, total;
	
	bool useMorphOps = true;
	
	//createTrackBarsHSL();
	//createTrackBarsHSV();
	//createTrackBarsLAB();
	
	VideoCapture capture;
	capture.open(0);
	capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
	capture.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
	
	
	//untuk memulai komunikasi data yang digunakan
    if ((fd=serialOpen ("/dev/ttyS0", 9600)) < 0){
       fprintf (stderr, "Unable to open serial device: %s\n", strerror (errno)) ;
    	return 1 ;
    }
    
    if (wiringPiSetup () == -1){
       fprintf (stdout, "Unable to start wiringPi: %s\n", strerror (errno)) ;
    	return 1 ;
    }
    
    if ( wiringPiISR (CMi, INT_EDGE_BOTH, &myInterrupt) < 0 ) {
      fprintf (stderr, "Unable to setup ISR: %s\n", strerror (errno));
		return 1;
    }
    
    initKomunikasi();
    
    char key = 0;
	double x = 0, y = 0;
	float x1=0, y1=0, x2=0, y2=0;
	int kx = 0, ky = 60;
	float p1 = 0, p2 = 0, p3 = 0, p4 = 0;
	float sudutgaris = 0;
	int Sensor;
	motion = 0;
	sendCM(motion);
	cout<<"lines.size()"<<"\t"<<"x1"<<"\t"<<"y1"<<"x2"<<"\t"<<"y2"<<"\t"<<"\n";
	
	while(key != 'Q' && key != 'q')
	{
		capture.read(cameraFeed);
	    camFeed = cameraFeed;
		cvtColor(cameraFeed, HSV, COLOR_BGR2HSV);
		cvtColor(cameraFeed, HSL, COLOR_BGR2HLS);
		cvtColor(cameraFeed, LAB, COLOR_BGR2Lab);
		
				

		Sensor = cekSensor();
		servoGerak(0, 60);
		
		//for(int i=0; i<5 ; i++) cout<<digitalRead(sm[i]);
		//cout<<"\t";
				
		if(Sensor == jatuhDepan) {
            motion = bangunDepan;
        }
        else if(Sensor == jatuhBelakang || Sensor == jatuhKanan || Sensor == jatuhKiri) {
            motion = bangunBelakang;
        }
		
		if(obj_target == cariGaris)
		{
			cout << Sensor << "\t";
			
			inRange(HSV, Scalar(H_MIN, S_MIN, V_MIN), Scalar(H_MAX, S_MAX, V_MAX), hsv);
			inRange(HSL, Scalar(H_MINL, S_MINL, L_MIN), Scalar(H_MAXL, S_MAXL, L_MAX), hsl);
			inRange(LAB, Scalar(L_MINL, A_MIN, B_MIN), Scalar(L_MAXL, A_MAX, B_MAX), lab);
			
			if(useMorphOps)
			{
				morphOps(hsv);
				morphOps(hsl);
				morphOps(lab);
			}
			
		createTrackBarsHSL();
		createTrackBarsHSV();
		createTrackBarsLAB();

		imshow("Original", cameraFeed);

		bitwise_and(hsv, lab, total);
		Canny(total, hasil_canny, 50, 200, 3);

		vector<Vec4i> lines;
		HoughLinesP(hasil_canny, lines, 1, CV_PI/180, 40, 40, 60);

		cout<<lines.size()<<"\t";
			
		for(size_t i = 0 ; i<lines.size() ; i++)
        {
            Vec4i l = lines[i];
            
            line(camFeed, Point(l[0],l[1]), Point(l[2],l[3]), Scalar(0,0,255), 3, CV_AA);
            
            x1 = l[0];
            y1 = l[1];
            x2 = l[2];
            y2 = l[3];
        }

       	cout << x1 << "\t" << y1 << "\t" << x2 << "\t" << y2 << "\t" << "\n";

        imshow("Merge", hasil_canny);
        imshow("Detect Lines", camFeed);

		if(waitKey(30) == 27)
		{
			cout<<"ESC key is PRESSED by USER"<<endl;
			break;
		}
	}
			//bitwise_and(white2, white, merge);
			//bitwise_and(merge, white3, total);
			//bitwise_and(white, white3, total);
			
			//Canny(total, etc, 50, 200, 3);
			
			vector<Vec4i> lines;
			//HoughLinesP(etc, lines, 1, CV_PI/180, 40, 40, 60);
			
			cout<<lines.size()<<"\t";
			
			for(size_t i = 0 ; i<lines.size() ; i++)
            {
                Vec4i l = lines[i];
                
                line(camFeed, Point(l[0],l[1]), Point(l[2],l[3]), Scalar(0,0,255), 3, CV_AA);
                
                p1 = l[0];
                p2 = l[1];
                p3 = l[2];
                p4 = l[3];
            }
            
            cout << p1 << "\t" << p2 << "\t" << p3 << "\t" << p4 << "\t";
            
            if(mulai) 
			{
                if(lines.size() > 0)
				{
                    if(p1 >= 45 && p1 <= 190 && p3 >= 90 && p3 <= 245)
						motion = maju;
					else
					if(p1 >= 200 && p1 <= 310 && p3 >= 200 && p3 <= 320)
						motion = putarKanan;
					else
					if(p1 >= 0 && p1 <= 70 && p3 >= 0 && p3 <= 190)
						motion = putarKiri;  	   
				}               
                else 
				{
                    motion = maju;
                }
           	}
            else 
			{
                motion = berdiri;
            }
		
			//imshow("Original", cameraFeed);
			//imshow("HSV", HSV);
			//imshow("Filtered HSV", white2);
			//imshow("Filtered LAB",white3);
			//imshow("Merge", total);
			//imshow("LAB", LAB);
			//imshow("HSL", HSL);
			//imshow("Filtered HSL", white);
			//imshow("Canny", etc);
			//imshow("Detect Lines", cmf);
			
		}
		cout<<endl;
		
		key = cvWaitKey(1);
		if(digitalRead(sm[3]) == 1) 
		{
       		//ledState(2);
			key ='q';
		}
		
		/*if(waitKey(30) == 27)
		{
			cout<<"ESC key is PRESSED by USER"<<endl;
			break;
		}*/
	}
	
