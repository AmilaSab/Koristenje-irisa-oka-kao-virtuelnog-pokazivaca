#include "mainwindow.h"
#include <QApplication>
#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include<sstream>
#include <ueye.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/objdetect/objdetect.hpp>
#include <cmath>
#include <vector>
#include <math.h>
#include <opencv2/plot.hpp>

#define PI 3.14159265358979323846

using namespace std ;
using namespace cv;

int thres_varijabla =30;
char TrackbarName[50];
char TrackbarName2[25];
int pauziraj;
int problem(0);
int odabrane_tri_tacke(0);
int nRet ;
int Mode ;
char strCamFileName[256] ;
int img_width = 641;
int img_height = 482;
int pritisnuta_jedinica(0), pritisnuta_dvica(0), pritisnuta_trica(0), pritisnuta_cetvorka(0);
int rekalibracija(0);
double x1, x2, y2, x3, y3, x4, y4;
double iks, iksic, ipsiloncic, ips;
int ips1;
int kalibracija(0);
int C;
double a1, b1, c1, a2, b2, c2;
int cetiri_kruga(0);
int thresh2 = 100;
int max_thresh2 = 255;
int varijabla(0);
char c;
int provjera_stanja (0);
char k;
int brojac(0);
int izvrsilo_se_barem_jednom(0);

vector<double> ipsilon;
vector<Point2f> curvePoints;
vector<Point> koordinate_krugova;
vector<vector<double> > iksevi;
vector<Point> centers;

Point gornji_lijevo, gornji_desno, donji_desno, donji_lijevo;
Point kut1, kut2, kut3, kut4;
Point lastPoint;
Point mousePoint;
Point2i pos_1, pos_2, pos_3, pos_4;
Point2i centar;

HIDS hCam = 200;
HIDS hCam2 = 250;

SENSORINFO sensor_info ;
CAMINFO camera_info ;


string upozorenje = "";

Mat slika_oka;
Mat src; Mat src_gray; Mat src_bin;
Mat src_cropped;  Mat gray;
Mat canny_output;
Mat bw;
Mat image;
Mat ekran;
Mat image1;
Mat image2;
Mat slika_druga;
Mat xyz;

CascadeClassifier eyeCascade;


/// Funkcije
void slider_callback(int, void*);
double Ugao_rotacije();
void inicijalizacijaUSBkamere1();
Point2i Plotaj(Mat);
void nacrtaj(double a1, double b1, double c1);
Mat grabFromUSBcamera(HIDS);
Point stabilize(vector<Point>*, int);
vector<vector<double> > Inverzna(vector<vector<double> >*);
void init_2Dvector(vector<vector<double> >& vec, int x_dim, int y_dim);
vector<double> Odredjivanje_parametara(double, int, double, double, double, double);
Rect getLeftmostEye(std::vector<cv::Rect>*);
void calculateInverse(vector< vector<double>>*);
void inicijalizacijaUSBkamere2();
void prikaz_svih_slika(Mat &slika1, Mat &slika2, Mat &slika3, Mat &slika4, Mat &slika5, Mat &slika6);
void CallBackFunc(int event, int x, int y, int flags, void* userdata);
void onMouse(int, int , int , int , void* );
Point2i Plotaj_drugi_nacin(Mat slika);
int  da_li_je_oko_zatvoreno(Mat, vector<Point2f>, vector<Point2f>);




 Mat show_histogram(std::string const& name, cv::Mat1b const& image)
 {

     int bins = 256;
     int histSize[] = {bins};

     float lranges[] = {0, 256};
     const float* ranges[] = {lranges};

     cv::Mat hist;
     int channels[] = {0};

     int const hist_height = 256;
     cv::Mat3b hist_image = cv::Mat3b::zeros(hist_height, bins);
     cv::calcHist(&image, 1, channels, cv::Mat(), hist, 1, histSize, ranges, true, false);

     double max_val=0;
     minMaxLoc(hist, 0, &max_val);

     for(int b = 0; b < bins; b++) {
         float const binVal = hist.at<float>(b);
         int   const height = cvRound(binVal*hist_height/max_val);
         cv::line
             ( hist_image
             , cv::Point(b, hist_height-height), cv::Point(b, hist_height)
             , cv::Scalar::all(255)
             );
     }
     cv::imshow(name, hist_image);
     return hist_image;
 }

 void calculateInverse(vector< vector<double> >& A) {
     int n = A.size();

     for (int i=0; i<n; i++) {
         // Trazenje maksimuma kolone
         double maxEl = abs(A[i][i]);
         int maxRow = i;
         for (int k=i+1; k<n; k++) {
             if (abs(A[k][i]) > maxEl) {
                 maxEl = A[k][i];
                 maxRow = k;
             }
         }


         for (int k=i; k<2*n;k++) {
             double tmp = A[maxRow][k];
             A[maxRow][k] = A[i][k];
             A[i][k] = tmp;
         }

         // Svi redovi ispod ovog nula u trenutnoj koloni
         for (int k=i+1; k<n; k++) {
             double c = -A[k][i]/A[i][i];
             for (int j=i; j<2*n; j++) {
                 if (i==j) {
                     A[k][j] = 0;
                 } else {
                     A[k][j] += c * A[i][j];
                 }
             }
         }
     }

     // Rijesiti Ax=b za gornju trougaonu matricu A
     for (int i=n-1; i>=0; i--) {
         for (int k=n; k<2*n;k++) {
             A[i][k] /= A[i][i];
         }

         A[i][i] = 1;

         for (int rowModify=i-1;rowModify>=0; rowModify--) {
             for (int columModify=n;columModify<2*n;columModify++) {
                 A[rowModify][columModify] -= A[i][columModify]
                                              * A[rowModify][i];
             }

             A[rowModify][i] = 0;
         }
     }
 }



 cv::Point stabilize(std::vector<cv::Point> &points, int windowSize)
 {
   float sumX = 0;
   float sumY = 0;
   int count = 0;
   for (int i = std::max(0, (int)(points.size() - windowSize)); i < points.size(); i++)
   {
       sumX += points[i].x;
       sumY += points[i].y;
       ++count;
   }
   if (count > 0)
   {
       sumX /= count;
       sumY /= count;
   }
   return cv::Point(sumX, sumY);
 }




Rect getLeftmostEye(std::vector<cv::Rect> &eyes)
{
  int leftmost = 99999999;
  int leftmostIndex = -1;
  for (int i = 0; i < eyes.size(); i++)
  {
      if (eyes[i].tl().x < leftmost)
      {
          leftmost = eyes[i].tl().x;
          leftmostIndex = i;
      }
  }
  return eyes[leftmostIndex];
}



void DetektujOci(cv::Mat &frame, cv::CascadeClassifier &eyeCascade){


    if(izvrsilo_se_barem_jednom == 1){

        image = frame.clone();
        Mat image_cb;
        cv::cvtColor(image, image_cb, CV_BGR2GRAY); // konvertuj sliku u grayscale
        equalizeHist(image_cb, image);


       double start_point_x = x1 + iks;
       double end_point_x = x3 + iks;
       vector<Point2f> curvePoints, curvePoints2;

       for (double x = start_point_x; x <= end_point_x; x+=1){
           double y = a1*x*x + b1*x + c1;
           Point2f new_point = Point2f(x, y);
           curvePoints.push_back(new_point);
       }


       for (double x = start_point_x; x <= end_point_x; x+=1){
           double y = a2*x*x + b2*x + c2;
           Point2f new_point = Point2f(x, y);
           curvePoints2.push_back(new_point);
       }


       Mat curve(curvePoints, true);
       curve.convertTo(curve, CV_32S); //adaptuj tip u polylines
       polylines(frame, curve, false, Scalar(255,0,0), 2, CV_AA);

      Mat curve2(curvePoints2, true);
       curve2.convertTo(curve2, CV_32S); //adaptuj tip u polylines
       polylines(frame, curve2, false, Scalar(255,0,0), 2, CV_AA);


    //Bijela pozadina

       for(int k=0; k < curvePoints.size(); k++){
           for(int i=0; i<image.cols; i++){

               if(i == curvePoints[k].x){
                   for(int j=0; j < curvePoints[k].y; j++){
                       image.at<char>(j,curvePoints[k].x) = 255;
                   }
               }
               if(i < x1+iks){
                   for(int j= 0; j < image.rows; j++){
                        image.at<char>(j,i) = 255;
                   }
               }
               if(i > x3+iks){
                   for(int j= 0; j < image.rows; j++){
                        image.at<char>(j,i) = 255;
                   }
               }

           }

        }


       for(int k=0; k < curvePoints2.size(); k++){
           for(int i=0; i<image.cols; i++){

               if(i == curvePoints2[k].x){
                   for(int j=curvePoints2[k].y+1; j < image.rows; j++){
                       image.at<char>(j,curvePoints2[k].x) = 255;
                   }
               }

           }
       }



    Mat blur;


    threshold(image,bw,0,255,THRESH_BINARY+THRESH_OTSU);

    Mat pom_bw = ~bw;

    namedWindow("varijabla bw");
    namedWindow("slika");
    moveWindow("varijabla bw", 1100, 50);
    moveWindow("slika", 5,500);



    if(izvrsilo_se_barem_jednom==1){

        string name = "slika_";
        string type = ".jpg";
        string tress = to_string(varijabla);

        string ime = name+tress+type;


        Point2f centar_rotacije = Point2f(x1+iks, ips1+ips);
        double ugao = Ugao_rotacije();

        Mat r = cv::getRotationMatrix2D(centar_rotacije, ugao, 1.0);

        warpAffine(pom_bw, pom_bw, r, bw.size());
        bw = ~ pom_bw;

        odabrane_tri_tacke = 0;


        centar = Plotaj(bw);
        circle(bw, centar, 3, Scalar(255,0,0),2);

        imshow("varijabla bw",bw);

        resize(image,slika_druga,Size(),0.3,0.3);

        if(pauziraj == 1){
        destroyWindow("slika");
        }

      }
    }





    if(pauziraj == 0){

        Mat slikica = frame.clone();

        cv::Mat grayscale;
        cv::cvtColor(frame, grayscale, CV_BGR2GRAY);


        cv::equalizeHist(grayscale, grayscale); // povecaj kontrast slike

        std::vector<cv::Rect> eyes;

        eyeCascade.detectMultiScale(frame, eyes, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(150, 150));

        if (eyes.size() != 1){
             problem = 1;
             return;// provjera da li je oko detektovano
         }

         problem =0;

        for (cv::Rect &eye : eyes)
        {
          rectangle(frame,  eye.tl(), eye.br(), cv::Scalar(0, 255, 0), 2);
          iksic = eye.tl().x;
          ipsiloncic = eye.tl().y;
        }


        Mat eye_cb;
        Mat eye = frame(eyes[0]); // uzmi lijevo oko
        cv::cvtColor(eye, eye_cb, CV_BGR2GRAY);
        equalizeHist(eye_cb, eye);

        slika_oka=eye;
        }

        namedWindow("Oko");
        moveWindow("Oko", 750,10);
        imshow("Oko", slika_oka);


        if(pauziraj == 1){

            setMouseCallback("Oko", CallBackFunc, NULL);

        }

    }


int da_li_je_oko_zatvoreno(Mat slika, vector<Point2f> curvePoints, vector<Point2f> curvePoints2){//tesko je na osnovu broja piksela provjeravati zatvorenost oka
    int broj_piksela(0);
    int broj_crnih(0);
    int broj_bijelih(0);



    for(int k=0; k < curvePoints.size(); k++){

        for(int ii=curvePoints[k].y; ii <= curvePoints2[k].y; ii++){
            if(slika.at<char>(ii,curvePoints[k].x) != 0) broj_bijelih++;
            if(slika.at<char>(ii,curvePoints[k].x) == 0) broj_crnih++;
          //  cout<<ii<<endl;
        }


    }
     broj_piksela = broj_bijelih + broj_crnih;


    return broj_crnih;

}


double Ugao_rotacije(){

    double a_1, a_2, alfa_r(0), alfa_d(0);

    double x_p_1(x1+iks), x_p_3(x3+iks), y_p_1(ips1+ips), y_p_3(y3+ips);

    if(y_p_1 < y_p_3){

        a_1 = x_p_3 - x_p_1;
        a_2 = y_p_3 - y_p_1;

        double rez = a_2/a_1;
        alfa_r = atan(rez);
        alfa_d = (180./PI)*alfa_r;

    }
    else if(y_p_1 > y_p_3){

        a_1 = x_p_3 - x_p_1;
        a_2 = y_p_3 - y_p_1;
        alfa_r = atan(a_2/a_1);
        alfa_d = (180./PI)*alfa_r;
    }

    return alfa_d;

}


Point2i Plotaj(Mat slika1){

    Mat slika2 = slika1.clone();
    Mat slika = ~slika2;

    GaussianBlur( slika, slika, Size(9, 9), 2, 2 );

    vector<float> elementix , elementiy;
    float suma = 0;


    for(int i=x1+iks+15; i<x3+iks-15; i++){

        for(int j=0; j < slika.rows; j++){
            suma = suma + slika.at<uchar>(j,i);
        }

        elementix.push_back(suma);
        suma=0;

    }

    Point2i centar;

    float max_x, max_y;
    int  centar_x, centar_y;

    max_x = elementix[0];
    centar_x = x1+iks+15;

    for(int i=0; i<elementix.size(); i++){

        if(elementix[i] > max_x && elementix[i]>200){
            max_x = elementix[i];
            centar_x = i+x1+iks+15;
        }
    }

    suma =0;
    for(int i=y2+ips+5; i<y4+ips-5; i++){
        for(int j=0; j < slika.cols; j++){
            suma = suma + slika.at<uchar>(i,j);
        }
        elementiy.push_back(suma);
        suma=0;
    }

    max_y = elementiy[0];
    centar_y = y2+ips+5;;

    for(int i=0; i<elementiy.size(); i++){

        if(elementiy[i] > max_y){
            max_y = elementiy[i];
            centar_y = i+y2+ips+5;
        }
    }

    centar.x=centar_x;
    centar.y = centar_y;

    return centar;


}




void CallBackFunc(int event, int x, int y, int flags, void* userdata)
{

     if  ( event == EVENT_LBUTTONDOWN)
     {
         iks = iksic;
         ips = ipsiloncic;
          cout << "Pritisnut lijevi klik misa - pozicija ("<< x << ", " << y << ")";
          if(brojac == 0){
              brojac++;
              x1=x;
              ips1=y;
               cout<<" - prva tacka"<<endl;
          }
          else if (brojac==1) {
              x2=x;
              y2=y;
              brojac++;
              cout<<" - druga tacka"<<endl;

          }
          else if(brojac == 2){
              x3=x;
              y3 = y;

              brojac++;
             vector<double> abc = Odredjivanje_parametara(x1+iks,ips1+ips,x2+iks,y2+ips,x3+iks,y3+ips);

             a1 = abc[0]; b1 = abc[1]; c1=abc[2];
             cout<<" - treca tacka"<<endl;
             odabrane_tri_tacke = 1;


          }
          else if(brojac == 3){
              x4 = x;
              y4 = y;
              brojac = 0;
              cout<<" - cetvrta tacka"<<endl;
              vector<double> abc = Odredjivanje_parametara(x1+iks,ips1+ips,x4+iks,y4+ips,x3+iks,y3+ips);
              a2 = abc[0]; b2 = abc[1]; c2 = abc[2];
              provjera_stanja = 1;
              izvrsilo_se_barem_jednom = 1;
              pauziraj=0;
              c=-1;
          }
     }
}



void init_2Dvector(vector<vector<double> >& vec, int x_dim, int y_dim){
    vec.resize(x_dim);
    for(int i=0; i < vec.size(); i++)
         vec[i].resize(y_dim);
}

vector<vector<double> > Inverzna(vector<vector<double> > &mat){
    vector<vector<double> > inverzna;
    float determinant = 0;
    init_2Dvector(mat, 3, 3);
    init_2Dvector(inverzna, 3, 3);

    for(int i = 0; i < 3; i++)
        determinant = determinant + (mat[0][i] * (mat[1][(i+1)%3] * mat[2][(i+2)%3] - mat[1][(i+2)%3] * mat[2][(i+1)%3]));

    for(int i = 0; i < 3; i++){
        for(int j = 0; j < 3; j++)
            inverzna[i][j]=((mat[(j+1)%3][(i+1)%3] * mat[(j+2)%3][(i+2)%3]) - (mat[(j+1)%3][(i+2)%3] * mat[(j+2)%3][(i+1)%3]))/ determinant;

    }
    return inverzna;
}


vector<double> Odredjivanje_parametara(double x_1, int y_1, double x_2, double y_2, double x_3, double y_3){
    vector<vector<double> > matrica;
    vector<vector<double> > ipsiloncek;
    vector<vector<double> > inverzna;
   vector<double> ikss;

    vector<double> parametri;

    ikss.push_back(x_1);
    ikss.push_back(x_2);
    ikss.push_back(x_3);

    init_2Dvector(ipsiloncek, 3, 1);
    init_2Dvector(iksevi, 3, 3);
     init_2Dvector(inverzna, 3, 3);

    ipsiloncek[0][0]=y_1;
    ipsiloncek[1][0]=y_2;
    ipsiloncek[2][0]=y_3;



   for(int i=0; i<3; i++)
   {

           iksevi[i][0]=ikss[i]*ikss[i];
           iksevi[i][1]=ikss[i];
           iksevi[i][2]=1;
   }

  inverzna = Inverzna(iksevi);
   init_2Dvector(matrica, 3, 1);

       for(int i=0; i<3;i++)
       {
           for(int j=0;j<1;j++)
           {
               matrica[i][j] = 0;

               for(int k=0;k<3;k++)
               {
                   matrica[i][j] += inverzna[i][k] * ipsiloncek[k][j];
               }
            parametri.push_back(matrica[i][j]);
           }
       }


       return parametri;

}

void DetektujEkran(Mat &slika_ekrana){

    Mat crnobela;
    Mat maska;
    cvtColor(slika_ekrana,maska, CV_BGR2GRAY);


    GaussianBlur( maska, maska, Size(9, 9), 2, 2 );

    vector<cv::Vec3f> kruzici;
    HoughCircles(maska, kruzici, CV_HOUGH_GRADIENT, 1, 100, 200, 30, 0, 0);

    if(kruzici.size() ==4){cetiri_kruga = 1;}
    for( size_t i = 0; i < kruzici.size(); i++ ){

       Point center(cvRound(kruzici[i][0]), cvRound(kruzici[i][1]));

       if(center.x <= 150 && center.y <=300){
           gornji_lijevo.x=center.x;
           gornji_lijevo.y=center.y;
       }
       else if(center.x >150 && center.y <=300){
           gornji_desno.x=center.x;
           gornji_desno.y=center.y;
       }
       else if(center.x <= 150 && center.y > 300){
           donji_lijevo.x = center.x;
           donji_lijevo.y = center.y;

       }
       else if(center.x > 150 && center.y >300){
           donji_desno.x = center.x;
           donji_desno.y = center.y;
       }
       int radius = cvRound(kruzici[i][2]);
       // centar kruga
       circle( slika_ekrana, center, 3, Scalar(0,255,0), -1, 8, 0 );
       // obris kruga
       circle( slika_ekrana, center, radius, Scalar(0,0,255), 3, 8, 0 );

     }

}



Mat Krugovi_i_kvadrati(){

    Mat slika(1080, 1920, CV_8UC3, Scalar(0,0, 0));

    circle(slika,Point(100,100),70,Scalar(255,255,255),CV_FILLED);
    circle(slika,Point(1819,100),70,Scalar(255,255,255),CV_FILLED);
    circle(slika,Point(1819,979),70,Scalar(255,255,255),CV_FILLED);
    circle(slika,Point(100,979),70,Scalar(255,255,255),CV_FILLED);

    rectangle(slika,Point(209,0), Point(580,269),Scalar(0,0,0), CV_FILLED);
    rectangle(slika,Point(580,0), Point(960,269),Scalar(255,255,255), CV_FILLED);
    rectangle(slika,Point(960,0), Point(1340,269),Scalar(0,0,0), CV_FILLED);
    rectangle(slika,Point(1340,0), Point(1720,269),Scalar(255,255,255), CV_FILLED);


    rectangle(slika,Point(209,269), Point(580,538),Scalar(255,255,255), CV_FILLED);
    rectangle(slika,Point(580,269), Point(960,538),Scalar(0,0,0), CV_FILLED);
    rectangle(slika,Point(960,269), Point(1340,538),Scalar(255,255,255), CV_FILLED);
    rectangle(slika,Point(1340,269), Point(1720,538),Scalar(0,0,0), CV_FILLED);

    rectangle(slika,Point(209,538), Point(580,807),Scalar(0,0,0), CV_FILLED);
    rectangle(slika,Point(580,538), Point(960,807),Scalar(255,255,255), CV_FILLED);
    rectangle(slika,Point(960,538), Point(1340,807),Scalar(0,0,0), CV_FILLED);
    rectangle(slika,Point(1340,538), Point(1720,807),Scalar(255,255,255), CV_FILLED);


    rectangle(slika,Point(209,807), Point(580,1076),Scalar(255,255,255), CV_FILLED);
    rectangle(slika,Point(580,807), Point(960,1076),Scalar(0,0,0), CV_FILLED);
    rectangle(slika,Point(960,807), Point(1340,1076),Scalar(255,255,255), CV_FILLED);
    rectangle(slika,Point(1340,807), Point(1720,1076),Scalar(0,0,0), CV_FILLED);


    return slika;

}


void Kalibracija(Mat &zaslon){

    int fontFace = CV_FONT_HERSHEY_SIMPLEX;
    double fontScale = 0.7;
    int thickness = 3;

    if(pritisnuta_jedinica == 1){

    if(cetiri_kruga == 1){

        kut1 = gornji_lijevo;
        kut2 = gornji_desno;
        kut3 = donji_lijevo;
        kut4 = donji_desno;
    }


        pos_1 = centar;
        cout<<"Pritisnuta 1  "<<pos_1.x<<"  "<<pos_1.y<<endl;

        putText(zaslon, "Pritisnuta 1", Point(394,100), fontFace,fontScale, Scalar::all(255), thickness, 8);

        pritisnuta_jedinica = 0;
        c=-1;

    }

    if(pritisnuta_dvica == 1){

        pos_2 = centar;
        putText(zaslon, "Pritisnuta 2", Point(1065,100), fontFace,fontScale, Scalar::all(255), thickness, 8);
        cout<<"Pritisnuta 2  "<<pos_2.x<<"  "<<pos_2.y<<endl;
        pritisnuta_dvica = 0;

    c=-1;
    }

    if(pritisnuta_trica == 1){

        pos_3 = centar;
        putText(zaslon, "Pritisnuta 3", Point(394,630), fontFace,fontScale, Scalar::all(255), thickness, 8);
        cout<<"Pritisnuta 3  "<<pos_3.x<<"  "<<pos_3.y<<endl;
        pritisnuta_trica = 0;

        c= -1;
    }

    if(pritisnuta_cetvorka == 1){
        pos_4 = centar;
        putText(zaslon, "Pritisnuta 4", Point(1065,630), fontFace,fontScale, Scalar::all(255), thickness, 8);
        cout<<"Pritisnuta 4  "<<pos_4.x<<"  "<<pos_4.y<<endl;
        pritisnuta_cetvorka = 0;
    c= -1;
        kalibracija = 1;
        rekalibracija = 0;

    }



}


    void gdje_gleda(Mat &zaslon){

        double sv1 = (pos_2.x - pos_1.x)/2;
        double sv2 = (pos_3.y - pos_1.y)/2;
        double sv3 = (pos_4.y-pos_2.y)/2;
        double sv4 = (pos_4.x - pos_3.x)/2;

        if(rekalibracija == 1){
            return;

        }

        else{

            if(centar.x <= (pos_1.x+sv1) && centar.y <= (pos_1.y + sv2)){

                rectangle(zaslon,Point(209,0), Point(960,538),Scalar(0,0,0), CV_FILLED);
            }

            else if(centar.x >= (pos_1.x+sv1) && centar.y <= (pos_2.y + sv3)){

                rectangle(zaslon,Point(960,0), Point(1720,538),Scalar(0,0,0), CV_FILLED);
            }
            else if((centar.x <= (pos_3.x + sv4) ||  centar.x <= (pos_1.x+sv1)) && centar.y >= (pos_1.y + sv2)){

             rectangle(zaslon, Point(209,538), Point(960,1076),Scalar(0,0,0), CV_FILLED);
            }
            else if( (centar.x >= (pos_3.x+sv4) || centar.x >= (pos_1.x+sv1)) && centar.y >= (pos_2.y + sv3)){

                rectangle(zaslon, Point(960,538), Point(1720,1076),Scalar(0,0,0), CV_FILLED);
            }
        }


    }



void main5() {

    inicijalizacijaUSBkamere1();
    inicijalizacijaUSBkamere2();


    char k='a';

    VideoCapture cap_oko(250);
    VideoCapture cap_zaslon(250);

    Mat slika;
    Mat slika2;

    slika = grabFromUSBcamera(hCam);
    slika2 = grabFromUSBcamera(hCam2);


    while(1){

        ekran = Krugovi_i_kvadrati();

        if(pauziraj == 0){

            image1 = grabFromUSBcamera(hCam);
            image2 = grabFromUSBcamera(hCam2);
        }


        DetektujOci(image1, eyeCascade);
        if(pauziraj == 0){
            DetektujEkran(image2);

        }


       if(izvrsilo_se_barem_jednom == 1){

            if(pauziraj == 0){

                namedWindow("slika");
                moveWindow("slika", 5,500);
                namedWindow("Zaslon", CV_WINDOW_NORMAL);
                setWindowProperty("Zaslon", CV_WND_PROP_FULLSCREEN, CV_WINDOW_FULLSCREEN);


                 string rec = "Centar";
                 string rec1 = "(";
                 string rec2 = to_string(centar.x);
                 string rec3 = ",";
                 string rec4 = to_string(centar.y);
                 string rec5 = ")";
                 string text = rec1 + rec2 + rec3 + rec4 + rec5;
                 int fontFace = CV_FONT_HERSHEY_SIMPLEX;
                 double fontScale = 0.7;
                 int thickness = 3;



                 int baseline=0;
                 Size textSize = getTextSize(text, fontFace,
                                             fontScale, thickness, &baseline);
                 baseline += thickness;


                 Point textOrg(1750,500);
                 Point textOrg1(1750,550);
                 Point textOrg2(1750,700);


                 putText(ekran, rec, textOrg, fontFace, fontScale,
                         Scalar::all(255), thickness, 8);
                 putText(ekran, text, textOrg1, fontFace, fontScale,
                         Scalar::all(255), thickness, 8);

                 if(problem == 1){
                     putText(ekran, "Problem sa detekcijom", Point(1730,600), fontFace, double(0.5), Scalar::all(255), int(2),8);
                      putText(ekran, "oka", Point(1730,630), fontFace, double(0.5), Scalar::all(255), int(2),8);

                 }



                 if(cetiri_kruga == 1){

                     string xgd = to_string(gornji_desno.x);
                     string ygd = to_string(gornji_desno.y);
                     string xgl = to_string(gornji_lijevo.x);
                     string ygl = to_string(gornji_lijevo.y);
                     string xdl = to_string(donji_lijevo.x);
                     string ydl = to_string(donji_lijevo.y);
                     string xdd = to_string(donji_desno.x);
                     string ydd = to_string(donji_desno.y);

                     putText(ekran, rec1 + xgd + rec3 + ygd + rec5, Point(1750,250), fontFace,fontScale, Scalar::all(255), thickness, 8);
                     putText(ekran, rec1 + xgl + rec3 + ygl + rec5, Point(50,250), fontFace,fontScale, Scalar::all(255), thickness, 8);
                     putText(ekran, rec1 + xdl + rec3 + ydl + rec5, Point(50,850), fontFace,fontScale, Scalar::all(255), thickness, 8);
                     putText(ekran, rec1 + xdd + rec3 + ydd + rec5, Point(1750,850), fontFace,fontScale, Scalar::all(255), thickness, 8);


                 }



                Kalibracija(ekran);

                if(kalibracija ==1){

                  gdje_gleda(ekran);

                }


                imshow("Zaslon", ekran);
                imshow("slika",slika_druga);
                cetiri_kruga = 0;


            }


                if(pauziraj == 1){
                    destroyWindow("Zaslon");
                }

                if(pauziraj == 1){
                    destroyWindow("slika");
                }


        }



            namedWindow("Prikaz prve kamere");
            moveWindow("Prikaz prve kamere", 100,10);

            namedWindow("Prikaz druge kamere");
            moveWindow("Prikaz druge kamere", 100,550);

            imshow("Prikaz prve kamere",image1);

            imshow("Prikaz druge kamere",image2);

        // Pritisnite ESC za izlaz

            c = (char)waitKey(1);

            if( c == 27 )
              break;

            if(c == 32){
                pauziraj = 1;
                rekalibracija = 1;
            }

            if(c == 49){

                pritisnuta_jedinica = 1;
            }

            if(c == 50){
                pritisnuta_dvica = 1;
            }

            if(c == 51){
                pritisnuta_trica = 1;
            }

            if(c == 52){
                pritisnuta_cetvorka = 1;
            }

     }



        cap_oko.release();
        cap_zaslon.release();


        // Zatvara sve prozore
        destroyAllWindows();


        is_ExitCamera(hCam);
        is_ExitCamera(hCam2);

  }







 int main() {


        if (!eyeCascade.load("/home/amila/PokusajSaSokicevimKodom/haarcascade_eye_tree_eyeglasses.xml"))
        {
            std::cerr << "Could not load eye detector." << std::endl;
            return -1;
        }

        cout<<"Dobrodosli! :)"<<endl;
        main5();

        return 1;
}



  void inicijalizacijaUSBkamere1() {
    //initialize camera
    nRet = is_InitCamera(&hCam, NULL);
    //cout << "Status Init: " << nRet << endl;

    //get sensor info
    //nRet = is_GetSensorInfo(hCam, &sensor_info);
    //cout << "Sensor Color Mode: " << sensor_info.nColorMode << endl;
    //cout << "Camera Model: " << sensor_info.strSensorName << endl;

    //get camera info
   // nRet = is_GetCameraInfo(hCam, &camera_info);
  //  cout << "Camera ID: " << camera_info.ID << endl;
    //cout << "Camera SerNum: " << camera_info.SerNo << endl;
    //cout << "Camera Version: " << camera_info.Version << endl;
    //cout << "Camera Type: " << camera_info.Type << endl;

    // color mode
    Mode = IS_CM_RGB8_PACKED;
    nRet = is_SetColorMode(hCam, Mode);
   // cout << "Color Mode: " << nRet << endl;

    UINT formatID = 13;
    nRet = is_ImageFormat(hCam, IMGFRMT_CMD_SET_FORMAT, &formatID, 4);
  // cout << "Status Image Format: " << nRet << endl;

    char* pMem = NULL;
    int memID = 0;
    nRet = is_AllocImageMem(hCam, img_width, img_height, 24, &pMem, &memID);
    nRet = is_SetImageMem(hCam, pMem, memID);

    // postavi display mode
    Mode = IS_SET_DM_DIB;
    nRet = is_SetDisplayMode(hCam, Mode);

    Mode = FOC_CMD_SET_DISABLE_AUTOFOCUS;
    nRet = is_Focus(hCam, Mode, NULL, 0);

  }

  void inicijalizacijaUSBkamere2() {
    //initialize camera
    nRet = is_InitCamera(&hCam2, NULL);
 //   cout << "Status Init: " << nRet << endl;

    //color mode
    Mode = IS_CM_RGB8_PACKED;
    nRet = is_SetColorMode(hCam, Mode);
 //   cout << "Color Mode: " << nRet << endl;

    UINT formatID = 13;
    nRet = is_ImageFormat(hCam2, IMGFRMT_CMD_SET_FORMAT, &formatID, 4);
  //  cout << "Status Image Format: " << nRet << endl;

    char* pMem = NULL;
    int memID = 0;
    nRet = is_AllocImageMem(hCam2, img_width, img_height, 24, &pMem, &memID);
    nRet = is_SetImageMem(hCam2, pMem, memID);

    //set display mode
    Mode = IS_SET_DM_DIB;
    nRet = is_SetDisplayMode(hCam2, Mode);

    Mode = FOC_CMD_SET_DISABLE_AUTOFOCUS;
    nRet = is_Focus(hCam2, Mode, NULL, 0);

  }

  Mat grabFromUSBcamera(HIDS broj) {
    if (is_FreezeVideo(broj, IS_WAIT) == IS_SUCCESS) {
      void* pMemVoid; //pointer to where the image is stored
      is_GetImageMem(broj, &pMemVoid);
      Mat frame;
      frame = Mat(Size(img_width, img_height), CV_8UC3, pMemVoid);
      return frame;
    } else {return Mat::zeros(Size(img_width,img_height), CV_8UC3 );};

  }


