#include <ros/ros.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <boost/property_tree/info_parser.hpp>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>

#define PI 3.14159265

using namespace cv;
using namespace std;

int hl = 0, hh = 179, sl = 0, sh = 255, low = 12, high = 254, thresh=41, stopCond=0;

int threshHough1=150,threshHough2=130;
int dp=2;
int threshPOG=50;
int threshForBoundaries=150;
int thresholdForGreen=44;
int threshForCorner=150;

int cameraToUse=1;                             //-------- 1 for external and 0 for internal camera
int ballDetected=0;
float maxPercent=0.0;

vector<Vec3f> findBallUsingCircleTemplate(Mat image)
{
    maxPercent=0.0;
    namedWindow("normal",WINDOW_NORMAL);
    namedWindow("GrayScale",WINDOW_NORMAL);

    Mat org=image.clone();

    cvtColor(image, image, COLOR_BGR2GRAY);
    int downsampled=0;
    while(image.rows>500 && image.cols>500) {
        downsampled++;
        GaussianBlur(image, image, Size(3, 3), 0, 0);
        pyrDown(image, image, Size(image.cols / 2, image.rows / 2));
    }

    Mat blur;

    int i,j;
    uchar* p;

    thresh=50;
    createTrackbar("Threshold", "GrayScale", &thresh, 255, NULL);
    Canny( image, blur, thresh, thresh*3, 3 );

    double min,max;
    minMaxLoc(blur, &min, &max);
    imshow("GrayScale", blur);

    int rect_x,rect_y,rect_width,previous_ratio=0;
    int start_size=90;
    while(start_size>40){

        Mat ballTemplate(start_size,start_size,blur.type(),Scalar::all(0));
        //----------- Circle using formula but still detects head
        /*
        int midI=round((start_size-1)/2),midJ=round((start_size-1)/2);
        for( i = 0; i < ballTemplate.rows; ++i)
        {
           p = ballTemplate.ptr<uchar>(i);
           for ( j = 0; j < ballTemplate.cols; ++j)
           {
              if(sqrt(pow((i-midI),2)+pow((j-midJ),2))==round((start_size-1)/2))
                 p[j] = 255;
              else
                 p[j] = 0;
           }
        }
        */
        //----------- Circle using inbuilt function
        circle(ballTemplate,Point(round((ballTemplate.rows-1)/2),round((ballTemplate.rows-1)/2)),round((ballTemplate.rows-1)/2),(0,0,255),1);
        for( i = 0; i < ballTemplate.rows; ++i)
        {
            p = ballTemplate.ptr<uchar>(i);
            for ( j = 0; j < ballTemplate.cols; ++j)
            {
                if(ballTemplate.at<uchar>(i,j)==255)
                    p[j] = 255;
                else
                    p[j] = 0;
            }
        }
        int NonZeroCount=countNonZero(ballTemplate);


        for(int rw=0;rw<=(blur.rows-ballTemplate.rows);rw++)
        {
            for(int cl=ceil((float)(ballTemplate.rows)/2);cl<=(blur.cols-ceil((float)(ballTemplate.rows)/2));cl++)
            {
                if(blur.at<uchar>(rw,cl)==255.0) {
                    Mat_<char> diff;
                    Mat sub_sec=blur(Range(rw, rw + (ballTemplate.rows)), Range(cl-floor((float)(ballTemplate.rows)/2), cl + ceil((float)(ballTemplate.rows)/2)));
                    subtract(ballTemplate, sub_sec,diff);
                    char *p1;
                    for (i = 0; i < diff.rows; ++i) {
                        p1 = diff.ptr<char>(i);
                        for (j = 0; j < diff.cols; ++j) {
                            if (diff.at<char>(i, j) < 127)
                                p1[j] = 0;
                        }
                    }
                    float l=(((float)(NonZeroCount-countNonZero(diff))/(NonZeroCount))*100);
                    if (l > maxPercent) {
                        rect_x = rw-1;
                        rect_y = cl-round((ballTemplate.rows)/2)-1;
                        rect_width = start_size%2==0?start_size-2:start_size-1;
                        maxPercent=l;
                    }
                }
            }
        }
        start_size--;

    }

    rectangle(image,Point(rect_y,rect_x),Point(rect_y+rect_width,rect_x+rect_width),Scalar(0,255,0),1);
    imshow("normal", image);
    rect_x*= pow(2, downsampled);
    rect_y*= pow(2, downsampled);
    rect_width*= pow(2, downsampled);
    float radius = (float)rect_width/2;
    vector<Vec3f> coord(1);
    coord[0][0]=(float)rect_y+radius;
    coord[0][1]=(float)rect_x+radius;
    coord[0][2]=radius;

    return coord;
}

vector<Vec3f> trackBall(Mat image)
{
    vector<Vec3f> coord;
    int downsampled=0;
    while(image.rows>1000 && image.cols>1000){
        downsampled++;
        GaussianBlur( image, image, Size( 3, 3 ), 0, 0);
        pyrDown( image, image, Size( image.cols/2, image.rows/2 ) );
    }
    namedWindow("blurred image", WINDOW_NORMAL);

    int nRows=image.rows;
    int nCols=image.cols;
    Mat thresholded=Mat::zeros(nRows, nCols, CV_8UC1);
    int rw,cl;
    uchar *p;
    int thresholdForGreen=45;
    for(rw=0;rw<nRows;rw++){
        p = thresholded.ptr<uchar>(rw);
        for(cl=0;cl<nCols;cl++){
            Vec3b intensities = image.at<Vec3b>(rw,cl);
            float percentOfGreen = (float) intensities.val[1]/(intensities.val[0]+intensities.val[1]+intensities.val[2]);
            if((percentOfGreen*100)>thresholdForGreen){
                p[cl]=0;
            }
            else
            {
                p[cl]=255;
            }
        }
    }
    Canny( thresholded, thresholded, threshPOG, threshPOG*3, 3 );

    cvtColor(image, image, COLOR_BGR2GRAY);
    Canny( image, image, thresh, thresh*3, 3 );
    add(thresholded, image, image);

    HoughCircles(image, coord, CV_HOUGH_GRADIENT, dp, image.rows, threshHough1, threshHough2);

    if(coord.size()>0) {
        ROS_INFO("no. of Ball Detected = %ld",coord.size());
        for(int i=0;i<coord.size();i++) {
            coord[i][0] *= pow(2, downsampled);
            coord[i][1] *= pow(2, downsampled);
            coord[i][2] *= pow(2, downsampled);
        }
    }
    return coord;
}

vector<vector<Point>> drawField(Mat *image)
{
    vector<vector<Point>> regions;
    int downsampled=0;
    while(image->rows>400 && image->cols>400){
        downsampled++;
        GaussianBlur( *image, *image, Size( 3, 3 ), 0, 0);
        pyrDown( *image, *image, Size( image->cols/2, image->rows/2 ) );
    }

    namedWindow("Thresholded using POG", WINDOW_NORMAL);
    int nRows=image->rows;
    int nCols=image->cols;
    Mat thresholded=Mat::zeros(nRows, nCols, CV_8UC1);
    int rw,cl;
    uchar *p;
    createTrackbar("thresholdForGreen", "Thresholded using POG", &thresholdForGreen, 255, NULL);
    for(rw=0;rw<nRows;rw++){
        p = thresholded.ptr<uchar>(rw);
        for(cl=0;cl<nCols;cl++){
            Vec3b intensities = image->at<Vec3b>(rw,cl);
            float percentOfGreen = (float) intensities.val[1]/(intensities.val[0]+intensities.val[1]+intensities.val[2]);
            if((percentOfGreen*100)>thresholdForGreen){
                p[cl]=0;
            }
            else
            {
                p[cl]=255;
            }
        }
    }
    imshow("Thresholded using POG",thresholded);

    Mat dst;

    Mat srcCanny;
    Canny(thresholded, srcCanny, 100, 100 * 2, 3, true);
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    findContours(srcCanny, contours, hierarchy, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));
    vector<vector<Point> > approx;
    approx.resize(contours.size());
    for (int k = 0; k < contours.size(); k++)
        approxPolyDP(Mat(contours[k]), approx[k], 3, true);
    Mat drawing = Mat::zeros(srcCanny.size(), CV_8UC3);
    for (int i = 0; i< approx.size(); i++)
    {
        if(approx[i].size()>=4)
        {
            int count_sides=0;
            vector<float> side_length;
            vector<Point> edges;
            float prev_slope=0.0;
            int limit=(int)approx[i].size();
            for(int j=0;j<approx[i].size();j++)
            {
                if(j==0)
                {
                    prev_slope=((approx[i][j+1].y-approx[i][j].y)<0.0 ^ (approx[i][j+1].x-approx[i][j].x)<0.0)?-1.0:1.0;
                    side_length.push_back((float)(sqrt(pow((approx[i][j+1].y-approx[i][j].y),2)+pow((approx[i][j+1].x-approx[i][j].x),2))));
                    edges.push_back(Point(approx[i][j].x,approx[i][j].y));
                    count_sides++;
                }
                else
                {
                    float slope=((approx[i][(j+1)%limit].y-approx[i][j].y)<0.0 ^ (approx[i][(j+1)%limit].x-approx[i][j].x)<0.0)?-1.0:1.0;
                    if(slope==(-1)*prev_slope){
                        side_length.push_back((float)(sqrt(pow((approx[i][(j+1)%limit].y-approx[i][j].y),2)+pow((approx[i][(j+1)%limit].x-approx[i][j].x),2))));
                        edges.push_back(Point(approx[i][j].x,approx[i][j].y));
                        count_sides++;
                    }
                    else
                    {
                        side_length[count_sides-1]+=sqrt((float)pow((approx[i][j+1].y-approx[i][j].y),2)+pow((approx[i][j+1].x-approx[i][j].x),2));
                    }
                    prev_slope=slope;
                }
            }
            if(count_sides==4)
            {
                if(abs(side_length[0]-side_length[2])<15 && abs(side_length[1]-side_length[3])<20){
                    regions.push_back(edges);
                    drawContours(*image, approx, i, Scalar(0,255,255), 1, 8, hierarchy, 0, Point());
                    float small_side,large_side,slope_large,slope_small;
                    if(side_length[0]>side_length[1])
                    {
                        slope_large=((float)(edges[1].y-edges[0].y)/(edges[1].x-edges[0].x));
                        slope_small=((float)(edges[2].y-edges[1].y)/(edges[2].x-edges[1].x));
                        small_side=side_length[1]>side_length[3]?side_length[1]:side_length[3];
                        large_side=side_length[0]>side_length[2]?side_length[0]:side_length[2];
                    }
                    else
                    {
                        slope_small=((float)(edges[1].y-edges[0].y)/(edges[1].x-edges[0].x));
                        slope_large=((float)(edges[2].y-edges[1].y)/(edges[2].x-edges[1].x));
                        small_side=side_length[0]>side_length[2]?side_length[0]:side_length[2];
                        large_side=side_length[1]>side_length[3]?side_length[1]:side_length[3];
                    }
                    int small,back_corner_edge=-1;
                    if(slope_large<0.0){
                        for (int j = 0; j < approx[i].size(); j++) {
                            if (j == 0) {
                                small = j;
                            } else {
                                if (approx[i][small].x <= approx[i][j].x)
                                    small = j;
                            }
                        }
                        for (int j = 0; j < approx[i].size(); j++) {
                            if (j != small)
                                if (back_corner_edge == -1) {
                                    back_corner_edge = j;
                                } else {
                                    if (approx[i][back_corner_edge].x < approx[i][j].x)
                                        back_corner_edge = j;
                                }
                        }
                        // For outer rectangle
                        float x1 = approx[i][back_corner_edge].x +
                                   round((float) (small_side) / sqrt(1 + pow(slope_large, 2)));
                        float y1 = approx[i][back_corner_edge].y +
                                   round((float) (small_side * slope_large) / sqrt(1 + pow(slope_large, 2)));
                        float x2=x1+round((float)(2*small_side)/sqrt(1+pow(slope_small,2)));
                        float y2=y1+round((float)(2*small_side*slope_small)/sqrt(1+pow(slope_small,2)));
                        line(*image,Point(x1,y1),Point(x2,y2),Scalar(255,0,255),1);
                        float x3=x2-round((float)(large_side+3*small_side)/sqrt(1+pow(slope_large,2)));
                        float y3=y2-round((float)((large_side+3*small_side)*slope_large)/sqrt(1+pow(slope_large,2)));
                        line(*image,Point(x3,y3),Point(x2,y2),Scalar(255,0,255),1);
                        float x4=x3-round((float)(2*small_side)/sqrt(1+pow(slope_small,2)));
                        float y4=y3-round((float)(2*small_side*slope_small)/sqrt(1+pow(slope_small,2)));
                        line(*image,Point(x3,y3),Point(x4,y4),Scalar(255,0,255),1);
                        vector<Point> outer_rect;
                        outer_rect.push_back(Point(x1,y1));
                        outer_rect.push_back(Point(x2,y2));
                        outer_rect.push_back(Point(x3,y3));
                        outer_rect.push_back(Point(x4,y4));
                        regions.push_back(outer_rect);

                        // For corner side lines
                        x1 = approx[i][back_corner_edge].x +
                             round((float) (2*small_side) / sqrt(1 + pow(slope_large, 2)));
                        y1 = approx[i][back_corner_edge].y +
                             round((float) ((2*small_side) * slope_large) / sqrt(1 + pow(slope_large, 2)));
                        x2=approx[i][back_corner_edge].x-round((float)(4*small_side+large_side)/sqrt(1+pow(slope_large,2)));
                        y2=approx[i][back_corner_edge].y-round((float)((4*small_side+large_side)*slope_large)/sqrt(1+pow(slope_large,2)));
                        line(*image,Point(x1,y1),Point(x2,y2),Scalar(255,255,0),1);
                        vector<Point> side_lines;
                        side_lines.push_back(Point(x1,y1));
                        side_lines.push_back(Point(x2,y2));
                        regions.push_back(side_lines);
                    }
                    else {
                        for (int j = 0; j < approx[i].size(); j++) {
                            if (j == 0) {
                                small = j;
                            } else {
                                if (approx[i][small].x >= approx[i][j].x)
                                    small = j;
                            }
                        }
                        for (int j = 0; j < approx[i].size(); j++) {
                            if (j != small)
                                if (back_corner_edge == -1) {
                                    back_corner_edge = j;
                                } else {
                                    if (approx[i][back_corner_edge].x > approx[i][j].x)
                                        back_corner_edge = j;
                                }
                        }
                        // For outer rectangle
                        float x1 = approx[i][back_corner_edge].x -
                                   round((float) small_side / sqrt(1 + pow(slope_large, 2)));
                        float y1 = approx[i][back_corner_edge].y -
                                   round((float) (small_side * slope_large) / sqrt(1 + pow(slope_large, 2)));
                        float x2=x1-round((float)(2*small_side)/sqrt(1+pow(slope_small,2)));
                        float y2=y1-round((float)(2*small_side*slope_small)/sqrt(1+pow(slope_small,2)));
                        line(*image,Point(x1,y1),Point(x2,y2),Scalar(255,0,255),1);
                        float x3=x2+round((float)(large_side+3*small_side)/sqrt(1+pow(slope_large,2)));
                        float y3=y2+round((float)((large_side+3*small_side)*slope_large)/sqrt(1+pow(slope_large,2)));
                        line(*image,Point(x3,y3),Point(x2,y2),Scalar(255,0,255),1);
                        float x4=x3+round((float)(2*small_side)/sqrt(1+pow(slope_small,2)));
                        float y4=y3+round((float)(2*small_side*slope_small)/sqrt(1+pow(slope_small,2)));
                        line(*image,Point(x3,y3),Point(x4,y4),Scalar(255,0,255),1);
                        vector<Point> outer_rect;
                        outer_rect.push_back(Point(x1,y1));
                        outer_rect.push_back(Point(x2,y2));
                        outer_rect.push_back(Point(x3,y3));
                        outer_rect.push_back(Point(x4,y4));
                        regions.push_back(outer_rect);

                        // For corner side lines
                        x1 = approx[i][back_corner_edge].x -
                                   round((float) (2*small_side) / sqrt(1 + pow(slope_large, 2)));
                        y1 = approx[i][back_corner_edge].y -
                                   round((float) ((2*small_side) * slope_large) / sqrt(1 + pow(slope_large, 2)));
                        x2=approx[i][back_corner_edge].x+round((float)(4*small_side+large_side)/sqrt(1+pow(slope_large,2)));
                        y2=approx[i][back_corner_edge].y+round((float)((4*small_side+large_side)*slope_large)/sqrt(1+pow(slope_large,2)));
                        line(*image,Point(x1,y1),Point(x2,y2),Scalar(255,255,0),1);
                        vector<Point> side_lines;
                        side_lines.push_back(Point(x1,y1));
                        side_lines.push_back(Point(x2,y2));
                        regions.push_back(side_lines);
                    }
                }
            }
        }
    }
	
    for(int i=0;i<regions.size();i++)
        for(int j=0;j<regions[i].size();j++)
        {
            regions[i][j].x *= pow(2, downsampled);
            regions[i][j].y *= pow(2, downsampled);
        }
    return regions;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "ball_vision");
    ros::NodeHandle nodeHandle;
    string templateImage;
    //nodeHandle.getParam("templateImg", templateImage);


    Mat image;
    VideoCapture cap(cameraToUse);

    if ( cap.isOpened() )
        while (ros::ok())
        {
            cap >> image;
			//------- If using template image passed as parameter
            //image=imread(templateImage);
            ROS_INFO("Image Read...");
            Mat org=image.clone();
            ROS_INFO("Processing the I/P image...");
            vector<Vec3f> coord;
            vector<vector<Point>> regions;
            //coord=findBallUsingCircleTemplate(image);
            coord=trackBall(image);
            regions=drawField(&image);
            for(int i=0;i<regions.size();i++) {
                if (i == 0) {
                    for (int j = 0; j < regions[i].size(); j++) {
                        line(org, Point(regions[i][j].x, regions[i][j].y),
                             Point(regions[i][(j + 1) % regions[i].size()].x,
                                   regions[i][(j + 1) % regions[i].size()].y), Scalar(255, 255, 0), 4);
                    }
                }
                else {
                    for (int j = 0; j < regions[i].size() - 1; j++) {
                        if (i == 1)
                            line(org, Point(regions[i][j].x, regions[i][j].y),
                                 Point(regions[i][(j + 1)].x, regions[i][(j + 1)].y), Scalar(255, 0, 255), 4);
                        else
                            line(org, Point(regions[i][j].x, regions[i][j].y),
                                 Point(regions[i][(j + 1)].x, regions[i][(j + 1)].y), Scalar(0, 255, 255), 4);
                    }
                }
            }
            if(coord.size()>0) {
                ROS_INFO("Ball detected...");
                for(int i=0;i<1;i++) {
                    Point center(cvRound(coord[i][0]), cvRound(coord[i][1]));
                    int radius = cvRound(coord[i][2]);
                    circle(org, center, radius, Scalar(0, 0, 255), 4, 8, 0);
                }
            }
            else
                ROS_INFO("Ball not found...");

			for(int i=0;i<100;i=(i+1)%100)
			   imwrite( "src/fields.jpg", gray_image );
			
            namedWindow("Original", WINDOW_NORMAL);
            imshow("Original", org);
            if (waitKey(30) > 0) {
                break;
            }
        }
    else
        cout<<"Something's wrong with the camera...video not captured"<<endl;
    return 0;
}
