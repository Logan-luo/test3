#include <cmath>
#include <iostream>

#include <Eigen/Core>
// #include <Eigen/Core>
#include <Eigen/Dense>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>

#define g 9.8
#define PAI 3.1415926535
using namespace std;
using namespace cv;
using namespace Eigen;

float getDis(Point2f, Point2f);
Point2f findcen(Point2f a, Point2f b, Point2f c, Point2f d);
Point2f setzero(Point2f);

int main(int argc, char *argv[])
{
    if (argc != 2)
        return -1;
    string path = argv[1];
    VideoCapture cap(path);
    // float fps = cap.get(CAP_PROP_FPS); // calculate the fps to get the time between each capture
    if (!cap.isOpened())
    {
        cout << "failed to open the video!" << endl;
        return -1;
    }
    vector<Point2f> cenPoints;
    vector<Point2f> cenPoints1; // to correct the left and right
    vector<Point2f> cenPoints2; // to correct the high and low

    Mat img;
    while (cap.read(img))
    {
        Point2f zero;
        setzero(zero);
        vector<RotatedRect> tubes;
        tubes.reserve(4);
        vector<Point2f> fourPoints;
        fourPoints.reserve(4);
        // get the black//
        vector<Mat> channels(3);
        split(img, channels.data());
        Mat minus_channel = channels[0] - channels[2];
        vector<vector<Point>> contours;
        Mat binary;
        threshold(minus_channel, binary, 40, 255, THRESH_BINARY);

        // preprocessing//
        Mat image = getStructuringElement(MORPH_RECT, Size(3, 3));
        morphologyEx(binary, binary, MORPH_CLOSE, image);

        // find the countour//
        findContours(binary, contours, RETR_EXTERNAL, RETR_LIST);

        cenPoints.reserve(4);

        for (size_t i = 0; i < contours.size(); i++)
        {
            if (30 < contourArea(contours[i]))
            {

                // auto norotaterect = boundingRect(contours[i]) ;
                // RotatedRect temp=norotaterect;
                RotatedRect temp = minAreaRect(contours[i]);
                // drawContours(img, contours, i, Scalar(0, 255, 0), 2);
                if (0.5 < temp.size.height / temp.size.width && temp.size.height / temp.size.width < 2)
                {
                    tubes.push_back(temp);
                    // cout << "THE i" << i << "THE SIZE" << temp.size.height / temp.size.width << endl;
                }
            }
        }
        int count = 0; // count the rectangle
        for (int i = 0; i < 4; i++)
        {
            tubes[i].points(fourPoints.data());
            float aline = getDis(fourPoints[0], fourPoints[1]);
            float bline = getDis(fourPoints[1], fourPoints[2]);
            if (aline > bline)
            {
                swap(aline, bline);
            }
            float a = aline / bline;
            if (0.5 < a)
            {
                for (int j = 0; j < 4; j++)
                {

                    line(img, fourPoints[j], fourPoints[(j + 1) % 4], Scalar(0, 255, 255), 2);
                }
                Point2f temp = findcen(fourPoints[0], fourPoints[1], fourPoints[2], fourPoints[3]);
                cenPoints.push_back(temp);
                count++;
            }
        }
        if (count == 4)
        {
            cenPoints.clear();
        }
        Point2f center;
        setzero(center);
        if (0.8 < (getDis(cenPoints[0], cenPoints[2]) / getDis(cenPoints[1], cenPoints[3])) && (getDis(cenPoints[0], cenPoints[2]) / getDis(cenPoints[1], cenPoints[3])) < 1.2)
        {
            for (int i = 0; i < 4; i++)
            {
                line(img, cenPoints[i], cenPoints[(i + 1) % 4], Scalar(0, 0, 255), 2);
                line(img, cenPoints[i], cenPoints[(i + 2) % 4], Scalar(0, 0, 255), 2);
            }
            center = findcen(cenPoints[0], cenPoints[1], cenPoints[2], cenPoints[3]);
            circle(img, center, 3, Scalar(100, 0, 0), 2);
        }
        // the above is to find the rectangle and the center//
        // calculate the x and y in real world//
        vector<Point3f> objP;
        Mat objM;
        objP.clear();
        objP.push_back(Point3f(120, -120, 0));
        objP.push_back(Point3f(-120, -120, 0));
        objP.push_back(Point3f(-120, 120, 0));
        objP.push_back(Point3f(120, 120, 0));

        //初始化相机参数Opencv
        double camD[9] = {
            5.9763827661155904e+02, 0, 4.1575511901601089e+02,
            0, 5.9922205940008985e+02, 2.6769310598084320e+02,
            0, 0, 1};
        Mat camera_matrix = Mat(3, 3, CV_64FC1, camD);

        //畸变参数
        double distCoeffD[5] = {5.9365728086275861e-02, 6.3271114889939875e-02,
                                5.5006940318826766e-03, -3.5032524991503678e-03, 0.};
        Mat distortion_coefficients = Mat(5, 1, CV_64FC1, distCoeffD);
        // out put
        vector<Point2f> Points2D(4);
        for (int i = 0; i < 4; i++)
        {

            if (cenPoints[i].x > center.x && cenPoints[i].y > center.y)
            {
                Points2D[0] = cenPoints[i];
            }
            if (cenPoints[i].x < center.x && cenPoints[i].y > center.y)
            {
                Points2D[1] = cenPoints[i];
            }
            if (cenPoints[i].x < center.x && cenPoints[i].y < center.y)
            {
                Points2D[2] = cenPoints[i];
            }
            if (cenPoints[i].x > center.x && cenPoints[i].y < center.y)
            {
                Points2D[3] = cenPoints[i];
            }
        }

        Mat rvec, tvec;
        solvePnP(objP, Points2D, camera_matrix, distortion_coefficients, rvec, tvec, false);
        Mat rovec;
        Rodrigues(rvec, rovec);
        Eigen::Vector3d tv;
        cv::cv2eigen(tvec, tv);
        float pitch = atan(rovec.at<float>(1, 0) / rovec.at<float>(0, 0)) * 2 * PAI / 360 - 23;
        float x=tv(0),y=tv(1),z=tv(2);
        string x_str = "x = " + to_string(x);
        putText(img, x_str, Point(1, 20), FONT_HERSHEY_COMPLEX, 0.75, Scalar(255, 0, 0));
        string y_str = "y = " + to_string(y);
        putText(img, y_str, Point(1, 40), FONT_HERSHEY_COMPLEX, 0.75, Scalar(0, 255, 0));
        string z_str = "z = " + to_string(z);
        putText(img, z_str, Point(1, 60), FONT_HERSHEY_COMPLEX, 0.75, Scalar(0, 0, 255));
        string pitch_str = "pitch = " + to_string(pitch);
        putText(img, pitch_str, Point(1, 80), FONT_HERSHEY_COMPLEX, 0.75, Scalar(255, 255, 255));
        // cout << "PITCH=" << pitch << "reg" << endl;
        // cout << sqrt(tv.transpose() * tv) << endl;
        // cout << tv(0) << endl;
        // cout << tv(1) << endl;

        // namedWindow("test1");
        imshow("test2", img);
        //imshow("test1", binary);
        // caculate the speed//

        if (waitKey(30) == 27) // Esc
            if (waitKey(0) == 27)
                break;
    }
}