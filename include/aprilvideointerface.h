#ifndef APRILINTERFACE_H
#define APRILINTERFACE_H
#include <list>
#include <vector>
#include <string>
#include <Eigen/Geometry>
// OpenCV library for easy access to USB camera and drawing of images
#include "opencv2/opencv.hpp"
#include "opencv2/core.hpp"
// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
#include "AprilTags/TagDetector.h"
// April tags detector and various families that can be selected by command line option
#include "AprilTags/Tag16h5.h"
#include "AprilTags/Tag25h7.h"
#include "AprilTags/Tag25h9.h"
#include "AprilTags/Tag36h9.h"
#include "AprilTags/Tag36h11.h"

#include "structures.h"
double tic();
//Normalize angle to be within the interval [-pi,pi].
inline double standardRad(double t);
//Convert rotation matrix to Euler angles
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll);
//for determining opencv matrix type
std::string type2str(int type);

//starts video capture and detects the april tags in the scene, also parses the command line options
class AprilInterfaceAndVideoCapture{
  public:
  AprilTags::TagDetector* m_tagDetector;
  AprilTags::TagCodes m_tagCodes;
  bool m_draw; // draw image and April tag detections?
  bool m_arduino; // send tag detections to serial port?
  bool m_timing; // print timing information for each tag extraction call

  int m_width; // image size in pixels
  int m_height;
  double m_tagSize; // April tag side length in meters of square black frame
  double m_fx; // camera focal length in pixels
  double m_fy;
  double m_px; // camera principal point
  double m_py;

  int m_deviceId; // camera id (in case of multiple cameras)
  std::list<std::string> m_imgNames;

  cv::VideoCapture m_cap;

  int m_exposure;
  int m_gain;
  int m_brightness;

  Eigen::Vector3d planeOrigin;
  Eigen::Vector3d x_axis;
  Eigen::Vector3d y_axis;
  
  std::vector<AprilTags::TagDetection> detections;
  //remember to add a class var to specify video device number, currently being assumed at 0 in setupvideo
  AprilInterfaceAndVideoCapture() :
    // default settings, most can be modified through command line opt. (see below)
    m_tagDetector(NULL),
    m_tagCodes(AprilTags::tagCodes36h11),

    m_draw(true),
    m_arduino(false),
    m_timing(false),

    //below parameters are the most important
    //use a camera calibration technique to find out the below parameters
    //below parameters are found using opencv calibration example module in opencv installation 
    m_width(640),
    m_height(480),
    m_tagSize(13.5),
    m_fx(6.4205269897923586e+02),//focal length in pixels
    m_fy(6.4205269897923586e+02),
    m_px(m_width/2),
    m_py(m_height/2),

    m_exposure(-1),
    m_gain(-1),
    m_brightness(-1), 
    m_deviceId(0){}

  // changing the tag family
  void setTagCodes(string s);
  // parse command line options to change default behavior
  void parseOptions(int argc, char* argv[]);
  void setup();
  void setupVideo();

  void pixelToWorld(double x,double y,double &xd,double &yd);
  //find the normal vector to the plane formed by the endpoints of tag
  void findNormal(Eigen::Vector3d &trans, Eigen::Matrix3d &rot, Eigen::Vector3d &result);
  void extractPlane(int ind);
  //robot is assumed to be facing positive y direction of it's apriltag
  void findRobotPose(int ind, robot_pose &rob);
  void processImage(cv::Mat& image, cv::Mat& image_gray);
  // Load and process a single image
  void loadImages();
  // check the image container to find if video is to be processed or image
  bool isVideo();
}; 
#endif
