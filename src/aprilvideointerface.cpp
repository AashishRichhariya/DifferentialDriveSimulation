#include <cmath>
#include <typeinfo>
#include <cstring>
#include <sys/time.h>
#include "aprilvideointerface.h"
using namespace Eigen;
using namespace std;
using namespace cv;

const string usage = "\n"
  "Usage:\n"
  "  apriltags_demo [OPTION...] [IMG1 [IMG2...]]\n"
  "\n"
  "Options:\n"
  "  -h  -?          Show help options\n"
  "  -a              Arduino (send tag ids over serial port)\n"
  "  -d              Disable graphics\n"
  "  -t              Timing of tag extraction\n"
  "  -C <bbxhh>      Tag family (default 36h11)\n"
  "  -D <id>         Video device ID (if multiple cameras present)\n"
  "  -F <fx>         Focal length in pixels\n"
  "  -W <width>      Image width (default 640, availability depends on camera)\n"
  "  -H <height>     Image height (default 480, availability depends on camera)\n"
  "  -S <size>       Tag size (square black frame) in meters\n"
  "  -E <exposure>   Manually set camera exposure (default auto; range 0-10000)\n"
  "  -G <gain>       Manually set camera gain (default auto; range 0-255)\n"
  "  -B <brightness> Manually set the camera brightness (default 128; range 0-255)\n"
  "\n";

const string intro = "\n"
    "April tags test code\n"
    "(C) 2012-2014 Massachusetts Institute of Technology\n"
    "Michael Kaess\n"
    "\n";
#ifndef __APPLE__
#define EXPOSURE_CONTROL // only works in Linux
#endif

#ifdef EXPOSURE_CONTROL
#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <errno.h>
#endif
// Needed for getopt / command line options processing
#include <unistd.h>
extern int optind;
extern char *optarg;
// utility function to provide current system time (used below in
// determining frame rate at which images are being processed)
double tic() {
  struct timeval t;
  gettimeofday(&t, NULL);
  return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}
//Normalize angle to be within the interval [-pi,pi].
inline double standardRad(double t) {
  if (t >= 0.) {
    t = fmod(t+PI, TWOPI) - PI;
  } else {
    t = fmod(t-PI, -TWOPI) + PI;
  }
  return t;
}
//Convert rotation matrix to Euler angles
void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}
//for determining opencv matrix type
string type2str(int type) {
  string r;

  uchar depth = type & CV_MAT_DEPTH_MASK;
  uchar chans = 1 + (type >> CV_CN_SHIFT);

  switch ( depth ) {
    case CV_8U:  r = "8U"; break;
    case CV_8S:  r = "8S"; break;
    case CV_16U: r = "16U"; break;
    case CV_16S: r = "16S"; break;
    case CV_32S: r = "32S"; break;
    case CV_32F: r = "32F"; break;
    case CV_64F: r = "64F"; break;
    default:     r = "User"; break;
  }

  r += "C";
  r += (chans+'0');

  return r;
}

//AprilInterfaceAndVideoCapture class definitions
// changing the tag family
void AprilInterfaceAndVideoCapture::setTagCodes(string s) {
  if (s=="16h5") {
    m_tagCodes = AprilTags::tagCodes16h5;
  } else if (s=="25h7") {
    m_tagCodes = AprilTags::tagCodes25h7;
  } else if (s=="25h9") {
    m_tagCodes = AprilTags::tagCodes25h9;
  } else if (s=="36h9") {
    m_tagCodes = AprilTags::tagCodes36h9;
  } else if (s=="36h11") {
    m_tagCodes = AprilTags::tagCodes36h11;
  } else {
    cout << "Invalid tag family specified" << endl;
    exit(1);
  }
}
// parse command line options to change default behavior
void AprilInterfaceAndVideoCapture::parseOptions(int argc, char* argv[]) {
  int c;
  while ((c = getopt(argc, argv, ":h?adtC:F:H:S:W:E:G:B:D:")) != -1) {
    // Each option character has to be in the string in getopt();
    // the first colon changes the error character from '?' to ':';
    // a colon after an option means that there is an extra
    // parameter to this option; 'W' is a reserved character
    switch (c) {
    case 'h':
    case '?':
      cout << intro;
      cout << usage;
      exit(0);
      break;
    case 'a':
      m_arduino = true;
      break;
    case 'd':
      m_draw = false;
      break;
    case 't':
      m_timing = true;
      break;
    case 'C':
      setTagCodes(optarg);
      break;
    case 'F':
      m_fx = atof(optarg);
      m_fy = m_fx;
      break;
    case 'H':
      m_height = atoi(optarg);
      m_py = m_height/2;
       break;
    case 'S':
      m_tagSize = atof(optarg);
      break;
    case 'W':
      m_width = atoi(optarg);
      m_px = m_width/2;
      break;
    case 'E':
#ifndef EXPOSURE_CONTROL
      cout << "Error: Exposure option (-E) not available" << endl;
      exit(1);
#endif
      m_exposure = atoi(optarg);
      break;
    case 'G':
#ifndef EXPOSURE_CONTROL
      cout << "Error: Gain option (-G) not available" << endl;
      exit(1);
#endif
      m_gain = atoi(optarg);
      break;
    case 'B':
#ifndef EXPOSURE_CONTROL
      cout << "Error: Brightness option (-B) not available" << endl;
      exit(1);
#endif
      m_brightness = atoi(optarg);
      break;
    case 'D':
      m_deviceId = atoi(optarg);
      break;
    case ':': // unknown option, from getopt
      cout << intro;
      cout << usage;
      exit(1);
      break;
    }
  }

  if (argc > optind) {
    for (int i=0; i<argc-optind; i++) {
      m_imgNames.push_back(argv[optind+i]);
    }
  }
}

void AprilInterfaceAndVideoCapture::setup(){
  m_tagDetector = new AprilTags::TagDetector(m_tagCodes);
}

void AprilInterfaceAndVideoCapture::setupVideo(){
#ifdef EXPOSURE_CONTROL
  // manually setting camera exposure settings; OpenCV/v4l1 doesn't
  // support exposure control; so here we manually use v4l2 before
  // opening the device via OpenCV; confirmed to work with Logitech
  // C270; try exposure=20, gain=100, brightness=150
  string video_str = "/dev/video0";
  video_str[10] = '0' + m_deviceId;
  int device = v4l2_open(video_str.c_str(), O_RDWR | O_NONBLOCK);

  if (m_exposure >= 0) {
    // not sure why, but v4l2_set_control() does not work for
    // V4L2_CID_EXPOSURE_AUTO...
    struct v4l2_control c;
    c.id = V4L2_CID_EXPOSURE_AUTO;
    c.value = 1; // 1=manual, 3=auto; V4L2_EXPOSURE_AUTO fails...
    if (v4l2_ioctl(device, VIDIOC_S_CTRL, &c) != 0) {
      cout << "Failed to set... " << strerror(errno) << endl;
    }
    cout << "exposure: " << m_exposure << endl;
    v4l2_set_control(device, V4L2_CID_EXPOSURE_ABSOLUTE, m_exposure*6);
  }
  if (m_gain >= 0) {
    cout << "gain: " << m_gain << endl;
    v4l2_set_control(device, V4L2_CID_GAIN, m_gain*256);
  }
  if (m_brightness >= 0) {
    cout << "brightness: " << m_brightness << endl;
    v4l2_set_control(device, V4L2_CID_BRIGHTNESS, m_brightness*256);
  }
  v4l2_close(device);
#endif 

  // find and open a USB camera (built in laptop camera, web cam etc)
  m_cap = cv::VideoCapture(m_deviceId);
      if(!m_cap.isOpened()) {
    cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
    exit(1);
  }
  m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
  m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
  cout << "Camera successfully opened (ignore error messages above...)" << endl;
  cout << "Actual resolution: "
       << m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
       << m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;
}

void AprilInterfaceAndVideoCapture::pixelToWorld(double x,double y,double &xd,double &yd){
  x = x-m_px;
  y = y-m_py;
  Eigen::Vector3d tp(x/m_fx,y/m_fy,1);
  Eigen::Vector3d normal = x_axis.cross(y_axis);
  double d = normal.dot(planeOrigin);
  double tpd = normal.dot(tp);
  double zdash = d/tpd;
  tp = zdash*tp;
  tp = tp-planeOrigin;
  xd = tp.dot(x_axis);
  yd = tp.dot(y_axis);
}

//find the normal vector to the plane formed by the endpoints of tag
void AprilInterfaceAndVideoCapture::findNormal(Eigen::Vector3d &trans, Eigen::Matrix3d &rot, Eigen::Vector3d &result){
  Eigen::Vector3d origin(0,0,0),xcoord(1,0,0),ycoord(0,1,0);
  origin = rot*origin+trans;
  xcoord = rot*xcoord+trans;
  ycoord = rot*ycoord+trans;
  xcoord = xcoord-origin;
  ycoord = ycoord-origin;
  result = xcoord.cross(ycoord);
}

void AprilInterfaceAndVideoCapture::extractPlane(int ind){
  Eigen::Vector3d translation;
  Eigen::Matrix3d rotation;
  detections[ind].getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,translation, rotation);
  Eigen::Vector3d ori(0,0,0);
  Eigen::Vector3d xone(1,0,0);
  Eigen::Vector3d yone(0,1,0);
  planeOrigin = (rotation*ori) + translation;
  x_axis = (rotation*xone) + translation;
  x_axis = x_axis - planeOrigin;
  x_axis.normalize();//this step is important
  y_axis = (rotation*yone) + translation;
  y_axis = y_axis - planeOrigin;
  y_axis.normalize();
}

//robot is assumed to be facing positive y direction of it's apriltag
void AprilInterfaceAndVideoCapture::findRobotPose(int ind, robot_pose &rob){
  Eigen::Vector3d translation;
  Eigen::Matrix3d rotation;
  detections[ind].getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,translation, rotation);
  Eigen::Vector3d ori(0,0,0);
  ori = (rotation*ori)+translation;//ori is now the robot centre
  ori = ori-planeOrigin;
  rob.x = ori.dot(x_axis);
  rob.y = ori.dot(y_axis);
  Eigen::Vector3d ycoord(0,1,0);
  ycoord = (rotation*ycoord)+translation;
  ycoord = ycoord-planeOrigin;
  double tempx = ycoord.dot(x_axis), tempy = ycoord.dot(y_axis);
  tempx = tempx-rob.x; tempy = tempy-rob.y;
  rob.omega = atan2(tempy,tempx);
}

void AprilInterfaceAndVideoCapture::processImage(cv::Mat& image, cv::Mat& image_gray) {
  // alternative way is to grab, then retrieve; allows for
  // multiple grab when processing below frame rate - v4l keeps a
  // number of frames buffered, which can lead to significant lag
  //      m_cap.grab();
  //      m_cap.retrieve(image);
  // detect April tags (requires a gray scale image)
  cv::cvtColor(image, image_gray, CV_BGR2GRAY);
  double t0;
  if (m_timing) {
    t0 = tic();
  }
  detections = m_tagDetector->extractTags(image_gray);
  if (m_timing) {
    double dt = tic()-t0;
    cout << "Extracting tags took " << dt << " seconds." << endl;
  }
  cout << detections.size() << " tags detected:" << endl;
}

// Load and process a single image
void AprilInterfaceAndVideoCapture::loadImages() {
  cv::Mat image;
  cv::Mat image_gray;
  for (list<string>::iterator it=m_imgNames.begin(); it!=m_imgNames.end(); it++) {
    image = cv::imread(*it); // load image with opencv
    processImage(image, image_gray);
    while (cv::waitKey(100) == -1) {}
  }
}

// check the image container to find if video is to be processed or image
bool AprilInterfaceAndVideoCapture::isVideo() {
  return m_imgNames.empty();
}
