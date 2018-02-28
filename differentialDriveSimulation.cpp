#include "aprilvideointerface.h"
#include "pathplanners.h"
#include "controllers.h"
// For Arduino: serial port access class
#include "Serial.h"
using namespace std;
using namespace cv;

int main(int argc, char* argv[]) {
  int x = 43, y =4343;
  int &ab = x;
  int &xy = y;
  ab = xy;
  cout<<ab<<endl;
  return 0;
}
