#include <ros/ros.h>
#include <celex5/celex5.h>
#include <celex5/celex5datamanager.h>
#include <celextypes.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <pthread.h>  
#include <celex5_msgs/event.h>
#include <celex5_msgs/eventVector.h>

#define MAT_ROWS 800
#define MAT_COLS 1280