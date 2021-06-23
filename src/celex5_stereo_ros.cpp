#include "celex5_ros.h"

class Celex5StereoRos : public CeleX5DataManager {
public:
  std::vector<EventData> vecEvent_;
  cv::Mat event_frame_;
  cv::Mat full_picture_;

  pthread_mutex_t event_frame_lock = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t event_vec_lock = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t full_picture_lock = PTHREAD_MUTEX_INITIALIZER;

  CX5SensorDataServer *m_pServer_;
  CeleX5 *celex_;

  CeleX5::emEventPicType type_;

  Celex5StereoRos(CX5SensorDataServer *pServer){
    // pthread_mutex_init(&event_frame_lock,NULL);
    // pthread_mutex_init(&event_vec_lock,NULL);
    m_pServer_ = pServer;
    m_pServer_->registerData(this, CeleX5DataManager::CeleX_Frame_Data);
  }

  ~Celex5StereoRos() {
    pthread_mutex_destroy(&event_frame_lock);
    pthread_mutex_destroy(&event_vec_lock);
    m_pServer_->unregisterData(this, CeleX5DataManager::CeleX_Frame_Data);
    delete celex_;
  }

  std::vector<EventData> getEventVector();
  cv::Mat getEventFrame();
  cv::Mat getFullPicture();
  // overrides the update operation
  virtual void onFrameDataUpdated(CeleX5ProcessedData *pSensorData);
};


// the callback update function
void Celex5StereoRos::onFrameDataUpdated(
    CeleX5ProcessedData *pSensorData) {

  // get event_frame
  pthread_mutex_lock(&event_frame_lock);
  event_frame_ = cv::Mat(800, 1280, CV_8UC1, pSensorData->getEventPicBuffer(type_));
  pthread_mutex_unlock(&event_frame_lock);
  // get raw event data
  pthread_mutex_lock(&event_vec_lock);
  vecEvent_ = pSensorData->getEventDataVector();
  pthread_mutex_unlock(&event_vec_lock);
  // get full_picture
  pthread_mutex_lock(&full_picture_lock);
  full_picture_ = cv::Mat(800, 1280, CV_8UC1, pSensorData->getFullPicBuffer());
  pthread_mutex_unlock(&full_picture_lock);
  }

cv::Mat Celex5StereoRos::getFullPicture(){
  pthread_mutex_lock(&event_frame_lock);
  auto tmp=this->full_picture_.clone(); //clone为深拷贝
  pthread_mutex_unlock(&event_frame_lock);
  return tmp; //mat有自己的数据管理，拷贝代价低
}

cv::Mat Celex5StereoRos::getEventFrame(){
  pthread_mutex_lock(&event_frame_lock);
  auto tmp=this->event_frame_.clone(); //clone为深拷贝
  pthread_mutex_unlock(&event_frame_lock);
  return tmp; //mat有自己的数据管理，拷贝代价低
}

std::vector<EventData> Celex5StereoRos::getEventVector(){
  pthread_mutex_lock(&event_vec_lock);
  auto tmp=this->vecEvent_; //vector默认是深拷贝
  pthread_mutex_unlock(&event_vec_lock);
  return tmp; //这里可以尝试用std::move 来减少一次拷贝
}


// std::vector<EventData> events[2];
// cv::Mat event_frame[2];
Celex5StereoRos *celex_ros_event[2];

struct publisher_with_id
{
  ros::NodeHandle node;
  ros::Publisher pub;
  int id;
  int event_frame_time;
};

// parameters
std::string celex_mode, event_pic_type;
int threshold, clock_rate, event_frame_time;



int main(int argc, char **argv) {
  ros::init(argc, argv, "celex_ros");
  ros::NodeHandle node_;
  CeleX5 *pCelex_;
  pCelex_ = new CeleX5;
  if (NULL == pCelex_)
    return 0;
  pCelex_->openSensor(CeleX5::CeleX5_MIPI);

  publisher_with_id events_pub_0;
  publisher_with_id event_frame_pub_0;
  publisher_with_id full_picture_pub_0;
  publisher_with_id events_pub_1;
  publisher_with_id event_frame_pub_1;
  publisher_with_id full_picture_pub_1;

  // grab the parameters
  node_.param<std::string>("/celex_ros/celex_mode", celex_mode,"Event_Address_Only_Mode");
  node_.param<std::string>("/celex_ros/event_pic_type", event_pic_type,"EventBinaryPic");
  node_.param<int>("/celex_ros/threshold", threshold, 170);   // 0-1024
  node_.param<int>("/celex_ros/clock_rate", clock_rate, 100); // 0-100
  node_.param<int>("/celex_ros/event_frame_time", event_frame_time, 1e4); // 0-100

  events_pub_0.pub = node_.advertise<celex5_msgs::eventVector>("celex5_0/events", 10);
  events_pub_0.id = 0;
  events_pub_0.node = node_;
  events_pub_0.event_frame_time = event_frame_time;
  event_frame_pub_0.pub = node_.advertise<sensor_msgs::Image>("celex5_0/event_frame", 10);
  event_frame_pub_0.id = 0;
  event_frame_pub_0.node = node_;
  event_frame_pub_0.event_frame_time = event_frame_time;
  full_picture_pub_0.pub = node_.advertise<sensor_msgs::Image>("celex5_0/full_picture", 10);
  full_picture_pub_0.id = 0;
  full_picture_pub_0.node = node_;
  full_picture_pub_0.event_frame_time = event_frame_time;
  events_pub_1.pub  = node_.advertise<celex5_msgs::eventVector>("celex5_1/events", 10);
  events_pub_1.id = 1;
  events_pub_1.node = node_;
  events_pub_1.event_frame_time = event_frame_time;
  event_frame_pub_1.pub = node_.advertise<sensor_msgs::Image>("celex5_1/event_frame", 10);
  event_frame_pub_1.id = 1;
  event_frame_pub_1.node = node_;
  event_frame_pub_1.event_frame_time = event_frame_time;
  full_picture_pub_1.pub = node_.advertise<sensor_msgs::Image>("celex5_1/full_picture", 10);
  full_picture_pub_1.id = 1;
  full_picture_pub_1.node = node_;
  full_picture_pub_1.event_frame_time = event_frame_time;

  pCelex_->setThreshold(threshold, 0);
  pCelex_->setThreshold(threshold, 1);
  pCelex_->setEventFrameTime(event_frame_time, 0);
  pCelex_->setEventFrameTime(event_frame_time, 1);

  CeleX5::CeleX5Mode mode;
  if (celex_mode == "Event_Address_Only_Mode")
    mode = CeleX5::Event_Address_Only_Mode;
  else if (celex_mode == "Event_Optical_Flow_Mode")
    mode = CeleX5::Event_Optical_Flow_Mode;
  else if (celex_mode == "Event_Intensity_Mode")
    mode = CeleX5::Event_Intensity_Mode;
  else if (celex_mode == "Full_Picture_Mode")
    mode = CeleX5::Full_Picture_Mode;
  else if (celex_mode == "Full_Optical_Flow_S_Mode")
    mode = CeleX5::Full_Optical_Flow_S_Mode;
  else if (celex_mode == "Full_Optical_Flow_M_Mode")
    mode = CeleX5::Full_Optical_Flow_M_Mode;
  pCelex_->setSensorFixedMode(mode, 0);
  pCelex_->setSensorFixedMode(mode, 1);

  CeleX5::emEventPicType type;
  if (event_pic_type == "EventBinaryPic")
    type = CeleX5::EventBinaryPic;
  else if (event_pic_type == "EventAccumulatedPic")
    type = CeleX5::EventAccumulatedPic;
  else if (event_pic_type == "EventGrayPic")
    type = CeleX5::EventGrayPic;
  else if (event_pic_type == "EventCountPic")
    type = CeleX5::EventCountPic;
  else if (event_pic_type == "EventDenoisedBinaryPic")
    type = CeleX5::EventDenoisedBinaryPic;
  else if (event_pic_type == "EventSuperimposedPic")
    type = CeleX5::EventSuperimposedPic;
  else if (event_pic_type == "EventDenoisedCountPic")
    type = CeleX5::EventDenoisedCountPic;

  pCelex_->setFpnFile("/home/robin/catkin_ws/src/celex5_stereo/celex5dependencies/stereo/config/FPN_M_2.txt", 0);
  pCelex_->setFpnFile("/home/robin/catkin_ws/src/celex5_stereo/celex5dependencies/stereo/config/FPN_S_2.txt", 1);

  celex_ros_event[0] = new Celex5StereoRos(pCelex_->getSensorDataServer(0));
  celex_ros_event[1] = new Celex5StereoRos(pCelex_->getSensorDataServer(1));

  celex_ros_event[0]->type_ = type;
  celex_ros_event[1]->type_ = type;

  // events[0] = celex_ros_0->vecEvent_;
  // events[1] = celex_ros_1->vecEvent_;
  // event_frame[0] = celex_ros_0->event_frame_;
  // event_frame[1] = celex_ros_1->event_frame_;
  
  pthread_t events_pub_thread_0;
  pthread_t events_pub_thread_1;
  pthread_t frame_pub_thread_0;
  pthread_t frame_pub_thread_1;   

  int rc;
  void *publish_event_frame(void *arg);
  void *publish_full_picture(void *arg);
  void *publish_events(void *arg);

  cout << "main() : 创建主相机事件向量发布线程" << endl;      
  rc = pthread_create(&events_pub_thread_0, NULL, publish_events, (void *)&events_pub_0);
  if (rc){
      cout << "Error:无法创建主相机事件向量发布线程" << endl;
      exit(-1);
  }

  if (mode == CeleX5::Event_Address_Only_Mode){
    cout << "main() : 创建主相机事件帧发布线程" << endl;      
    rc = pthread_create(&frame_pub_thread_0, NULL, publish_event_frame, (void *)&event_frame_pub_0);
    if (rc){
        cout << "Error:无法创建主相机事件帧发布线程" << endl;
        exit(-1);
    }
  }
  else if (mode == CeleX5::Full_Picture_Mode){
    cout << "main() : 创建主相机灰度图发布线程" << endl;      
    rc = pthread_create(&frame_pub_thread_0, NULL, publish_full_picture, (void *)&full_picture_pub_0);
    if (rc){
        cout << "Error:无法创建主相机灰度图发布线程" << endl;
        exit(-1);
    }
  }

  cout << "main() : 创建从相机事件向量发布线程" << endl;      
  rc = pthread_create(&events_pub_thread_1, NULL, publish_events, (void *)&events_pub_1);
  if (rc){
      cout << "Error:无法创建主相机事件向量发布线程" << endl;
      exit(-1);
  }

  if (mode == CeleX5::Event_Address_Only_Mode){
    cout << "main() : 创建从相机事件帧发布线程, " << endl;      
    rc = pthread_create(&frame_pub_thread_1, NULL, publish_event_frame, (void *)&event_frame_pub_1);
    if (rc){
        cout << "Error:无法创建主相机事件帧发布线程," << endl;
        exit(-1);
    }
  }
  else if (mode == CeleX5::Full_Picture_Mode){
    cout << "main() : 创建从相机灰度图发布线程" << endl;      
    rc = pthread_create(&frame_pub_thread_1, NULL, publish_full_picture, (void *)&full_picture_pub_1);
    if (rc){
        cout << "Error:无法创建从相机灰度图发布线程" << endl;
        exit(-1);
    }
  }

  ros::Rate loop_rate(1e6/event_frame_time);
  while (node_.ok()) {
    // events[0] = celex_ros_0->vecEvent_;
    // events[1] = celex_ros_1->vecEvent_;
    // event_frame[0] = celex_ros_0->event_frame_;
    // event_frame[1] = celex_ros_1->event_frame_;
    // ros::spinOnce();
    loop_rate.sleep(); //理论上主线程可以退出了，具体得看ros那边
  }
  return EXIT_SUCCESS;
}

void *publish_event_frame(void *arg){
    struct publisher_with_id *publisher;
    publisher = (struct publisher_with_id *) arg;
    ros::Rate loop_rate(1e6/publisher->event_frame_time);
  while (publisher->node.ok()) {
    auto data=celex_ros_event[publisher->id]->getEventFrame();
    sensor_msgs::ImagePtr event_frame_ros = cv_bridge::CvImage(std_msgs::Header(), "mono8", data).toImageMsg();
    // sensor_msgs::ImagePtr event_frame_ros = cv_bridge::CvImage(std_msgs::Header(), "mono8", event_frame[publisher->id]).toImageMsg();
    publisher->pub.publish(event_frame_ros);
    // ros::spinOnce();
    loop_rate.sleep();
  }
}

void *publish_full_picture(void *arg){
    struct publisher_with_id *publisher;
    publisher = (struct publisher_with_id *) arg;
    ros::Rate loop_rate(1e6/publisher->event_frame_time);
  while (publisher->node.ok()) {
    auto data=celex_ros_event[publisher->id]->getFullPicture();
    sensor_msgs::ImagePtr full_picture_ros = cv_bridge::CvImage(std_msgs::Header(), "mono8", data).toImageMsg();
    // sensor_msgs::ImagePtr event_frame_ros = cv_bridge::CvImage(std_msgs::Header(), "mono8", event_frame[publisher->id]).toImageMsg();
    publisher->pub.publish(full_picture_ros);
    // ros::spinOnce();
    loop_rate.sleep();
  }
}

void *publish_events(void *arg){
  struct publisher_with_id *publisher;
  publisher = (struct publisher_with_id *) arg;
  celex5_msgs::eventVector event_vector;
  ros::Rate loop_rate(1e6/publisher->event_frame_time);
  int tmp = 0;
  while (publisher->node.ok()) {
      auto data=  celex_ros_event[publisher->id]->getEventVector();
      int dataSize = data.size();
      event_vector.vectorLength = dataSize;
      celex5_msgs::event event_;
      for (int i = 0; i < dataSize; i++) {
        event_.x = data[i].row;
        event_.y = data[i].col;
        event_.brightness = 255;
        event_.timestamp.sec = floor(data[i].t_increasing/5e4);
        event_.timestamp.nsec = data[i].t_increasing%50000*2e4;
        event_vector.events.push_back(event_);
        // if (floor(data[i].t_increasing/5e4) == tmp && publisher->id == 0)
        // {
        //   std::cout << "master: " << data[i].t_increasing/5e4 << std::endl;
        //   tmp++ ;
        // }
        // if (floor(data[i].t_increasing/5e4) == tmp && publisher->id == 1)
        // {
        //   std::cout << "slave: " << data[i].t_increasing/5e4 << std::endl;
        //   tmp++ ;
        // }
      }
      publisher->pub.publish(event_vector);
      event_vector.events.clear();

    loop_rate.sleep();      
    }
}
