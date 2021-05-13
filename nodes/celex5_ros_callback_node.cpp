#include "celex5_ros.h"
#include <pthread.h>
#include <chrono>   

static const std::string OPENCV_WINDOW = "Image window";
static const std::string OPENCV_WINDOW_1 = "Image window_1";
namespace celex_ros_callback {
class CelexRosCallBackNode : public CeleX5DataManager {
public:
  std::vector<EventData> vecEvent_;
  cv::Mat event_frame_;

  pthread_mutex_t frame_lock = PTHREAD_MUTEX_INITIALIZER;
  pthread_mutex_t event_vec_lock = PTHREAD_MUTEX_INITIALIZER;

  CX5SensorDataServer *m_pServer_;
  CeleX5 *celex_;

  CelexRosCallBackNode(CX5SensorDataServer *pServer){
    // pthread_mutex_init(&frame_lock,NULL);
    // pthread_mutex_init(&event_vec_lock,NULL);
    m_pServer_ = pServer;
    m_pServer_->registerData(this, CeleX5DataManager::CeleX_Frame_Data);
  }

  ~CelexRosCallBackNode() {
    pthread_mutex_destroy(&frame_lock);
    pthread_mutex_destroy(&event_vec_lock);
    m_pServer_->unregisterData(this, CeleX5DataManager::CeleX_Frame_Data);
    delete celex_;
  }

  std::vector<EventData> getEventVector();
  cv::Mat getFrame();
  // overrides the update operation
  virtual void onFrameDataUpdated(CeleX5ProcessedData *pSensorData);
};


// the callback update function
void CelexRosCallBackNode::onFrameDataUpdated(
    CeleX5ProcessedData *pSensorData) {

  // get event_frame
  pthread_mutex_lock(&frame_lock);
  event_frame_ = cv::Mat(800, 1280, CV_8UC1, pSensorData->getEventPicBuffer(CeleX5::EventBinaryPic));
  pthread_mutex_unlock(&frame_lock);
  // get raw event data
  pthread_mutex_lock(&event_vec_lock);
  vecEvent_ = pSensorData->getEventDataVector();
  pthread_mutex_unlock(&event_vec_lock);
  }

cv::Mat CelexRosCallBackNode::getFrame(){
  pthread_mutex_lock(&frame_lock);
  auto tmp=this->event_frame_.clone(); //clone为深拷贝
  pthread_mutex_unlock(&frame_lock);
  return tmp; //mat有自己的数据管理，拷贝代价低
}

std::vector<EventData> CelexRosCallBackNode::getEventVector(){
  pthread_mutex_lock(&event_vec_lock);
  auto tmp=this->vecEvent_; //vector默认是深拷贝
  pthread_mutex_unlock(&event_vec_lock);
  return tmp; //这里可以尝试用std::move 来减少一次拷贝
}

}

// std::vector<EventData> events[2];
// cv::Mat event_frame[2];
celex_ros_callback::CelexRosCallBackNode *celex_ros_event[2];

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
  ros::init(argc, argv, "celex_ros_callback");
  ros::NodeHandle node_;
  CeleX5 *pCelex_;
  pCelex_ = new CeleX5;
  if (NULL == pCelex_)
    return 0;
  pCelex_->openSensor(CeleX5::CeleX5_MIPI);

  publisher_with_id events_pub_0;
  publisher_with_id events_pub_1;
  publisher_with_id event_frame_pub_0;
  publisher_with_id event_frame_pub_1;

  // grab the parameters
  node_.param<std::string>("/celex_ros_callback/celex_mode", celex_mode,"Event_Address_Only_Mode");
  node_.param<std::string>("/celex_ros_callback/event_pic_type", event_pic_type,"EventBinaryPic");
  node_.param<int>("/celex_ros_callback/threshold", threshold, 170);   // 0-1024
  node_.param<int>("/celex_ros_callback/clock_rate", clock_rate, 100); // 0-100
  node_.param<int>("/celex_ros_callback/event_frame_time", event_frame_time, 1e4); // 0-100

  events_pub_0.pub = node_.advertise<celex5_msgs::eventVector>("celex5_0/events", 10);
  events_pub_0.id = 0;
  events_pub_0.node = node_;
  events_pub_0.event_frame_time = event_frame_time;
  events_pub_1.pub  = node_.advertise<celex5_msgs::eventVector>("celex5_1/events", 10);
  events_pub_1.id = 1;
  events_pub_1.node = node_;
  events_pub_1.event_frame_time = event_frame_time;
  event_frame_pub_0.pub = node_.advertise<sensor_msgs::Image>("celex5_0/event_frame", 10);
  event_frame_pub_0.id = 0;
  event_frame_pub_0.node = node_;
  event_frame_pub_0.event_frame_time = event_frame_time;
  event_frame_pub_1.pub = node_.advertise<sensor_msgs::Image>("celex5_1/event_frame", 10);
  event_frame_pub_1.id = 1;
  event_frame_pub_1.node = node_;
  event_frame_pub_1.event_frame_time = event_frame_time;

  pCelex_->setThreshold(threshold,0);
  pCelex_->setThreshold(threshold, 1);
  pCelex_->setEventFrameTime(event_frame_time,0);
  pCelex_->setEventFrameTime(event_frame_time,1);

  CeleX5::CeleX5Mode mode;
  if (celex_mode == "Event_Address_Only_Mode")
    mode = CeleX5::Event_Address_Only_Mode;
  else if (celex_mode == "Event_Optical_Flow_Mode")
    mode = CeleX5::Event_Optical_Flow_Mode;
  pCelex_->setSensorFixedMode(mode, 0);
  pCelex_->setSensorFixedMode(mode, 1);

  celex_ros_event[0] = new celex_ros_callback::CelexRosCallBackNode(pCelex_->getSensorDataServer(0));
  celex_ros_event[1]= new celex_ros_callback::CelexRosCallBackNode(pCelex_->getSensorDataServer(1));

  // events[0] = celex_ros_0->vecEvent_;
  // events[1] = celex_ros_1->vecEvent_;
  // event_frame[0] = celex_ros_0->event_frame_;
  // event_frame[1] = celex_ros_1->event_frame_;
  
  pthread_t events_pub_thread_0;
  pthread_t events_pub_thread_1;
  pthread_t event_frame_pub_thread_0;
  pthread_t event_frame_pub_thread_1;   

  int rc;
  void *publish_event_frame(void *arg);
  void *publish_events(void *arg);

  cout << "main() : 创建主相机事件向量发布线程" << endl;      
  rc = pthread_create(&events_pub_thread_0, NULL, publish_events, (void *)&events_pub_0);
  if (rc){
      cout << "Error:无法创建主相机事件向量发布线程" << endl;
      exit(-1);
  }
  cout << "main() : 创建主相机事件帧发布线程" << endl;      
  rc = pthread_create(&event_frame_pub_thread_0, NULL, publish_event_frame, (void *)&event_frame_pub_0);
  if (rc){
      cout << "Error:无法创建主相机事件帧发布线程" << endl;
      exit(-1);
  }

  cout << "main() : 创建从相机事件向量发布线程" << endl;      
  rc = pthread_create(&events_pub_thread_1, NULL, publish_events, (void *)&events_pub_1);
  if (rc){
      cout << "Error:无法创建主相机事件向量发布线程" << endl;
      exit(-1);
  }
  cout << "main() : 创建从相机事件帧发布线程, " << endl;      
  rc = pthread_create(&event_frame_pub_thread_1, NULL, publish_event_frame, (void *)&event_frame_pub_1);
  if (rc){
      cout << "Error:无法创建主相机事件帧发布线程," << endl;
      exit(-1);
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
    auto data=celex_ros_event[publisher->id]->getFrame();
    sensor_msgs::ImagePtr event_frame_ros = cv_bridge::CvImage(std_msgs::Header(), "mono8", data).toImageMsg();
    // sensor_msgs::ImagePtr event_frame_ros = cv_bridge::CvImage(std_msgs::Header(), "mono8", event_frame[publisher->id]).toImageMsg();
    publisher->pub.publish(event_frame_ros);
    // ros::spinOnce();
    loop_rate.sleep();
  }
}

void *publish_events(void *arg){
  struct publisher_with_id *publisher;
  publisher = (struct publisher_with_id *) arg;
  celex5_msgs::eventVector event_vector;
  ros::Rate loop_rate(1e6/publisher->event_frame_time);
  while (publisher->node.ok()) {
      auto data=  celex_ros_event[publisher->id]->getEventVector();
      int dataSize = data.size();
      event_vector.vectorLength = dataSize;
      celex5_msgs::event event_;
      for (int i = 0; i < dataSize; i++) {
        event_.x = data[i].row;
        event_.y = data[i].col;
        event_.brightness = 255;
        event_vector.events.push_back(event_);
      }
      publisher->pub.publish(event_vector);
      event_vector.events.clear();

      // int dataSize = events[publisher->id].size();
      // event_vector.vectorLength = dataSize;
      // celex5_msgs::event event_;
      // for (int i = 0; i < dataSize; i++) {
      //   event_.x = events[publisher->id][i].row;
      //   event_.y = events[publisher->id][i].col;
      //   event_.brightness = 255;
      //   event_vector.events.push_back(event_);
      // }
      // publisher->pub.publish(event_vector);
      // event_vector.events.clear();

    // ros::spinOnce();
    loop_rate.sleep();      
    }
}
