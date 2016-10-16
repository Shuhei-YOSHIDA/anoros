//Nodelet2.cpp
//Shuhei-YOSHIDA 2016/10/17

// ヘッダをインクルード
#include "nodelet_tutorials/Nodelet2.h"

// 実装側の.cppにこれを置く
#include <pluginlib/class_list_macros.h>
// 大文字のモノは大抵マクロ
//PLUGINLIB_EXPORT_CLASS(nodelet_tutorials::Nodelet2, nodelet::Nodelet)
PLUGINLIB_DECLARE_CLASS(nodelet_tutorials, Nodelet2, nodelet_tutorials::Nodelet2, nodelet::Nodelet);

// ヘッダに定義したクラスの実装を記述
namespace nodelet_tutorials {

Nodelet2::Nodelet2()
{
    ROS_INFO("Nodelet2 Class constructor");
    std::cout << "NODELET2 constucted" << std::endl;
    loopPtr = new ros::Rate(10);
}

Nodelet2::~Nodelet2()
{
    ROS_INFO("Nodelet2 Class destructor");
    std::cout << "NODELET2 destructed" << std::endl;
    delete loopPtr;
    threadPtr->join();
}

void Nodelet2::onInit()
{
    NODELET_INFO("Nodelet2 Class %s -:(NODELET_INFO):- %s", getName().c_str(), __FUNCTION__);
    NODELET_DEBUG("initializing Nodelet2");
    ROS_INFO("Nodelet2 Class -:(ROS_INFO):-, %s", __FUNCTION__);
    std::cout << "NODELET2 initialized" << std::endl;

    spinThread_nh = getMTPrivateNodeHandle();
    nh = getPrivateNodeHandle();
    pub = spinThread_nh.advertise<std_msgs::String>("outmsg", 10);
    sub = nh.subscribe("inmsg", 10, &Nodelet2::inmsgCB, this);

    threadPtr = boost::shared_ptr<boost::thread>(new boost::thread(&Nodelet2::spinThread, this));
}

void Nodelet2::inmsgCB(const std_msgs::String::ConstPtr& input)
{
    NODELET_INFO("normal Callback");
}

void Nodelet2::spinThread()
{
    while (ros::ok())
    {
        ROS_INFO("Thread is spined");
        std_msgs::String msg; msg.data = "NowSpin!";
        pub.publish(msg);
        ros::spinOnce();
        loopPtr->sleep();
    }
}

} // namespace nodelet_tutorials 


