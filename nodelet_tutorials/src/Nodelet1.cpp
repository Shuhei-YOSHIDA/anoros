// Nodelet1.cpp
// Shuhei-YOSHIDA 2016/10/15

// ヘッダをインクルード
#include "nodelet_tutorials/Nodelet1.h"

// 実装側の.cppにこれを置く
#include <pluginlib/class_list_macros.h>
// 大文字のモノは大抵マクロ
//PLUGINLIB_EXPORT_CLASS(nodelet_tutorials::Nodelet1, nodelet::Nodelet)
PLUGINLIB_DECLARE_CLASS(nodelet_tutorials, Nodelet1, nodelet_tutorials::Nodelet1, nodelet::Nodelet);

// ヘッダに定義したクラスの実装を記述
namespace nodelet_tutorials {

Nodelet1::Nodelet1()
{
    ROS_INFO("Nodelet1 Class constructor");
    std::cout << "NODELET1 constucted" << std::endl;
}

Nodelet1::~Nodelet1()
{
    ROS_INFO("Nodelet1 Class destructor");
    std::cout << "NODELET1 destructed" << std::endl;
}

void Nodelet1::onInit()
{
    NODELET_INFO("Nodelet1 Class %s -:(NODELET_INFO):- %s", getName().c_str(), __FUNCTION__);
    NODELET_DEBUG("initializing Nodelet1");
    ROS_INFO("Nodelet1 Class -:(ROS_INFO):-, %s", __FUNCTION__);
    std::cout << "NODELET1 initialized" << std::endl;

    ros::NodeHandle &nh = getPrivateNodeHandle();
    pub = nh.advertise<std_msgs::String>("outmsg", 10);
    sub = nh.subscribe("inmsg", 10, &Nodelet1::inmsgCB, this);
}

void Nodelet1::inmsgCB(const std_msgs::String::ConstPtr& input)
{
    std_msgs::StringPtr output(new std_msgs::String());
    output->data = "outed!";
    NODELET_INFO("Callback");
    pub.publish(output);
}

} // namespace nodelet_tutorials 


