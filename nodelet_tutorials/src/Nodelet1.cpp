// Nodelet1.cpp
// Shuhei-YOSHIDA 2016/10/15

// ヘッダをインクルード
#include "nodelet_tutorials/Nodelet1.h"

// 実装側の.cppにこれを置く
#include <pluginlib/class_list_macros.h>

// ヘッダに定義したクラスの実装を記述
namespace _Nodelet1 {

Nodelet1::Nodelet1()
{
    ROS_INFO("Nodelet1 Class constructor");
}

Nodelet1::~Nodelet1()
{
    ROS_INFO("Nodelet1 Class destructor");
}

void Nodelet1::onInit()
{
    NODELET_INFO("Nodelet1 Class -:(NODELET_INFO):- %s", __FUNCTION__);
    NODELET_DEBUG("initializing Nodelet1");
    ROS_INFO("Nodelet1 Class -:(ROS_INFO):-, %s", __FUNCTION__);
}

} // namespace _Nodelet1


// 大文字のモノは大抵マクロ
PLUGINLIB_EXPORT_CLASS(_Nodelet1::Nodelet1, nodelet::Nodelet)
