// Nodelet2.h
// Shuhei-YOSHIDA 2016/10/17
#pragma once
#include <nodelet/nodelet.h>
#include <iostream>
#include <std_msgs/String.h>
#include <ros/ros.h>
// NodeからNodeletへ
// * 必要なインクルードを加える
// * main関数はない
// * onInitで初期設定
// * whileループによるコードは，タイマのサブスクライブなどで記述
// * ヘッダ.hでの定義と .cppでの実装で分ける
// and so on

// マルチスレッドを試してみる．
#include<boost/thread.hpp>
#include<boost/shared_ptr.hpp>

namespace nodelet_tutorials {
    
    class Nodelet2 : public nodelet::Nodelet{
        private:
            ros::Publisher pub;
            ros::Subscriber sub;
            void inmsgCB(const std_msgs::String::ConstPtr& input);
            // Nodeletで必要なクラスメソッド．
            virtual void onInit();
            void spinThread();
            ros::Rate* loopPtr;
            ros::NodeHandle spinThread_nh;
            ros::NodeHandle nh;
            boost::shared_ptr<boost::thread> threadPtr;
        public:
            Nodelet2();  // コンストラクタ
            ~Nodelet2(); // デストラクタ
    };
} // namespace nodelet_tutorials 
//
