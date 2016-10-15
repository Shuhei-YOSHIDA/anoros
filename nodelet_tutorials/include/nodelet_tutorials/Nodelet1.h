// Nodelet1.h
// Shuhei-YOSHIDA 2016/10/15
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

namespace nodelet_tutorials {
    
    class Nodelet1 : public nodelet::Nodelet{
        private:
            ros::Publisher pub;
            ros::Subscriber sub;
            void inmsgCB(const std_msgs::String::ConstPtr& input);
            // Nodeletで必要なクラスメソッド．
            virtual void onInit();
        public:
            Nodelet1();  // コンストラクタ
            ~Nodelet1(); // デストラクタ
    };
} // namespace nodelet_tutorials 
