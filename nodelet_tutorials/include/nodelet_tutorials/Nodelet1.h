// Nodelet1.h
// Shuhei-YOSHIDA 2016/10/15
#pragma once
#include <nodelet/nodelet.h>

// NodeからNodeletへ
// * 必要なインクルードを加える
// * main関数はない
// * onInitで初期設定
// * whileループによるコードは，タイマのサブスクライブなどで記述
// * ヘッダ.hでの定義と .cppでの実装で分ける
// and so on

namespace _Nodelet1 {
    
    class Nodelet1 : public nodelet::Nodelet{
        
        public:
            Nodelet1();  // コンストラクタ
            ~Nodelet1(); // デストラクタ
            // Nodeletで必要なクラスメソッド．
            virtual void onInit();
    };
} // namespace _Nodelet1
