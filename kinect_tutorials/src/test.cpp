//------------------------------------------------------------
// test.cpp since 20151020 Shuhei YOSHIDA
// This is a Example for "anoros". 
// This code is based on talker.cpp and listener.cpp on the page: 
// http://wiki.ros.org/ja/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29
//------------------------------------------------------------

// +++++++++++++++++++++++++++++++++
// memo
// 書き方としては，個人的にはc++っぽくクラスとか使いたいのですけど，
// B4あたりを対象とするためと実践編初回なので，できるだけcらしく書きます．
// そのため元ネタのtalkerなどに比べ，グローバル変数が増えました．
// 使われている関数はrosのwikiのroscppのAPI集に説明がだいたいあります．
// オーバーロード(c++用語)されているのでわかりづらいかもしれませんが，
// クラスから呼び出すときなどに同名の関数の別の形を使うことがあります．

// +++++++++++++++++++++++++++++++++
// published topics
// /image_check 
// /scan_check
// /points_check
// /chatter

// subscribed topics
// /camera/rgb/image_rect_color @ RGB画像
// /scan @ LRFのデータ
// /camera/depth_registered/points @ ポイントクラウド

// +++++++++++++++++++++++++++++++++
// 基本のインクルード　
// インストールしたROS関係は普通/opt/ros/indigo/include/~以下にヘッダが存在する．
#include "ros/ros.h" 
// msgファイルのインクルード .msgファイルからc++用にcatkin_makeで自動生成される．
#include "std_msgs/String.h" // 基本のメッセージ．文字列型．
#include "sensor_msgs/LaserScan.h" // LRFデータ用 他の型のインクルードも芋づる式にされるっぽい．
#include "sensor_msgs/Image.h" // RGB,Depth画像用
#include "sensor_msgs/PointCloud2.h" // 点群用
// std_msgs, sensor_msgsというパッケージがある．
// c++標準のインクルード
#include <sstream> // c++でよく用いられるstringstream. 数字を文字列に変換できて便利．

// 今回main関数外で使うpublish,subscribeのための変数．寿命に注意．
// ros::Publisherなどの変数は，使っている間はスコープ外にならないようにすること．
ros::Publisher image_pub, scan_pub, points_pub; 
ros::Subscriber image_sub, scan_sub, points_sub;

// +++++++++++++++++++++++++++++++++
// subscribeのためのコールバックと設定
// topic がpublishされた時，割り込みの用にこれらのCallback関数が呼び出される
// 引数の型は パッケージ名::msg名::ConstPtr 自動的に生成済み．
// subscribeされたトピックの内容は引数の中に入っていると思え．
// またCallback関数はsubscribeするトピック名と共に登録される必要がある

// トピック /camera/rgb/image_rect_colorに対するコールバック
void image_Callback(const sensor_msgs::Image::ConstPtr& msg) 
{
	// std::string型
	std::string frame_info;
	// subscribeしたトピックから情報を受け取る．
	frame_info = msg->header.frame_id; // sensor_msgs/Imageの中のheaderの中のframe_idというデータ

	// トピック /image_checkをpublishする
	std_msgs::String pub_msg; // std_msgs/String型
	pub_msg.data = frame_info; // publishするデータを収納

	image_pub.publish(pub_msg); // トピックをpublish
}


// トピック /scanに対するコールバック
void scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
	// std::stringstream型
	std::stringstream timestamp_info;
	// subscribeしたトピックから情報を受け取る．
	timestamp_info << "Timestamp is " <<
					  msg->header.stamp.sec << " s:" <<
					  msg->header.stamp.nsec << " ns"; //headerの中のtimeというデータ
	//ros::Time として定義されている，msgの基本の型の一つ

	// トピック /scan_checkをpublishする
	std_msgs::String pub_msg; // std_msgs/String型
	pub_msg.data = timestamp_info.str(); // publishするデータを収納 .str()はstringstreamをstringに変換

	scan_pub.publish(pub_msg); // トピックをpublish
}


// トピック /camera/depth_registered_pointsに対するコールバック
void points_Callback(const sensor_msgs::PointCloud2::ConstPtr& msg) 
{
	// std::stringstream型
	std::stringstream hw_info; //heightとwidthに関する情報 点群データの2次元構造
	// subscribeしたトピックから情報を受け取る．
	hw_info << "height: " << msg->height << 
					 " width: " << msg->width ; // sensor_msgs/PointCloud2 の中のheaderとwidth

	// トピック /points_checkをpublishする
	std_msgs::String pub_msg; // std_msgs/String型
	pub_msg.data = hw_info.str(); // publishするデータを収納 .str()はstringstreamをstringに変換

	points_pub.publish(pub_msg); // トピックをpublish
}

// 初期設定用関数
void Setting(ros::NodeHandle& n)
{
	// +++++++++++++++++++++++++++++++++
	// publishのための設定
	// publishするトピックのmsg型とトピック名を宣言する．
	// 2番めのパラメータの数字は，トピックのメッセージが可能な速度より早くpublishされるとき，
	// 順番待ちにできる数．その数より増えたら古いものから捨てられるはず．
	// トピック /image_check をpublish 
	image_pub = n.advertise<std_msgs::String>("image_check", 10);
	// トピック /scan_check をpublish
	scan_pub = n.advertise<std_msgs::String>("scan_check", 10);
	// トピック /points_check をpublish
	points_pub = n.advertise<std_msgs::String>("points_check", 10);

	// ++++++++++++++++++++++++++++++++
	// subscribeのための設定
	// 登録
	image_sub = n.subscribe("/camera/rgb/image_rect_color", 10, image_Callback);
	// 登録
	scan_sub = n.subscribe("/scan", 10, scan_Callback);
	// 登録
	points_sub = n.subscribe("/camera/depth_registered/points", 10, points_Callback);
}

// +++++++++++++++++++++++++++++++++
// main部分 
// 個人的にはあまり直接処理をここで書かないほうがいいと思う．
// のちのち，nodeletにする必要が生じたりするかも．クラスとかうまく使うべき．
int main(int argc, char **argv) //型と引数はこの形にしとく
{
	// ノードを立ち上げます．ノードの名前を設定してください
	ros::init(argc, argv, "test"); //NodeHandle より早く作ること．

	// node handleの宣言．
	// このノードでrosの機能を使うためのキーだと思ってください．
	// これを使って，publish や subscribeのための関数を呼び出します．
	ros::NodeHandle n;

	// 設定呼び出し
	Setting(n);

	// トピック /chatterをpublishする設定
	ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

	// ノードの処理周期の設定
	ros::Rate loop_rate(20); // 20Hzに設定．
	// この例だと，この周期ごとにsubscribe の処理がされる．

	int count=0; 
	while(ros::ok()) //ctrl-Cが押されるかノードが殺されるまでループする
	{
		// ループ回数をpublishしてみる
		std_msgs::String msg; // std_msgs/String 型のメッセージをc++で扱う．

		std::stringstream ss; // ROSで情報を表示するためによく使うと思われ．
		ss<<"loop num is... " << count; // 文字列と数字を同時にまとめる．
		msg.data = ss.str(); // stringstream から string型へと変換．std_msgs/String のdataに代入．

		chatter_pub.publish(msg); // メッセージをtopicにpublishする．送ります．

		// 情報を端末に表示．(実はlogにも流れている)
		ROS_INFO("%s", msg.data.c_str()); // 基本使い方はprintf()と同じ．stringをchar* へと変換している．

		// subscribe と 処理周期待ち
		ros::spinOnce(); //subscribeのため．
		loop_rate.sleep(); //余った処理時間分休む．

		++count;
	}


	// 昔から，intで始まるmain の正常終了を示すためにゼロを返すと決まっておる．
	return 0; 
}
