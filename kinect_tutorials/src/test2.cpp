//------------------------------------------------------------
// test2.cpp since 20151021 Shuhei YOSHIDA
// This is a Example for "anoros". 
// This code is based on Tutorial for OpenCV and PCL
// http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages
// http://wiki.ros.org/pcl/Tutorials/hydro?action=AttachFile&do=view&target=example_planarsegmentation.cpp
//------------------------------------------------------------

// +++++++++++++++++++++++++++++++++
// memo
// OpenCVとかPCLの使用例を含んだ例です．
// OpenCVではcv::Matという型で画像を扱います．
// なので，ROSのsensor_msgs/Imageとcv::Matは相互変換される必要があります．
// その中で役立つのが，cv_bridgeというパッケージです．
// また，参考ページの中では，sensor_msgs/Imageをpublish, subscribeするとき，
// image_transport パッケージを使って効率的にやっているみたいです．
// 今回はスルーします．他のnodeletといった代替手段がありますし，
// そんなに説明が必要なほど導入が難しいものではなさそうなので．
// 
// PCLでは．pcl::PointCloud<pcl::PointXYZ>やpcl::PCLPointCloud2に
// sensor_msgs/PointCloud2 msgを変換する必要があります
// 複数，やり方が存在するようなので，ROSのチュートリアル等を確認してください．

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

// OpenCVのインクルード /usr/include/opencv2/coreを指しているはず．
#include "opencv2/opencv.hpp" // 無駄なインクルードも入るかもしれないけど，必要なものそろってるはず．
// OpenCVのCV::MatとROSのsensor_msgs/Imageの相互変換
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h" // 画像エンコードを意味する文字列？が定義

// Point Cloud Library(PCL)のインクルード
// http://wiki.ros.org/pcl/Tutorials を参考に．
// http://wiki.ros.org/ja/pcl/Tutorials も参考に．微妙に違う．
// sensor_msgs/PointCloud2 と pcl::PCLPointCloud2に互換ありっぽい 
#include "pcl_conversions/pcl_conversions.h" // pcl/conversion.hを含む
// #include "pcl/ros/conversions.h" // deprecatedみたいです．
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
// PCL の機能群
#include "pcl/segmentation/sac_segmentation.h" // pclでRANSAC使うとき必要っぽい

// LRFのデータ処理表示用にrvizでマーカ情報をtopicに送る．
#include "visualization_msgs/Marker.h"

// msgファイルのインクルード .msgファイルからc++用にcatkin_makeで自動生成される．
#include "std_msgs/String.h" // 基本のメッセージ．文字列型．
#include "sensor_msgs/LaserScan.h" // LRFデータ用 他の型のインクルードも芋づる式にされるっぽい．
#include "sensor_msgs/Image.h" // RGB,Depth画像用
#include "sensor_msgs/PointCloud2.h" // 点群用
// c++標準のインクルード
#include <sstream> // c++でよく用いられるstringstream. 数字を文字列に変換できて便利．

// 今回main関数外で使うpublish,subscribeのための変数．寿命に注意．
ros::Publisher image_pub, scan_pub, points_pub; 
ros::Subscriber image_sub, scan_sub, points_sub;

// +++++++++++++++++++++++++++++++++
// subscribeのためのコールバックと設定

// トピック /camera/rgb/image_rect_colorに対するコールバック
void image_Callback(const sensor_msgs::Image::ConstPtr& msg) 
{
	cv_bridge::CvImagePtr cv_ptr; // 相互変換クラスCvImage型へのポインタ
	try { // 例外処理
		// sensor_msgs/ImageをcvImage型に変換
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch(cv_bridge::Exception& e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// OpenCVを使って，画像処理をする．
	// 参考 http://opencv.jp/opencv-2svn/cpp/feature_detection.html
	cv::Mat dst, color_dst;
	cv::Canny(cv_ptr->image, dst, 50, 200, 3); // Canny変換
	cv::cvtColor(dst, color_dst, CV_GRAY2BGR); // 色入れられるように変数のサイズ設定

	std::vector<cv::Vec4i> lines; // 始点 終点での線分ベクトル
	cv::HoughLinesP(dst, lines, 1, CV_PI/180, 50, 50, 10); // 確率的ハフ変換
	for( size_t i = 0; i < lines.size(); i++ ) // linesに発見された線が格納されている．
    {
    	//  color_dstに見つけた線を引く．
        cv::line( color_dst, cv::Point(lines[i][0], lines[i][1]),
            cv::Point(lines[i][2], lines[i][3]), cv::Scalar(0,0,255), 3, 8 );
    }

    // cvImageクラスのimage メンバがcv::Mat．面倒なのでそこに代入．
    cv_ptr->image = color_dst;    

    // OpenCVの機能で画像表示
    // cv::imshow("image window", cv_ptr->image);
    // cv::waitKey(3);

    // cv_ptrからsensor_msgs/Image を生成して，publish 
    cv_ptr->header.stamp = ros::Time::now(); // header のタイムスタンプを現在に変更
    image_pub.publish(cv_ptr->toImageMsg()); 

}


// トピック /scanに対するコールバック
void scan_Callback(const sensor_msgs::LaserScan::ConstPtr& msg) 
{
	// sensor_msgs/LaserScan 型のLRFの点のデータは
	// 極座標系で表現されています．xyz系に変換して重心を求めてみる．

	float ang_min = msg->angle_min; // データのスタート角度
	float ang_inc = msg->angle_increment; // データの角度差
	float cod[3] = {0,0,0}; // データの重心(中心)

	// ROS msg でfloat32[] のような可変配列はc++だとstd::vectorにあたる．
	// イテレータを用いて，実際のデータにアクセスすることがある．
	// 特に全部のvector の全部のデータにfor文とかwhile文でアクセスするときに多用されるっぽい
	// sensor_msgs/LaserScan 型の距離データはrangesという要素にfloat32[]として入ってる
	std::vector<float>::const_iterator itr;// = msg->ranges.begin(); // 最初の要素を指すイテレータ．
	// ↑subscribeの場合はconst_iterator．ただのmsg を使うときは普通iterator でOK
	int count=0;
	for (itr = msg->ranges.begin(); itr != msg->ranges.end(); ++itr)
	{
		float x,y,z;
		// ある角度の距離値が計測不能の時，nanになるのでチェックすること．
		if ((msg->range_min < *itr) && (*itr < msg->range_max) ) {
			x = (*itr)*cos(ang_min + ang_inc * count); // *itr がrange[count]に相当する．
			y = (*itr)*sin(ang_min + ang_inc * count);
			z = 0; // LRFデータはxy座標平面上にあるので．

			// 逐次的に重心計算
			cod[0] = ((float)count/((float)count+1))*cod[0] + 1/((float)count+1)*x; // x方向
			cod[1] = ((float)count/((float)count+1))*cod[1] + 1/((float)count+1)*y; // y方向
			cod[2] = 0; // z方向の重心，常にゼロでいいや
		}
		count++;
	}

	// 重心の座標をマーカー情報に載せてpublishする．rvizで確認
	visualization_msgs::Marker output;
	output.header = msg->header; // 座標系の名前を共有したいので．
	output.type = visualization_msgs::Marker::SPHERE; // マーカの形状は球体
	output.action = visualization_msgs::Marker::ADD; // マーカを追加する．（実は修正と同じ）
	output.scale.x = 0.10; output.scale.y = 0.10; output.scale.z = 0.10; 
	output.color.g = 1.0; output.color.a = 0.8;
	output.pose.position.x = cod[0]; // マーカのx座標
	output.pose.position.y = cod[1]; // y座標
	output.pose.position.z = cod[2]; // z座標

	ROS_INFO("x:%f, y:%f, z:%f", cod[0], cod[1], cod[2]);

	//トピックをpublish
	scan_pub.publish(output);
}


// トピック /camera/depth_registered_pointsに対するコールバック
void points_Callback(const sensor_msgs::PointCloud2::ConstPtr& msg) 
{
	// Convert the sensor_msgs/PointCloud2 data to pcl/PointCloud
	pcl::PointCloud<pcl::PointXYZRGB> cloud;
	pcl::fromROSMsg (*msg, cloud);

	// 平面を点群から見つける
	pcl::ModelCoefficients coefficients;
	pcl::PointIndices inliers;
	// Create the segmentation object
	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
	// Optional
	seg.setOptimizeCoefficients (true);
	// Mandatory
	seg.setModelType (pcl::SACMODEL_PLANE);
	seg.setMethodType (pcl::SAC_RANSAC);
	seg.setDistanceThreshold (0.01);

	seg.setInputCloud (cloud.makeShared ());
	// 平面探索RANSAC inliersに平面上にある点のindex，coefficientsに平面方程式の係数？
	seg.segment (inliers, coefficients);

	if (inliers.indices.size () == 0)  
    {  
    	ROS_INFO ("Could not estimate a planar model for the given dataset.");  
    	return;  
    }  

    // 平面上の点を赤く染める
    for (size_t i = 0; i < inliers.indices.size (); ++i) {    
    	cloud.points[inliers.indices[i]].r = 0;  
    	cloud.points[inliers.indices[i]].g = 0;  
    	cloud.points[inliers.indices[i]].b = 255;  
	}  

	// 染まった点群をPublishする
	sensor_msgs::PointCloud2 output; 
	pcl::toROSMsg (cloud, output); // ROS msgに変換
	output.header.stamp = ros::Time::now();
	points_pub.publish (output); // publish
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
	image_pub = n.advertise<sensor_msgs::Image>("image_check", 10);
	// トピック /scan_check をpublish
	scan_pub = n.advertise<visualization_msgs::Marker>("scan_check", 10);
	// トピック /points_check をpublish
	points_pub = n.advertise<sensor_msgs::PointCloud2>("points_check", 10);

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
	ros::init(argc, argv, "test2"); //NodeHandle より早く作ること．

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
