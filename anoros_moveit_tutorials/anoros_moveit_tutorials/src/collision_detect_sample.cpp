// collision_detect_sample.cpp
// Shuhei-YOSHIDA 2016/11/11

#include<ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_scene/planning_scene.h> 

#include <moveit/kinematic_constraints/utils.h>
#include <eigen_conversions/eigen_msg.h>

// MoveIt! msg
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>
//planning_scene::PlanningScene のクラスはmsgの形式で引数を取るものが多い

// msg
#include <sensor_msgs/JointState.h>
#include <visualization_msgs/MarkerArray.h>

// Rviz URDF干渉を目視で確認
// TODO ロボット本体が環境に対して動くように設定すること

ros::Publisher markerArray_pub;
ros::Publisher planningScene_pub;
ros::Subscriber jointState_sub;

// joint_state callback
sensor_msgs::JointState joint_state;
void jointState_Callback(const sensor_msgs::JointState::ConstPtr& msg)
{
    joint_state = *msg;
}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "collision_detect_sample");
    ROS_INFO("sample_detect_sample start");
    ros::AsyncSpinner spinner(1);
    spinner.start(); //urdf読み込み〜planning_scene初期化で必要みたい

    ros::NodeHandle nh;
    markerArray_pub = nh.advertise<visualization_msgs::MarkerArray>("contacts_point", 1);
    planningScene_pub = nh. advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    jointState_sub = nh.subscribe("joint_states", 1, jointState_Callback);

    //urdfをパラメータから読み込み
    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
    planning_scene::PlanningScene planning_scene(kinematic_model);

    //collision detect 要求と結果の変数
    collision_detection::CollisionRequest collision_request;
    collision_detection::CollisionResult collision_result;

    // Self Collision
    planning_scene.checkSelfCollision(collision_request, collision_result);
    ROS_INFO_STREAM("Test 1: Current state is "
            << (collision_result.collision ? "in" : "not in")
            << " self collision"); 

    // ロボットの状態を変更 robot_state::RobotState は moveit::core::RobotStateに入れ替わってる？
    //robot_state::RobotState& current_state = planning_scene.getCurrentStateNonConst(); //これでも通るけど内容的には下．
    moveit::core::RobotState& current_state = planning_scene.getCurrentStateNonConst();
    //↑ ロボットの状態を返す．planning_sceneにつながっていて変更掛けられる
    // RobotStateに関節の状態を突っ込む関数はオーバーロードされてて色々使える
    /*
     * 以下，groovyまで？もう使えない 
     * bool setStateValues(const std::vector<double> &joint_state_values) //最速
     * void setStateValues(const sensor_msgs::JointState &msg) //msg受けられて楽
     * etc.
     * Rootの座標系について．きちんとTFのframe_idで管理されるべき
     * const Eigen::Affine3d & getRootTransform() const //linkツリー全体に適用される変換
     * void setRootTransform(const Eigen::Affine3d &transform) //TFで構造見て親Frameをチェックすべき
     * 少なくともindigo以上は… Positionは何を示す？ 多分オブジェクトロボットとか障害物とかの位置？
     * void setVariablePositions(const double *position)
     * void setVariablePositions(const std::map<std::string, double> &variable_map) //変なの入れたら例外
     * etc. 他にも速度とか加速度を設定できるみたい
     * void setJointPositions(...) とVariablePositionsってどう違うのか？
     * RootLinkの位置設定は？//world_jointと/odom_conbinedがデフォルト値？
     */

    // CollisionRequestの設定 costってなんだ
    /*
     * bool contacts //trueのとき，接触計算
     * bool cost //collision costが計算される
     * bool distance //proximity distanceが計算される
     * std::string group_name //干渉計算されるgroupの計算．無いときロボット全体
     * auto is_done
     * std::size_t max_contacts //計算される最大干渉点数
     * std::size_t max_contacts_per_pair // ボディ組に対する最大干渉点数
     * std::size_t max_cost_sources //コストが計算されるとき，返されるべきtop cost sourceがどれだけかを意味
     * double min_cost_density //コスト計算されるとき，結果に含まれるCostSourceに対する最小コスト密度を示す．
     */
    collision_request.contacts = true;
    collision_request.max_contacts = 1000;
    
    // CollisionResultの設定 
    /*
     * typedef std::map<std::pair<std::string, std::string>, std::vector<Contact>> ContactMap 
     * //Contactは接触する二つの物体の名前や型，干渉深さや接触点とその法線ベクトル
     * void clear()
     * bool collision //干渉見つかったかどうか
     * std::size_t contact_count //帰ってきた接触の数
     * ContactMap contacts //実際の干渉の情報．
     * std::set< CostSource > //各コスト源 コストって干渉する物体の存在確率？
     * double distance //ボディ最小接近距離
     * 
     * ContactMap は std::map<std::pair<std::string, std::string>, std::vector<Contact>>
     * iteratorでアクセス．最初のmapのfirstが衝突している二つの物体名のpair, secondが接触情報Contactクラス
     * Contactがvectorになっているのは，二つの物体の間に衝突点が複数あるから？
     */

    // 何か干渉物体を置く rvizによる表示用にplanning_sceneトピックを吐く必要？
    moveit_msgs::CollisionObject colObj; // 座標系に固定されるのかされないのか？
    colObj.header.frame_id = "base_footprint";
    colObj.header.stamp = ros::Time::now();
    colObj.id = "colObj1";
    shape_msgs::SolidPrimitive primitive;
    primitive.type = shape_msgs::SolidPrimitive::BOX; // meshどうやるんだ
    primitive.dimensions.resize(3);
    primitive.dimensions[0] = 1.0;
    primitive.dimensions[1] = 0.04; 
    primitive.dimensions[2] = 0.04; 
    colObj.primitives.push_back(primitive);
    geometry_msgs::Pose priPose;
    priPose.position.x = 0.5;priPose.position.y = 0.5;priPose.position.z = 0.5;
    priPose.orientation.w = 1;
    colObj.primitive_poses.push_back(priPose);
    colObj.operation = moveit_msgs::CollisionObject::ADD;
    planning_scene.processCollisionObjectMsg(colObj);
    //更新が必要？

    //getPlanningSceneMsgとかgetPlanningSceneDiffMsgでmoveit_msgs::PlanningSceneを確保できる．
    moveit_msgs::PlanningScene sceneMsg;
    planning_scene.getPlanningSceneMsg(sceneMsg);
    planningScene_pub.publish(sceneMsg);


    ros::Rate rate(0.5);
    ROS_INFO("collision checker");
    ROS_INFO_STREAM("Planning Frame: " << planning_scene.getPlanningFrame()); // /odom_conbinedが返ってくる
    unsigned int count = 0;
    //check
    ROS_INFO("printTransform");
    current_state.printTransforms();

    while (ros::ok()) {
        ROS_INFO("check");
        // わかりにくいので状態吐き出し
        // VariablePositions : 特に何も入っていない
        //const double * varPos = current_state.getVariablePositions();
        //ROS_INFO_STREAM("VariablePositions size:" << sizeof(varPos)/sizeof(varPos[0]));
        //for (int i = 0; i < sizeof(varPos)/sizeof(varPos[0]); i++) {
        //    ROS_INFO_STREAM("num:" << i << " position:" << varPos[i]);
        //}
        
        // joint_stateを受け取って干渉計算して，接触点をトピックで吐き出す

        // RootLinkの位置設定
        sensor_msgs::JointState tmp = joint_state;
        unsigned int tmpNum = 0;
        for (std::vector<std::string>::iterator itr = tmp.name.begin(); itr < tmp.name.end(); itr++) {
            current_state.setJointPositions(*itr, &(tmp.position[tmpNum]));
            tmpNum++;
        }
        
        //現在の状態publish
        planning_scene.getPlanningSceneMsg(sceneMsg);
        planningScene_pub.publish(sceneMsg);

        //干渉計算
        collision_result.clear();
        //updateCollisionBodyTransforms()?
        //planning_scene.checkSelfCollision(collision_request, collision_result);//自己干渉
        planning_scene.checkCollision(collision_request, collision_result); //世界干渉+自己干渉
        ROS_INFO_STREAM("Test " << count << ": Current state is "
                << (collision_result.collision ? "in" : "not in")
                << " self collision");

        //接触点位置にmarkerを送る //送っているものがinf かnanあるかも
        visualization_msgs::MarkerArray mad; visualization_msgs::Marker md;
        md.action = 3; mad.markers.push_back(md); markerArray_pub.publish(mad);//全消し
        
        if (collision_result.collision) {
            visualization_msgs::MarkerArray ma;
            collision_detection::CollisionResult::ContactMap::const_iterator it;
            unsigned int mcount = 0;
            for(it = collision_result.contacts.begin();it != collision_result.contacts.end();++it)
            {
                ROS_INFO("Contact between: %s and %s",
                        it->first.first.c_str(),
                        it->first.second.c_str());

                std::vector<collision_detection::Contact>::const_iterator it2;
                for (it2 = it->second.begin(); it2 != it->second.end(); ++it2)
                {
                    visualization_msgs::Marker m;
                    m.header.frame_id = "base_footprint"; 
                    m.header.stamp = ros::Time::now();
                    m.action = visualization_msgs::Marker::ADD;
                    m.id = mcount; mcount++;
                    m.type = visualization_msgs::Marker::SPHERE;
                    m.scale.x = 0.02; m.scale.y = 0.02; m.scale.z = 0.02; 
                    m.color.r = 1; m.color.a = 1;

                    //it2->pos; //Eigen::Vector3d 座標系は？ urdfのRoot?
                    m.pose.position.x = it2->pos(0);
                    m.pose.position.y = it2->pos(1);
                    m.pose.position.z = it2->pos(2);
                    //m.pose.orientation.w = 1;
                    ma.markers.push_back(m);

                    //normal vector
                    m.type = visualization_msgs::Marker::ARROW;
                    m.id = mcount; mcount++;
                    //開始点と方向を設定 下の場合とベクトルの向きが逆になる．クォータニオン逆？
                    //Eigen::Quaterniond q = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitX(), 
                    //        Eigen::Vector3d(it2->normal(0),it2->normal(1),it2->normal(2)));
                    //m.pose.orientation.w = q.w(); m.pose.orientation.x = q.x(); m.pose.orientation.y = q.y(); m.pose.orientation.z = q.z();
                    //m.scale.x = 0.1; m.scale.y = 0.001; m.scale.z = 0.001; 
                    //開始点と終端点で設定
                    m.points.push_back(m.pose.position);
                    m.pose.position.x += (it2->normal(0))*0.05;//矢印の長さを0.05[m]
                    m.pose.position.y += (it2->normal(1))*0.05;
                    m.pose.position.z += (it2->normal(2))*0.05;
                    m.points.push_back(m.pose.position);
                    m.pose = geometry_msgs::Pose();
                    m.scale.x = 0.01; m.scale.y = 0.02; m.scale.z = 0.04; 

                    m.color.r = 0; m.color.b = 1;
                    ma.markers.push_back(m);
                }


            }
            markerArray_pub.publish(ma);
        }
        //ros::spinOnce();
        rate.sleep();
        count++;
    }


}
