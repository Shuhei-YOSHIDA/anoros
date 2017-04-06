// collision_detect_sample_fcl.cpp
// Shuhei-YOSHIDA 2017/3/29
// planning_scene::PlanningScene class 
// ->getCollisionWorld() ->collision_detection::CollisionWorld ->FCLObject
// でプラグインのFCLにアクセスできるはず
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

#include <fcl/collision_object.h>
#include <fcl/shape/geometric_shapes.h>
#include <fcl/collision_data.h>
#include <fcl/distance.h>

using namespace fcl;
//using namespace std;
int main(int argc, char** argv)
{
    DistanceResult result;
    FCL_REAL distance;

    Matrix3f rotSphere(1.0, 0.0, 0.0,
                       0.0, 1.0, 0.0,
                       0.0, 0.0, 1.0);

    Vec3f trSphere(0.0, 0.0, 0.0);

    Matrix3f rotBox(1.0, 0.0, 0.0,
                    0.0, 1.0, 0.0,
                    0.0, 0.0, 1.0);

    Vec3f trBox(0.0, 5.0, 3.0);


    Sphere sphere(1);
    Box box(10, 2, 10); //Box-Sphere collision is strange?
    Capsule capsule(1, 10);//Capsule-Sphere may be ok

    boost::shared_ptr< CollisionGeometry > cgeomSphere_(&sphere);
    boost::shared_ptr< CollisionGeometry > cgeomBox_(&box);
    boost::shared_ptr< CollisionGeometry > cgeomCapsule_(&capsule);

    CollisionObject *objSphere = new CollisionObject(cgeomSphere_, rotSphere, trSphere);
    CollisionObject *objBox = new CollisionObject(cgeomBox_, rotBox, trBox);//strange
    CollisionObject *objCapsule = new CollisionObject(cgeomCapsule_, rotBox, trBox);//ok

    result.clear();
    DistanceRequest request(true);
    distance = fcl::distance(objSphere, objBox, request, result) ;

    std::cout << "distance = " << distance << std::endl;
    std::cout << " point on sphere: x = " << result.nearest_points[0].data[0] << " y = " << result.nearest_points[0].data[1] << " z = " << result.nearest_points[0].data[2] << std::endl;
    std::cout << " point on box: x = " << result.nearest_points[1].data[0] << " y = " << result.nearest_points[1].data[1] << " z = " << result.nearest_points[1].data[2] << std::endl;

    result.clear();
    DistanceRequest request2(true);
    distance = fcl::distance(objSphere, objCapsule, request2, result) ;

    std::cout << "distance = " << distance << std::endl;
    std::cout << " point on sphere: x = " << result.nearest_points[0].data[0] << " y = " << result.nearest_points[0].data[1] << " z = " << result.nearest_points[0].data[2] << std::endl;
    std::cout << " point on capsule: x = " << result.nearest_points[1].data[0] << " y = " << result.nearest_points[1].data[1] << " z = " << result.nearest_points[1].data[2] << std::endl;

    return 0;
}

