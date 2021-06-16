#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "ros/ros.h"
#include "arm_gazebo/ModelJoints.h"
#include "arm_gazebo/pos.h"
#include "arm_gazebo/ik.h"

namespace gazebo
{
  class Controller1 : public ModelPlugin
  {
    public: Controller1() : ModelPlugin()
    {
        printf("Hello World!\n");
    }

    private: void updateJointAnglesCallback(const arm_gazebo::ModelJoints& msg)
    {
        ROS_INFO("Received angle");

        std::string name1 = this->joint1->GetScopedName();

        this->jointController->SetPositionPID(name1, this->pidJoint1);
        this->jointController->SetPositionTarget(name1, getRad(msg.jointOneAngle));

        std::string name2 = this->joint2->GetScopedName();

        this->jointController->SetPositionPID(name2, this->pidJoint2);
        this->jointController->SetPositionTarget(name2, getRad(msg.jointTwoAngle));

        std::string name3 = this->joint3->GetScopedName();

        this->jointController->SetPositionPID(name3, this->pidJoint3);
        this->jointController->SetPositionTarget(name3, getRad(msg.jointThreeAngle));

        std::string name4 = this->joint4->GetScopedName();

        this->jointController->SetPositionPID(name4, this->pidJoint4);
        this->jointController->SetPositionTarget(name4, getRad(msg.jointFourAngle));
    }

    private: static float getRad(float angleDegree) {
        float rad = M_PI * angleDegree/180 ;
        return rad;
    }

    private: static float getDegree(float angleRad) {
        float degree = (angleRad * 180)/M_PI ;
        return degree;
    }

    private: void updateEndEffectorPosition(const arm_gazebo::pos& msg)
    {
        ROS_INFO("Received pos");
        testIK(msg.x, msg.y, msg.z);
    }

    public: int testIK(const float x, const float y, const float z){
        ros::ServiceClient client = this->nodeHandle.serviceClient<arm_gazebo::ik>("ik");

        arm_gazebo::ik srv;

        srv.request.link1_length = 0.1;
        srv.request.link2_length = 0.05;
        srv.request.link3_length = 2;
        srv.request.link4_length = 1;
        srv.request.link5_length = 0.5;
        srv.request.link6_length = 0.04;
        srv.request.link7_length = 0.1;

        srv.request.x = x;
        srv.request.y = y;
        srv.request.z = z;

        if (client.call(srv))
        {
            updateModel(
                srv.response.joint1_angle, srv.response.joint2_angle, srv.response.joint3_angle,
                srv.response.joint4_angle, srv.response.joint5_angle, srv.response.joint6_angle);
        }
        else
        {
            ROS_ERROR("Failed to call service add_two_ints");
        return 1;
        }
        return 0;
    }

    private: void updateModel(
            const float joint1_angle, const float joint2_angle,
            const float joint3_angle, const float joint4_angle, 
            const float joint5_angle, const float joint6_angle) {
        
        std::string name1 = this->joint1->GetScopedName();

        this->jointController->SetPositionPID(name1, pidJoint1);
        this->jointController->SetPositionTarget(name1, joint1_angle);

        std::string name2 = this->joint2->GetScopedName();

        this->jointController->SetPositionPID(name2, pidJoint2);
        this->jointController->SetPositionTarget(name2, joint2_angle);

        std::string name3 = this->joint3->GetScopedName();

        this->jointController->SetPositionPID(name3, pidJoint3);
        this->jointController->SetPositionTarget(name3, joint3_angle);

        std::string name4 = this->joint4->GetScopedName();

        this->jointController->SetPositionPID(name4, pidJoint4);
        this->jointController->SetPositionTarget(name4, joint4_angle);

        std::string name5 = this->joint_arm4_arm5->GetScopedName();

        this->jointController->SetPositionPID(name5, pidJoint5);
        this->jointController->SetPositionTarget(name5, -0.44);//joint5_angle);

        std::string name6 = this->joint_arm5_palm->GetScopedName();

        this->jointController->SetPositionPID(name6, pidJoint6);
        this->jointController->SetPositionTarget(name6, joint6_angle);
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        this->model = _parent;

        this->jointController = this->model->GetJointController();

        this->pidJoint1 = common::PID(31.6, 10.02, 15.45);

        this->pidJoint2 = common::PID(15, 5.1, 10);

        this->pidJoint3 = common::PID(15, 5.1, 10);

        this->pidJoint4 = common::PID(15, 5.1, 10);

        this->pidJoint5 = common::PID(15, 5.1, 10);

        this->pidJoint6 = common::PID(15, 5.1, 10);

        auto joint_name1 = "base_arm1";
        auto joint_name2 = "arm1_arm2";
        auto joint_name3 = "arm2_arm3";
        auto joint_name4 = "arm3_arm4";
        auto joint_name5 = "arm4_arm5";
        auto joint_name6 = "arm5_palm";

        this->joint1 = this->model->GetJoint(joint_name1);
        this->joint2 = this->model->GetJoint(joint_name2);
        this->joint3 = this->model->GetJoint(joint_name3);
        this->joint4 = this->model->GetJoint(joint_name4);
        this->joint_arm4_arm5 = this->model->GetJoint(joint_name5);
        this->joint_arm5_palm = this->model->GetJoint(joint_name6);

        updateModel(-0.4444, -0.37568, -1.04219, -0.4873, -0.44, 0.11432);

        this->jointPublisher = this->nodeHandle.advertise<arm_gazebo::ModelJoints>("modelJoints", 10);
        this->jointSubscriber = this->nodeHandle.subscribe("updateJointAngles", 1000, &Controller1::updateEndEffectorPosition, this);

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&Controller1::OnUpdate, this)
        );
    }
    //called by the world update start event
    public : void OnUpdate()
    {
        arm_gazebo::ModelJoints msg;

        msg.jointOneAngle = getDegree(this->joint1->Position());
        msg.jointTwoAngle = getDegree(this->joint2->Position());
        msg.jointThreeAngle = getDegree(this->joint3->Position());
        msg.jointFourAngle = getDegree(this->joint4->Position());

        this->jointPublisher.publish(msg);

        this->jointController->Update();
    }

    //Pointer to the model
    private: physics::ModelPtr model;

    //Pointer to the model
    private: physics::JointControllerPtr jointController;

    //Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;

    private: common::PID pidJoint1;
    private: common::PID pidJoint2;
    private: common::PID pidJoint3;
    private: common::PID pidJoint4;
    private: common::PID pidJoint5;
    private: common::PID pidJoint6;

    private: physics::JointPtr joint1;
    private: physics::JointPtr joint2;
    private: physics::JointPtr joint3;
    private: physics::JointPtr joint4;
    private: physics::JointPtr joint_arm4_arm5;
    private: physics::JointPtr joint_arm5_palm;

    private: ros::NodeHandle nodeHandle;
    private: ros::Publisher jointPublisher;
    private: ros::Subscriber jointSubscriber;
  };
  GZ_REGISTER_MODEL_PLUGIN(Controller1)
}