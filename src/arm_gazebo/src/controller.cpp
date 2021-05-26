#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "ros/ros.h"
#include "arm_gazebo/ModelJoints.h"

namespace gazebo
{
  class Controller1 : public ModelPlugin
  {
    public: Controller1() : ModelPlugin()
    {
        printf("Hello World!\n");
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        this->model = _parent;

        this->jointController = this->model->GetJointController();

        this->pidJoint1 = common::PID(10, 5.01, 5.03);

        this->pidJoint2 = common::PID(15, 10.1, 10);

        this->pidJoint3 = common::PID(15, 10.1, 10);

        this->pidJoint4 = common::PID(15, 10.1, 10);

        auto joint_name1 = "base_arm1";
        auto joint_name2 = "arm1_arm2";
        auto joint_name3 = "arm2_arm3";
        auto joint_name4 = "arm3_arm4";

        this->joint1 = this->model->GetJoint(joint_name1);
        this->joint2 = this->model->GetJoint(joint_name2);
        this->joint3 = this->model->GetJoint(joint_name3);
        this->joint4 = this->model->GetJoint(joint_name4);

        this->jointPublisher = this->nodeHandle.advertise<arm_gazebo::ModelJoints>("modelJoints", 10);

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&Controller1::OnUpdate, this)
        );
    }
    //called by the world update start event
    public : void OnUpdate()
    {
        arm_gazebo::ModelJoints msg;

        msg.jointOneAngle = this->joint1->Position();
        msg.jointTwoAngle = this->joint2->Position();
        msg.jointThreeAngle = this->joint3->Position();
        msg.jointFourAngle = this->joint4->Position();

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

    private: physics::JointPtr joint1;
    private: physics::JointPtr joint2;
    private: physics::JointPtr joint3;
    private: physics::JointPtr joint4;

    private: ros::NodeHandle nodeHandle;
    private: ros::Publisher jointPublisher;
  };
  GZ_REGISTER_MODEL_PLUGIN(Controller1)
}