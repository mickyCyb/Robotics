#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include "ros/ros.h"
#include "arm_gazebo/UpdateGripper.h"
#include "arm_gazebo/fk.h"

namespace gazebo
{
  class GripperController : public ModelPlugin
  {
    public: GripperController() : ModelPlugin()
    {
        printf("Hello World!\n");
    }

    private: void updateGripper(const arm_gazebo::UpdateGripper& msg)
    {
        std::string joint_palm_left_finger_name = this->joint_palm_left_finger->GetScopedName();
        std::string joint_palm_right_finger_name = this->joint_palm_right_finger->GetScopedName();

        this->jointController->SetPositionPID(joint_palm_left_finger_name, this->pidJoint7);
        this->jointController->SetPositionPID(joint_palm_right_finger_name, this->pidJoint8);


        if(msg.release){
            this->jointController->SetPositionTarget(joint_palm_left_finger_name, 0);
            
            this->jointController->SetPositionTarget(joint_palm_right_finger_name, 0);
        } else{
            this->jointController->SetPositionTarget(joint_palm_left_finger_name, -0.21);
            
            this->jointController->SetPositionTarget(joint_palm_right_finger_name, 0.21);
        }
        testFK();
    }

    public: int testFK(){
        ros::ServiceClient client = this->nodeHandle.serviceClient<arm_gazebo::fk>("fk");

        arm_gazebo::fk srv;

        srv.request.link1_length = 0.1;
        srv.request.link2_length = 0.05;
        srv.request.link3_length = 2;
        srv.request.link4_length = 1;
        srv.request.link5_length = 0.5;
        srv.request.link6_length = 0.04;
        srv.request.link7_length = 0.1;

        srv.request.joint1_angle = this->joint1->Position();
        srv.request.joint2_angle = this->joint2->Position();
        srv.request.joint3_angle = this->joint3->Position();
        srv.request.joint4_angle = this->joint4->Position();
        srv.request.joint5_angle = this->joint_arm4_arm5->Position();
        srv.request.joint6_angle = this->joint_arm5_palm->Position();

        if (client.call(srv))
        {
            ROS_INFO("x: %ld", (long int)srv.response.x);
        }
        else
        {
            ROS_ERROR("Failed to call service add_two_ints");
        return 1;
        }
        return 0;
    }

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
    {
        this->model = _parent;

        this->jointController = this->model->GetJointController();

        auto joint_name1 = "base_arm1";
        auto joint_name2 = "arm1_arm2";
        auto joint_name3 = "arm2_arm3";
        auto joint_name4 = "arm3_arm4";
        auto joint_name5 = "arm4_arm5";
        auto joint_name6 = "arm5_palm";
        auto joint_name7 = "palm_left_finger";
        auto joint_name8 = "palm_right_finger";

        this->joint1 = this->model->GetJoint(joint_name1);
        this->joint2 = this->model->GetJoint(joint_name2);
        this->joint3 = this->model->GetJoint(joint_name3);
        this->joint4 = this->model->GetJoint(joint_name4);
        this->joint_arm4_arm5 = this->model->GetJoint(joint_name5);
        this->joint_arm5_palm = this->model->GetJoint(joint_name6);
        this->joint_palm_left_finger = this->model->GetJoint(joint_name7);
        this->joint_palm_right_finger = this->model->GetJoint(joint_name8);

        this->pidJoint1 = common::PID(31.6, 10.02, 15.45);

        this->pidJoint2 = common::PID(15, 5.1, 10);

        this->pidJoint3 = common::PID(15, 5.1, 10);

        this->pidJoint4 = common::PID(15, 5.1, 10);

        this->pidJoint5 = common::PID(15, 5.1, 10);

        this->pidJoint6 = common::PID(15, 5.1, 10);

        this->pidJoint7 = common::PID(5, 2.1, 3);

        this->pidJoint8 = common::PID(5, 2.1, 3);

        this->jointSubscriber = this->nodeHandle.subscribe("updateGripper", 1000, &GripperController::updateGripper, this);

        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
            std::bind(&GripperController::OnUpdate, this)
        );
    }
    public : void OnUpdate()
    {
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
    private: common::PID pidJoint7;
    private: common::PID pidJoint8;

    private: physics::JointPtr joint1;
    private: physics::JointPtr joint2;
    private: physics::JointPtr joint3;
    private: physics::JointPtr joint4;
    private: physics::JointPtr joint_arm4_arm5;
    private: physics::JointPtr joint_arm5_palm;
    private: physics::JointPtr joint_palm_left_finger;
    private: physics::JointPtr joint_palm_right_finger;

    private: ros::NodeHandle nodeHandle;
    // private: ros::Publisher jointPublisher;
    private: ros::Subscriber jointSubscriber;
  };
  GZ_REGISTER_MODEL_PLUGIN(GripperController)
}