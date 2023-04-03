#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"

//#include <unistd.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <algorithm>
using namespace std;

#include <chrono>
using namespace std::chrono;

#include <time.h>

#include "mini_pupper.h"
using namespace mini_pupper;


class motion_node : public rclcpp::Node
{
public:
	motion_node() : Node("motion_node")
	{
		RCLCPP_INFO(this->get_logger(),"Starting...");

		// free joints
		for(size_t index=0; index<12; ++index)
		{
			control.goal_position[index]= 512;
			control.torque_enable[index] = 0;
		}
		int result = servo.update(control,feedback);
		//std::cout << "result:" << result << std::endl;		




		/*
		this->declare_parameter("acc_x",_acceleration_x);
		this->declare_parameter("acc_z",_acceleration_z);

		this->declare_parameter("pwm_max",_pwm_max);

		this->declare_parameter("x_kp",_x_kp);
		this->declare_parameter("x_ki",_x_ki);
        this->declare_parameter("x_kd",_x_kd);
        this->declare_parameter("x_kff",_x_kff);
        this->declare_parameter("x_d_alpha",_x_d_alpha);
        this->declare_parameter("x_o_alpha",_x_o_alpha);

        this->declare_parameter("w_kp",_w_kp);
        this->declare_parameter("w_ki",_w_ki);
        this->declare_parameter("w_kd",_w_kd);
        this->declare_parameter("w_kff",_w_kff);
        this->declare_parameter("w_d_alpha",_w_d_alpha);
        this->declare_parameter("w_o_alpha",_w_o_alpha);

        _acceleration_x = this->get_parameter("acc_x").as_double();
        _acceleration_z = this->get_parameter("acc_z").as_double();

        _pwm_max = this->get_parameter("pwm_max").as_int();

        _x_kp = this->get_parameter("x_kp").as_double();
        _x_ki = this->get_parameter("x_ki").as_double();
        _x_kd = this->get_parameter("x_kd").as_double();
        _x_kff = this->get_parameter("x_kff").as_double();
        _x_d_alpha = this->get_parameter("x_d_alpha").as_double();
        _x_o_alpha = this->get_parameter("x_o_alpha").as_double();

        _w_kp = this->get_parameter("w_kp").as_double();
        _w_ki = this->get_parameter("w_ki").as_double();
        _w_kd = this->get_parameter("w_kd").as_double();
        _w_kff = this->get_parameter("w_kff").as_double();
        _w_d_alpha = this->get_parameter("w_d_alpha").as_double();
        _w_o_alpha = this->get_parameter("w_o_alpha").as_double();

        _x_pid._d_alpha = _x_d_alpha;
        _x_pid._out_alpha = _x_o_alpha;

        _w_pid._d_alpha = _w_d_alpha;
        _w_pid._out_alpha = _w_o_alpha;

		_param_cb_ptr = this->add_on_set_parameters_callback(std::bind(&driver_node::dynamic_parameters_cb, this, std::placeholders::_1));
*/
		
		_cmd_vel_sub = create_subscription<geometry_msgs::msg::Twist>(
			"/cmd_vel",
			10,
			std::bind(&motion_node::_cmd_vel_callback, this, std::placeholders::_1)
		);	
/*
		// odometry publisher
 		_odom_pub = create_publisher<nav_msgs::msg::Odometry>(
 			"/odom_wheel",
 			10
 		);

 		_speed_setpoint_pub = create_publisher<geometry_msgs::msg::Twist>(
 			"/speed_setpoint",
 			10
 		);

		// get time
		rcutils_time_point_value_t now;
		if (rcutils_system_time_now(&now) != RCUTILS_RET_OK)
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to get system time");
		}
		_stat_start_time_s = RCL_NS_TO_S(now);	
*/
		// get time
		
		if (rcutils_system_time_now(&start_time) != RCUTILS_RET_OK)
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to get system time");
		}

		_timer = this->create_wall_timer(
			std::chrono::microseconds((long int)(cfg.dt*1000000.0f)),
			std::bind(&motion_node::update, this)
		);
	}

	~motion_node()
	{
		// free joints
		for(size_t index=0; index<12; ++index)
		{
			control.goal_position[index]= 512;
			control.torque_enable[index] = 0;
		}
		int result = servo.update(control,feedback);
		//std::cout << "result:" << result << std::endl;
	}
/*    
	rcl_interfaces::msg::SetParametersResult dynamic_parameters_cb(
        const std::vector<rclcpp::Parameter> & parameters
	)
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        result.reason = "success";
        for (const auto &parameter : parameters)
        {
            if (parameter.get_name() == "x_kp" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
				_x_kp = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter %s changed: %f", parameter.get_name().c_str(), parameter.as_double());
            }
            if (parameter.get_name() == "x_ki" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
				_x_ki = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter %s changed: %f", parameter.get_name().c_str(), parameter.as_double());
            }
            if (parameter.get_name() == "x_kd" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
				_x_kd = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter %s changed: %f", parameter.get_name().c_str(), parameter.as_double());
            }
            if (parameter.get_name() == "x_kff" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
				_x_kff = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter %s changed: %f", parameter.get_name().c_str(), parameter.as_double());
            }
            if (parameter.get_name() == "w_kp" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
				_w_kp = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter %s changed: %f", parameter.get_name().c_str(), parameter.as_double());
            }
            if (parameter.get_name() == "w_ki" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
				_w_ki = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter %s changed: %f", parameter.get_name().c_str(), parameter.as_double());
            }
            if (parameter.get_name() == "w_kd" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
				_w_kd = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter %s changed: %f", parameter.get_name().c_str(), parameter.as_double());
            }
            if (parameter.get_name() == "w_kff" && parameter.get_type() == rclcpp::ParameterType::PARAMETER_DOUBLE)
            {
				_w_kff = parameter.as_double();
                RCLCPP_INFO(this->get_logger(), "Parameter %s changed: %f", parameter.get_name().c_str(), parameter.as_double());
            }
        }
        return result;
    }
*/
	void update()
	{
		// get time
		rcutils_time_point_value_t now;
		if (rcutils_system_time_now(&now) != RCUTILS_RET_OK)
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to get system time");
		}
		float time_s = (float)((now-start_time)/1000)/1000000.0f;	
		RCLCPP_INFO(this->get_logger(), "time: %f", time_s);

		//RCLCPP_INFO(this->get_logger(), "Vx: %f - Vy: %f - Wz: %f", _setpoint_vx, _setpoint_vy, _setpoint_wz);

        float dx {0.0f};
        float dy {0.0f};
        float dz {0.0f};

        float pitch {0.0f};
        float roll {0.0f};
        float yaw {0.0f};

        // gait computation
        vel_smo.update(_setpoint_vx,_setpoint_vy,_setpoint_wz);
        for(auto & p : phase)
            p.update(time_s,vel_smo.is_moving());
        for(auto & l : leg)
            l.update(vel_smo.get_vx(),vel_smo.get_vy(),vel_smo.get_wz());

		//RCLCPP_INFO(this->get_logger(), "Moving: %d, Vx: %f - Vy: %f - Wz: %f", vel_smo.is_moving(), vel_smo.get_vx(), vel_smo.get_vy(), vel_smo.get_wz());
		//RCLCPP_INFO(this->get_logger(), "State: %d - Alpha: %f", phase[0].get_state(), phase[0].get_alpha());

        // compute in-place transformation
        Eigen::Matrix<float,3,4> feet_BRF;
        Eigen::Vector3f const translation { dx, dy, dz };
        Eigen::Matrix3f const rotation { rotation_from_euler(roll,pitch,yaw) };
        for(size_t leg_id=0; leg_id<4; ++leg_id)
            feet_BRF.col(leg_id) = rotation * ( leg[leg_id].get_foot_BRF() + translation );

        // compute IK, joint and servo position
        Eigen::Matrix<float,3,4> joint_position { kin.four_leg_inverse_kinematics_BRF(feet_BRF) };
        int servo_position[12] {0};
        j.position_setpoint(joint_position,servo_position);

        // ESP32
        for(size_t index=0; index<12; ++index)
        {
            control.goal_position[index]= servo_position[index];
            control.torque_enable[index] = 1;
        }
        int result = servo.update(control,feedback);
        if(result!=mini_pupper::API_OK)
			RCLCPP_ERROR(this->get_logger(), "Failed to communication with ESP32!");

		//RCLCPP_INFO(this->get_logger(), "Servo: %d %d %d", servo_position[0],servo_position[1],servo_position[2]);

	}

private:
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel_sub;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom_pub;
	OnSetParametersCallbackHandle::SharedPtr _param_cb_ptr;
	rclcpp::TimerBase::SharedPtr _timer;

	void _cmd_vel_callback(const std::shared_ptr<geometry_msgs::msg::Twist> msg) //, const std::string & key)
	{
		//RCLCPP_INFO(this->get_logger(),"CMD_VEL %.3f %.3f",msg->linear.x,msg->angular.z);
		_setpoint_vx = msg->linear.x;
		_setpoint_vy = msg->linear.y;
		_setpoint_wz = msg->angular.z;
		// get time
		rcutils_time_point_value_t now;
		if (rcutils_system_time_now(&now) != RCUTILS_RET_OK)
		{
			RCLCPP_ERROR(this->get_logger(), "Failed to get system time");
		}
		_setpoint_timestamp_s = RCL_NS_TO_S(now);
	}

	rcutils_time_point_value_t start_time;

	// setpoints
	float _setpoint_vx = 0.0f;
	float _setpoint_vy = 0.0f;
	float _setpoint_wz = 0.0f;
	int32_t _setpoint_timestamp_s = 0;

	// motion
	config cfg;
	velocity_smoother vel_smo {cfg};
	vector<gait_phase> phase {
        { cfg, LEG_FR },
        { cfg, LEG_FL },
        { cfg, LEG_RR },
        { cfg, LEG_RL }
    };
    kinematic kin;
    vector<leg_trajectory> leg {
        { cfg, LEG_FR, phase[LEG_FR], kin },
        { cfg, LEG_FL, phase[LEG_FL], kin },
        { cfg, LEG_RR, phase[LEG_RR], kin },
        { cfg, LEG_RL, phase[LEG_RL], kin }
    };
    joints j;
    esp32_api servo;
    parameters_control_instruction_format control;
    parameters_control_acknowledge_format feedback;

};

int main(int argc, char ** argv)
{
	rclcpp::init(argc,argv);
	auto nd = std::make_shared<motion_node>();
	rclcpp::spin(nd);
	rclcpp::shutdown();
	return 0;
}
