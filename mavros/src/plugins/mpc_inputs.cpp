
#include <mavros/mavros_plugin.h>

#include <mavros_msgs/MPC_Inputs.h>

// #include <algorithm>
#include <iostream>
#include <pluginlib/class_list_macros.h>


namespace mavros
{
namespace std_plugins
{
class MPCInputsPlugin : public plugin::PluginBase
{
public:
	MPCInputsPlugin() : PluginBase(),
		nh("~")
    { }

    void initialize(UAS &uas_) override
	{
		PluginBase::initialize(uas_);

        nh.param<std::string>("frame_id", frame_id, "map");

		mpc_inputs_pub = nh.advertise<mavros_msgs::MPC_Inputs>("mpc_inputs_topic", 1);

		// enable_connection_cb();
	}

    Subscriptions get_subscriptions() override
	{
		return {
			       make_handler(&MPCInputsPlugin::handle_mpc_inputs),
		};
	}

private:
	ros::NodeHandle nh;
	std::string frame_id;
	// mavros_msgs::MPC_Inputs ros_msg;

	ros::Publisher mpc_inputs_pub;

	// float prev_time=0.0;

	void handle_mpc_inputs(const mavlink::mavlink_message_t *msg, mavlink::common::msg::MPC_INPUTS &mpc_inputs)
	{
		auto ros_msg = boost::make_shared<mavros_msgs::MPC_Inputs>();
		ros_msg->header = m_uas->synchronized_header(frame_id, mpc_inputs.time_usec);

		// ros_msg.header.stamp = m_uas->synchronise_stamp(mpc_inputs.time_usec);

		// std::cout<<"MPC_Inputs: "<<mpc_inputs.mpc_ref_in[0]<<" ; "<<mpc_inputs.mpc_mo_in[0]<<"\n";

		ros_msg->timestamp=mpc_inputs.time_usec;


		// Testing time sync
		// float mpc_ins_dt=mpc_inputs.time_usec*1e-6f-prev_time;

		// prev_time=mpc_inputs.time_usec*1e-6f;

		// std::cout<<"MPC_Inputs_dt : "<<mpc_ins_dt<<"\n";
		// std::cout<<"MPC_Inputs_time : "<<mpc_inputs.time_usec<<"\n";

        // for(int i=0;i<60;i++){ 
        //     ros_msg->mpc_ref_in[i] = mpc_inputs.mpc_ref_in[i];
		// 	// ros_msg.mpc_ref_in[i] = mpc_inputs.mpc_ref_in[i];
        // }

		ros_msg->mpc_ref_in[0] = mpc_inputs.mpc_ref_in[0];
		ros_msg->mpc_ref_in[1] = mpc_inputs.mpc_ref_in[1];

        ros_msg->mpc_mo_in[0] = mpc_inputs.mpc_mo_in[0];
        ros_msg->mpc_mo_in[1] = mpc_inputs.mpc_mo_in[1];
		// ros_msg.mpc_mo_in[0] = mpc_inputs.mpc_mo_in[0];
        // ros_msg.mpc_mo_in[1] = mpc_inputs.mpc_mo_in[1];
		ros_msg->state = mpc_inputs.state;
		ros_msg->grd_vel = mpc_inputs.grd_vel;
		ros_msg->ramp_trajectory = mpc_inputs.ramp_trajectory;
		ros_msg->mpc_activate = mpc_inputs.mpc_activate;
		ros_msg->mpc_land = mpc_inputs.mpc_land;

		mpc_inputs_pub.publish(ros_msg);
	}

};
}
}


PLUGINLIB_EXPORT_CLASS(mavros::std_plugins::MPCInputsPlugin, mavros::plugin::PluginBase)