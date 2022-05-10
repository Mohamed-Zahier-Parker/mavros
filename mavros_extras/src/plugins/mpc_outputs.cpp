#include <mavros/mavros_plugin.h>
#include <pluginlib/class_list_macros.h>
#include <mavros_msgs/MPC_Outputs.h>
#include <iostream>

namespace mavros {
 namespace extra_plugins{

 class MPCOutputsPlugin : public plugin::PluginBase {
 public:
     MPCOutputsPlugin() : PluginBase(),
         nh("~mpc_outputs")

    { };

     void initialize(UAS &uas_)
     {
         PluginBase::initialize(uas_);
         mpc_outputs_sub = nh.subscribe("mpc_outputs_topic", 1, &MPCOutputsPlugin::mpc_outputs_cb, this);
     };

     Subscriptions get_subscriptions()
     {
         return {/* RX disabled */ };
     }


private:
     ros::NodeHandle nh;
     ros::Subscriber mpc_outputs_sub;

    void mpc_outputs_cb(const mavros_msgs::MPC_Outputs::ConstPtr &req)
    {
     
        mavlink::common::msg::MPC_OUTPUTS mpc_out {};
         mpc_out.time_usec = req->header.stamp.toNSec() / 1000;
         mpc_out.mpc_mv_out[0] = req->mpc_mv_out[0];
         mpc_out.mpc_mv_out[1] = req->mpc_mv_out[1];

        // std::cout << "mpc_out plugin : "<<mpc_out.mpc_mv_out[0]<<"\n";
        // std::cout<<"MPC_outputs_time : "<<mpc_out.time_usec<<"\n";
     
        UAS_FCU(m_uas)->send_message_ignore_drop(mpc_out);
    }


 };
 }   // namespace extra_plugins
 }   // namespace mavros

PLUGINLIB_EXPORT_CLASS(mavros::extra_plugins::MPCOutputsPlugin, mavros::plugin::PluginBase)