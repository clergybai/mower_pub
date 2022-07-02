#ifndef _DYNAMIC_RECONFIGURE_CLIENT_H_
#define _DYNAMIC_RECONFIGURE_CLIENT_H_

#include <ros/ros.h>
#include <ros/subscriber.h>

#include <boost/thread/mutex.hpp>

#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/Config.h>

class DynamicReconfigureClient
{
public:
	DynamicReconfigureClient(ros::NodeHandle& nh, const std::string& dynamic_reconfigure_service_name, const std::string& parameter_updates_topic)
	: dynamic_reconfigure_current_config_received_(false), node_handle_(nh), dynamic_reconfigure_service_name_(dynamic_reconfigure_service_name)
	{
		dynamic_reconfigure_sub_ = node_handle_.subscribe(parameter_updates_topic, 1, &DynamicReconfigureClient::dynamic_reconfigure_current_config_callback, this);

		// receive current configuration
		ros::Duration sleep_rate(0.5);
		while (dynamic_reconfigure_current_config_received_ == false)
		{
			ros::spinOnce();
			sleep_rate.sleep();
		}
	}

	dynamic_reconfigure::Config& getConfig()
	{
		boost::mutex::scoped_lock lock(dynamic_reconfigure_lock_);
		return dynamic_reconfigure_config_;
	}

	bool setConfig(const std::string& param_name, const bool param_value)
	{
		boost::mutex::scoped_lock lock(dynamic_reconfigure_lock_);

		if (dynamic_reconfigure_current_config_received_ == false)
		{
			ROS_WARN("DynamicReconfigureClient: Did not receive the current configuration, yet.");
			return false;
		}

		bool found = false;
		for (size_t i=0; i<dynamic_reconfigure_config_.bools.size(); ++i)
		{
			if (param_name.compare(dynamic_reconfigure_config_.bools[i].name) == 0)
			{
				dynamic_reconfigure_config_.bools[i].value = param_value;
				found = true;
				break;
			}
		}
		if (found == false)
		{
			ROS_WARN("DynamicReconfigureClient: Parameter %s does not exist. Appending it to the reconfigure message.", param_name.c_str());
			dynamic_reconfigure::BoolParameter cfg_param;
			cfg_param.name = param_name;
			cfg_param.value = param_value;
			dynamic_reconfigure_config_.bools.push_back(cfg_param);
		}
		return sendConfiguration(dynamic_reconfigure_config_);
	}

	bool setConfig(const std::string& param_name, const double param_value)
	{
		boost::mutex::scoped_lock lock(dynamic_reconfigure_lock_);

		if (dynamic_reconfigure_current_config_received_ == false)
		{
			ROS_WARN("DynamicReconfigureClient: Did not receive the current configuration, yet.");
			return false;
		}

		bool found = false;
		for (size_t i=0; i<dynamic_reconfigure_config_.doubles.size(); ++i)
		{
			if (param_name.compare(dynamic_reconfigure_config_.doubles[i].name) == 0)
			{
				dynamic_reconfigure_config_.doubles[i].value = param_value;
				found = true;
				break;
			}
		}
		if (found == false)
		{
			ROS_WARN("DynamicReconfigureClient: Parameter %s does not exist. Appending it to the reconfigure message.", param_name.c_str());
			dynamic_reconfigure::DoubleParameter cfg_param;
			cfg_param.name = param_name;
			cfg_param.value = param_value;
			dynamic_reconfigure_config_.doubles.push_back(cfg_param);
		}
		return sendConfiguration(dynamic_reconfigure_config_);
	}

	bool setConfig(const std::string& param_name, const int param_value)
	{
		boost::mutex::scoped_lock lock(dynamic_reconfigure_lock_);

		if (dynamic_reconfigure_current_config_received_ == false)
		{
			ROS_WARN("DynamicReconfigureClient: Did not receive the current configuration, yet.");
			return false;
		}

		bool found = false;
		for (size_t i=0; i<dynamic_reconfigure_config_.ints.size(); ++i)
		{
			if (param_name.compare(dynamic_reconfigure_config_.ints[i].name) == 0)
			{
				dynamic_reconfigure_config_.ints[i].value = param_value;
				found = true;
				break;
			}
		}
		if (found == false)
		{
			ROS_WARN("DynamicReconfigureClient: Parameter %s does not exist. Appending it to the reconfigure message.", param_name.c_str());
			dynamic_reconfigure::IntParameter cfg_param;
			cfg_param.name = param_name;
			cfg_param.value = param_value;
			dynamic_reconfigure_config_.ints.push_back(cfg_param);
		}
		return sendConfiguration(dynamic_reconfigure_config_);
	}

	bool setConfig(const std::string& param_name, const std::string& param_value)
	{
		boost::mutex::scoped_lock lock(dynamic_reconfigure_lock_);

		if (dynamic_reconfigure_current_config_received_ == false)
		{
			ROS_WARN("DynamicReconfigureClient: Did not receive the current configuration, yet.");
			return false;
		}

		bool found = false;
		for (size_t i=0; i<dynamic_reconfigure_config_.strs.size(); ++i)
		{
			if (param_name.compare(dynamic_reconfigure_config_.strs[i].name) == 0)
			{
				dynamic_reconfigure_config_.strs[i].value = param_value;
				found = true;
				break;
			}
		}
		if (found == false)
		{
			ROS_WARN("DynamicReconfigureClient: Parameter %s does not exist. Appending it to the reconfigure message.", param_name.c_str());
			dynamic_reconfigure::StrParameter cfg_param;
			cfg_param.name = param_name;
			cfg_param.value = param_value;
			dynamic_reconfigure_config_.strs.push_back(cfg_param);
		}
		return sendConfiguration(dynamic_reconfigure_config_);
	}

private:
	void dynamic_reconfigure_current_config_callback(const dynamic_reconfigure::ConfigConstPtr& current_config)
	{
		boost::mutex::scoped_lock lock(dynamic_reconfigure_lock_);

		dynamic_reconfigure_config_ = *current_config;
		dynamic_reconfigure_current_config_received_ = true;
	}

	bool sendConfiguration(const dynamic_reconfigure::Config& dynamic_reconfigure_config)
	{
		dynamic_reconfigure::ReconfigureRequest req;
		dynamic_reconfigure::ReconfigureResponse res;
		req.config = dynamic_reconfigure_config;
		const bool success = ros::service::call(dynamic_reconfigure_service_name_, req, res);
		return success;
	}

	// parameters
	ros::NodeHandle node_handle_;
	ros::Subscriber dynamic_reconfigure_sub_;
	dynamic_reconfigure::Config dynamic_reconfigure_config_;
	bool dynamic_reconfigure_current_config_received_;
	std::string dynamic_reconfigure_service_name_;

	boost::mutex dynamic_reconfigure_lock_;
};

#endif //_DYNAMIC_RECONFIGURE_CLIENT_H_
