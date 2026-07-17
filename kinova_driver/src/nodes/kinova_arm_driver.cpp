//============================================================================
// Name        : kinova_arm_driver.cpp
// Author      : WPI, Clearpath Robotics
// Version     : 0.5
// Copyright   : BSD
// Description : A ROS driver for controlling the Kinova Kinova robotic manipulator arm
//============================================================================

#include <libudev.h>
#include <poll.h>
#include <cstring>
#include <stdexcept>

#include "kinova_driver/kinova_api.h"
#include "kinova_driver/kinova_arm.h"
#include "kinova_driver/kinova_tool_pose_action.h"
#include "kinova_driver/kinova_joint_angles_action.h"
#include "kinova_driver/kinova_fingers_action.h"
#include "kinova_driver/kinova_joint_trajectory_controller.h"

constexpr const char *KINOVA_VENDOR_ID = "22cd";
constexpr const char *KINOVA_JACO_PRODUCT_ID = "0000";

void wait_for_kinova_arm()
{
    struct udev *udev = udev_new();
    if (!udev)
        throw std::runtime_error("Failed to create udev context");

    RCLCPP_INFO(rclcpp::get_logger("kinova_arm_driver"), "Waiting for Kinova Jaco2 to become available...");

    // First check whether the Kinova arm is already connected
    struct udev_enumerate *enumerate = udev_enumerate_new(udev);
    udev_enumerate_add_match_subsystem(enumerate, "usb");
    udev_enumerate_scan_devices(enumerate);

    struct udev_list_entry *devices =
        udev_enumerate_get_list_entry(enumerate);

    struct udev_list_entry *entry;

    udev_list_entry_foreach(entry, devices)
    {
        const char *path = udev_list_entry_get_name(entry);

        struct udev_device *dev =
            udev_device_new_from_syspath(udev, path);

        const char *vendor =
            udev_device_get_sysattr_value(dev, "idVendor");

        const char *product =
            udev_device_get_sysattr_value(dev, "idProduct");
        
        if (vendor &&
            product &&
            strcmp(KINOVA_VENDOR_ID, vendor) == 0 &&
            strcmp(KINOVA_JACO_PRODUCT_ID, product) == 0)
        {
            udev_device_unref(dev);
            udev_enumerate_unref(enumerate);
            udev_unref(udev);
            return;
        }

        udev_device_unref(dev);
    }

    udev_enumerate_unref(enumerate);

    // Kinova arm not present; wait for it
    struct udev_monitor *mon =
        udev_monitor_new_from_netlink(udev, "udev");

    udev_monitor_filter_add_match_subsystem_devtype(
        mon,
        "usb",
        "usb_device");

    udev_monitor_enable_receiving(mon);

    int fd = udev_monitor_get_fd(mon);

    while (true)
    {
        struct pollfd pfd{};
        pfd.fd = fd;
        pfd.events = POLLIN;

        if (poll(&pfd, 1, -1) <= 0)
            continue;

        struct udev_device *dev =
            udev_monitor_receive_device(mon);

        if (!dev)
            continue;

        const char *action =
            udev_device_get_action(dev);

        if (action && strcmp(action, "add") == 0)
        {
            const char *vendor =
                udev_device_get_sysattr_value(dev, "idVendor");

            const char *product =
                udev_device_get_sysattr_value(dev, "idProduct");

            if (vendor &&
                product &&
                strcmp(KINOVA_VENDOR_ID, vendor) == 0 &&
                strcmp(KINOVA_JACO_PRODUCT_ID, product) == 0)
            {
                udev_device_unref(dev);
                break;
            }
        }

        udev_device_unref(dev);
    }

    udev_monitor_unref(mon);
    udev_unref(udev);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = std::make_shared<rclcpp::Node>("kinova_arm");
    std::shared_ptr<rclcpp::Node> node_pose_action = std::make_shared<rclcpp::Node>("pose_action","pose_action");
    std::shared_ptr<rclcpp::Node> node_joints_action = std::make_shared<rclcpp::Node>("joints_action","joints_action");
    std::shared_ptr<rclcpp::Node> node_fingers_action = std::make_shared<rclcpp::Node>("fingers_action","fingers_action");

    boost::recursive_mutex api_mutex;

    bool is_first_init = true;
    std::string kinova_robotType = "";
    std::string kinova_robotName = "";

    if (!node->has_parameter("kinova_robotType"))
        node->declare_parameter("kinova_robotType", kinova_robotType);
    node->get_parameter("kinova_robotType", kinova_robotType);
    if (!node->has_parameter("kinova_robotName"))
        node->declare_parameter("kinova_robotName", kinova_robotName);
    node->get_parameter("kinova_robotName", kinova_robotName);

    if (kinova_robotType == "")
        return -1;

    RCLCPP_INFO(node->get_logger(), "kinova_robotType is %s.", kinova_robotType.c_str());
    RCLCPP_INFO(node->get_logger(), "kinova_robotName is %s.", kinova_robotName.c_str());

    // Wait for Kinova Jaco to be detected through USB
    wait_for_kinova_arm();
    RCLCPP_INFO(node->get_logger(), "Kinova Jaco2 is available");

    
    try
    {
        kinova::KinovaComm comm(node, api_mutex, is_first_init, kinova_robotType);
        kinova::KinovaArm kinova_arm(comm, node, kinova_robotType, kinova_robotName);
        kinova::KinovaPoseActionServer pose_server(comm, node_pose_action, kinova_robotType, kinova_robotName);
        kinova::KinovaAnglesActionServer angles_server(comm, node, node_joints_action, kinova_robotType, kinova_robotName);
        kinova::KinovaFingersActionServer fingers_server(comm, node_fingers_action, kinova_robotType, kinova_robotName);
        kinova::JointTrajectoryController joint_trajectory_controller(comm, node);
        is_first_init = false;

        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.add_node(node_pose_action);
        executor.add_node(node_joints_action);
        executor.add_node(node_fingers_action);
        executor.spin();
    }
    catch(const std::exception& e)
    {
        RCLCPP_ERROR_STREAM(node->get_logger(), e.what());
        kinova::KinovaAPI api;
        boost::recursive_mutex::scoped_lock lock(api_mutex);
        api.closeAPI();
        rclcpp::sleep_for(std::chrono::seconds(1));
    }

    rclcpp::shutdown();
    return 0;
}
