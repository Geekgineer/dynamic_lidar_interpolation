/**
 * @file pointcloud_interpolation_node.cpp
 * @brief ROS2 Node for LiDAR Point Cloud Interpolation with Dynamic Parameter Updates.
 *
 * This node subscribes to a LiDAR topic, applies interpolation to the point cloud,
 * and publishes the resulting dense point cloud. It supports dynamic parameter
 * updates at runtime using ROS2's parameter callback mechanism.
 *
 * Updated to handle dynamic parameter updates, maintain flexibility and responsiveness,
 * and ensure optimal performance through pre-allocation and thread safety.
 *
 * Original Author: Abdalrahman M. Amer
 * Updated by: [Your Name]
 *
 * @license AGPL-3.0
 *
 * This file is part of a project that is licensed under the Affero General Public License
 * (AGPL) version 3 or later. You should have received a copy of the AGPL license along with
 * this program. If not, see <https://www.gnu.org/licenses/>.
 *
 * The software is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the AGPL license for more details.
 */

#include "dynamic_lidar_interpolation/pointcloud_interpolation.hpp"

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <memory>
#include <mutex>
#include <Eigen/Dense>

using namespace std::chrono_literals;
using rcl_interfaces::msg::SetParametersResult;

/**
 * @brief ROS2 Node for LiDAR Point Cloud Interpolation with Dynamic Parameter Updates.
 *
 * This node subscribes to a LiDAR topic, applies interpolation to the point cloud,
 * and publishes the resulting dense point cloud. It supports dynamic parameter
 * updates at runtime using ROS2's parameter callback mechanism.
 */
class LidarInterpolationNode : public rclcpp::Node
{
public:
    LidarInterpolationNode()
        : Node("lidar_interpolation_node")
    {
        RCLCPP_INFO(this->get_logger(), "Initializing LidarInterpolationNode...");

        // Declare and retrieve parameters
        declare_parameters();

        // Cache parameters to avoid retrieving them on every callback
        cache_parameters();

        // Initialize publishers and subscribers
        initialize_publishers();
        initialize_subscribers();

        // Initialize PointCloud Interpolator
        range_utils_ = std::make_shared<pointcloud_interpolation::PointCloudInterpolator>();

        // Pre-allocate point cloud objects
        preallocate_pointclouds();

        // Register parameter callback for dynamic updates
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            std::bind(&LidarInterpolationNode::on_parameter_event, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Lidar Interpolation Node Initialized.");
    }

private:
    /**
     * @brief Declares all the ROS2 parameters used by the node.
     */
    void declare_parameters()
    {
        // LiDAR parameters
        this->declare_parameter<double>("lidar.max_range", std::numeric_limits<double>::max()); // Maximum range for filtering
        this->declare_parameter<double>("lidar.min_range", 0.0);                                // Minimum range for filtering

        // Interpolation parameters
        this->declare_parameter<std::string>("interpolation.method", "linear");                                               // Interpolation method (linear, nearest, etc.)
        this->declare_parameter<double>("interpolation.scale_factor_x", 1.0);                                                 // Scale factor for the point cloud enlargement in X
        this->declare_parameter<double>("interpolation.scale_factor_y", 2.0);                                                 // Scale factor for the point cloud enlargement in Y
        this->declare_parameter<double>("interpolation.interpolation_max_var", 50.0);                                         // Maximum allowed variance for filtering in the interpolation process
        this->declare_parameter<bool>("interpolation.apply_variance_filter", false);                                           // Apply variance filter
        this->declare_parameter<double>("interpolation.rotation_angle_x", 0.0);                                               // Rotation angle around the X axis (degrees)
        this->declare_parameter<std::string>("interpolation.extrapolation_value", "NaN");                                     // Extrapolation value
        this->declare_parameter<std::vector<double>>("interpolation.sensor_translation", std::vector<double>{0.0, 0.0, 0.0}); // Translation of the sensor (in 3D space)

        // Range Image Parameters
        this->declare_parameter<double>("range_image.angular_resolution_x", 0.25); // Angular resolution in X (degrees)
        this->declare_parameter<double>("range_image.angular_resolution_y", 2.05); // Angular resolution in Y (degrees)
        this->declare_parameter<double>("range_image.max_angle_width", 360.0);     // Maximum angular width (degrees)
        this->declare_parameter<double>("range_image.max_angle_height", 180.0);    // Maximum angular height (degrees)

        // Define the camera's FOV limits (in degrees)
        this->declare_parameter<double>("range_image.min_ang_fov", 0.0);
        this->declare_parameter<double>("range_image.max_ang_fov", 360.0);

        // Processing parameters
        this->declare_parameter<bool>("processing.filter_pc", false); // Flag for filtering the point cloud

        // Topics
        this->declare_parameter<std::string>("topics.lidar_topic", "velodyne_points");
        this->declare_parameter<std::string>("topics.interpolated_point_cloud_topic", "interpolated_point_cloud");
    }

    /**
     * @brief Caches parameters after declaration to avoid retrieving them in every callback.
     */
    void cache_parameters()
    {
        // LiDAR parameters
        lidar_min_range_ = this->get_parameter("lidar.min_range").as_double();
        lidar_max_range_ = this->get_parameter("lidar.max_range").as_double();

        // Interpolation parameters
        interpolation_method_ = this->get_parameter("interpolation.method").as_string();

        if (!is_supported_interpolation_method(interpolation_method_))
        {
            RCLCPP_ERROR(this->get_logger(), "Unsupported interpolation method: %s", interpolation_method_.c_str());
            throw std::invalid_argument("Unsupported interpolation method.");
        }

        scale_factor_x_ = this->get_parameter("interpolation.scale_factor_x").as_double();
        scale_factor_y_ = this->get_parameter("interpolation.scale_factor_y").as_double();
        interpolation_max_var_ = this->get_parameter("interpolation.interpolation_max_var").as_double();
        apply_variance_filter_ = this->get_parameter("interpolation.apply_variance_filter").as_bool();
        rotation_angle_x_ = this->get_parameter("interpolation.rotation_angle_x").as_double();
        extrapolation_value_str_ = this->get_parameter("interpolation.extrapolation_value").as_string();

        // Handle extrapolation_value_
        if (is_nan_string(extrapolation_value_str_)) {
            extrapolation_value_ = std::numeric_limits<double>::quiet_NaN();
            RCLCPP_INFO(this->get_logger(), "Extrapolation value set to NaN.");
        } else {
            try {
                extrapolation_value_ = std::stod(extrapolation_value_str_);
                RCLCPP_INFO(this->get_logger(), "Extrapolation value set to %f.", extrapolation_value_);
            } catch (const std::invalid_argument &e) {
                RCLCPP_WARN(this->get_logger(), "Invalid extrapolation_value string '%s'. Defaulting to NaN.", extrapolation_value_str_.c_str());
                extrapolation_value_ = std::numeric_limits<double>::quiet_NaN();
            }
        }

        // Sensor translation
        std::vector<double> sensor_translation_vec = this->get_parameter("interpolation.sensor_translation").as_double_array();
        if (sensor_translation_vec.size() != 3)
        {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'interpolation.sensor_translation' must have exactly 3 elements.");
            throw std::runtime_error("Invalid 'interpolation.sensor_translation' parameter size.");
        }

        sensor_translation_ = Eigen::Vector3f(
            static_cast<float>(sensor_translation_vec[0]),
            static_cast<float>(sensor_translation_vec[1]),
            static_cast<float>(sensor_translation_vec[2]));

        // Range Image Parameters
        angular_resolution_x_ = this->get_parameter("range_image.angular_resolution_x").as_double();
        angular_resolution_y_ = this->get_parameter("range_image.angular_resolution_y").as_double();
        max_angle_width_ = this->get_parameter("range_image.max_angle_width").as_double();
        max_angle_height_ = this->get_parameter("range_image.max_angle_height").as_double();

        // FOV limits in degrees
        min_ang_fov_ = this->get_parameter("range_image.min_ang_fov").as_double();
        max_ang_fov_ = this->get_parameter("range_image.max_ang_fov").as_double();

        // Processing parameters
        filter_pc_ = this->get_parameter("processing.filter_pc").as_bool();

        // Topics
        lidar_topic_ = this->get_parameter("topics.lidar_topic").as_string();
        interpolated_point_cloud_topic_ = this->get_parameter("topics.interpolated_point_cloud_topic").as_string();
    }

    /**
     * @brief Initializes all publishers with appropriate QoS settings.
     */
    void initialize_publishers()
    {
        // PointCloud2 Publisher with Best Effort QoS
        interpolated_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            interpolated_point_cloud_topic_,
            rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());

        RCLCPP_INFO(this->get_logger(), "Initialized PointCloud2 publisher for '%s'.",
                    interpolated_point_cloud_topic_.c_str());
    }

    /**
     * @brief Initializes subscribers.
     */
    void initialize_subscribers()
    {
        // Initialize subscriber for LiDAR data with Best Effort QoS
        rclcpp::QoS qos_lidar = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
        sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            lidar_topic_,
            qos_lidar,
            std::bind(&LidarInterpolationNode::fusionCallback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Subscribed to LiDAR topic '%s'.",
                    lidar_topic_.c_str());
    }

    /**
     * @brief Pre-allocates point cloud objects to reuse in the callback.
     */
    void preallocate_pointclouds()
    {
        lidar_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        filtered_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        sor_filtered_cloud_ = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        RCLCPP_INFO(this->get_logger(), "Pre-allocated point cloud objects.");
    }

    /**
     * @brief Callback function for LiDAR messages.
     *
     * Processes the incoming LiDAR point cloud, applies interpolation, and publishes the result.
     *
     * @param lidar_msg Incoming LiDAR point cloud message.
     */
    void fusionCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr lidar_msg)
    {
        std::lock_guard<std::mutex> lock(mutex_);

        RCLCPP_DEBUG(this->get_logger(), "Fusion callback triggered.");

        try
        {
            /*───────────────────────────────────────────────────────*/
            //           Step 0: PointCloud Conversion
            /*───────────────────────────────────────────────────────*/

            // Clear pre-allocated point clouds
            lidar_cloud_->clear();
            filtered_cloud_->clear();
            sor_filtered_cloud_->clear();

            // Convert ROS message to PCL format
            pcl::fromROSMsg(*lidar_msg, *lidar_cloud_);

            RCLCPP_DEBUG(this->get_logger(), "Converted PointCloud2 message to PCL format. Total points: %zu", lidar_cloud_->points.size());

            // Step 1: Filter Point Cloud - Remove NaNs
            std::vector<int> indices;
            pcl::removeNaNFromPointCloud(*lidar_cloud_, *filtered_cloud_, indices);

            RCLCPP_DEBUG(this->get_logger(), "Removed NaN points: %zu points remaining.", filtered_cloud_->points.size());

            // Apply Statistical Outlier Removal (Optional)
            if (filter_pc_)
            {
                pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
                sor.setInputCloud(filtered_cloud_);
                sor.setMeanK(50);
                sor.setStddevMulThresh(1.0);
                sor.filter(*sor_filtered_cloud_);

                sor_filtered_cloud_->is_dense = true; // Set to true after filtering

                RCLCPP_DEBUG(this->get_logger(), "Statistical outlier removal: %zu points remaining.", sor_filtered_cloud_->points.size());
            }
            else
            {
                sor_filtered_cloud_->clear();
                *sor_filtered_cloud_ = *filtered_cloud_;
                RCLCPP_DEBUG(this->get_logger(), "Statistical outlier removal skipped.");
            }

            /*───────────────────────────────────────────*/
            //        Step 2: PointCloud Interpolation    /
            /*───────────────────────────────────────────*/

            // Generate 3D interpolated PointCloud
            auto interpolated_dense_cloud = range_utils_->interpolatePointCloud(
                sor_filtered_cloud_,    // The filtered point cloud to be interpolated
                angular_resolution_x_,  // Angular resolution in the X direction
                angular_resolution_y_,  // Angular resolution in the Y direction
                max_angle_width_,       // Maximum angular width of the range image
                max_angle_height_,      // Maximum angular height of the range image
                interpolation_method_,  // Interpolation method ("linear", "nearest", etc.)
                scale_factor_x_,        // Scale factor in the X direction
                scale_factor_y_,        // Scale factor in the Y direction
                lidar_min_range_,       // Minimum range for the lidar
                lidar_max_range_,       // Maximum range for the lidar
                apply_variance_filter_, // Whether to apply point cloud variance filtering
                interpolation_max_var_, // Maximum allowed variance for filtering.
                sensor_translation_,    // The translation of the sensor in 3D space (Eigen::Vector3f)
                rotation_angle_x_,      // Rotation angle around the X axis
                extrapolation_value_,   // Value for extrapolation outside the range
                min_ang_fov_,           // Minimum angle for the field of view (degree)
                max_ang_fov_            // Maximum angle for the field of view (degree)
            );

            if (!interpolated_dense_cloud || interpolated_dense_cloud->points.empty())
            {
                RCLCPP_WARN(this->get_logger(), "Dense point cloud is empty after interpolation.");
                return;
            }

            RCLCPP_DEBUG(this->get_logger(), "Interpolated dense point cloud has %zu points.", interpolated_dense_cloud->points.size());

            // Step 3: Publish interpolated point cloud
            sensor_msgs::msg::PointCloud2 fused_pcl_msg;
            pcl::toROSMsg(*interpolated_dense_cloud, fused_pcl_msg);
            fused_pcl_msg.header = lidar_msg->header;
            interpolated_point_cloud_pub_->publish(fused_pcl_msg);

            RCLCPP_DEBUG(this->get_logger(), "Published interpolated point cloud.");
        }
        catch (const std::exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Exception in fusionCallback: %s", e.what());
        }
    }

    /**
     * @brief Callback function for dynamic parameter updates.
     *
     * This function is called whenever a parameter is set or updated. It updates the
     * cached parameter values and performs validation as necessary.
     *
     * @param parameters Vector of parameters that are being set.
     * @return SetParametersResult indicating success or failure.
     */
    SetParametersResult on_parameter_event(const std::vector<rclcpp::Parameter> &parameters)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        SetParametersResult result;
        result.successful = true;
        result.reason = "success";

        for (const auto &param : parameters)
        {
            try
            {
                if (param.get_name() == "lidar.min_range")
                {
                    double new_min_range = param.as_double();
                    if (new_min_range < 0.0)
                    {
                        result.successful = false;
                        result.reason = "'lidar.min_range' must be non-negative.";
                        RCLCPP_WARN(this->get_logger(), "Attempted to set 'lidar.min_range' to a negative value: %f", new_min_range);
                        break;
                    }
                    lidar_min_range_ = new_min_range;
                    RCLCPP_INFO(this->get_logger(), "Updated 'lidar.min_range' to %f", lidar_min_range_);
                }
                else if (param.get_name() == "lidar.max_range")
                {
                    double new_max_range = param.as_double();
                    if (new_max_range <= lidar_min_range_)
                    {
                        result.successful = false;
                        result.reason = "'lidar.max_range' must be greater than 'lidar.min_range'.";
                        RCLCPP_WARN(this->get_logger(), "Attempted to set 'lidar.max_range' to %f, which is not greater than 'lidar.min_range' (%f).",
                                    new_max_range, lidar_min_range_);
                        break;
                    }
                    lidar_max_range_ = new_max_range;
                    RCLCPP_INFO(this->get_logger(), "Updated 'lidar.max_range' to %f", lidar_max_range_);
                }
                else if (param.get_name() == "interpolation.method")
                {
                    std::string new_method = param.as_string();
                    if (!is_supported_interpolation_method(new_method))
                    {
                        result.successful = false;
                        result.reason = "Unsupported interpolation method.";
                        RCLCPP_WARN(this->get_logger(), "Unsupported interpolation method: %s", new_method.c_str());
                        break;
                    }
                    interpolation_method_ = new_method;
                    RCLCPP_INFO(this->get_logger(), "Updated 'interpolation.method' to %s", interpolation_method_.c_str());
                }
                else if (param.get_name() == "interpolation.scale_factor_x")
                {
                    double new_scale_x = param.as_double();
                    if (new_scale_x <= 0.0)
                    {
                        result.successful = false;
                        result.reason = "'interpolation.scale_factor_x' must be positive.";
                        RCLCPP_WARN(this->get_logger(), "Attempted to set 'interpolation.scale_factor_x' to a non-positive value: %f", new_scale_x);
                        break;
                    }
                    scale_factor_x_ = new_scale_x;
                    RCLCPP_INFO(this->get_logger(), "Updated 'interpolation.scale_factor_x' to %f", scale_factor_x_);
                }
                else if (param.get_name() == "interpolation.scale_factor_y")
                {
                    double new_scale_y = param.as_double();
                    if (new_scale_y <= 0.0)
                    {
                        result.successful = false;
                        result.reason = "'interpolation.scale_factor_y' must be positive.";
                        RCLCPP_WARN(this->get_logger(), "Attempted to set 'interpolation.scale_factor_y' to a non-positive value: %f", new_scale_y);
                        break;
                    }
                    scale_factor_y_ = new_scale_y;
                    RCLCPP_INFO(this->get_logger(), "Updated 'interpolation.scale_factor_y' to %f", scale_factor_y_);
                }
                else if (param.get_name() == "interpolation.interpolation_max_var")
                {
                    double new_max_var = param.as_double();
                    if (new_max_var < 0.0)
                    {
                        result.successful = false;
                        result.reason = "'interpolation.interpolation_max_var' must be non-negative.";
                        RCLCPP_WARN(this->get_logger(), "Attempted to set 'interpolation.interpolation_max_var' to a negative value: %f", new_max_var);
                        break;
                    }
                    interpolation_max_var_ = new_max_var;
                    RCLCPP_INFO(this->get_logger(), "Updated 'interpolation.interpolation_max_var' to %f", interpolation_max_var_);
                }
                else if (param.get_name() == "interpolation.apply_variance_filter")
                {
                    apply_variance_filter_ = param.as_bool();
                    RCLCPP_INFO(this->get_logger(), "Updated 'interpolation.apply_variance_filter' to %s", apply_variance_filter_ ? "true" : "false");
                }
                else if (param.get_name() == "interpolation.rotation_angle_x")
                {
                    double new_rotation_angle_x = param.as_double();
                    // Optionally, you can enforce angle limits here
                    rotation_angle_x_ = new_rotation_angle_x;
                    RCLCPP_INFO(this->get_logger(), "Updated 'interpolation.rotation_angle_x' to %f", rotation_angle_x_);
                }
                else if (param.get_name() == "interpolation.extrapolation_value")
                {
                    std::string new_extrapolation_str = param.as_string();
                    if (is_nan_string(new_extrapolation_str))
                    {
                        extrapolation_value_ = std::numeric_limits<double>::quiet_NaN();
                        RCLCPP_INFO(this->get_logger(), "Extrapolation value set to NaN.");
                    }
                    else
                    {
                        try
                        {
                            extrapolation_value_ = std::stod(new_extrapolation_str);
                            RCLCPP_INFO(this->get_logger(), "Extrapolation value set to %f.", extrapolation_value_);
                        }
                        catch (const std::invalid_argument &e)
                        {
                            result.successful = false;
                            result.reason = "Invalid 'interpolation.extrapolation_value' string.";
                            RCLCPP_WARN(this->get_logger(), "Invalid extrapolation_value string '%s'.", new_extrapolation_str.c_str());
                            break;
                        }
                    }
                }
                else if (param.get_name() == "interpolation.sensor_translation")
                {
                    std::vector<double> new_translation = param.as_double_array();
                    if (new_translation.size() != 3)
                    {
                        result.successful = false;
                        result.reason = "'interpolation.sensor_translation' must have exactly 3 elements.";
                        RCLCPP_WARN(this->get_logger(), "Attempted to set 'interpolation.sensor_translation' with %zu elements.", new_translation.size());
                        break;
                    }
                    sensor_translation_ = Eigen::Vector3f(
                        static_cast<float>(new_translation[0]),
                        static_cast<float>(new_translation[1]),
                        static_cast<float>(new_translation[2]));
                    RCLCPP_INFO(this->get_logger(), "Updated 'interpolation.sensor_translation' to [%f, %f, %f].",
                                sensor_translation_.x(), sensor_translation_.y(), sensor_translation_.z());
                }
                else if (param.get_name() == "range_image.angular_resolution_x")
                {
                    double new_ang_res_x = param.as_double();
                    if (new_ang_res_x <= 0.0)
                    {
                        result.successful = false;
                        result.reason = "'range_image.angular_resolution_x' must be positive.";
                        RCLCPP_WARN(this->get_logger(), "Attempted to set 'range_image.angular_resolution_x' to a non-positive value: %f", new_ang_res_x);
                        break;
                    }
                    angular_resolution_x_ = new_ang_res_x;
                    RCLCPP_INFO(this->get_logger(), "Updated 'range_image.angular_resolution_x' to %f", angular_resolution_x_);
                }
                else if (param.get_name() == "range_image.angular_resolution_y")
                {
                    double new_ang_res_y = param.as_double();
                    if (new_ang_res_y <= 0.0)
                    {
                        result.successful = false;
                        result.reason = "'range_image.angular_resolution_y' must be positive.";
                        RCLCPP_WARN(this->get_logger(), "Attempted to set 'range_image.angular_resolution_y' to a non-positive value: %f", new_ang_res_y);
                        break;
                    }
                    angular_resolution_y_ = new_ang_res_y;
                    RCLCPP_INFO(this->get_logger(), "Updated 'range_image.angular_resolution_y' to %f", angular_resolution_y_);
                }
                else if (param.get_name() == "range_image.max_angle_width")
                {
                    double new_max_ang_width = param.as_double();
                    if (new_max_ang_width <= 0.0 || new_max_ang_width > 360.0)
                    {
                        result.successful = false;
                        result.reason = "'range_image.max_angle_width' must be in the range (0, 360].";
                        RCLCPP_WARN(this->get_logger(), "Attempted to set 'range_image.max_angle_width' to an invalid value: %f", new_max_ang_width);
                        break;
                    }
                    max_angle_width_ = new_max_ang_width;
                    RCLCPP_INFO(this->get_logger(), "Updated 'range_image.max_angle_width' to %f", max_angle_width_);
                }
                else if (param.get_name() == "range_image.max_angle_height")
                {
                    double new_max_ang_height = param.as_double();
                    if (new_max_ang_height <= 0.0 || new_max_ang_height > 180.0)
                    {
                        result.successful = false;
                        result.reason = "'range_image.max_angle_height' must be in the range (0, 180].";
                        RCLCPP_WARN(this->get_logger(), "Attempted to set 'range_image.max_angle_height' to an invalid value: %f", new_max_ang_height);
                        break;
                    }
                    max_angle_height_ = new_max_ang_height;
                    RCLCPP_INFO(this->get_logger(), "Updated 'range_image.max_angle_height' to %f", max_angle_height_);
                }
                else if (param.get_name() == "range_image.min_ang_fov")
                {
                    double new_min_fov = param.as_double();
                    if (new_min_fov < 0.0 || new_min_fov >= max_ang_fov_)
                    {
                        result.successful = false;
                        result.reason = "'range_image.min_ang_fov' must be in the range [0, max_ang_fov).";
                        RCLCPP_WARN(this->get_logger(), "Attempted to set 'range_image.min_ang_fov' to an invalid value: %f", new_min_fov);
                        break;
                    }
                    min_ang_fov_ = new_min_fov;
                    RCLCPP_INFO(this->get_logger(), "Updated 'range_image.min_ang_fov' to %f", min_ang_fov_);
                }
                else if (param.get_name() == "range_image.max_ang_fov")
                {
                    double new_max_fov = param.as_double();
                    if (new_max_fov <= min_ang_fov_ || new_max_fov > 360.0)
                    {
                        result.successful = false;
                        result.reason = "'range_image.max_ang_fov' must be in the range (min_ang_fov, 360].";
                        RCLCPP_WARN(this->get_logger(), "Attempted to set 'range_image.max_ang_fov' to an invalid value: %f", new_max_fov);
                        break;
                    }
                    max_ang_fov_ = new_max_fov;
                    RCLCPP_INFO(this->get_logger(), "Updated 'range_image.max_ang_fov' to %f", max_ang_fov_);
                }
                else if (param.get_name() == "processing.filter_pc")
                {
                    filter_pc_ = param.as_bool();
                    RCLCPP_INFO(this->get_logger(), "Updated 'processing.filter_pc' to %s", filter_pc_ ? "true" : "false");
                }
                else if (param.get_name() == "topics.lidar_topic")
                {
                    std::string new_lidar_topic = param.as_string();
                    if (new_lidar_topic.empty())
                    {
                        result.successful = false;
                        result.reason = "'topics.lidar_topic' cannot be empty.";
                        RCLCPP_WARN(this->get_logger(), "Attempted to set 'topics.lidar_topic' to an empty string.");
                        break;
                    }
                    lidar_topic_ = new_lidar_topic;
                    // Reinitialize subscriber with the new topic
                    sub_lidar_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
                        lidar_topic_,
                        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
                        std::bind(&LidarInterpolationNode::fusionCallback, this, std::placeholders::_1));
                    RCLCPP_INFO(this->get_logger(), "Updated 'topics.lidar_topic' to '%s'", lidar_topic_.c_str());
                }
                else if (param.get_name() == "topics.interpolated_point_cloud_topic")
                {
                    std::string new_pub_topic = param.as_string();
                    if (new_pub_topic.empty())
                    {
                        result.successful = false;
                        result.reason = "'topics.interpolated_point_cloud_topic' cannot be empty.";
                        RCLCPP_WARN(this->get_logger(), "Attempted to set 'topics.interpolated_point_cloud_topic' to an empty string.");
                        break;
                    }
                    interpolated_point_cloud_topic_ = new_pub_topic;
                    // Reinitialize publisher with the new topic
                    interpolated_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                        interpolated_point_cloud_topic_,
                        rclcpp::QoS(rclcpp::KeepLast(10)).best_effort());
                    RCLCPP_INFO(this->get_logger(), "Updated 'topics.interpolated_point_cloud_topic' to '%s'", interpolated_point_cloud_topic_.c_str());
                }
                // Add additional parameter handlers as needed
                else
                {
                    RCLCPP_WARN(this->get_logger(), "Unhandled parameter: %s", param.get_name().c_str());
                }
            }
            catch (const std::exception &e)
            {
                result.successful = false;
                result.reason = e.what();
                RCLCPP_ERROR(this->get_logger(), "Exception in parameter callback: %s", e.what());
                break;
            }
        }

        return result;
    }

    /**
     * @brief Checks if the given interpolation method is supported.
     *
     * @param method Interpolation method as a string.
     * @return true if supported, false otherwise.
     */
    bool is_supported_interpolation_method(const std::string &method) const
    {
        static const std::vector<std::string> supported_methods = {
            "linear", "nearest", "bilateral", "edgeAware", "spline"};
        return std::find(supported_methods.begin(), supported_methods.end(), method) != supported_methods.end();
    }

    /**
     * @brief Checks if the given string represents a NaN value.
     *
     * @param str Input string.
     * @return true if the string represents NaN, false otherwise.
     */
    bool is_nan_string(const std::string &str) const
    {
        return (str == "NaN" || str == "nan" || str == "NAN");
    }

    // Cached Parameters
    double lidar_max_range_;
    double lidar_min_range_;

    std::string interpolation_method_;
    double scale_factor_x_;
    double scale_factor_y_;
    double interpolation_max_var_;
    bool apply_variance_filter_;
    double rotation_angle_x_;

    std::string extrapolation_value_str_;
    double extrapolation_value_;

    Eigen::Vector3f sensor_translation_;

    double angular_resolution_x_;
    double angular_resolution_y_;
    double max_angle_width_;
    double max_angle_height_;
    double min_ang_fov_;
    double max_ang_fov_;

    bool filter_pc_;

    std::string lidar_topic_;
    std::string interpolated_point_cloud_topic_;

    // Parameters and utilities
    std::shared_ptr<pointcloud_interpolation::PointCloudInterpolator> range_utils_;

    // Publishers
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr interpolated_point_cloud_pub_;

    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar_;

    // Mutex for thread safety
    std::mutex mutex_;

    // Pre-allocated PointClouds
    pcl::PointCloud<pcl::PointXYZ>::Ptr lidar_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr sor_filtered_cloud_;

    // Parameter callback handle
    OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    try
    {
        auto node = std::make_shared<LidarInterpolationNode>();
        rclcpp::spin(node);
    }
    catch (const std::exception &e)
    {
        RCLCPP_FATAL(rclcpp::get_logger("rclcpp"), "Exception in node: %s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
