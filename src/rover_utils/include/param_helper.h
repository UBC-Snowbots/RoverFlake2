#include <rclcpp/rclcpp.hpp>

// Might not work for string parameters
// Must be used after construction of a node (ie, make a public init function called from main, after the node shared pointer is created)
template<typename T>
T get_or_declare_parameter(
    const std::shared_ptr<rclcpp::Node> & node,
    const std::string & name,
    const T & default_value)
{
    if (!node->has_parameter(name)) {
        node->declare_parameter<T>(name, default_value);
    }
    return node->get_parameter(name).get_value<T>();
}