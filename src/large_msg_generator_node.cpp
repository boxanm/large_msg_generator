//
// Created by mbo on 02/03/23.
//
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

class LargeMsgGenerator : public rclcpp::Node {
public:
	LargeMsgGenerator():
	Node("large_msg_generator_node") {
		path_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("large_msg", 2);
		ptcloud_ = sensor_msgs::msg::PointCloud2();
		ptcloud_.header.frame_id = "map";
		ptcloud_.height = 32;
		ptcloud_.width = 1800;
		ptcloud_.is_bigendian = false;
		ptcloud_.point_step = 20;
		ptcloud_.row_step = 36000;
		ptcloud_.is_dense = false;
// uint8 FLOAT32 = 7
		auto point_fields = std::vector<sensor_msgs::msg::PointField>();
		auto point_field = sensor_msgs::msg::PointField();
		point_field.name = "x";
		point_field.offset = 0;
		point_field.datatype = 7;
		point_field.count = 1;
		point_fields.push_back(point_field);
		point_field.name = "y";
		point_field.offset = 4;
		point_field.datatype = 7;
		point_field.count = 1;
		point_fields.push_back(point_field);
		point_field.name = "z";
		point_field.offset = 8;
		point_field.datatype = 7;
		point_field.count = 1;
		point_fields.push_back(point_field);
		point_field.name = "intensity";
		point_field.offset = 12;
		point_field.datatype = 7;
		point_field.count = 1;
		point_fields.push_back(point_field);
		point_field.name = "time";
		point_field.offset = 16;
		point_field.datatype = 7;
		point_field.count = 1;
		point_fields.push_back(point_field);
		ptcloud_.fields = point_fields;

		RCLCPP_INFO(this->get_logger(), "Generation data");
		union float_bytes {
			float val;
			std::uint8_t bytes[sizeof(float)];
		} point;

		const float radius = 10.0;
		const int num_points = 57600;
		std::vector<std::uint8_t> data;
		float theta = 0;
		float phi = -M_PI/2;
		for (int i = 0; i < std::sqrt(num_points); ++i) {
			phi += M_PI/std::sqrt(num_points);
			const float z = std::cos(phi) * radius;
			const float k = std::sin(phi) * radius;
			theta = 0;
			for (int j = 0; j < std::sqrt(num_points); ++j) {
				theta += M_PI/std::sqrt(num_points);
				const float x = std::cos(theta) * k;
				const float y = std::sin(theta) * k;
				const float intensity = z;

				point.val = x;
				for (const std::uint8_t byte:point.bytes) {
					data.push_back(byte);
				}
				point.val = y;
				for (const std::uint8_t byte:point.bytes) {
					data.push_back(byte);
				}
				point.val = z;
				for (const std::uint8_t byte:point.bytes) {
					data.push_back(byte);
				}
				point.val = intensity;
				for (const std::uint8_t byte:point.bytes) {
					data.push_back(byte);
				}
				point.val = 0.0;
				for (const std::uint8_t byte:point.bytes) {
					data.push_back(byte);
				}
			}
		}
		ptcloud_.data = data;

		RCLCPP_INFO(this->get_logger(), "Done");
		timer_ = this->create_wall_timer(1000ms, [this] { publishLargeMsg(); });

	}
private:
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr path_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	sensor_msgs::msg::PointCloud2 ptcloud_;

	void publishLargeMsg() {
		ptcloud_.header.stamp = this->get_clock()->now();
		path_publisher_->publish(ptcloud_);
		RCLCPP_INFO(this->get_logger(), "Message published");
	}
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LargeMsgGenerator>());
	rclcpp::shutdown();
	return 0;
}