//
// Created by mbo on 02/03/23.
//
#include <chrono>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

using namespace std::chrono_literals;

template <typename T>
union word_union {
	T val;
	std::uint8_t bytes[sizeof(T)];
};

class LargeMsgGeneratorTalker : public rclcpp::Node {
public:
	LargeMsgGeneratorTalker():
	Node("large_msg_generator_talker") {
		this->declare_parameter("num_points", 500000);
		this->declare_parameter("rate", 10);
		const int num_points = this->get_parameter("num_points").get_parameter_value().get<int>();
		const int rate = this->get_parameter("rate").get_parameter_value().get<int>();
		const int timer_period = std::ceil(1000/rate);
		const int num_points_root = std::floor(std::sqrt(num_points));
		path_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("large_msg", 2);
		ptcloud_ = sensor_msgs::msg::PointCloud2();
		ptcloud_.header.frame_id = "map";
		ptcloud_.height = 1;
		ptcloud_.width = num_points_root*num_points_root;
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

		RCLCPP_INFO(this->get_logger(), "Generation data with %d points", num_points);

		const float radius = 10.0;
		const float time = 0.0;
		float phi = -M_PI/2;
		std::vector<std::uint8_t> data;
		float theta;
		for (int i = 0; i < num_points_root; ++i) {
			phi += M_PI/num_points_root;
			const float z = std::cos(phi) * radius;
			const float intensity = z;
			const float k = std::sin(phi) * radius;
			theta = 0;
			for (int j = 0; j < num_points_root; ++j) {
				theta += M_PI/num_points_root;
				const float x = std::cos(theta) * k;
				const float y = std::sin(theta) * k;

				push_data(data, x);
				push_data(data, y);
				push_data(data, z);
				push_data(data, intensity);
				push_data(data, time);
			}
		}
		ptcloud_.data = data;

		RCLCPP_INFO(this->get_logger(), "Done");
		RCLCPP_INFO(this->get_logger(), "Publishing data each %d ms", timer_period);
		timer_ = this->create_wall_timer(std::chrono::milliseconds{timer_period}, [this] { publishLargeMsg(); });

	}
private:
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr path_publisher_;
	rclcpp::TimerBase::SharedPtr timer_;
	sensor_msgs::msg::PointCloud2 ptcloud_;

	void publishLargeMsg() {
		ptcloud_.header.stamp = this->get_clock()->now();
		path_publisher_->publish(ptcloud_);
		RCLCPP_INFO(this->get_logger(), "Message published with time: %d.%d", ptcloud_.header.stamp.sec, ptcloud_.header.stamp.nanosec);
	}

	template<typename T>
	void push_data(std::vector<std::uint8_t> &data, T value) {
		word_union<T> point{};
		point.val = value;
		for (const std::uint8_t byte:point.bytes) {
			data.push_back(byte);
		}
	}
};

int main(int argc, char **argv) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<LargeMsgGeneratorTalker>());
	rclcpp::shutdown();
	return 0;
}