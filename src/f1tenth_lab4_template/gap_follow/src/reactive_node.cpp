#include "rclcpp/rclcpp.hpp"
#include <string>
#include <vector>
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include <cmath>
/// CHECK: include needed ROS msg type headers and libraries

class ReactiveFollowGap : public rclcpp::Node {
// Implement Reactive Follow Gap on the car
// This is just a template, you are free to implement your own node!

public:
    ReactiveFollowGap() : Node("reactive_node")
    {
        /// TODO: create ROS subscribers and publishers
        scan_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        lidarscan_topic, 10, std::bind(&ReactiveFollowGap::lidar_callback, this, std::placeholders::_1));
        
        drive_publisher_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(
        drive_topic, 10);
           
    }

private:
    std::string lidarscan_topic = "/scan";
    std::string drive_topic = "/drive";
    float angle_increment2 = 0.0f;
    /// TODO: create ROS subscribers and publishers
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_subscriber_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_publisher_;
    
    int preprocess_lidar(std::vector<float>& range)
    {   
    
    float prev_val , next_val;
    for(int i = 0; i < range.size()-1; i++){
    prev_val , next_val = range[i], range[i+1];
    if(std::abs(next_val-prev_val) > 1.8){
       float small = std::min(prev_val, next_val);
       int index_num = (int)(0.4/ (small*angle_increment2));
       if(i < index_num ){
        for(int j = 0; j < i+index_num; j++){
            range[i+j] = small;
        }}
        else if(i > range.size()- index_num){
            for(int j = (int)(i - index_num); j <(int)(range.size()- (index_num+1));j++){
                range[i+j] = small ;
            }

        }
                        
       else{
        for(int j = -index_num ; j < index_num; j++ ){
            range[i+j] = small ;

        }
       }
        
    }
}

auto it = std::min_element(range.begin(),range.end());
        int index = std::distance(range.begin(), it);
        int i = 1;
        int j = -1;
        while(index+i < range.size()){
            if(range[index+i] < 1.0f){
                range[index+i] = 0 ;
                i++;
            }else{
                break;
            }
        }
        while( -1 < index+j ){
            if(range[index+j]<1.0f){
                range[index+j] = 0 ;
                j--;
            }else{
                break;
            }
        }





       std::pair<int, int> gap = find_max_gap(range);
        int min_index = gap.first;
        int max_index = gap.second;
        int target_index = (min_index + max_index )/2;

        // Preprocess the LiDAR scan array. Expert implementation includes:
        // 1.Setting each value to the mean over some window
        // 2.Rejecting high values (eg. > 3m)
        return target_index;
    }

    std::pair<int,int> find_max_gap(const std::vector<float>& ranges){
        float threshold = 1.2f;
        float over = 5.4f;
        // 이게 강의자료에 의하면 고정되어 있을 필요가 없음
    int start,max_start;
    int current_length, max_length;

    start = 0;
    max_start = 0;
    current_length = 0;
    max_length = 0;

    for (int i = 0; i < ranges.size(); i++){
    if (ranges[i] > threshold && ranges[i] < over ){
        if(current_length == 0){
            start = i ;
        }
        current_length++;

    }else{
    if(current_length > max_length){
        max_length = current_length;
        max_start = start;

    }
    current_length = 0;
}
} 

return {max_start, max_start + max_length -1};


    }

    // void find_best_point(float* ranges, int* indice)
    // {   
    //     // Start_i & end_i are start and end indicies of max-gap range, respectively
    //     // Return index of best point in ranges
	//     // Naive: Choose the furthest point within ranges and go there
    //     return;
    // }


    void lidar_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) 
    {   
        // Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message

        /// TODO:
        // Find closest point to LiDAR
        std::vector<float> range = scan_msg->ranges;
        angle_increment2 = scan_msg->angle_increment;
        int target_index = preprocess_lidar(range);
        float target_angle = scan_msg->angle_min + target_index * scan_msg->angle_increment;
        ackermann_msgs::msg::AckermannDriveStamped drive_msg;
        drive_msg.drive.steering_angle = target_angle;
        float target_angle2 = std::abs(target_angle);
        if(target_angle2 > 0.34f){
            drive_msg.drive.speed = 0.2f;
           

        }else if(target_angle2 > 0.19f){

        
        
        drive_msg.drive.speed = 0.4f;
        }else{
            drive_msg.drive.speed = 0.8f;
        }

        drive_publisher_->publish(drive_msg);
        std::cout<< drive_msg.drive.steering_angle <<std::endl;

    }



};
int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ReactiveFollowGap>());
    rclcpp::shutdown();
    return 0;
}







// 연속된 구간 찾기
// 처음부터 여기까지 도움 없이 build 해보기 ---> 이거할줄 알아야지


int start, max_start;
int current_length, max_length;


