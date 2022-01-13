#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>

class TestPotential{
public:
    TestPotential();
    void run();
    ros::NodeHandle nh_;
    ros::Publisher pot_publisher;
    
    // std::vector<float> publish_vec;
    Eigen::MatrixXf potential_map;
    std_msgs::Float32MultiArray array;
};

TestPotential::TestPotential(){
    pot_publisher = nh_.advertise<std_msgs::Float32MultiArray>("potential_map", 1);
    potential_map = Eigen::MatrixXf::Zero(50, 50);

    potential_map.block(0, 3, 20, 3) = Eigen::MatrixXf::Constant(20, 3, 1);
    potential_map.block(6, 8, 10, 5) = Eigen::MatrixXf::Constant(10, 5, 1);
    potential_map.block(15, 8, 10, 10) = Eigen::MatrixXf::Constant(10, 10, 1);
    potential_map.block(0, 24, 35, 1) = Eigen::MatrixXf::Constant(35, 1, 1);
    potential_map.block(5, 30, 10, 3) = Eigen::MatrixXf::Constant(10, 3, 1);
    potential_map.block(10, 40, 5, 1) = Eigen::MatrixXf::Constant(5, 1, 1);
    potential_map.block(40, 40, 7, 7) = Eigen::MatrixXf::Constant(7, 7, 1);
    potential_map.block(0, 40, 45, 7) = Eigen::MatrixXf::Constant(45, 7, 1);
    potential_map.block(45, 0, 3, 7) = Eigen::MatrixXf::Constant(3, 7, 1);
    potential_map.block(40, 40, 7, 7) = Eigen::MatrixXf::Constant(7, 7, 1);

    // std::cout << potential_map << std::endl;
    array.data.clear();
    array.layout.dim.push_back(std_msgs::MultiArrayDimension());
    array.layout.dim.push_back(std_msgs::MultiArrayDimension());

    array.layout.dim[0].size = potential_map.rows();
    array.layout.dim[0].stride = potential_map.cols();
    array.layout.dim[0].label = "y";
    array.layout.dim[1].size = potential_map.cols();
    array.layout.dim[1].stride = 1;
    array.layout.dim[1].label = "x";
    std::vector<float> publish_vec(potential_map.data(), potential_map.data() + potential_map.rows() * potential_map.cols());
    array.data = publish_vec;
}

void TestPotential::run(){
    std::cout << "wait pot_publisher" << std::endl;
    while (pot_publisher.getNumSubscribers() < 1){
        std::cout << "wait pot_publisher" << std::endl;
    }
    // while(ros::ok() == 1){
    pot_publisher.publish(array);
    // }
}
    

int main(int argc, char** argv){
    ros::init(argc, argv, "test_potential");
    TestPotential Testpotential;
    // sleep(3);
    Testpotential.run();

	//ros::spin();

    return 0;
}