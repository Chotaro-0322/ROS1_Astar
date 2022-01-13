#include <ros/ros.h>
#include <eigen3/Eigen/Core>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <iostream>
#include <algorithm>

class Itolab_Astar{
public:
    ros::NodeHandle nh_;
    Itolab_Astar();
    void run();
    void map_callback(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void search_around_openspace(std::vector<std::vector<float>> current_pose);
    std::vector<float> cal_cost_and_minimumnode(void);
    std::vector<std::vector<float>> cal_next_route(std::vector<float> minimum_index);
    std::vector<std::vector<float>> feed_back(std::vector<float> start_node, std::vector<float> goal_node);
private:
    Eigen::MatrixXf map_mat;
    std::vector<Eigen::MatrixXf> parent_eachnode;
    Eigen::MatrixXf route_map;
    std::vector<std::vector<float>> obstacle_info;
    std::vector<float> start;
    std::vector<float> goal;
    std::vector<std::vector<float>> current_node;
    std::vector<std::vector<float>> route_vec;
    
    int moving_count;
    int search_limit;

    std::vector<std::vector<float>> opened_space_list;

    ros::Subscriber map_subscriber;
    ros::Subscriber start_position;
    ros::Subscriber current_waypoint;
    ros::Publisher avoid_waypoint;

    int map_h = 0;
    int map_w = 0;

    Eigen::MatrixXf c_cost_map;
    Eigen::MatrixXf h_cost_map;
    Eigen::MatrixXf score_map;
};

Itolab_Astar::Itolab_Astar(){
    std::cout << "construct" << std::endl;
    start = {0, 0};
    goal = {0, 49};
    current_node = {start};
    moving_count = 1;
    search_limit = 30;
    map_subscriber = nh_.subscribe("/potential_map", 1, &Itolab_Astar::map_callback, this);

    std::cout << "start : " << std::endl;
    for(int i=0; i < start.size(); i++) std::cout << start[0] << std::endl;
    std::cout << "goal : " << std::endl;
    for(int i=0; i < goal.size(); i++) std::cout << goal[0] << std::endl;
    std::cout << "current_node" << std::endl;
    for(int child=0; child < current_node.size(); child++){
        for(int j=0; j < current_node[child].size(); j++){
            std::cout << current_node[child][j] << " ";
        }std::cout << std::endl;
    }
    std::cout << "moving_count : " << moving_count << std::endl;
    std::cout << "search_limit : " << search_limit << std::endl;
}

void Itolab_Astar::map_callback(const std_msgs::Float32MultiArray::ConstPtr& msg){
    // std::cout << "hello " << std::endl;
    std::vector<float> msg_vec = msg->data;

    map_h = msg->layout.dim[0].size;
    map_w = msg->layout.dim[1].size;

    // Eigen::Map<Eigen::Matrix<float, -1, -1, Eigen::RowMajor>> map(msg_vec.data(), map_h, map_w);
    Eigen::Map<Eigen::Matrix<float, -1, -1>> map(msg_vec.data(), map_h, map_w);
    map_mat = map;
    c_cost_map = Eigen::MatrixXf::Constant(map_h, map_w, -1);
    h_cost_map = Eigen::MatrixXf::Constant(map_h, map_w, -1);
    score_map = Eigen::MatrixXf::Constant(map_h, map_w, -1);
    score_map(start[0], start[1]) = std::max(goal[0], goal[1]);
    parent_eachnode = {Eigen::MatrixXf::Constant(map_h, map_w, 0), Eigen::MatrixXf::Constant(map_h, map_w, 0)};
    route_map = map_mat;
    // std::cout << map << std::endl;
    std::cout << "Map Processing ..." << std::endl;
}

void Itolab_Astar::search_around_openspace(std::vector<std::vector<float>> current_node){
    // std::cout << "score_map : \n" << score_map << std::endl;
    // std::cout << "c_map : \n" << c_cost_map << std::endl;
    // std::cout << "h_map : \n" << h_cost_map << std::endl;
    for(int pose_i=0; pose_i < current_node.size(); pose_i++){
        for(int y=-1; y < 2; y++){
            for(int x=-1; x < 2; x++){
                // current_node 
                // for (int vec = 0; vec < current_node.size(); vec++){
                //     for (int i = 0; i < current_node[vec].size(); i++)
                //         std::cout << current_node[vec][i] << " ";
                // }std::cout << std::endl;
                if ((x == 0 && y == 0) == 0){
                    // std::cout << "(x, y) : " << x << " " << y << std::endl;
                    // std::cout << "current_node[pose_i][0] + y = " << current_node[pose_i][0] + y << std::endl;
                    // std::cout << "current_node[pose_i][1] + x = " << current_node[pose_i][1] + x << std::endl;
                    // std::cout << "current_node[pose_i][0] + y = " << current_node[pose_i][0] + y << std::endl;
                    // std::cout << "current_node[pose_i][1] + x = " << current_node[pose_i][1] + x << std::endl;
                    if(((current_node[pose_i][0] + y) >= 0) && ((current_node[pose_i][1] + x) >= 0) && ((current_node[pose_i][0] + y) < map_h) && ((current_node[pose_i][1] + x) < map_w)){
                        // std::cout << "{y, x} : " << current_node[pose_i][0] + y << ", " << current_node[pose_i][1] + x << " | score_value : " << score_map(current_node[pose_i][0] + y, current_node[pose_i][1] + x) << std::endl;
                        if ((map_mat(current_node[pose_i][0] + y, current_node[pose_i][1] + x) != 1) && (score_map(current_node[pose_i][0] + y, current_node[pose_i][1] + x) == -1)){
                            // std::cout << "before tmp_vec" << std::endl;
                            std::vector<float> tmp_vec = {current_node[pose_i][0] + y, current_node[pose_i][1] + x};
                            // for (int i = 0; i < tmp_vec.size(); i++){
                            //     std::cout << tmp_vec.at(i) << " ";
                            // }std::cout << std::endl;
                            if (std::count(opened_space_list.begin(), opened_space_list.end(), tmp_vec) == 0){
                                opened_space_list.push_back(tmp_vec);
                            }

                            parent_eachnode[0](current_node[pose_i][0] + y, current_node[pose_i][1] + x) = current_node[pose_i][0];
                            parent_eachnode[1](current_node[pose_i][0] + y, current_node[pose_i][1] + x) = current_node[pose_i][1];
                        }
                    }
                }
            }
        }
    }
}

std::vector<float> Itolab_Astar::cal_cost_and_minimumnode(){
    std::vector<float> score_list;
    float score;
    for(int space_i = 0; space_i < opened_space_list.size(); space_i++){
        if (score_map(opened_space_list[space_i][0], opened_space_list[space_i][1]) == -1){
            // std::cout << "space : " <<  opened_space_list[space_i][0] << " " << opened_space_list[space_i][1] << std::endl;
            c_cost_map(opened_space_list[space_i][0], opened_space_list[space_i][1]) = map_mat(opened_space_list[space_i][0], opened_space_list[space_i][1]);
            // std::cout << "c_cost : " << moving_count << "\n" << std::endl;
            h_cost_map(opened_space_list[space_i][0], opened_space_list[space_i][1]) = std::max(std::abs(goal[0] - opened_space_list[space_i][0]), std::abs(goal[1] - opened_space_list[space_i][1]));
            // std::cout << "h_cost : " << std::max(goal[0] - opened_space_list[space_i][0], goal[1] - opened_space_list[space_i][1]) << std::endl;
            // std::cout << "{y, x} : " << opened_space_list[space_i][0] << ", " << opened_space_list[space_i][1] \
                << " | {y_h, x_h} : " << std::abs(goal[0] - opened_space_list[space_i][0]) << ", " << std::abs(goal[1] - opened_space_list[space_i][1]) \
                << " | H_value : " << std::max(goal[0] - opened_space_list[space_i][0], goal[1] - opened_space_list[space_i][1]) << std::endl;
            score = c_cost_map(opened_space_list[space_i][0], opened_space_list[space_i][1]) + h_cost_map(opened_space_list[space_i][0], opened_space_list[space_i][1]);
            score_map(opened_space_list[space_i][0], opened_space_list[space_i][1]) = score;
            // std::cout << score_map << "\n" << std::endl;
            map_mat(opened_space_list[space_i][0], opened_space_list[space_i][1]) = 2;
        }else{
            score = score_map(opened_space_list[space_i][0], opened_space_list[space_i][1]);
        }
        score_list.push_back(score);
    }

    std::vector<float> minimum_index;
    float minimum_score = *std::min_element(score_list.begin(), score_list.end());
    for (int i = 0; i < score_list.size(); i++){
        if (score_list[i] == minimum_score){
            minimum_index.push_back(i);
        }
    }

    return minimum_index;
}

std::vector<std::vector<float>> Itolab_Astar::cal_next_route(std::vector<float> minimum_index){
    std::vector<std::vector<float>> next_node;
    // std::cout << "minimum_index : " << std::endl;
    // for(int i = 0; i < minimum_index.size(); i++){
        // std::cout << minimum_index[i] << ", ";
    // }std::cout << std::endl;

    for (int i = 0; i < minimum_index.size(); i++){
        next_node.push_back(opened_space_list[minimum_index[i]]);
    }

    for (int i = minimum_index.size() - 1 ; i >= 0; i--){
        // std::cout << "erase index : " << minimum_index[i] << std::endl;
        opened_space_list.erase(opened_space_list.begin() + minimum_index[i]);
    }

    // std::cout << "erased_opened_space : " << std::endl;
    // for (int vec = 0; vec < opened_space_list.size(); vec++){
        // std::cout << "{" << opened_space_list.at(vec).at(0) << ", " << opened_space_list.at(vec).at(1) << "}, " << " ";
    // }std::cout << std::endl;

    return next_node;
}

std::vector<std::vector<float>> Itolab_Astar::feed_back(std::vector<float> start_node, std::vector<float> goal_node){
    std::vector<std::vector<float>> waypoint = {goal_node};
    // std::cout << "parent_node y : " << parent_eachnode.at(0) << std::endl;
    // std::cout << "parent_node x : " << parent_eachnode.at(1) << std::endl;
    while(1){ 
        std::vector<float> parent_node = {parent_eachnode.at(0)(goal_node[0], goal_node[1]), parent_eachnode.at(1)(goal_node[0], goal_node[1])};

        waypoint.push_back(parent_node);

        goal_node = parent_node;
        
        // std::cout << "goal (x, y) : " << goal_node.at(0) << ", " << goal_node.at(0) << std::endl;

        if(goal_node[0] == start_node[0] && goal_node[1] == start_node[1]){
            return waypoint;
        }
    }
}

void Itolab_Astar::run(){
    ros::Rate loop_rate(10);
    while(ros::ok()){
        // std::cout << "map_mat rows : " << map_mat.rows() << std::endl;
        if (map_mat.rows() != 0){
            // std::cout << map_mat << std::endl;
            Itolab_Astar::search_around_openspace(current_node);
            // std::cout << "opened_space : " << std::endl;
            // for (int vec = 0; vec < opened_space_list.size(); vec++){
                    // std::cout << "{" << opened_space_list.at(vec).at(0) << ", " << opened_space_list.at(vec).at(1) << "}, " << " ";
            // }std::cout << std::endl;
            
            std::vector<float> minimum_index = Itolab_Astar::cal_cost_and_minimumnode();
            current_node = Itolab_Astar::cal_next_route(minimum_index);
            // std::cout << "current_node" << std::endl;
            // for (int vec = 0; vec < current_node.size(); vec++){
                    // std::cout << "{" << current_node.at(vec).at(0) << ", " << current_node.at(vec).at(1) << "}, " << " ";
            // }std::cout << std::endl;
            // std::cout << "current_node" << std::endl;
            // for (int vec = 0; vec < current_node.size(); vec++){
                // for (int i = 0; i < current_node[vec].size(); i++){
                    // std::cout << current_node[vec][i] << " ";
                // }std::cout << std::endl;
            // }

            moving_count += 1;

            if(score_map(goal.at(0), goal.at(1)) != -1){
                break;
            }
        }
        ros::spinOnce();
		loop_rate.sleep();
    }
    route_vec = feed_back(start, goal);

    for(int i = 0; i < route_vec.size(); i++){
        // std::cout << "{y, x} : " << "{" << route_vec.at(i).at(0) << ", " << route_vec.at(i).at(1) << "}, ";
        route_map(route_vec.at(i).at(0), route_vec.at(i).at(1)) = 7;
    }std::cout << std::endl;

    std::cout << "route_result : " << std::endl;
    std::cout << route_map << std::endl;
}

int main(int argc, char** argv){
    ros::init(argc, argv, "itolab_astar");
    Itolab_Astar Itolabastar;
    Itolabastar.run();

	// ros::spin();

    return 0;
}