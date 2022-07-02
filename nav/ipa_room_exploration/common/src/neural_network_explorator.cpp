#include <ipa_room_exploration/neural_network_explorator.h>

// Default constructor
NeuralNetworkExplorator::NeuralNetworkExplorator()
{
	step_size_ = 0.008; // 0.008
	A_ = 17; // 17
	B_ = 5; // 5
	D_ = 7; // 7
	E_ = 80; // E >> B, 80
	mu_ = 1.03; // 1.03
	delta_theta_weight_ = 0.15; // 0.15
}

// Function that calculates an exploration path trough the given map s.t. everything has been covered by the robot-footprint
// or the field of view. The algorithm is programmed after
void NeuralNetworkExplorator::getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path, const float map_resolution,
					 const cv::Point starting_position, const cv::Point2d map_origin, const double grid_spacing_in_pixel,
					 const bool plan_for_footprint, const Eigen::Matrix<float, 2, 1> robot_to_fov_vector, bool show_path_computation)
{
	const int grid_spacing_as_int = std::floor(grid_spacing_in_pixel);
	const int half_grid_spacing_as_int = std::floor(grid_spacing_in_pixel*0.5);

	// *********************** I. Find the main directions of the map and rotate it in this manner. ***********************
	cv::Mat R;
	cv::Rect bbox;
	cv::Mat rotated_room_map;
	RoomRotator room_rotation;
	room_rotation.computeRoomRotationMatrix(room_map, R, bbox, map_resolution);
	room_rotation.rotateRoom(room_map, rotated_room_map, R, bbox);

	// compute min/max room coordinates
	cv::Point min_room(1000000, 1000000), max_room(0, 0);
	for (int v=0; v<rotated_room_map.rows; ++v)
	{
		for (int u=0; u<rotated_room_map.cols; ++u)
		{
			if (rotated_room_map.at<uchar>(v,u)==255)
			{
				min_room.x = std::min(min_room.x, u);
				min_room.y = std::min(min_room.y, v);
				max_room.x = std::max(max_room.x, u);
				max_room.y = std::max(max_room.y, v);
			}
		}
	}
	cv::Mat inflated_rotated_room_map;
	cv::erode(rotated_room_map, inflated_rotated_room_map, cv::Mat(), cv::Point(-1, -1), half_grid_spacing_as_int);

	// ****************** II. Create the neural network ******************
	// reset previously computed neurons
	neurons_.clear();

	// go trough the map and create the neurons
	int number_of_free_neurons = 0;
	for(int y=min_room.y+half_grid_spacing_as_int; y<max_room.y; y+=grid_spacing_as_int)
	{
		// for the current row create a new set of neurons to span the network over time
		std::vector<Neuron> current_network_row;
		for(int x=min_room.x+half_grid_spacing_as_int; x<max_room.x; x+=grid_spacing_as_int)
		{
			// create free neuron
			cv::Point cell_center(x,y);
			if (GridGenerator::completeCellTest(inflated_rotated_room_map, cell_center, grid_spacing_as_int) == true)
			//if(rotated_room_map.at<uchar>(y,x) == 255)
			{
				Neuron current_neuron(cell_center, A_, B_, D_, E_, mu_, step_size_, false);
				current_network_row.push_back(current_neuron);
				++number_of_free_neurons;
			}
			else // obstacle neuron
			{
				Neuron current_neuron(cell_center, A_, B_, D_, E_, mu_, step_size_, true);
				current_network_row.push_back(current_neuron);
			}
		}

		// insert the current row into the network
		neurons_.push_back(current_network_row);
	}

	// todo: do not limit to direct neighbors but cycle through all neurons for finding the best next
	// go trough the found neurons and get the direct neighbors of each
	for(size_t row=0; row<neurons_.size(); ++row)
	{
		for(size_t column=0; column<neurons_[row].size(); ++column)
		{
			for(int dy=-1; dy<=1; ++dy)
			{
				// don't exceed the current row
				if(row+dy < 0 || row+dy >= neurons_.size())
					continue;

				// get the neighbors left from the current neuron
				if(column > 0)
					neurons_[row][column].addNeighbor(&neurons_[row+dy][column-1]);

				// get the neurons on the same column as the current neuron
				if(dy != 0)
					neurons_[row][column].addNeighbor(&neurons_[row+dy][column]);

				// get the neurons right from the current neuron
				if(column < neurons_[row].size()-1)
					neurons_[row][column].addNeighbor(&neurons_[row+dy][column+1]);
			}
		}
	}

//	testing
//	cv::Mat black_map = cv::Mat(rotated_room_map.rows, rotated_room_map.cols, rotated_room_map.type(), cv::Scalar(0));
//	for(size_t i=0; i<neurons_.size(); ++i)
//	{
//		for(size_t j=0; j<neurons_[i].size(); ++j)
//		{
//			std::vector<Neuron*> neighbors;
//			neurons_[i][j].getNeighbors(neighbors);
//			for(size_t k=0; k<neighbors.size(); ++k)
//			{
//				cv::circle(black_map, neighbors[k]->getPosition(), 2, cv::Scalar(127), CV_FILLED);
//			}
//		}
//	}
//	cv::imshow("neighbors", black_map);
//	cv::waitKey();

	// ****************** III. Find the coverage path ******************
	// mark the first non-obstacle neuron as starting node
	Neuron* starting_neuron = 0;
	bool found = false;
	for(size_t row=0; row<neurons_.size(); ++row)
	{
		for(size_t column=0; column<neurons_[row].size(); ++column)
		{
			if(neurons_[row][column].isObstacle() == false && found == false)
			{
				found = true;
				starting_neuron = &neurons_[row][column];
				break;
			}
		}

		if(found == true)
			break;
	}
	if (starting_neuron==0)
	{
		std::cout << "Warning: there are no accessible points in this room." << std::endl;
		return;
	}
	starting_neuron->markAsVisited();

	// initial updates of the states to mark obstacles and unvisited free neurons as such
	for(size_t init=1; init<=100; ++init)
	{
		for(size_t row=0; row<neurons_.size(); ++row)
			for(size_t column=0; column<neurons_[row].size(); ++column)
				neurons_[row][column].saveState();
		for(size_t row=0; row<neurons_.size(); ++row)
			for(size_t column=0; column<neurons_[row].size(); ++column)
				neurons_[row][column].updateState();
	}

//	testing
//	cv::Mat black_map = cv::Mat(rotated_room_map.rows, rotated_room_map.cols, CV_32F, cv::Scalar(0));
//	for(size_t row=0; row<neurons_.size(); ++row)
//	{
//		for(size_t column=0; column<neurons_[row].size(); ++column)
//		{
//			std::cout << neurons_[row][column].getState(false) << " ";
//			black_map.at<float>(row*fitting_radius_as_int, column*fitting_radius_as_int) = (float) 1000.0*neurons_[row][column].getState(false);
//		}
//		std::cout << std::endl;
//	}
//	std::cout << std::endl;
//	cv::namedWindow("states", cv::WINDOW_NORMAL);
//	cv::imshow("states", black_map);
//	cv::resizeWindow("states", 600, 600);
//	cv::waitKey();

	// iteratively choose the next neuron until all neurons have been visited or the algorithm is stuck in a
	// limit cycle like path (i.e. the same neurons get visited over and over)
	int visited_neurons = 1;
	bool stuck_in_cycle = false;
	std::vector<cv::Point> fov_coverage_path;
	fov_coverage_path.push_back(cv::Point(starting_neuron->getPosition().x, starting_neuron->getPosition().y));
	double previous_traveling_angle = 0.0; // save the travel direction to the current neuron to determine the next neuron
	cv::Mat black_map = rotated_room_map.clone();
	Neuron* previous_neuron = starting_neuron;
	int loop_counter = 0;
	do
	{
		//std::cout << "Point: " << previous_neuron->getPosition() << std::endl;
		++loop_counter;

		// get the current neighbors and choose the next out of them
		std::vector<Neuron*> neighbors;
		previous_neuron->getNeighbors(neighbors);
		Neuron* next_neuron = 0;

		// go through the neighbors and find the next one
		double max_value = -1e10, travel_angle = 0.0, best_angle = 0.0;
		for(size_t neighbor=0; neighbor<neighbors.size(); ++neighbor)
		{
			// get travel angle to this neuron
			travel_angle = std::atan2(neighbors[neighbor]->getPosition().y-previous_neuron->getPosition().y, neighbors[neighbor]->getPosition().x-previous_neuron->getPosition().x);

			// compute penalizing function y_j
			double diff_angle = travel_angle - previous_traveling_angle;
			while (diff_angle < -PI)
				diff_angle += 2*PI;
			while (diff_angle > PI)
				diff_angle -= 2*PI;
			double y = 1 - (std::abs(diff_angle)/PI);

			// compute transition function value
			//std::cout << " Neighbor: " << neighbors[neighbor]->getPosition() << "   " << neighbors[neighbor]->getState(false) << ", " << delta_theta_weight_ * y << std::endl;
			double trans_fct_value = neighbors[neighbor]->getState(false) + delta_theta_weight_ * y;

			// check if neighbor is next neuron to be visited
			if(trans_fct_value > max_value && rotated_room_map.at<uchar>(neighbors[neighbor]->getPosition()) != 0)
			{
				max_value = trans_fct_value;
				next_neuron = neighbors[neighbor];
				best_angle = travel_angle;
			}
		}
		// catch errors
		if (next_neuron == 0)
		{
			if (loop_counter <= 20)
				continue;
			else
				break;
		}
		loop_counter = 0;

		// if the next neuron was previously uncleaned, increase number of visited neurons
		if(next_neuron->visitedNeuron() == false)
			++visited_neurons;

		// mark next neuron as visited
		next_neuron->markAsVisited();
		previous_traveling_angle = best_angle;

		// add neuron to path
		const cv::Point current_pose(next_neuron->getPosition().x, next_neuron->getPosition().y);
		fov_coverage_path.push_back(current_pose);

		// check the fov path for a limit cycle by searching the path for the next neuron, if it occurs too often
		// and the previous/following neuron is always the same the algorithm probably is stuck in a cycle
		int number_of_neuron_in_path = 0;
		for(std::vector<cv::Point>::iterator pose=fov_coverage_path.begin(); pose!=fov_coverage_path.end(); ++pose)
			if(*pose==current_pose)
				++number_of_neuron_in_path;

		if(number_of_neuron_in_path >= 20)
		{
			// check number of previous neuron
			cv::Point previous_pose = fov_coverage_path[fov_coverage_path.size()-2];
			int number_of_previous_neuron_in_path = 0;
			for(std::vector<cv::Point>::iterator pose=fov_coverage_path.begin()+1; pose!=fov_coverage_path.end()-1; ++pose)
			{
				// check if the the previous pose always has the current pose as neighbor
				if(*pose==previous_pose)
				{
					if(*(pose+1)==current_pose)
						++number_of_previous_neuron_in_path;
					else if(*(pose-1)==current_pose)
						++number_of_previous_neuron_in_path;
				}
			}

			//if(number_of_previous_neuron_in_path >= number_of_neuron_in_path)
			if(number_of_previous_neuron_in_path >= 20)
			{
				std::cout << "Warning: the algorithm is probably stuck in a cycle. Aborting." << std::endl;
				stuck_in_cycle = true;
			}
		}

		// update the states of the network
		for (int i=0; i<100; ++i)
		{
			for(size_t row=0; row<neurons_.size(); ++row)
				for(size_t column=0; column<neurons_[row].size(); ++column)
					neurons_[row][column].saveState();
			for(size_t row=0; row<neurons_.size(); ++row)
				for(size_t column=0; column<neurons_[row].size(); ++column)
					neurons_[row][column].updateState();
		}

//		printing of the path computation
		if(show_path_computation == true)
		{
			cv::circle(black_map, next_neuron->getPosition(), 2, cv::Scalar((visited_neurons*5)%250), CV_FILLED);
			cv::line(black_map, previous_neuron->getPosition(), next_neuron->getPosition(), cv::Scalar(128), 1);
			cv::imshow("next_neuron", black_map);
			cv::waitKey();
		}

		// save neuron that has been visited
		previous_neuron = next_neuron;
	} while (visited_neurons < number_of_free_neurons && stuck_in_cycle == false); //TODO: test terminal condition

	// transform the calculated path back to the originally rotated map
	std::vector<geometry_msgs::Pose2D> fov_poses;
	std::vector<cv::Point2f> fov_coverage_path_2f(fov_coverage_path.size());
	for (size_t i=0; i<fov_coverage_path.size(); ++i)
		fov_coverage_path_2f[i] = cv::Point2f(fov_coverage_path[i].x, fov_coverage_path[i].y);
	room_rotation.transformPathBackToOriginalRotation(fov_coverage_path_2f, fov_poses, R);

//	// go trough the found fov-path and compute the angles of the poses s.t. it points to the next pose that should be visited
//	for(unsigned int point_index=0; point_index<fov_path.size(); ++point_index)
//	{
//		// get the vector from the current point to the next point
//		geometry_msgs::Pose2D current_point = fov_path[point_index];
//		geometry_msgs::Pose2D next_point = fov_path[(point_index+1)%(fov_path.size())];
//
//		float angle = std::atan2(next_point.y - current_point.y, next_point.x - current_point.x);
//
//		// save the found angle
//		fov_path[point_index].theta = angle;
//	}

	// if the path should be planned for the robot footprint create the path and return here
	if(plan_for_footprint == true)
	{
		for(std::vector<geometry_msgs::Pose2D>::iterator pose=fov_poses.begin(); pose != fov_poses.end(); ++pose)
		{
			geometry_msgs::Pose2D current_pose;
			current_pose.x = (pose->x * map_resolution) + map_origin.x;
			current_pose.y = (pose->y * map_resolution) + map_origin.y;
			current_pose.theta = pose->theta;
			path.push_back(current_pose);
		}
		return;
	}

	// ****************** III. Map the found fov path to the robot path ******************
	// go trough all computed fov poses and compute the corresponding robot pose
	ROS_INFO("Starting to map from field of view pose to robot pose");
	cv::Point robot_starting_position = (fov_poses.size()>0 ? cv::Point(fov_poses[0].x, fov_poses[0].y) : starting_position);
	cv::Mat inflated_room_map;
	cv::erode(room_map, inflated_room_map, cv::Mat(), cv::Point(-1, -1), half_grid_spacing_as_int);
	mapPath(inflated_room_map, path, fov_poses, robot_to_fov_vector, map_resolution, map_origin, robot_starting_position);
}
