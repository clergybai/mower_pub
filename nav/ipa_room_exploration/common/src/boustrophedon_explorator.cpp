#include <ipa_room_exploration/boustrophedon_explorator.h>

//#define DEBUG_VISUALIZATION

// Constructor
BoustrophedonExplorer::BoustrophedonExplorer()
{

}

// This method takes the given map and separates it into several cells. Each cell is obstacle free and so allows an
// easier path planning. For each cell then a boustrophedon path is planned, which goes up, down and parallel to the
// upper and lower boundaries of the cell, see the referenced paper for details. This function does the following steps:
void BoustrophedonExplorer::getExplorationPath(const cv::Mat& room_map, std::vector<geometry_msgs::Pose2D>& path,
		const float map_resolution, const cv::Point starting_position, const cv::Point2d map_origin,
		const double grid_spacing_in_pixel, const double grid_obstacle_offset, const double path_eps, const int cell_visiting_order,
		const bool plan_for_footprint, const Eigen::Matrix<float, 2, 1> robot_to_fov_vector, const double min_cell_area, const int max_deviation_from_track)
{
	ROS_INFO("Planning the boustrophedon path trough the room.");
	const int grid_spacing_as_int = (int)std::floor(grid_spacing_in_pixel); // convert fov-radius to int
	const int half_grid_spacing_as_int = (int)std::floor(0.5*grid_spacing_in_pixel); // convert fov-radius to int
	const int min_cell_width = half_grid_spacing_as_int + 2.*grid_obstacle_offset/map_resolution;

	// *********************** I. Find the main directions of the map and rotate it in this manner. ***********************
	// *********************** II. Sweep a slice trough the map and mark the found cell boundaries. ***********************
	// *********************** III. Find the separated cells. ***********************
	cv::Mat R;
	cv::Rect bbox;
	cv::Mat rotated_room_map;
	std::vector<GeneralizedPolygon> cell_polygons;
	std::vector<cv::Point> polygon_centers;
	computeCellDecompositionWithRotation(room_map, map_resolution, min_cell_area, min_cell_width, 0., R, bbox, rotated_room_map, cell_polygons, polygon_centers);
	// does not work so well: findBestCellDecomposition(room_map, map_resolution, min_cell_area, R, bbox, rotated_room_map, cell_polygons, polygon_centers);

	ROS_INFO("Found the cells in the given map.");


	// *********************** IV. Determine the cell paths. ***********************
	// determine the start cell that contains the start position
	std::vector<cv::Point> starting_point_vector(1, starting_position); // opencv syntax
	cv::transform(starting_point_vector, starting_point_vector, R);
	const cv::Point rotated_starting_point = starting_point_vector[0]; // point that keeps track of the last point after the boustrophedon path in each cell

//	testing
//	cv::Mat center_map = room_map.clone();
//	for(size_t i=0; i<cell_polygons.size(); ++i)
//		cv::circle(center_map, cell_polygons[i].getCenter(), 2, cv::Scalar(127), CV_FILLED);
//	cv::circle(center_map, starting_position, 4, cv::Scalar(100), CV_FILLED);
//	cv::imshow("centers", center_map);
//	cv::waitKey();

	int start_cell_index = 0;
	for(std::vector<GeneralizedPolygon>::iterator cell=cell_polygons.begin(); cell!=cell_polygons.end(); ++cell)
		if(cv::pointPolygonTest(cell->getVertices(), rotated_starting_point, false) >= 0)
			start_cell_index = cell - cell_polygons.begin();

	// determine the visiting order of the cells
	std::vector<int> optimal_order;
	if (cell_visiting_order == OPTIMAL_TSP)
	{
		// determine the optimal visiting order of the cells
	//	ConcordeTSPSolver tsp_solver;
	//	std::vector<int> optimal_order = tsp_solver.solveConcordeTSP(rotated_room_map, polygon_centers, 0.25, 0.0, map_resolution, start_cell_index, 0);
		GeneticTSPSolver tsp_solver;
		optimal_order = tsp_solver.solveGeneticTSP(rotated_room_map, polygon_centers, 0.25, 0.0, map_resolution, start_cell_index, 0);
		if (optimal_order.size()!=polygon_centers.size())
		{
			std::cout << "=====================> Genetic TSP failed with 25% resolution, falling back to 100%. <=======================" << std::endl;
			optimal_order = tsp_solver.solveGeneticTSP(rotated_room_map, polygon_centers, 1.0, 0.0, map_resolution, start_cell_index, 0);
		}
	}
	else if (cell_visiting_order == LEFT_TO_RIGHT)
	{
		// we are using an alternative ordering here, which visits the cells in a more obvious fashion to the human observer (though it is not optimal)
		std::multimap<int, int> y_coordinate_ordering;		// <y-coordinate of room centers, index>
		for (size_t i=0; i<polygon_centers.size(); ++i)
			y_coordinate_ordering.insert(std::pair<int, int>(polygon_centers[i].y, (int)i));
		for (std::multimap<int,int>::iterator it=y_coordinate_ordering.begin(); it!=y_coordinate_ordering.end(); ++it)
			optimal_order.push_back(it->second);
	}
	else
	{
		std::cout << "Error: BoustrophedonExplorer::getExplorationPath: The specified cell_visiting_order=" << cell_visiting_order << " is invalid." << std::endl;
		return;
	}

	// go trough the cells [in optimal visiting order] and determine the boustrophedon paths
	ROS_INFO("Starting to get the paths for each cell, number of cells: %d", (int)cell_polygons.size());
	std::cout << "Boustrophedon grid_spacing_as_int=" << grid_spacing_as_int << std::endl;
	cv::Point robot_pos = rotated_starting_point;	// point that keeps track of the last point after the boustrophedon path in each cell
	std::vector<cv::Point2f> fov_middlepoint_path;	// this is the trajectory of centers of the robot footprint or the field of view
	for(size_t cell=0; cell<cell_polygons.size(); ++cell)
	{
		computeBoustrophedonPath(rotated_room_map, map_resolution, cell_polygons[optimal_order[cell]], fov_middlepoint_path,
				robot_pos, grid_spacing_as_int, half_grid_spacing_as_int, path_eps, max_deviation_from_track, grid_obstacle_offset/map_resolution);
	}

	// transform the calculated path back to the originally rotated map and create poses with an angle
	RoomRotator room_rotation;
	std::vector<geometry_msgs::Pose2D> fov_poses;	// this is the trajectory of poses of the robot footprint or the field of view, in [pixels]
	room_rotation.transformPathBackToOriginalRotation(fov_middlepoint_path, fov_poses, R);
#ifdef DEBUG_VISUALIZATION
	std::cout << "printing path" << std::endl;
	cv::Mat room_map_path = room_map.clone();
	cv::circle(room_map_path, starting_position, 3, cv::Scalar(160), CV_FILLED);
	for(size_t i=0; i<fov_poses.size()-1; ++i)
	{
		cv::circle(room_map_path, cv::Point(cvRound(fov_poses[i].x), cvRound(fov_poses[i].y)), 1, cv::Scalar(200), CV_FILLED);
		cv::line(room_map_path, cv::Point(cvRound(fov_poses[i].x), cvRound(fov_poses[i].y)), cv::Point(cvRound(fov_poses[i+1].x), cvRound(fov_poses[i+1].y)), cv::Scalar(100), 1);
	}
	cv::circle(room_map_path, cv::Point(cvRound(fov_poses.back().x), cvRound(fov_poses.back().y)), 1, cv::Scalar(200), CV_FILLED);
	//	for(size_t i=0; i<optimal_order.size()-1; ++i)
	//		cv::line(room_map_path, polygon_centers[optimal_order[i]], polygon_centers[optimal_order[i+1]], cv::Scalar(100), 1);
	cv::imshow("room_map_path_intermediate", room_map_path);
	if (plan_for_footprint == true)
		cv::waitKey();
#endif
	ROS_INFO("Found the cell paths.");

	// if the path should be planned for the robot footprint create the path and return here
	if (plan_for_footprint == true)
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

	// *********************** V. Get the robot path out of the fov path. ***********************
	// go trough all computed fov poses and compute the corresponding robot pose
	ROS_INFO("Starting to map from field of view pose to robot pose");
	cv::Point robot_starting_position = (fov_poses.size()>0 ? cv::Point(cvRound(fov_poses[0].x), cvRound(fov_poses[0].y)) : starting_position);
	cv::Mat inflated_room_map;
	cv::erode(room_map, inflated_room_map, cv::Mat(), cv::Point(-1, -1), half_grid_spacing_as_int);
	mapPath(inflated_room_map, path, fov_poses, robot_to_fov_vector, map_resolution, map_origin, robot_starting_position);


#ifdef DEBUG_VISUALIZATION
	// testing
	std::cout << "printing path" << std::endl;
	cv::Mat fov_path_map = room_map.clone();
	if (path.size() > 0)
		cv::circle(fov_path_map, cv::Point((path[0].x-map_origin.x)/map_resolution, (path[0].y-map_origin.y)/map_resolution), 2, cv::Scalar(150), CV_FILLED);
	for(size_t i=1; i<path.size(); ++i)
	{
		cv::Point p1((path[i-1].x-map_origin.x)/map_resolution, (path[i-1].y-map_origin.y)/map_resolution);
		cv::Point p2((path[i].x-map_origin.x)/map_resolution, (path[i].y-map_origin.y)/map_resolution);
		cv::circle(fov_path_map, p2, 1, cv::Scalar(200), CV_FILLED);
		cv::line(fov_path_map, p1, p2, cv::Scalar(100), 1);
		cv::Point p3(p2.x+5*cos(path[i].theta), p2.y+5*sin(path[i].theta));
		cv::line(fov_path_map, p2, p3, cv::Scalar(50), 1);
//		cv::imshow("cell path", fov_path_map);
//		cv::waitKey();
	}
	cv::imshow("room_map_path_final", fov_path_map);
	cv::waitKey();
#endif
}


void BoustrophedonExplorer::findBestCellDecomposition(const cv::Mat& room_map, const float map_resolution, const double min_cell_area,
		const int min_cell_width, cv::Mat& R, cv::Rect& bbox, cv::Mat& rotated_room_map,
		std::vector<GeneralizedPolygon>& cell_polygons, std::vector<cv::Point>& polygon_centers)
{
	// *********************** I. Find the main directions of the map and rotate it in this manner. ***********************
	// *********************** II. Sweep a slice trough the map and mark the found cell boundaries. ***********************
	// *********************** III. Find the separated cells. ***********************
	cv::Mat R_1, R_2;
	cv::Rect bbox_1, bbox_2;
	cv::Mat rotated_room_map_1, rotated_room_map_2;
	std::vector<GeneralizedPolygon> cell_polygons_1, cell_polygons_2;
	std::vector<cv::Point> polygon_centers_1, polygon_centers_2;
	computeCellDecompositionWithRotation(room_map, map_resolution, min_cell_area, min_cell_width, 0., R_1, bbox_1, rotated_room_map_1, cell_polygons_1, polygon_centers_1);
	computeCellDecompositionWithRotation(room_map, map_resolution, min_cell_area, min_cell_width, 90./180.*CV_PI, R_2, bbox_2, rotated_room_map_2, cell_polygons_2, polygon_centers_2);

	// select the cell decomposition with good axis alignment which produces less cells
	if (cell_polygons_1.size() <= cell_polygons_2.size())
	{
		R = R_1;
		bbox = bbox_1;
		rotated_room_map = rotated_room_map_1;
		cell_polygons = cell_polygons_1;
		polygon_centers = polygon_centers_1;
	}
	else
	{
		R = R_2;
		bbox = bbox_2;
		rotated_room_map = rotated_room_map_2;
		cell_polygons = cell_polygons_2;
		polygon_centers = polygon_centers_2;
	}
}

void BoustrophedonExplorer::computeCellDecompositionWithRotation(const cv::Mat& room_map, const float map_resolution, const double min_cell_area,
		const int min_cell_width, const double rotation_offset, cv::Mat& R, cv::Rect& bbox, cv::Mat& rotated_room_map,
		std::vector<GeneralizedPolygon>& cell_polygons, std::vector<cv::Point>& polygon_centers)
{
	// *********************** I. Find the main directions of the map and rotate it in this manner. ***********************
	RoomRotator room_rotation;
	room_rotation.computeRoomRotationMatrix(room_map, R, bbox, map_resolution, 0, rotation_offset);
	room_rotation.rotateRoom(room_map, rotated_room_map, R, bbox);

#ifdef DEBUG_VISUALIZATION
//	// testing
//	cv::Mat room_map_disp = room_map.clone();
//	cv::circle(room_map_disp, starting_position, 3, cv::Scalar(160), CV_FILLED);
//	cv::imshow("room_map", room_map_disp);
//	std::vector<cv::Point> tester;
//	tester.push_back(cv::Point(10,10));
//	cv::Mat tester_map = room_map.clone();
//	cv::circle(tester_map, tester[0], 3, cv::Scalar(127), CV_FILLED);
//	cv::transform(tester, tester, R);
//	cv::circle(rotated_room_map, tester[0], 3, cv::Scalar(127), CV_FILLED);
//	cv::imshow("original", tester_map);
//	cv::imshow("rotated_im.png", rotated_room_map);
//	cv::waitKey();
#endif

	// *********************** II. Sweep a slice trough the map and mark the found cell boundaries. ***********************
	// *********************** III. Find the separated cells. ***********************
	computeCellDecomposition(rotated_room_map, map_resolution, min_cell_area, min_cell_width, cell_polygons, polygon_centers);
}

void BoustrophedonExplorer::computeCellDecomposition(const cv::Mat& room_map, const float map_resolution, const double min_cell_area,
		const int min_cell_width, std::vector<GeneralizedPolygon>& cell_polygons, std::vector<cv::Point>& polygon_centers)
{
	// *********************** II. Sweep a slice trough the map and mark the found cell boundaries. ***********************
	// create a map copy to mark the cell boundaries
	cv::Mat cell_map = room_map.clone();

	// find smallest y-value for that a white pixel occurs, to set initial y value and find initial number of segments
	size_t y_start = 0;
	bool found = false, obstacle = false;
	int previous_number_of_segments = 0;
	std::vector<int> previous_obstacles_end_x;		// keep track of the end points of obstacles
	for(size_t y=0; y<room_map.rows; ++y)
	{
		for(size_t x=0; x<room_map.cols; ++x)
		{
			if(found == false && room_map.at<uchar>(y,x) == 255)
			{
				y_start = y;
				found = true;
			}
			else if(found == true && obstacle == false && room_map.at<uchar>(y,x) == 0)
			{
				++previous_number_of_segments;
				obstacle = true;
			}
			else if(found == true && obstacle == true && room_map.at<uchar>(y,x) == 255)
			{
				obstacle = false;
				previous_obstacles_end_x.push_back(x);
			}
		}

		if(found == true)
			break;
	}

	// sweep trough the map and detect critical points
	for(size_t y=y_start+1; y<room_map.rows; ++y) // start at y_start+1 because we know number of segments at y_start
	{
		int number_of_segments = 0; // int to count how many segments at the current slice are
		std::vector<int> current_obstacles_start_x;
		std::vector<int> current_obstacles_end_x;
		bool obstacle_hit = false; // bool to check if the line currently hit an obstacle, s.t. not all black pixels trigger an event
		bool hit_white_pixel = false; // bool to check if a white pixel has been hit at the current slice, to start the slice at the first white pixel

		// count number of segments within this row
		for(size_t x=0; x<room_map.cols; ++x)
		{
			if(hit_white_pixel == false && room_map.at<uchar>(y,x) == 255)
				hit_white_pixel = true;
			else if(hit_white_pixel == true)
			{
				if(obstacle_hit == false && room_map.at<uchar>(y,x) == 0) // check for obstacle
				{
					++number_of_segments;
					obstacle_hit = true;
					current_obstacles_start_x.push_back(x);
				}
				else if(obstacle_hit == true && room_map.at<uchar>(y,x) == 255) // check for leaving obstacle
				{
					obstacle_hit = false;
					current_obstacles_end_x.push_back(x);
				}
			}
		}

		// if the number of segments did not change, check whether the position of segments has changed so that there is a gap between them
		bool segment_shift_detected = false;
		if (previous_number_of_segments == number_of_segments && current_obstacles_start_x.size() == previous_obstacles_end_x.size()+1)
		{
			for (size_t i=0; i<previous_obstacles_end_x.size(); ++i)
				if (current_obstacles_start_x[i] > previous_obstacles_end_x[i])
				{
					segment_shift_detected = true;
					break;
				}
		}

		// reset hit_white_pixel to use this Boolean later
		hit_white_pixel = false;

		// check if number of segments has changed --> event occurred
		if(previous_number_of_segments < number_of_segments || segment_shift_detected == true) // IN event (or shift)
		{
			// check the current slice again for critical points
			for(int x=0; x<room_map.cols; ++x)
			{
				if(hit_white_pixel == false && room_map.at<uchar>(y,x) == 255)
					hit_white_pixel = true;
				else if(hit_white_pixel == true && room_map.at<uchar>(y,x) == 0)
				{
					// check over black pixel for other black pixels, if none occur a critical point is found
					bool critical_point = true;
					for(int dx=-1; dx<=1; ++dx)
						if(room_map.at<uchar>(y-1,std::max(0,std::min(x+dx, room_map.cols-1))) == 0)
							critical_point = false;

					// if a critical point is found mark the separation, note that this algorithm goes left and right
					// starting at the critical point until an obstacle is hit, because this prevents unnecessary cells
					// behind other obstacles on the same y-value as the critical point
					if(critical_point == true)
					{
						// to the left until a black pixel is hit
						for(int dx=-1; x+dx>=0; --dx)
						{
							uchar& val = cell_map.at<uchar>(y,x+dx);
							if(val == 255 && cell_map.at<uchar>(y-1,x+dx) == 255)
								val = BORDER_PIXEL_VALUE;
							else if(val == 0)
								break;
						}

						// to the right until a black pixel is hit
						for(int dx=1; x+dx<room_map.cols; ++dx)
						{
							uchar& val = cell_map.at<uchar>(y,x+dx);
							if(val == 255 && cell_map.at<uchar>(y-1,x+dx) == 255)
								val = BORDER_PIXEL_VALUE;
							else if(val == 0)
								break;
						}
					}
				}
			}
		}
		else if(previous_number_of_segments > number_of_segments) // OUT event
		{
			// check the previous slice again for critical points --> y-1
			for(int x=0; x<room_map.cols; ++x)
			{
				if(room_map.at<uchar>(y-1,x) == 255 && hit_white_pixel == false)
					hit_white_pixel = true;
				else if(hit_white_pixel == true && room_map.at<uchar>(y-1,x) == 0)
				{
					// check over black pixel for other black pixels, if none occur a critical point is found
					bool critical_point = true;
					for(int dx=-1; dx<=1; ++dx)
						if(room_map.at<uchar>(y,std::max(0,std::min(x+dx, room_map.cols-1))) == 0) // check at side after obstacle
							critical_point = false;

					// if a critical point is found mark the separation, note that this algorithm goes left and right
					// starting at the critical point until an obstacle is hit, because this prevents unnecessary cells
					// behind other obstacles on the same y-value as the critical point
					if(critical_point == true)
					{
						const int ym2 = std::max(0,(int)y-2);

						// to the left until a black pixel is hit
						for(int dx=-1; x+dx>=0; --dx)
						{
							uchar& val = cell_map.at<uchar>(y-1,x+dx);
							if(val == 255 && cell_map.at<uchar>(ym2,x+dx) == 255)
								val = BORDER_PIXEL_VALUE;
							else if(val == 0)
								break;
						}

						// to the right until a black pixel is hit
						for(int dx=1; x+dx<room_map.cols; ++dx)
						{
							uchar& val = cell_map.at<uchar>(y-1,x+dx);
							if(val == 255 && cell_map.at<uchar>(ym2,x+dx) == 255)
								val = BORDER_PIXEL_VALUE;
							else if(val == 0)
								break;
						}
					}
				}
			}
		}

		// save the found number of segments and the obstacle end points
		previous_number_of_segments = number_of_segments;
		previous_obstacles_end_x = current_obstacles_end_x;
	}

#ifdef DEBUG_VISUALIZATION
	cv::imshow("cell_map", cell_map);
#endif

	// *********************** II.b) merge too small cells into bigger cells ***********************
	cv::Mat cell_map_labels;
	const int number_of_cells = mergeCells(cell_map, cell_map_labels, min_cell_area, min_cell_width);

	// *********************** III. Find the separated cells. ***********************
	std::vector<std::vector<cv::Point> > cells;
	for (int i=1; i<=number_of_cells; ++i)
	{
		cv::Mat cell_copy(cell_map_labels == i);
		std::vector<std::vector<cv::Point> > cellsi;
		cv::findContours(cell_copy, cellsi, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
		cells.insert(cells.end(), cellsi.begin(), cellsi.end());
	}

#ifdef DEBUG_VISUALIZATION
//	// testing
//	cv::Mat black_map = cv::Mat::zeros(cell_map.rows, cell_map.cols, cell_map.type());
//	for(size_t i=0; i<cells.size(); ++i)
//	{
//		cv::drawContours(black_map, cells, i, cv::Scalar(127), CV_FILLED);
//		cv::imshow("contours", black_map);
//		cv::waitKey();
//	}
#endif

	// create generalized Polygons out of the contours to handle the cells
	for(size_t cell=0; cell<cells.size(); ++cell)
	{
		GeneralizedPolygon current_cell(cells[cell], map_resolution);
//		std::cout << cell+1 << ": " << current_cell.getArea() << std::endl;
		if(current_cell.getArea()>=min_cell_area)
		{
			cell_polygons.push_back(current_cell);
			polygon_centers.push_back(current_cell.getCenter());
		}
		else
		{
			std::cout << "WARN: BoustrophedonExplorer::computeCellDecomposition: dropped cell " << cell+1 << " with area=" << current_cell.getArea() << ". This should only happen for small unconnected cells." << std::endl;
		}
	}
}

int BoustrophedonExplorer::mergeCells(cv::Mat& cell_map, cv::Mat& cell_map_labels, const double min_cell_area, const int min_cell_width)
{
	// label all cells
	//   --> create a label map with 0=walls/obstacles, -1=cell borders, 1,2,3,4...=cell labels
	cell_map.convertTo(cell_map_labels, CV_32SC1, 256, 0);
	//   --> re-assign the cell borders with -1
	for (int v=0; v<cell_map_labels.rows; ++v)
		for (int u=0; u<cell_map_labels.cols; ++u)
			if (cell_map_labels.at<int>(v,u) == BORDER_PIXEL_VALUE*256)
				cell_map_labels.at<int>(v,u) = -1;
	//   --> flood fill cell regions with unique id labels
	std::map<int, boost::shared_ptr<BoustrophedonCell> > cell_index_mapping;		// maps each cell label --> to the cell object
	int label_index = 1;
	for (int v=0; v<cell_map_labels.rows; ++v)
	{
		for (int u=0; u<cell_map_labels.cols; ++u)
		{
			// if the map has already received a label for that pixel --> skip
			if (cell_map_labels.at<int>(v,u)!=65280)
				continue;

			// fill each cell with a unique id
			cv::Rect bounding_box;
			const double area = cv::floodFill(cell_map_labels, cv::Point(u,v), label_index, &bounding_box, 0, 0, 4);
			cell_index_mapping[label_index] = boost::shared_ptr<BoustrophedonCell>(new BoustrophedonCell(label_index, area, bounding_box));
			label_index++;
			if (label_index == INT_MAX)
				std::cout << "WARN: BoustrophedonExplorer::mergeCells: label_index exceeds range of int." << std::endl;
		}
	}
	std::cout << "INFO: BoustrophedonExplorer::mergeCells: found " << label_index-1 << " cells before merging." << std::endl;

	// determine the neighborhood relationships between all cells
	for (int v=1; v<cell_map_labels.rows-1; ++v)
	{
		for (int u=1; u<cell_map_labels.cols-1; ++u)
		{
			if (cell_map_labels.at<int>(v,u)==-1)	// only check the border points for neighborhood relationships
			{
				const int label_left = cell_map_labels.at<int>(v,u-1);
				const int label_right = cell_map_labels.at<int>(v,u+1);
				if (label_left>0 && label_right>0)
				{
					cell_index_mapping[label_left]->neighbors_.insert(cell_index_mapping[label_right]);
					cell_index_mapping[label_right]->neighbors_.insert(cell_index_mapping[label_left]);
				}
				const int label_up = cell_map_labels.at<int>(v-1,u);
				const int label_down = cell_map_labels.at<int>(v+1,u);
				if (label_up>0 && label_down>0)
				{
					cell_index_mapping[label_up]->neighbors_.insert(cell_index_mapping[label_down]);
					cell_index_mapping[label_down]->neighbors_.insert(cell_index_mapping[label_up]);
				}
			}
		}
	}
#ifdef DEBUG_VISUALIZATION
//	printCells(cell_index_mapping);
//	cv::imshow("cell_map",cell_map);
//	cv::waitKey();
#endif

	// iteratively merge cells
	mergeCellsSelection(cell_map, cell_map_labels, cell_index_mapping, min_cell_area, min_cell_width);

	// re-assign area labels to 1,2,3,4,...
	int new_cell_label = 1;
	for (std::map<int, boost::shared_ptr<BoustrophedonCell> >::iterator itc=cell_index_mapping.begin(); itc!=cell_index_mapping.end(); ++itc, ++new_cell_label)
		for (int v=0; v<cell_map_labels.rows; ++v)
			for (int u=0; u<cell_map_labels.cols; ++u)
				if (cell_map_labels.at<int>(v,u)==itc->second->label_)
					cell_map_labels.at<int>(v,u) = new_cell_label;

	std::cout << "INFO: BoustrophedonExplorer::mergeCells: " << cell_index_mapping.size() << " cells remaining after merging." << std::endl;
	return cell_index_mapping.size();
}

void BoustrophedonExplorer::mergeCellsSelection(cv::Mat& cell_map, cv::Mat& cell_map_labels, std::map<int, boost::shared_ptr<BoustrophedonCell> >& cell_index_mapping,
		const double min_cell_area, const int min_cell_width)
{
	// iteratively merge cells
	// merge small cells below min_cell_area with their largest neighboring cell
	std::multimap<double, boost::shared_ptr<BoustrophedonCell> > area_to_region_id_mapping;		// maps the area of each cell --> to the respective cell
	for (std::map<int, boost::shared_ptr<BoustrophedonCell> >::iterator itc=cell_index_mapping.begin(); itc!=cell_index_mapping.end(); ++itc)
		area_to_region_id_mapping.insert(std::pair<double, boost::shared_ptr<BoustrophedonCell> >(itc->second->area_, itc->second));
	for (std::multimap<double, boost::shared_ptr<BoustrophedonCell> >::iterator it=area_to_region_id_mapping.begin(); it!=area_to_region_id_mapping.end();)
	{
		// skip if segment is large enough (area and side length criteria)
		if (it->first >= min_cell_area && it->second->bounding_box_.width >= min_cell_width && it->second->bounding_box_.height >= min_cell_width)
		{
			++it;
			continue;
		}

		// skip segments which have no neighbors
		if (it->second->neighbors_.size() == 0)
		{
			std::cout << "WARN: BoustrophedonExplorer::mergeCells: skipping small cell without neighbors." << std::endl;
			++it;
			continue;
		}

		// determine the largest neighboring cell
		const BoustrophedonCell& small_cell = *(it->second);
		std::multimap<double, boost::shared_ptr<BoustrophedonCell>, std::greater<double> > area_sorted_neighbors;
		for (BoustrophedonCell::BoustrophedonCellSetIterator itn = small_cell.neighbors_.begin(); itn != small_cell.neighbors_.end(); ++itn)
			area_sorted_neighbors.insert(std::pair<double, boost::shared_ptr<BoustrophedonCell> >((*itn)->area_, *itn));
		BoustrophedonCell& large_cell = *(area_sorted_neighbors.begin()->second);

		// merge the cells
		mergeTwoCells(cell_map, cell_map_labels, small_cell, large_cell, cell_index_mapping);

		// update area_to_region_id_mapping
		area_to_region_id_mapping.clear();
		for (std::map<int, boost::shared_ptr<BoustrophedonCell> >::iterator itc=cell_index_mapping.begin(); itc!=cell_index_mapping.end(); ++itc)
			area_to_region_id_mapping.insert(std::pair<double, boost::shared_ptr<BoustrophedonCell> >(itc->second->area_, itc->second));
		it = area_to_region_id_mapping.begin();

#ifdef DEBUG_VISUALIZATION
//		printCells(cell_index_mapping);
//		cv::imshow("cell_map",cell_map);
//		cv::waitKey();
#endif
	}

	// label remaining border pixels with label of largest neighboring region label
	for (int v=1; v<cell_map.rows-1; ++v)
	{
		for (int u=1; u<cell_map.cols-1; ++u)
		{
			if (cell_map.at<uchar>(v,u) == BORDER_PIXEL_VALUE)
			{
				std::set<int> neighbor_labels;
				for (int dv=-1; dv<=1; ++dv)
				{
					for (int du=-1; du<=1; ++du)
					{
						const int& val = cell_map_labels.at<int>(v+dv,u+du);
						if (val>0)
							neighbor_labels.insert(val);
					}
				}
				if (neighbor_labels.size() > 0)
				{
					int new_label = -1;
					for (std::multimap<double, boost::shared_ptr<BoustrophedonCell> >::reverse_iterator it=area_to_region_id_mapping.rbegin(); it!=area_to_region_id_mapping.rend(); ++it)
					{
						if (neighbor_labels.find(it->second->label_) != neighbor_labels.end())
						{
							cell_map_labels.at<int>(v,u) = it->second->label_;
							break;
						}
					}
				}
				else
					std::cout << "WARN: BoustrophedonExplorer::mergeCells: border pixel has no labeled neighbors." << std::endl;
			}
		}
	}
}

void BoustrophedonExplorer::mergeTwoCells(cv::Mat& cell_map, cv::Mat& cell_map_labels, const BoustrophedonCell& minor_cell, BoustrophedonCell& major_cell,
		std::map<int, boost::shared_ptr<BoustrophedonCell> >& cell_index_mapping)
{
	// execute merging the minor cell into the major cell
	//   --> remove border from maps
	for (int v=0; v<cell_map.rows; ++v)
		for (int u=0; u<cell_map.cols; ++u)
			if (cell_map.at<uchar>(v,u) == BORDER_PIXEL_VALUE &&
					((cell_map_labels.at<int>(v,u-1)==minor_cell.label_ && cell_map_labels.at<int>(v,u+1)==major_cell.label_) ||
					(cell_map_labels.at<int>(v,u-1)==major_cell.label_ && cell_map_labels.at<int>(v,u+1)==minor_cell.label_) ||
					(cell_map_labels.at<int>(v-1,u)==minor_cell.label_ && cell_map_labels.at<int>(v+1,u)==major_cell.label_) ||
					(cell_map_labels.at<int>(v-1,u)==major_cell.label_ && cell_map_labels.at<int>(v+1,u)==minor_cell.label_)))
			{
				cell_map.at<uchar>(v,u) = 255;
				cell_map_labels.at<int>(v,u) = major_cell.label_;
				major_cell.area_ += 1;
			}
	//   --> update old label in cell_map_labels
	for (int v=0; v<cell_map_labels.rows; ++v)
		for (int u=0; u<cell_map_labels.cols; ++u)
			if (cell_map_labels.at<int>(v,u) == minor_cell.label_)
				cell_map_labels.at<int>(v,u) = major_cell.label_;
	//   --> update major_cell
	major_cell.area_ += minor_cell.area_;
	for (BoustrophedonCell::BoustrophedonCellSetIterator itn = major_cell.neighbors_.begin(); itn != major_cell.neighbors_.end(); ++itn)
		if ((*itn)->label_ == minor_cell.label_)
		{
			major_cell.neighbors_.erase(itn);
			break;
		}
	for (BoustrophedonCell::BoustrophedonCellSetIterator itn = minor_cell.neighbors_.begin(); itn != minor_cell.neighbors_.end(); ++itn)
		if ((*itn)->label_ != major_cell.label_)
			major_cell.neighbors_.insert(*itn);

	// clean all references to minor_cell
	cell_index_mapping.erase(minor_cell.label_);
	for (std::map<int, boost::shared_ptr<BoustrophedonCell> >::iterator itc=cell_index_mapping.begin(); itc!=cell_index_mapping.end(); ++itc)
		for (BoustrophedonCell::BoustrophedonCellSetIterator itn = itc->second->neighbors_.begin(); itn != itc->second->neighbors_.end(); ++itn)
			if ((*itn)->label_ == minor_cell.label_)
			{
				(*itn)->label_ = major_cell.label_;
				break;
			}
}

void BoustrophedonExplorer::correctThinWalls(cv::Mat& room_map)
{
	for (int v=1; v<room_map.rows; ++v)
	{
		for (int u=1; u<room_map.cols; ++u)
		{
			if (room_map.at<uchar>(v-1,u-1)==255 && room_map.at<uchar>(v-1,u)==0 && room_map.at<uchar>(v,u-1)==0 && room_map.at<uchar>(v,u)==255)
				room_map.at<uchar>(v,u)=0;
			else if (room_map.at<uchar>(v-1,u-1)==0 && room_map.at<uchar>(v-1,u)==255 && room_map.at<uchar>(v,u-1)==255 && room_map.at<uchar>(v,u)==0)
				room_map.at<uchar>(v,u-1)=0;
		}
	}
}

void BoustrophedonExplorer::computeBoustrophedonPath(const cv::Mat& room_map, const float map_resolution, const GeneralizedPolygon& cell,
		std::vector<cv::Point2f>& fov_middlepoint_path, cv::Point& robot_pos,
		const int grid_spacing_as_int, const int half_grid_spacing_as_int, const double path_eps, const int max_deviation_from_track, const int grid_obstacle_offset)
{
	// get a map that has only the current cell drawn in
	//	Remark:	single cells are obstacle free so it is sufficient to use the cell to check if a position can be reached during the
	//			execution of the coverage path
	cv::Mat cell_map;
	cell.drawPolygon(cell_map, cv::Scalar(255));

	// align the longer dimension of the cell horizontally with the x-axis
	cv::Point cell_center = cell.getBoundingBoxCenter();
	cv::Mat R_cell;
	cv::Rect cell_bbox;
	cv::Mat rotated_cell_map;
	RoomRotator cell_rotation;
	cell_rotation.computeRoomRotationMatrix(cell_map, R_cell, cell_bbox, map_resolution, &cell_center);
	cell_rotation.rotateRoom(cell_map, rotated_cell_map, R_cell, cell_bbox);

	// create inflated obstacles room map and rotate according to cell
	//  --> used later for checking accessibility of Boustrophedon path inside the cell
	cv::Mat inflated_room_map, rotated_inflated_room_map;
	cv::erode(room_map, inflated_room_map, cv::Mat(), cv::Point(-1, -1), half_grid_spacing_as_int+grid_obstacle_offset);
	cell_rotation.rotateRoom(inflated_room_map, rotated_inflated_room_map, R_cell, cell_bbox);
	cv::Mat rotated_inflated_cell_map = rotated_cell_map.clone();
	for (int v=0; v<rotated_inflated_cell_map.rows; ++v)
		for (int u=0; u<rotated_inflated_cell_map.cols; ++u)
			if (rotated_inflated_cell_map.at<uchar>(v,u)!=0 && rotated_inflated_room_map.at<uchar>(v,u)==0)
				rotated_inflated_cell_map.at<uchar>(v,u) = 128;
#ifdef DEBUG_VISUALIZATION
	cv::imshow("rotated_cell_map_with_inflation", rotated_inflated_cell_map);
#endif

	// this was deactivated because it is not as accurate as the direct check within GridGenerator::generateBoustrophedonGrid,
	// because the rotation introduces some rounding errors
//	// get the min/max x/y values for this cell
//	int min_x=100000, max_x=0, min_y=100000, max_y=0;
//	std::vector<cv::Point> rotated_vertexes = cell.getVertices();
//	cv::transform(rotated_vertexes, rotated_vertexes, R_cell);
//	for(size_t point=0; point<rotated_vertexes.size(); ++point)
//	{
//		if(rotated_vertexes[point].x > max_x)
//			max_x = rotated_vertexes[point].x;
//		if(rotated_vertexes[point].y > max_y)
//			max_y = rotated_vertexes[point].y;
//		if(rotated_vertexes[point].x < min_x)
//			min_x = rotated_vertexes[point].x;
//		if(rotated_vertexes[point].y < min_y)
//			min_y = rotated_vertexes[point].y;
//	}

	// compute the basic Boustrophedon grid lines
	BoustrophedonGrid grid_lines;
	GridGenerator::generateBoustrophedonGrid(rotated_cell_map, rotated_inflated_cell_map, -1, grid_lines, cv::Vec4i(-1, -1, -1, -1), //cv::Vec4i(min_x, max_x, min_y, max_y),
			grid_spacing_as_int, half_grid_spacing_as_int, 1, max_deviation_from_track);

#ifdef DEBUG_VISUALIZATION
	cv::Mat rotated_cell_map_disp = rotated_cell_map.clone();
	for (size_t i=0; i<grid_lines.size(); ++i)
	{
		for (size_t j=0; j+1<grid_lines[i].upper_line.size(); ++j)
		{
			cv::circle(rotated_cell_map_disp, grid_lines[i].upper_line[j], 1, cv::Scalar(64), CV_FILLED);
			cv::line(rotated_cell_map_disp, grid_lines[i].upper_line[j], grid_lines[i].upper_line[j+1], cv::Scalar(128), 1);
		}
		for (size_t j=0; j+1<grid_lines[i].lower_line.size(); ++j)
		{
			cv::circle(rotated_cell_map_disp, grid_lines[i].lower_line[j], 1, cv::Scalar(64), CV_FILLED);
			cv::line(rotated_cell_map_disp, grid_lines[i].lower_line[j], grid_lines[i].lower_line[j+1], cv::Scalar(196), 1);
		}
	}
	cv::imshow("rotated_cell_map", rotated_cell_map_disp);
#endif

	// if no edge could be found in the cell (e.g. if it is too small), ignore it
	if(grid_lines.size()==0)
		return;

	// get the edge nearest to the current robot position to start the boustrophedon path at, by looking at the
	// upper and lower horizontal path (possible nearest locations) for the edges transformed to the original coordinates (easier)
	std::vector<cv::Point> outer_corners(4);
	outer_corners[0] = grid_lines[0].upper_line[0];		// upper left corner
	outer_corners[1] = grid_lines[0].upper_line.back();	// upper right corner
	outer_corners[2] = grid_lines.back().upper_line[0];	// lower left corner
	outer_corners[3] = grid_lines.back().upper_line.back();	// lower right corner
	cv::Mat R_cell_inv;
	cv::invertAffineTransform(R_cell, R_cell_inv);	// invert the rotation matrix to remap the determined points to the original cell
	cv::transform(outer_corners, outer_corners, R_cell_inv);
	double min_corner_dist = path_planner_.planPath(room_map, robot_pos, outer_corners[0], 1.0, 0.0, map_resolution);
	int min_corner_index = 0;
	for (int i=1; i<4; ++i)
	{
		double dist = path_planner_.planPath(room_map, robot_pos, outer_corners[i], 1.0, 0.0, map_resolution);
		if (dist < min_corner_dist)
		{
			min_corner_dist = dist;
			min_corner_index = i;
		}
	}
	bool start_from_upper_path = (min_corner_index<2 ? true : false);
	bool start_from_left = (min_corner_index%2==0 ? true : false); // boolean to determine on which side the path should start and to check where the path ended

#ifdef DEBUG_VISUALIZATION
	cv::Mat room_map_disp = room_map.clone();
	for (size_t i=0; i<outer_corners.size(); i+=2)
		cv::line(room_map_disp, outer_corners[i], outer_corners[i+1], cv::Scalar(128), 1);
	cv::circle(room_map_disp, robot_pos, 3, cv::Scalar(160), CV_FILLED);
	if (start_from_upper_path == true)
	{
		if (start_from_left == true)
			cv::circle(room_map_disp, outer_corners[0], 3, cv::Scalar(64), CV_FILLED);
		else
			cv::circle(room_map_disp, outer_corners[1], 3, cv::Scalar(64), CV_FILLED);
	}
	else
	{
		if (start_from_left == true)
			cv::circle(room_map_disp, outer_corners[2], 3, cv::Scalar(64), CV_FILLED);
		else
			cv::circle(room_map_disp, outer_corners[3], 3, cv::Scalar(64), CV_FILLED);
	}
	cv::imshow("rotated_room_map", room_map_disp);
#endif

	// connect the boustrophedon paths
	cv::Point cell_robot_pos;
	bool start = true;
	std::vector<cv::Point> current_fov_path;
	if(start_from_upper_path == true) // plan the path starting from upper horizontal line
	{
		for(BoustrophedonGrid::iterator line=grid_lines.begin(); line!=grid_lines.end(); ++line)
		{
			if(start == true) // at the beginning of path planning start at first horizontal line --> no vertical points between lines
			{
				if(start_from_left == true)
					cell_robot_pos = line->upper_line[0];
				else
					cell_robot_pos = line->upper_line.back();
				start = false;
			}

			if(start_from_left == true) // plan path from left to right corner
			{
				// get points on transition between horizontal lines by using the Astar-path
				std::vector<cv::Point> astar_path;
				path_planner_.planPath(rotated_inflated_cell_map, cell_robot_pos, line->upper_line[0], 1.0, 0.0, map_resolution, 0, &astar_path);
				downsamplePath(astar_path, current_fov_path, cell_robot_pos, path_eps);

				// get points between left and right corner
				downsamplePath(line->upper_line, current_fov_path, cell_robot_pos, path_eps);

				// add the lower path of the current line if available (and then start from the left side again next time)
				if (line->has_two_valid_lines == true)
					downsamplePathReverse(line->lower_line, current_fov_path, cell_robot_pos, path_eps);
				else
					start_from_left = false;	// start from the right side next time
			}
			else // plan path from right to left corner
			{
				// get points on transition between horizontal lines by using the Astar-path
				std::vector<cv::Point> astar_path;
				path_planner_.planPath(rotated_inflated_cell_map, cell_robot_pos, line->upper_line.back(), 1.0, 0.0, map_resolution, 0, &astar_path);
				downsamplePath(astar_path, current_fov_path, cell_robot_pos, path_eps);

				// get points between right and left corner
				downsamplePathReverse(line->upper_line, current_fov_path, cell_robot_pos, path_eps);

				// add the lower path of the current line if available (and then start from the right side again next time)
				if (line->has_two_valid_lines == true)
					downsamplePath(line->lower_line, current_fov_path, cell_robot_pos, path_eps);
				else
					start_from_left = true;	// start from the left side next time
			}
		}
	}
	else // plan the path from the lower horizontal line
	{
		for(BoustrophedonGrid::reverse_iterator line=grid_lines.rbegin(); line!=grid_lines.rend(); ++line)
		{
			if(start == true) // at the beginning of path planning start at first horizontal line --> no vertical points between lines
			{
				if(start_from_left == true)
					cell_robot_pos = line->upper_line[0];
				else
					cell_robot_pos = line->upper_line.back();
				start = false;
			}

			if(start_from_left == true) // plan path from left to right corner
			{
				// get points on transition between horizontal lines by using the Astar-path
				std::vector<cv::Point> astar_path;
				path_planner_.planPath(rotated_inflated_cell_map, cell_robot_pos, line->upper_line[0], 1.0, 0.0, map_resolution, 0, &astar_path);
				downsamplePath(astar_path, current_fov_path, cell_robot_pos, path_eps);

				// get points between left and right corner
				downsamplePath(line->upper_line, current_fov_path, cell_robot_pos, path_eps);

				// add the lower path of the current line if available (and then start from the left side again next time)
				if (line->has_two_valid_lines == true)
					downsamplePathReverse(line->lower_line, current_fov_path, cell_robot_pos, path_eps);
				else
					start_from_left = false;	// start from the right side next time
			}
			else // plan path from right to left corner
			{
				// get points on transition between horizontal lines by using the Astar-path
				std::vector<cv::Point> astar_path;
				path_planner_.planPath(rotated_inflated_cell_map, cell_robot_pos, line->upper_line.back(), 1.0, 0.0, map_resolution, 0, &astar_path);
				downsamplePath(astar_path, current_fov_path, cell_robot_pos, path_eps);

				// get points between right and left corner
				downsamplePathReverse(line->upper_line, current_fov_path, cell_robot_pos, path_eps);

				// add the lower path of the current line if available (and then start from the right side again next time)
				if (line->has_two_valid_lines == true)
					downsamplePath(line->lower_line, current_fov_path, cell_robot_pos, path_eps);
				else
					start_from_left = true;	// start from the left side next time
			}
		}
	}
#ifdef DEBUG_VISUALIZATION
	cv::Mat rotated_cell_fov_path_disp = rotated_cell_map.clone();
	for (size_t i=1; i<current_fov_path.size(); ++i)
	{
		cv::circle(rotated_cell_fov_path_disp, current_fov_path[i], 1, cv::Scalar(196), 1);
		cv::line(rotated_cell_fov_path_disp, current_fov_path[i-1], current_fov_path[i], cv::Scalar(128), 1);
	}
	cv::imshow("rotated_cell_fov_path", rotated_cell_fov_path_disp);
#endif

	// remap the fov path to the originally rotated cell and add the found points to the global path
	std::vector<cv::Point2f> fov_middlepoint_path_part;
	for(std::vector<cv::Point>::iterator point=current_fov_path.begin(); point!=current_fov_path.end(); ++point)
		fov_middlepoint_path_part.push_back(cv::Point2f(point->x, point->y));
	cv::transform(fov_middlepoint_path_part, fov_middlepoint_path_part, R_cell_inv);
	fov_middlepoint_path.insert(fov_middlepoint_path.end(), fov_middlepoint_path_part.begin(), fov_middlepoint_path_part.end());

#ifdef DEBUG_VISUALIZATION
	cv::Mat cell_fov_path_disp = cell_map.clone();
	for (size_t i=1; i<fov_middlepoint_path.size(); ++i)
	{
		cv::circle(cell_fov_path_disp, fov_middlepoint_path[i], 1, cv::Scalar(196), 1);
		cv::line(cell_fov_path_disp, fov_middlepoint_path[i-1], fov_middlepoint_path[i], cv::Scalar(128), 1);
	}
	cv::imshow("cell_fov_path", cell_fov_path_disp);
	cv::waitKey();
#endif

	// also update the current robot position
	std::vector<cv::Point> current_pos_vector(1, cell_robot_pos);
	cv::transform(current_pos_vector, current_pos_vector, R_cell_inv);
	robot_pos = current_pos_vector[0];
}

void BoustrophedonExplorer::downsamplePath(const std::vector<cv::Point>& original_path, std::vector<cv::Point>& downsampled_path,
		cv::Point& robot_pos, const double path_eps)
{
	// downsample path
	for(size_t path_point=0; path_point<original_path.size(); ++path_point)
	{
		if(cv::norm(robot_pos-original_path[path_point]) >= path_eps)
		{
			downsampled_path.push_back(original_path[path_point]);
			robot_pos = original_path[path_point];
		}
	}
	// add last element
	if (original_path.size() > 0)
	{
		downsampled_path.push_back(original_path.back());
		robot_pos = original_path.back();
	}
}

void BoustrophedonExplorer::downsamplePathReverse(const std::vector<cv::Point>& original_path, std::vector<cv::Point>& downsampled_path,
		cv::Point& robot_pos, const double path_eps)
{
	// downsample path
	for(size_t path_point=original_path.size()-1; ; --path_point)
	{
		if(cv::norm(robot_pos-original_path[path_point]) >= path_eps)
		{
			downsampled_path.push_back(original_path[path_point]);
			robot_pos = original_path[path_point];
		}
		if (path_point == 0)
			break;
	}
	// add last element
	if (original_path.size() > 0)
	{
		downsampled_path.push_back(original_path[0]);
		robot_pos = original_path[0];
	}
}

void BoustrophedonExplorer::printCells(std::map<int, boost::shared_ptr<BoustrophedonCell> >& cell_index_mapping)
{
	std::cout << "---\n";
	for (std::map<int, boost::shared_ptr<BoustrophedonCell> >::iterator itc=cell_index_mapping.begin(); itc!=cell_index_mapping.end(); ++itc)
	{
		std::cout << itc->first << ": l=" << itc->second->label_ << "   a=" << itc->second->area_ << "   n=";
		for (BoustrophedonCell::BoustrophedonCellSetIterator its=itc->second->neighbors_.begin(); its!=itc->second->neighbors_.end(); ++its)
			std::cout << (*its)->label_ << ", ";
		std::cout << std::endl;
	}
}


/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// BoustrophedonVariantExplorer
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


void BoustrophedonVariantExplorer::mergeCellsSelection(cv::Mat& cell_map, cv::Mat& cell_map_labels, std::map<int, boost::shared_ptr<BoustrophedonCell> >& cell_index_mapping,
		const double min_cell_area, const int min_cell_width)
{
	// iteratively merge cells
	//todo:
	// - take one major cell (the largest) and its major direction
	// - merge every other cell into the major cell, except
	//   - the width along the major direction is too small and the cell is sufficiently large
	//   - the bounding box orientation (side length ratio) deviates strongly from the major direction
	//   - the cell main direction is not well aligned with the major direction (skew, 90 deg)


	RoomRotator room_rotator;
	//double rotation_angle = room_rotator.computeRoomMainDirection(cell_map, map_resolution);

	// merge small cells below min_cell_area with their largest neighboring cell
	std::multimap<double, boost::shared_ptr<BoustrophedonCell> > area_to_region_id_mapping;		// maps the area of each cell --> to the respective cell
	for (std::map<int, boost::shared_ptr<BoustrophedonCell> >::iterator itc=cell_index_mapping.begin(); itc!=cell_index_mapping.end(); ++itc)
		area_to_region_id_mapping.insert(std::pair<double, boost::shared_ptr<BoustrophedonCell> >(itc->second->area_, itc->second));
	for (std::multimap<double, boost::shared_ptr<BoustrophedonCell> >::iterator it=area_to_region_id_mapping.begin(); it!=area_to_region_id_mapping.end();)
	{
		// abort if no cells below min_cell_area remain unmerged into bigger cells
		if (it->first >= min_cell_area && it->second->bounding_box_.width >= min_cell_width && it->second->bounding_box_.height >= min_cell_width)
		{
			++it;
			continue;
		}

		// skip segments which have no neighbors
		if (it->second->neighbors_.size() == 0)
		{
			std::cout << "WARN: BoustrophedonExplorer::mergeCells: skipping small cell without neighbors." << std::endl;
			++it;
			continue;
		}

		// determine the largest neighboring cell
		const BoustrophedonCell& small_cell = *(it->second);
		std::multimap<double, boost::shared_ptr<BoustrophedonCell>, std::greater<double> > area_sorted_neighbors;
		for (BoustrophedonCell::BoustrophedonCellSetIterator itn = small_cell.neighbors_.begin(); itn != small_cell.neighbors_.end(); ++itn)
			area_sorted_neighbors.insert(std::pair<double, boost::shared_ptr<BoustrophedonCell> >((*itn)->area_, *itn));
		BoustrophedonCell& large_cell = *(area_sorted_neighbors.begin()->second);

		// merge the cells
		mergeTwoCells(cell_map, cell_map_labels, small_cell, large_cell, cell_index_mapping);

		// update area_to_region_id_mapping
		area_to_region_id_mapping.clear();
		for (std::map<int, boost::shared_ptr<BoustrophedonCell> >::iterator itc=cell_index_mapping.begin(); itc!=cell_index_mapping.end(); ++itc)
			area_to_region_id_mapping.insert(std::pair<double, boost::shared_ptr<BoustrophedonCell> >(itc->second->area_, itc->second));
		it = area_to_region_id_mapping.begin();

#ifdef DEBUG_VISUALIZATION
//		printCells(cell_index_mapping);
//		cv::imshow("cell_map",cell_map);
//		cv::waitKey();
#endif
	}

	// label remaining border pixels with label of largest neighboring region label
	for (int v=1; v<cell_map.rows-1; ++v)
	{
		for (int u=1; u<cell_map.cols-1; ++u)
		{
			if (cell_map.at<uchar>(v,u) == BORDER_PIXEL_VALUE)
			{
				std::set<int> neighbor_labels;
				for (int dv=-1; dv<=1; ++dv)
				{
					for (int du=-1; du<=1; ++du)
					{
						const int& val = cell_map_labels.at<int>(v+dv,u+du);
						if (val>0)
							neighbor_labels.insert(val);
					}
				}
				if (neighbor_labels.size() > 0)
				{
					int new_label = -1;
					for (std::multimap<double, boost::shared_ptr<BoustrophedonCell> >::reverse_iterator it=area_to_region_id_mapping.rbegin(); it!=area_to_region_id_mapping.rend(); ++it)
					{
						if (neighbor_labels.find(it->second->label_) != neighbor_labels.end())
						{
							cell_map_labels.at<int>(v,u) = it->second->label_;
							break;
						}
					}
				}
				else
					std::cout << "WARN: BoustrophedonExplorer::mergeCells: border pixel has no labeled neighbors." << std::endl;
			}
		}
	}
}

//void BoustrophedonVariantExplorer::computeCellDecomposition(const cv::Mat& room_map, const float map_resolution, const double min_cell_area,
//		std::vector<GeneralizedPolygon>& cell_polygons, std::vector<cv::Point>& polygon_centers)
//{
//	std::cout << "Calling BoustrophedonVariantExplorer::computeCellDecomposition..." << std::endl;
//
//	// *********************** II. Sweep a slice trough the map and mark the found cell boundaries. ***********************
//	// create a map copy to mark the cell boundaries
//	cv::Mat cell_map = room_map.clone();
//#ifdef DEBUG_VISUALIZATION
//	cv::imshow("cell_map", cell_map);
//#endif
//
//
//	// *********************** III. Find the separated cells. ***********************
//	std::vector<std::vector<cv::Point> > cells;
//	cv::Mat cell_copy = cell_map.clone();
//	correctThinWalls(cell_copy);	// just adds a few obstacle pixels to avoid merging independent segments
//	cv::findContours(cell_copy, cells, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
//#ifdef DEBUG_VISUALIZATION
////	 testing
////	cv::Mat black_map = cv::Mat(cell_map.rows, cell_map.cols, cell_map.type(), cv::Scalar(0));
////	for(size_t i=0; i<cells.size(); ++i)
////	{
////		cv::drawContours(black_map, cells, i, cv::Scalar(127), CV_FILLED);
////		cv::imshow("contours", black_map);
////		cv::waitKey();
////	}
//#endif
//
//	// create generalized Polygons out of the contours to handle the cells
//	for(size_t cell=0; cell<cells.size(); ++cell)
//	{
//		if(cv::contourArea(cells[cell])>=min_cell_area)
//		{
//			GeneralizedPolygon current_cell(cells[cell], map_resolution);
//			cell_polygons.push_back(current_cell);
//			polygon_centers.push_back(current_cell.getCenter());
//		}
//	}
//}
