
#include "gflags/gflags.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "amrl_msgs/AckermannCurvatureDriveMsg.h"
#include "amrl_msgs/Pose2Df.h"
#include "amrl_msgs/VisualizationMsg.h"
#include "glog/logging.h"
#include "ros/ros.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"
#include "shared/ros/ros_helpers.h"
#include "navigation.h"
#include "visualization/visualization.h"
#include <algorithm>
#include "simple_queue.h"
#include <unordered_map>
#include <chrono> 
#include <iostream>

using namespace std::chrono;
using Eigen::Vector2f;
using Eigen::Rotation2Df;
using amrl_msgs::AckermannCurvatureDriveMsg;
using amrl_msgs::VisualizationMsg;
using std::string;
using std::vector;
using visualization::DrawPathOption;
using visualization::DrawArc;
using visualization::DrawPoint;
using visualization::ClearVisualizationMsg;
using visualization::DrawLine;
using visualization::DrawCross;
using geometry::line2f;

using namespace math_util;
using namespace ros_helpers;

namespace {
	ros::Publisher drive_pub_;
	ros::Publisher viz_pub_;
	VisualizationMsg local_viz_msg_;
	VisualizationMsg global_viz_msg_;
	AckermannCurvatureDriveMsg drive_msg_;
	// Epsilon value for handling limited numerical precision.
	const float kEpsilon = 1e-5;
} //namespace

namespace navigation {


	Navigation::Navigation(const std::string& map_file, ros::NodeHandle* n) :
		robot_loc_(0, 0),
		robot_angle_(0),
		robot_vel_(0, 0),
		robot_omega_(0),
		nav_complete_(false),
		nav_goal_loc_(0, 0),
		nav_goal_angle_(0) {
		drive_pub_ = n->advertise<AckermannCurvatureDriveMsg>(
			"ackermann_curvature_drive", 1);
		viz_pub_ = n->advertise<VisualizationMsg>("visualization", 1);
		local_viz_msg_ = visualization::NewVisualizationMessage(
			"base_link", "navigation_local");
		global_viz_msg_ = visualization::NewVisualizationMessage(
			"map", "navigation_global");
		InitRosHeader("base_link", &drive_msg_.header);
	}

	void Navigation::InitializeMap(const std::string& map_file) {
		map_.Load("maps/" + map_file + ".txt");
		printf("Initializing map!\n");
	}
	
	void Navigation::CreateObstacles(float unit, int number_of_points) {
		for(int i=0;i<number_of_points;++i) {

			double x1 = GenRand(map_space_x_.x(), map_space_x_.y()); 
			double y1 = GenRand(map_space_y_.x(), map_space_y_.y()); 
			map_.lines.push_back(line2f(Vector2f(x1, y1), Vector2f(x1+unit, y1)));
			map_.lines.push_back(line2f(Vector2f(x1, y1), Vector2f(x1, y1+unit)));
			map_.lines.push_back(line2f(Vector2f(x1, y1+unit), Vector2f(x1+unit, y1+unit)));
			map_.lines.push_back(line2f(Vector2f(x1+unit, y1), Vector2f(x1+unit, y1+unit)));
			DrawLine(Vector2f(x1, y1), Vector2f(x1+unit, y1), 0x000000, local_viz_msg_);
			DrawLine(Vector2f(x1, y1), Vector2f(x1, y1+unit), 0x000000, local_viz_msg_);
			DrawLine(Vector2f(x1, y1+unit), Vector2f(x1+unit, y1+unit), 0x000000, local_viz_msg_);
			DrawLine(Vector2f(x1+unit, y1), Vector2f(x1+unit, y1+unit), 0x000000, local_viz_msg_);
		}	
	}

	double Navigation::GenRand(double max, double min ) {
		static std::mt19937 gen;
		static std::uniform_real_distribution<double> dis(0.0,1.0);

		return min + (max-min) * dis(gen);
	}
	
// *****************************************************************
// ****************** INFORMED RRT* ********************************
// *****************************************************************
	Vector2f Navigation::Sample(float Cmax) {
		Vector2f Xrand;
		if (Cmax<1e12) {
			printf("Sampling from ellipse: Cmax=%f, Cmin=%f\n", Cmax, Cmin_);
			// Sample a unit circle in (x,y)
			float ang = rng_.UniformRandom() * M_2PI; // random angle in [0,2pi]
			float r = sqrt(rng_.UniformRandom()); // sqrt of random radius
			Vector2f Xball(r*cosf(ang) , r*sinf(ang));
			// Transform circle to an ellipse
			Xball.x() *= 0.5 * Cmax;
			Xball.y() *= 0.5 * sqrt(Sq(Cmax)-Sq(Cmin_));
			// Rotate and translate the ellipse
			Xrand = Rot_ * Xball + Xcenter_;
		}
		else {
			// Generate random sample in the map
			float x = rng_.UniformRandom(map_space_x_.x() , map_space_x_.y());
			float y = rng_.UniformRandom(map_space_y_.x() , map_space_y_.y());
			Xrand = Vector2f(x, y);
			printf("SampleFreeSpace: (%f,%f)\n", x, y);
		}
		return Xrand;
	}
	
	Vector2f Navigation::find_endpoint(float Ax, float Ay, float C, float start_angle, float L, bool clockwise) {
		float r = 1/C; //sqrtf(Sq(Ax - Cx) + Sq(Ay - Cy));
		r = abs(r);
		float Cx, Cy;
		float angle ;
		if (clockwise) {
			Cx = Ax;
			angle = M_PI/2 + start_angle - L / r;
			Cy = Ay - r;
		}
		else {
			Cx = Ax; 
			angle = -M_PI/2 + start_angle + L / r;
			Cy = Ay + r;
		}
		float Bx = Cx + r * cosf(angle);
		float By = Cy + r * sinf(angle);
		return Vector2f(Bx, By);
}

	void Navigation::extend (Vector2f& x, float& xnew_angle, float start_angle, Vector2f& x_start, int k, float step_len_){
		
		if (curvatures_[k] == 0){
			x.x() = x_start.x() + step_len_;
			x.y() = x_start.y() ;
			xnew_angle =  start_angle;
		} 
		else{
			bool clockwise;
			if (curvatures_[k]<0)
				clockwise = false;
			else 
				clockwise = true;
			
			x = find_endpoint(x_start.x(),x_start.y(), curvatures_[k] ,start_angle, step_len_,clockwise);
			xnew_angle = atan2f(x.y()-x_start.y(), x.x()-x_start.x());
			
/* 			float delta_theta;
			Vector2f dxdy;
			Matrix2f E;
			
			E << cosf(start_angle - M_PI/2), -sinf(start_angle - M_PI/2),
				  sinf(start_angle - M_PI/2),    cosf(start_angle - M_PI/2);
			float a = sqrtf(abs(curvatures_[k])/(2*step_len_));
			float C = 0;
			float S = 0;
			for (float i = 0; i < sqrtf(2/M_PI) * a * step_len_ ; i+= 0.05){
				S+= sinf(M_PI/2)*Sq(i);
			}
			S = sqrtf(M_PI/2)* S;
			for (float i = 0; i < sqrtf(2/M_PI) * a * step_len_ ; i+= 0.05){
				C+= cosf(M_PI/2)*Sq(i);
			}
			C = sqrtf(M_PI/2)* C;
			
			float sign = 0;
			if (curvatures_[k]<0)
				sign = -1;
			else 
				sign = 1;
			dxdy = E * Vector2f(sign* 1/a * C, 1/a * S);
			delta_theta = - (curvatures_[k] * Sq(step_len_))/(2*step_len_); 
				
			x.x() = x_start.x() + dxdy.x();
			x.y() = x_start.y() + dxdy.y();
			xnew_angle = start_angle + delta_theta;
			printf("Extend %f, %f, %f, %f, %f, %f\n", x_start.x(), x_start.y(), start_angle, dxdy.x(),dxdy.y(),delta_theta);*/
		}
		
	}
	//Steer(Xnearest, Xrand, Xnearest_theta, Xnew, Xnew_angle)
	void Navigation::Steer(Vector2f x_start, Vector2f x_goal, float steering_angle, Vector2f& xnew, float& xnew_angle) { 
	    //Xnearest, Xrand, x nearest steering angle
		/* float dmin = 1e12;
		Vector2f x;
		//float k_optimal;
		float dist, x_angle;
		
		for (size_t k = 0; k < curvatures_.size() ; k++){
			extend (x, x_angle, steering_angle, x_start, k, (x_goal-x_start).norm());
			dist = (x - x_goal).norm();
			if (dist<dmin){
				dmin = dist;
				//k_optimal = k;
				xnew = x;
				xnew_angle = x_angle;
			}
		}   */
		
		//Implementation 1
		float dist = std::min(step_len_, (x_goal-x_start).norm());
		float theta = atan2f(x_goal.y()-x_start.y(), x_goal.x()-x_start.x());
		xnew_angle = theta;
		xnew = Vector2f(x_start.x() + dist * cosf(theta), x_start.y() + dist * sinf(theta)); 
		
		//Implementation 2
        /*float theta = atan2f(x_goal.y()-x_start.y(), x_goal.x()-x_start.x());
		if (theta > steer_kinematic_angle_constraint_)
			theta = steer_kinematic_angle_constraint_ ;
		else if( theta < -steer_kinematic_angle_constraint_)
			theta = -steer_kinematic_angle_constraint_; */
		
		//Implementation 3
		/* Vector2f Xnew(x_start.x() + dist * cosf(theta), x_start.y() + dist * sinf(theta));
		Vector2f Xnew_odom = Rotation2Df(-odom_angle_) * (Xnew - odom_loc_);
		float ang = atan2f(Xnew_odom.y(), Xnew_odom.x());
		if (ang > M_PI/6)
			ang = M_PI/6;
		else if( ang < -M_PI/6)
			ang = -M_PI/6;
		Xnew_odom = Vector2f(dist * cosf(theta), dist * sinf(theta)); */
		
        //return Vector2f(x_start.x() + dist * cosf(theta), x_start.y() + dist * sinf(theta));
		
	}
	bool Navigation::CollisionFree(Vector2f start, Vector2f end) {
		bool intersects = false;
		bool intersects_up = false;
		bool intersects_down = false;
		bool intersects_left = false;
		bool intersects_right= false; 
		
		Vector2f intersection_point;
		line2f my_line(start.x(), start.y(), end.x(), end.y());
		line2f virtual_line_up(start.x(), start.y()+safe_distance_, end.x(), end.y()+safe_distance_);
		line2f virtual_line_down(start.x(), start.y()-safe_distance_, end.x(), end.y()-safe_distance_);
		line2f virtual_line_left(start.x()-safe_distance_, start.y(), end.x()-safe_distance_, end.y());
		line2f virtual_line_right(start.x()+safe_distance_, start.y(), end.x()+safe_distance_, end.y());
		
		for (size_t i = 0; i < map_.lines.size() ; i++) {
			const line2f map_line = map_.lines[i];
			intersects = map_line.Intersection(my_line, &intersection_point);
			intersects_up = map_line.Intersection(virtual_line_up, &intersection_point);
			intersects_down = map_line.Intersection(virtual_line_down, &intersection_point);
			intersects_left = map_line.Intersection(virtual_line_left, &intersection_point);
			intersects_right = map_line.Intersection(virtual_line_right, &intersection_point);
			if (intersects || intersects_up || intersects_down || intersects_left || intersects_right) {
				return false;
			}
		}
		return true;
	}
	
	vector<int> Navigation::Near(Vector2f Xnew) {
		// find the vertices that are within a given dist from Xnew
		gamma_rrt_star_ = sqrtf(2 * 1.5) * sqrtf(1600/3.1416) + 10; // Hassan
		//float gamma_rrt_star = 2.0; // Jamie
		//float gamma_rrt_star = 20; // Trent 
		int N = Tree_.size();
		float radius = std::min(gamma_rrt_star_ * sqrtf(log(N)/N), eta_);
		
		vector<int> near_indices;
		for (size_t i=0 ; i<Tree_.size() ; i++) {
			if ((Xnew-Tree_[i].loc).norm() < radius) { near_indices.push_back(i); }
		}
		return near_indices;
	}
	
	int Navigation::Nearest(Vector2f& Xnew) {
		// find the closest node to Xnew
		float min_dist = (Xnew-Tree_[0].loc).norm();
		int min_ndx = 0;
		for (size_t k=1 ; k<Tree_.size() ; k++) {
			float dist_i = (Xnew-Tree_[k].loc).norm();
			if (dist_i < min_dist) {
				min_dist = dist_i;
				min_ndx = k;
			}
		}
		return min_ndx;
	}
	
	void Navigation::CreatePath(int goal_ndx) {
		path_.clear();
		int current = goal_ndx;
		int Nnodes = 0;
		while (current>=0) {
			path_.push_back(current);
			current = Tree_[current].parent;
			Nnodes++;
		}
		std::reverse(path_.begin(), path_.end());
		curr_path_ndx_ = 0;
		printf("Path created with %i nodes\n", Nnodes);
	}
	
	bool Navigation::CostMakesSense(int current) {
		float Cc = 0.0;
		float Cp1 = Tree_[current].cost;
		float Cx = 0.0;
		Vector2f Xp1 = Tree_[current].loc;
		printf("X = (%f,%f) : Cost = %f\n", Tree_[current].loc.x(), Tree_[current].loc.y(), Tree_[current].cost);
		current = Tree_[current].parent;
		while (current>=0) {
			printf("X = (%f,%f) : Cost = %f\n", Tree_[current].loc.x(), Tree_[current].loc.y(), Tree_[current].cost);
			printf("dX = %f : dC = %f\n", (Xp1 - Tree_[current].loc).norm(), Cp1 - Tree_[current].cost);
			Cc += Cp1 - Tree_[current].cost;
			Cx += (Xp1 - Tree_[current].loc).norm();
			Cp1 = Tree_[current].cost;
			Xp1 = Tree_[current].loc;
			current = Tree_[current].parent;
		}
		
		if (std::abs(Cc-Cx)>0.001) { return false; }
		else { return true; }
	}
	
	void Navigation::InformedRRTstar() {
		printf("Entering IRRT*\n");
		printf("Starting point is (%f,%f)\n", odom_loc_.x(), odom_loc_.y());
		
		auto start = high_resolution_clock::now();
		
		Tree_.clear();
		Tree_.push_back(Node(odom_loc_, 0.0f, -1, odom_angle_)); // first node
		soln_.clear();
		Cmin_ = (nav_goal_loc_-odom_loc_).norm();
		Xcenter_ = (odom_loc_+nav_goal_loc_) / 2;
		Rot_ = Rotation2Df(atan2f(nav_goal_loc_.y()-odom_loc_.y(), nav_goal_loc_.x()-odom_loc_.x()));
		float Cbest = 1e16; // init to inf
		int best_ndx = 0;
		int ndx_min = 0;
		float Cmin, Cnew, Cnear;
		Vector2f Xrand, Xnearest, Xnew, Xmin, Xnear;
		float Xnearest_theta, Xnew_angle;
		int nearest_ndx;
		int num_iter_without_change = 0;
		float last_Cbest = -1.0;
		//for (int k=0 ; k<max_iter_ ; k++) {
		while (num_iter_without_change < max_iter_) {
			// Find best path so far (if any)
			for (int ndx : soln_) {
				if (Tree_[ndx].cost < Cbest) {
					Cbest = Tree_[ndx].cost;
					best_ndx = ndx;
					num_iter_without_change = 0;
				}
			}
			if ((!soln_.empty()) && (Cbest==last_Cbest)) { num_iter_without_change++; }
			else { num_iter_without_change = 0; }
			last_Cbest = Cbest;
			
			Xrand = Sample(Cbest); // generate sample inside ellipse
			nearest_ndx = Nearest(Xrand); // find closest node in Tree to Xrand
			Xnearest = Tree_[nearest_ndx].loc;
			Xnearest_theta = Tree_[nearest_ndx].pose;
			Steer(Xnearest, Xrand, Xnearest_theta, Xnew, Xnew_angle); // if Xrand is too far away, make it closer
			printf("Xrand=(%f,%f) , Xnearest=(%f,%f) , Xnew=(%f,%f)\n", Xrand.x(), Xrand.y(), Xnearest.x(), Xnearest.y(), Xnew.x(), Xnew.y());
			if (CollisionFree(Xnearest, Xnew)) {
				Cmin = Tree_[nearest_ndx].cost + (Xnew-Xnearest).norm(); // cost to Xnew
				Xmin = Xnearest;
				ndx_min = nearest_ndx;
				//printf("Cnearest=%f + NearestNew=%f = Cnew=%f\n", Tree_[nearest_ndx].cost, (Xnew-Xnearest).norm(), Cmin);
				// Find shortest path to Xnew
				vector<int> near_indices = Near(Xnew); // find all the nodes in the neighborhood of Xnew
				for (int near_ndx : near_indices) {
					Xnear = Tree_[near_ndx].loc;
					Cnew = Tree_[near_ndx].cost + (Xnew-Xnear).norm();
					if ((Cnew<Cmin) && (CollisionFree(Xnear, Xnew))) {
						Xmin = Xnear;
						Cmin = Cnew;
						ndx_min = near_ndx;
					}
				}
				
				// Rewire
				for (int near_ndx : near_indices) {
					Cnear = Tree_[near_ndx].cost;
					Xnear = Tree_[near_ndx].loc;
					Cnew = Cmin + (Xnew-Xnear).norm(); // Cnew = Cost(Xnew) + Line(Xnew,Xnear)
					if ((Cnew<Cnear) && (CollisionFree(Xnear,Xnew))) {
						//printf("Rewiring %i to %zu\n", near_ndx, Tree_.size());
						//printf("Changing cost from %f to %f\n", Cnear, Cnew);
						Tree_[near_ndx].parent = Tree_.size(); // parent is now Xnew (which isn't added yet)
						Tree_[near_ndx].cost = Cnew;
					}
				}
				
				// Test if we're close enough to the goal
				if ((Xnew-nav_goal_loc_).norm() < 1.0) {
					// move Xnew to Xgoal
					float Cgoal = Tree_[ndx_min].cost + (nav_goal_loc_-Xmin).norm();
					Node new_node = {nav_goal_loc_, Cgoal, ndx_min, nav_goal_angle_};
					Tree_.push_back(new_node); // put Xgoal in the Tree
					soln_.push_back(Tree_.size()-1); // add index to Xnew to the solution set
					/*
					if (!CostMakesSense(soln_.back())) {
						//Vector2f Xsoln = Tree_[soln_.back()].loc;
						//printf("\nSolution found: loc=(%f,%f) , ndx=%i , cost=%f\n", Xsoln.x(), Xsoln.y(), soln_.back(), Cgoal);
						//printf("Solution cost the old fashion way=%f\n\n", C_g);
					}
					*/
				}
				else {
					Tree_.push_back(Node(Xnew, Cmin, ndx_min, Xnew_angle)); // put Xnew in the Tree
					printf("Adding Xnew to Tree: C=%f\n", Cmin);
					/*
					if (!CostMakesSense(Tree_.size()-1)) {
						printf("\nProblem!!\n\n");
						Sleep(1.0);
					}
					*/
				}
			}
			else { printf("Collision ... Try Again\n\n"); }
			if (Tree_.size() % 20 == 0) {
				// Draw Tree
				for (Node node : Tree_) {
					if (node.parent==-1) { continue; } // this is the root
					DrawLine(Tree_[node.parent].loc, node.loc, 0xffd970, global_viz_msg_);
				}
				viz_pub_.publish(global_viz_msg_);
				ClearVisualizationMsg(global_viz_msg_);
			}
		} // iterations
		
		// Create path
		CreatePath(best_ndx);
		path_is_ready_ = true;
		auto stop = high_resolution_clock::now(); 
		auto duration = duration_cast<microseconds>(stop - start);
		printf("Duration for finding path\n");
		cout << duration.count() << endl; 
	}
	
		void Navigation::FigureAddMap(double yg, double hg, double h, double x1, double y1){
		//lower rectangle 
	 	map_.lines.push_back(line2f(Vector2f(x1, y1), Vector2f(x1+yg, y1)));
		map_.lines.push_back(line2f(Vector2f(x1, y1), Vector2f(x1, y1+yg)));
		map_.lines.push_back(line2f(Vector2f(x1, y1+yg), Vector2f(x1+yg, y1+yg)));
		map_.lines.push_back(line2f(Vector2f(x1+yg, y1), Vector2f(x1+yg, y1+yg))); 

		
		//upper rectangle	
		double new_y = y1+yg+hg;
		double new_delta = h -hg-yg;
		map_.lines.push_back(line2f(Vector2f(x1, new_y), Vector2f(x1+yg, new_y)));
		map_.lines.push_back(line2f(Vector2f(x1, new_y), Vector2f(x1, new_y+new_delta)));
		map_.lines.push_back(line2f(Vector2f(x1, new_y+new_delta), Vector2f(x1+yg, new_y+new_delta)));
		map_.lines.push_back(line2f(Vector2f(x1+yg, y1+ yg+hg), Vector2f(x1+yg, new_y+new_delta))); 
	}
	
	void Navigation::FigureDisplay(double yg, double hg, double h, double x1, double y1){
		//lower rectangle 
		DrawLine(Vector2f(x1, y1), Vector2f(x1+yg, y1), 0x000000, global_viz_msg_);
		DrawLine(Vector2f(x1, y1), Vector2f(x1, y1+yg), 0x000000, global_viz_msg_);
		DrawLine(Vector2f(x1, y1+yg), Vector2f(x1+yg, y1+yg), 0x000000, global_viz_msg_);
		DrawLine(Vector2f(x1+yg, y1), Vector2f(x1+yg, y1+yg), 0x000000, global_viz_msg_);
		
		//upper rectangle	
		double new_y = y1+yg+hg;
		double new_delta = h -hg-yg;
		DrawLine(Vector2f(x1, new_y), Vector2f(x1+yg, new_y), 0x000000, global_viz_msg_);
		DrawLine(Vector2f(x1, new_y), Vector2f(x1, new_y+new_delta), 0x000000, global_viz_msg_);
		DrawLine(Vector2f(x1, new_y+new_delta), Vector2f(x1+yg, new_y+new_delta), 0x000000, global_viz_msg_);
		DrawLine(Vector2f(x1+yg, y1+ yg+hg), Vector2f(x1+yg, new_y+new_delta), 0x000000, global_viz_msg_);	
		
	}
// *****************************************************************
// **************** END INFORMED RRT* ******************************
// *****************************************************************

	void Navigation::SetNavGoal(const Vector2f& loc, float angle) {
		printf("Setting Nav Goal to %f %f\n", loc[0], loc[1]);
		nav_goal_loc_ = loc;
		nav_goal_angle_ = angle;
		nav_target_set_ = true;
		path_is_ready_ = false;
		
		InformedRRTstar();
	}

	void Navigation::UpdateLocation(const Eigen::Vector2f& loc, float angle) {
		start_loc_ = loc;
	}

	void Navigation::UpdateOdometry(const Vector2f& loc,
		float angle,
		const Vector2f& vel,
		float ang_vel) {

		if (!is_odom_init_) // First time odometry is available
		{
			initial_odom_loc_ = loc;
			initial_odom_angle_ = angle;
			is_odom_init_ = true;
			
			//populate admissible curvatures for steer function
			curvatures_.clear();
			for (float curvature = -curvature_range_; curvature <= curvature_range_; curvature += 0.05)
			{
				curvatures_.push_back(curvature);
			}
			
			FigureAddMap(1.5,0.45,6.0,32.0,-18.0);
			FigureDisplay(1.5,0.45,6.0,32.0,-18.0);
			//viz_pub_.publish(global_viz_msg_);
		}

		odom_loc_ = loc;
		odom_angle_ = angle;
		robot_vel_ = vel;
		robot_omega_ = ang_vel;
	}

	void Navigation::ObservePointCloud(const vector<Vector2f>& cloud, double time) {
		for (Vector2f point : cloud) {
			DrawPoint(point, 0xe62020, local_viz_msg_);
		}
		if (!nav_complete_ && path_is_ready_) {
			FindBestPath(cloud);
		}
	}

	void Navigation::TOC() {
		float obs_vel = robot_vel_.norm();

		dist_to_stop_ = Sq(obs_vel) / (2.0 * max_decel_) + obs_vel * (dt_ + latency_) + safety_margin_;
		//printf("Dist to stop = %f\n", dist_to_stop);

		if (curr_fpl_ < dist_to_stop_) {
			printf("Decel : %f\n", curr_fpl_);
			setVel_ = std::max(0.0f, setVel_ - (max_decel_ * dt_));
		}
		else if (setVel_ < max_vel_) {
			printf("Accel : %f\n", curr_fpl_);
			setVel_ = std::min(max_vel_, setVel_ + (max_accel_ * dt_));
		}
		else {
			printf("Cruise : %f\n", curr_fpl_);
			setVel_ = max_vel_;
		}
	}

	float Navigation::CalcFreePathLen(float r, float x, float y) {
		// r is always positive
		float L = 10000;

		float rmax = sqrtf(Sq(std::abs(r) + maxPtCar_y_) + Sq(maxPtCar_x_));
		//printf("rmax : %f  ", rmax);
		float rmin = r - maxPtCar_y_;
		//printf("rmin : %f  ", rmin);
		float robs = sqrtf(Sq(x) + Sq(y - r));
		//printf("robs : %f\n", robs);

		if ((robs >= rmin) & (robs <= rmax) & (x > 0.0)) {  // there's a collision
			float ang_car;
			float ang_obs = atan2f(x, r - y);
			float rmid = sqrtf(Sq(rmin) + Sq(maxPtCar_x_));
			if (robs >= rmid) { // front collision
				ang_car = atan2f(maxPtCar_x_, rmin);
				//printf("Front Collision: ang_obs=%f , ang_car=%f , rorxrm=(%f,%f,%f) , \n", ang_obs, ang_car, robs, rmax, rmin);
				//printf("Front Collision\n");
			}
			else { // side collision
				ang_car = acosf(rmin / robs);
				//printf("Side Collision: ang_car=%f\n", ang_car);
				//printf("Side Collision\n");
			}
			//if (ang_car>ang_obs) { printf("Negative L : robs=(%f) , p = (%f , %f)\n", robs, x, y); }

			L = r * std::max(0.0f, (ang_obs - ang_car));
		}

		if ((x < 0.0) & (x >= -minPtCar_x_) & (y < 0.0)) { // tail end might collide
			float rback = sqrtf(Sq(maxPtCar_y_) + Sq(minPtCar_x_));
			float magP = sqrtf(Sq(x) + Sq(y));
			if (rback >= magP) {
				L = 0.0;
				printf("Back Collision p=(%f,%f) robs=(%f) rback=(%f)\n", x, y, robs, rback);
				sleep(3);
			}
		}

		return L;
	}

	float Navigation::CalcPathScore(float fpl, float clearance, float curvature, float r, bool isStr8) {
		if (fpl < dist_to_stop_) { return -1000.0; } // path too short: no longer an option
		if (clearance < 0.0) { return -1000.0; } // path doesn't have enough clearance: no longer an option

		float w0 = 1.0; // fpl/goal_dist_
		float w1 = 1.0 / curr_clearance_; // clearance
		float w2 = 1.0; // dist_toward_goal
		float w3 = -0.125; // delta_curvature
		float score;

		//compute dist toward goal
		Vector2f vec_path;
		if (isStr8) { vec_path = Vector2f(fpl , 0.0); } // zero curvature
		else {
			float theta = fpl / r;
			vec_path = Vector2f(r*sinf(theta) , r*(1.0-cosf(theta)));
		}
		float dist_toward_goal = immediate_goal_.norm() - (immediate_goal_-vec_path).norm();
		//float dist_toward_goal;
		//if (isStr8) { dist_toward_goal = fpl; } // zero curvature
		//else { dist_toward_goal = abs(r) * sinf(fpl * abs(curvature)); } // non-zero curvature
		
		float s1 = fpl / goal_dist_;
		float s2 = std::min(1.0f, clearance / maxClearance_);
		float s3 = dist_toward_goal / fpl;
		float s4 = Sq(Sq(curr_curvature_ - curvature));

		score = w0 * s1 + w1 * s2 + w2 * s3 + w3 * s4;
		//printf("Stats for curv %f: fpl=%f , clc=%f , dtg=%f , dcv=%f, score=%f\n", curvature, s1, s2, s3, s4, score);

		return score;
	}

	void Navigation::FindBestPath(const vector<Vector2f>& point_cloud) {
		float r, L;
		float bestFPL = goal_dist_;
		float bestCurvature = 0.0;
		float fpl = goal_dist_;
		float bestClearance = maxClearance_;
		float clearance = maxClearance_;

		printf("Curr Clearance = %f\n", curr_clearance_);
		// ----- ZERO CURVATURE -----
		for (Vector2f p : point_cloud) {
			if ((abs(p.y()) <= maxPtCar_y_) & (p.x() >= maxPtCar_x_) & (p.x() <= immediate_goal_.x())) { // observation is directly in front of me
				fpl = std::min(fpl, p.x() - maxPtCar_x_);
			}
		}
		for (Vector2f p : point_cloud) {
			if ((p.x() <= fpl + maxPtCar_x_) & (p.x() >= maxPtCar_x_)) {
				clearance = std::min(clearance, abs(p.y()) - maxPtCar_y_);
			}
		}
		float bestScore = CalcPathScore(fpl, clearance, 0.0f, 0.0f, true); // zero curvature
		bestFPL = fpl;
		bestClearance = clearance;
		DrawLine(Vector2f(0.0,0.0), Vector2f(fpl,0.0), 0xc0c0c0, local_viz_msg_);
		
		float score;
		for (float curvature = 0.05; curvature <= 1.0; curvature += 0.05) {
			r = 1 / curvature;
			clearance = maxClearance_;
			float rmax = sqrtf(Sq(r + maxPtCar_y_) + Sq(maxPtCar_x_));
			float rmin = r - maxPtCar_y_;

			// ----- POSITIVE CURVATURE -----
			if (immediate_goal_.y() > r) {
				fpl = r * std::min(M_PI_2, (M_PI - atan2(immediate_goal_.x(), immediate_goal_.y()-r)));
			}
			else {
				fpl = r * std::min((float)M_PI_2, atan2(immediate_goal_.x(), r-immediate_goal_.y())); // length of arc at min dist_goal
			}
			//printf("Orig FPL for curv %f : %f\n", curvature, fpl);
			for (Vector2f p : point_cloud) {
				L = CalcFreePathLen(r, p.x(), p.y());
				if (L < fpl) { fpl = L; }
			}
			for (Vector2f p : point_cloud) {
				clearance = std::min(clearance, CalcClearance(r, p[0], p[1], fpl, rmax, rmin));
			}
			DrawArc(Vector2f(0.0,r), r, -M_PI/2.0, fpl*curvature-(M_PI/2.0), 0xc0c0c0, local_viz_msg_);
			score = CalcPathScore(fpl, clearance, curvature, r, false);
			if (score > bestScore) {
				bestScore = score;
				bestCurvature = curvature;
				bestFPL = fpl;
				bestClearance = clearance;
			}

			// ----- NEGATIVE CURVATURE -----
			if (-immediate_goal_.y() > r) {
				fpl = r * std::min(M_PI_2, (M_PI - atan2(immediate_goal_.x(), -immediate_goal_.y()-r)));
			}
			else {
				fpl = r * std::min((float)M_PI_2, atan2(immediate_goal_.x(), r+immediate_goal_.y())); // length of arc at min dist_goal
			}
			clearance = maxClearance_;
			//printf("Orig FPL for curv %f : %f\n", -curvature, fpl);
			for (Vector2f p : point_cloud) {
				L = CalcFreePathLen(r, p[0], -p[1]);
				if (L < fpl) { fpl = L; }
			}
			for (Vector2f p : point_cloud) {
				clearance = std::min(clearance, CalcClearance(r, p[0], -p[1], fpl, rmax, rmin));
			}
			DrawArc(Vector2f(0.0,-r), r, M_PI/2.0-(fpl*curvature), M_PI/2.0, 0xc0c0c0, local_viz_msg_);
			score = CalcPathScore(fpl, clearance, -curvature, -r, false);
			if (score > bestScore) {
				bestScore = score;
				bestCurvature = -curvature;
				bestFPL = fpl;
				bestClearance = clearance;
			}
		} // loop over curvatures
		printf("Best path found to be (%f , %f)\n", bestFPL, bestCurvature);
		//}
		curr_fpl_ = bestFPL;
		curr_curvature_ = bestCurvature;
		curr_clearance_ = bestClearance;
	}

	float Navigation::CalcClearance(float r, float x, float y, float fpl, float rmax, float rmin) {
		// r is always positive
		float clearance = 100.0; // arbitrary large number
		float ang_max = fpl / r + atan2f(maxPtCar_x_, rmin); // ang from base_link to tip of car at fpl
		float robs = sqrtf(Sq(x) + Sq(y - r)); // can optimize by storing this calculation elsewhere
		float ang_obs = atan2f(x, r - y);

		if ((ang_obs > 0.0) & (ang_obs <= ang_max)) { // points in consideration
			if (robs > r) { clearance = robs - rmax; }
			else { clearance = rmin - robs; }
		}

		return clearance;
	}

	void Navigation::DrawCarAndPath() {
		DrawLine(Vector2f(-minPtCar_x_, maxPtCar_y_), Vector2f(maxPtCar_x_, maxPtCar_y_), 0x000000, local_viz_msg_);
		DrawLine(Vector2f(maxPtCar_x_, maxPtCar_y_), Vector2f(maxPtCar_x_, -maxPtCar_y_), 0x000000, local_viz_msg_);
		DrawLine(Vector2f(-minPtCar_x_, -maxPtCar_y_), Vector2f(maxPtCar_x_, -maxPtCar_y_), 0x000000, local_viz_msg_);
		DrawLine(Vector2f(-minPtCar_x_, maxPtCar_y_), Vector2f(-minPtCar_x_, -maxPtCar_y_), 0x000000, local_viz_msg_);
		DrawPathOption(curr_curvature_, curr_fpl_, curr_clearance_, local_viz_msg_);
	}

	void Navigation::DrawTreeandCarrot() {
		// Draw Tree
		for (Node node : Tree_) {
			if (node.parent==-1) { continue; } // this is the root
			DrawLine(Tree_[node.parent].loc, node.loc, 0xffd970, global_viz_msg_);
		}
		
		for (size_t k = 0; k < path_.size() - 1; k++) {
			Vector2f vec1(Tree_[path_[k]].loc.x() , Tree_[path_[k]].loc.y());
			Vector2f vec2(Tree_[path_[k+1]].loc.x(), Tree_[path_[k+1]].loc.y());
			DrawLine(vec1, vec2, 0x620200, global_viz_msg_);
		}
		DrawCross(immediate_goal_, 0.2, 0xFF0000, local_viz_msg_);
		DrawCross(nav_goal_loc_, 0.2, 0xFF0000, global_viz_msg_);
		//DrawArc(Vector2f(0, 0), goal_dist_, -M_PI, M_PI, 0x800080, local_viz_msg_);
	}
	
	void Navigation::CarrotPositioning() {
		need_to_replan_ = true;
		//int pathSize_m1 = path_.size() - 1;
		float dist_to_path = goal_dist_;
		float min_dist_to_path = 100000.0;
		int closest_path_ndx = curr_path_ndx_;
		for (size_t k = curr_path_ndx_; k < path_.size(); k++) {
			dist_to_path = (Tree_[path_[k]].loc - odom_loc_).norm();
			if (dist_to_path < min_dist_to_path) {
				min_dist_to_path = dist_to_path;
				closest_path_ndx = k;
				need_to_replan_ = false;
			}
			
			if (dist_to_path > goal_dist_) {
				immediate_goal_ = Rotation2Df(-odom_angle_) * (Tree_[path_[k]].loc - odom_loc_);
				break;
			}
		}
		curr_path_ndx_ = closest_path_ndx;
		
		// if immediate goal is behind base_link.x -> replan
		if (immediate_goal_.x()<0.0) { need_to_replan_ = true; }
		
		printf("Immediate goal=(%f,%f)\n", immediate_goal_.x(), immediate_goal_.y());
	}
	
	void Navigation::Run() {
		if (!is_odom_init_) {
			printf("odom not initialized ... skipping\n");
			return;
		}
		if (!nav_target_set_) {
			printf("nav target not set ... skipping\n");
			return;
		}
		if (!path_is_ready_) {
			printf("path isn't ready ... skipping\n");
			return;
		}
		//double curr_time = GetMonotonicTime(); // arbitrary time reference

		// Goal Reached
		if ((nav_goal_loc_ - odom_loc_).norm() < goal_dist_) {
			printf("Nav target reached!\n");
			nav_complete_ = true; // FindBestPath() no longer called
			immediate_goal_ = Rotation2Df(-odom_angle_) * (nav_goal_loc_ - odom_loc_);
			if (immediate_goal_.norm() > curr_fpl_) { curr_fpl_ = 0.0; } // stop if we start to get further away from the target
			else {
				curr_fpl_ = immediate_goal_.norm();
				//curr_curvature_ = 0.0;
			}
	    }
	    else {
			// Get the Next Carrot, Return need_to_replan_=true if no carrot available
			CarrotPositioning();
			
			// Need to Re-Calculate the Path
			if (need_to_replan_) {
				printf("Recalculating path!\n");
				path_is_ready_ = false;
				while (!path_is_ready_){
					drive_msg_.velocity = 0.0; // m/s
					drive_msg_.curvature = 0.0; // 1/radius
					drive_pub_.publish(drive_msg_);
					InformedRRTstar();
				}
				CarrotPositioning();
				//Sleep(2.0);
				//return;
			}
		}
		
		DrawCarAndPath();
		DrawTreeandCarrot();
		TOC();
	    
		//printf("Odom: (%f,%f) \n", odom_loc_.x(), odom_loc_.y());
		//printf("Goal: (%f,%f) \n", nav_goal_loc_.x(), nav_goal_loc_.y());

		printf("Setting vel(%f) : Setting curv(%f)\n", setVel_, curr_curvature_);
		drive_msg_.velocity = setVel_; // m/s
		drive_msg_.curvature = curr_curvature_; // 1/radius
		drive_pub_.publish(drive_msg_);
		viz_pub_.publish(local_viz_msg_);
		ClearVisualizationMsg(local_viz_msg_);
		viz_pub_.publish(global_viz_msg_);
		ClearVisualizationMsg(global_viz_msg_);
	}

}   // namespace navigation