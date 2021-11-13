
#include <vector>
#include <unordered_map>
#include "eigen3/Eigen/Dense"
#include "vector_map/vector_map.h"
#include "shared/util/random.h"

#ifndef NAVIGATION_H
#define NAVIGATION_H

namespace ros {
    class NodeHandle;
}  // namespace ros

namespace navigation {

    struct PathOption {
        float curvature;
        float clearance;
        float free_path_length;
        Eigen::Vector2f obstruction;
        Eigen::Vector2f closest_point;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

	struct Node {
	  Eigen::Vector2f loc;
	  float cost;
	  int parent;
	  float pose;
	  // constructor
	  Node(Eigen::Vector2f loc_in, float c_in, int p_in, float pose_in) : loc(loc_in), cost(c_in), parent(p_in), pose(pose_in) {}
	};

    class Navigation {
    public:

        // Constructor
        explicit Navigation(const std::string& map_file, ros::NodeHandle* n);

        // Used in callback from localization to update position.
        void UpdateLocation(const Eigen::Vector2f& loc, float angle);

        //Load the map
        void InitializeMap(const std::string& map_file);

        // Used in callback for odometry messages to update based on odometry.
        void UpdateOdometry(const Eigen::Vector2f& loc,
            float angle,
            const Eigen::Vector2f& vel,
            float ang_vel);

        // Updates based on an observed laser scan
        void ObservePointCloud(const std::vector<Eigen::Vector2f>& cloud,
            double time);

        // Main function called continously from main
        void Run();
        // Used to set the next target pose.
        void SetNavGoal(const Eigen::Vector2f& loc, float angle);
        // Time Optimal Control
        void TOC();
        // Calculate Free Path Length
        float CalcFreePathLen(float r, float x, float y);
        // Calculate Path Score
        float CalcPathScore(float fpl, float clearance, float curvature, float r, bool isStr8);
        // Find best curvature, with it's FPL, given a point p=(x,y)
        void FindBestPath(const std::vector<Eigen::Vector2f>& p);
        // Calculate Clearance
        float CalcClearance(float r, float x, float y, float fpl, float rmax, float rmin);
        // Draw car outline and selected path
        void DrawCarAndPath();
        // Draw Graph with edges
        void DrawTreeandCarrot();
        // Find carrot in base_link frame
        void CarrotPositioning();
			
        // Draw random sample in map
        Eigen::Vector2f Sample(float Cmax);
		// Move randomly sampled point close to Tree
        void Steer(Eigen::Vector2f x_start, Eigen::Vector2f x_goal, float steering_angle, Eigen::Vector2f& xnew, float& xnew_angle);
        //extend edges keeping in view the kinematic constraints 
		void extend (Eigen::Vector2f& x, float& xnew_angle, float angle, Eigen::Vector2f& x_start, int k, float step_len_);
        // Test if the random sample point is valid
        bool CollisionFree(Eigen::Vector2f start, Eigen::Vector2f end);
        // Find all nodes within a given range from new random sample
        std::vector<int> Near(Eigen::Vector2f Xnew);
        // Find closest node to new random sample
        int Nearest(Eigen::Vector2f& Xnew);
        // Create path to follow
        void CreatePath(int goal_ndx);
        // Informed RRT* algorithm
        void InformedRRTstar();
        // Calculate cost by traversing Tree
        bool CostMakesSense(int current);
        // Extra obstacles
        void CreateObstacles(float unit, int number_of_points);
        // Used by CreateObstacles()
        double GenRand(double max, double min );
		// Find end point of arc given other end point and curvature
		Eigen::Vector2f find_endpoint(float Ax, float Ay, float C, float start_angle, float L, bool clockwise);
		//obstacles from the paper, added to map and visualization
		void FigureDisplay(double yg, double hg, double h, double x1, double y1);
		void FigureAddMap(double yg, double hg, double h, double x1, double y1);
        
		
	private:

		// Map of the environment.
		vector_map::VectorMap map_;

		// Random number generator.
		util_random::Random rng_;
  
        // Current robot location in Map ref frame
        Eigen::Vector2f robot_loc_;
        // Current robot orientation in Map ref frame
        float robot_angle_;
        // Current robot velocity in base_link ref frame
        Eigen::Vector2f robot_vel_;
        // Current robot angular speed in base_link ref frame
        float robot_omega_;
        // Odometry-reported robot location.
        Eigen::Vector2f odom_loc_;
        // Odometry-reported robot angle.
        float odom_angle_;

        // Whether navigation is complete.
        bool nav_complete_ = false;
        // Navigation goal location.
        Eigen::Vector2f nav_goal_loc_;
        // Navigation goal angle.
        float nav_goal_angle_;

        /* MASC STUFF */
        Eigen::Vector2f initial_odom_loc_; // Initial odometry-reported robot location.
        float initial_odom_angle_; // Initial odometry-reported robot angle.
        bool is_odom_init_ = false; // Has the odometry been set yet?
        bool nav_target_set_ = false; // Has a nav target been identified?
        float dt_ = 0.05; // Length of one iteration
        float setVel_ = 0.0; // m/s
        float max_accel_ = 4.0; // m/s/s
        float max_decel_ = 4.0; // m/s/s
        float max_vel_ = 2.0; // m/s
        float latency_ = 0.015; // sec
        float safety_margin_ = 0.05; // meters
        float maxPtCar_x_ = (0.535 + 0.324) / 2 + safety_margin_;
        float minPtCar_x_ = (0.535 - 0.324) / 2 + safety_margin_;
        float maxPtCar_y_ = 0.281 / 2 + safety_margin_;
        float maxSensorRange_ = 10.0; // m
        float goal_dist_ = 2.5; // meters
		float curr_fpl_ = 0.0; // meters
        float curr_curvature_ = 0.0; // 1/m
        float curr_clearance_ = 2.0; // meters
        float maxClearance_ = 1.5; //
        float dist_to_stop_; // meters
		bool need_to_replan_ = true; // we have deviated beyond goal_dist_ from the path
		int curr_path_ndx_ = 0;
		bool path_is_ready_ = false;
		
		float safe_distance_ = maxPtCar_y_;
		Eigen::Vector2f start_loc_;
        Eigen::Vector2f immediate_goal_;
        
        // Informed RRT*
        std::vector<Node> Tree_;
        std::vector<int> soln_;
        std::vector<int> path_;
        int max_iter_ = 1000;
        Eigen::Rotation2Df Rot_ = Eigen::Rotation2Df(0.0);
		float Cmin_; // distance from Xstart to Xgoal
		Eigen::Vector2f Xcenter_; // mid-point between Xstart & Xgoal
		float step_len_ = 0.25;
		float delta_ = 0.5;
		Eigen::Vector2f map_space_x_ = Eigen::Vector2f(-42.0 , 42.0);
		Eigen::Vector2f map_space_y_ = Eigen::Vector2f(-10.0 , 23.0);
		float eta_ = 20.0;
		float gamma_rrt_star_ = 20; //sqrtf(2 * 1.5) * sqrt(10);
		float steer_kinematic_angle_constraint_ = M_PI/1.25;
		float curvature_range_ = 2.0;
		std::vector<float> curvatures_;
    };

}  // namespace navigation

#endif  // NAVIGATION_H