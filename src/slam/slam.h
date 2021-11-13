
#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"

#ifndef SRC_SLAM_H_
#define SRC_SLAM_H_

namespace slam {

class SLAM {
 public:
  // Default Constructor.
  SLAM();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Get latest map.
  std::vector<Eigen::Vector2f> GetMap();

  // Get latest robot pose.
  void GetPose(Eigen::Vector2f* loc, float* angle) const;
  
  // Get XY-coord of scans in base_link frame
  void GetScanXY(const std::vector<float>& ranges,
					 float range_max,
					 float angle_min,
					 float angle_max,
					 float buffer,
					 std::vector<Eigen::Vector2f>* scan_ptr);
  
 // Get XY.......
 void GetScanXY(const std::vector<float>& ranges,
					 float range_max,
					 float angle_min,
					 float angle_max,
					 Eigen::Vector2f ref_loc,
					 float ref_ang,
					 std::vector<Eigen::Vector2f>* scan_ptr);
 
 // Add points to map
 void AugmentMap(const std::vector<Eigen::Vector2f>& scan_xy);
 
 // Calc Size & Instantiate Lookup Table
 void InitializeLUT();

 // Create Lookup Table for p(z|x,m) Calculation
 void CreateLookupTable(const std::vector<Eigen::Vector2f>& scanXY);
 
 // Extract values from Lookup Table
 float IndexLookupTable(const std::vector<Eigen::Vector2f>& points, Eigen::Vector2f dxdy);


 private:

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_ = false;
  bool LUT_initialized_ = false;
  // MASC STUFF
  std::vector<Eigen::Vector2f> map_; // map points (all of them!!)
			// Caveat: this method cannot change the map aposteriori since the poses & measurements aren't preserved
			//           I'm doing this because I didn't want to figure out how to store all the measurements in a manner
			// 			 that could be indexed from the pose index.
  std::vector<Eigen::Vector3f> poses_; // (x,y,ang) for all poses
  //std::vector<std::vector<Eigen::Vector2f>> scans_; // all the laser scans for each pose
  float dist_since_update_ = 0.0; // [m] translation since last SLAM update
  Eigen::Vector2f dxdy_since_update_ = Eigen::Vector2f(0.0 , 0.0); // [m] delta (x,y) since last SLAM update
  float ang_since_update_ = 0.0; // [rad] rotation since last SLAM update
  float abs_ang_since_update_ = 0.0; // [rad] abs rotation since last SLAM update
  std::vector<std::vector<float>> LUT_;
  float LUT_x_start_;
  float LUT_y_start_;
  int LUT_Nx_;
  int LUT_Ny_;
  
  float sigmaSq_rng_ = math_util::Sq(0.025); // var of range measurements [meters]
  float d_reqd_ = 0.5; // [m] translation of this amount will result in a SLAM update
  float ang_reqd_ = math_util::DegToRad(5.0); // [rad] rotation of this amount will result in a SLAM update
  int scan_decimation_ = 1; // use every Nth measurement for table lookup
  float max_range_ = 4.0; // [m], only consider measurements at ranges less than this
  float buffer_ = 0.5;
  float dx_step_ = 0.05;
  float dy_step_ = 0.05;
  float dtheta_step_ = math_util::DegToRad(0.2);
  float resolution_ = 0.05; // [m]
  int NskipInMap_ = 0; // skips n scans in AugmentMap()
  Eigen::Vector2f ka_ = Eigen::Vector2f(0.05 , 1.0); // (dist,ang)
  Eigen::Vector2f kx_ = Eigen::Vector2f(0.2 , 0.02); // (dist,ang)
  Eigen::Vector2f ky_ = Eigen::Vector2f(0.1 , 0.02); // (dist,ang)
  // Recommendation from Dr Biswas in Piazza:
  //   Angular error per unit rotation (radians / radian):  0.05 - 0.2
  //   Angular error per unit translation (radians / meter): 0.02 - 0.1
  //   Translation error per unit rotation (meters / radian): 0.01
  //   Translation error per unit translation (meters / meter): 0.1 - 0.2
};
}  // namespace slam

#endif   // SRC_SLAM_H_
