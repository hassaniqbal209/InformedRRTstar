
#include <algorithm>
#include <vector>

#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "shared/math/line2d.h"
#include "shared/util/random.h"
#include "vector_map/vector_map.h"
#include "shared/math/math_util.h"

#ifndef SRC_PARTICLE_FILTER_H_
#define SRC_PARTICLE_FILTER_H_

namespace particle_filter {

struct Particle {
  Eigen::Vector2f loc;
  float angle;
  double weight;
};

class ParticleFilter {
 public:
  // Default Constructor.
   ParticleFilter();

  // Observe a new laser scan.
  void ObserveLaser(const std::vector<float>& ranges,
                    float range_min,
                    float range_max,
                    float angle_min,
                    float angle_max);

  // Observe new odometry-reported location.
  void ObserveOdometry(const Eigen::Vector2f& odom_loc,
                       const float odom_angle);

  // Initialize the robot location.
  void Initialize(const std::string& map_file,
                  const Eigen::Vector2f& loc,
                  const float angle);

  // Return the list of particles.
  void GetParticles(std::vector<Particle>* particles) const;

  // Get robot's current location.
  void GetLocation(Eigen::Vector2f* loc, float* angle) const;

  // Update particle weight based on laser.
  void Update(const std::vector<float>& ranges,
              float range_min,
              float range_max,
              float angle_min,
              float angle_max,
              std::vector<bool> valid_rng,
              Particle* p);

  // Resample particles.
  void Resample();

  // For debugging: get predicted point cloud from current location.
  void GetPredictedPointCloud(const Eigen::Vector2f& loc,
                              const float angle,
                              int num_ranges,
                              float range_min,
                              float range_max,
                              float angle_min,
                              float angle_max,
                              bool isCartesian,
                              std::vector<Eigen::Vector2f>* scan);
  void GetPredictedRng(const Eigen::Vector2f& loc,
						const float angle,
						int num_ranges,
						float range_min,
						float range_max,
						float angle_min,
						float angle_max,
						std::vector<bool>* valid,
						std::vector<float>* scan);

  // Calculate lookup table for observation likelihood
  void CalcObsLogLikelihoodFx();
  
  // Disregard measurements that are estimated to be un-mapped abstacles
  void IdentifyObstacles(const std::vector<float>& ranges, std::vector<bool>* valid_ptr, std::vector<float>& pred_rng);
  
  // Draw Observation Lines
  //void DrawObservations(const std::vector<float>& ranges, float angle_min, float angle_max);
  
 private:

  // List of particles being tracked.
  std::vector<Particle> particles_;

  // Map of the environment.
  vector_map::VectorMap map_;

  // Random number generator.
  util_random::Random rng_;

  // Previous odometry-reported locations.
  Eigen::Vector2f prev_odom_loc_;
  float prev_odom_angle_;
  bool odom_initialized_ = false;
  bool part_initialized_ = false;
  
  // MASC Stuff
  float sigma_init_loc_ = 0.25; // stdev of initial loc position, [meters]
  float sigma_init_heading_ = math_util::DegToRad(1.0); // stddev of initial loc heading, [radians]
  int scan_decimation_ = 10; // use every scan_decimation scan from sensor
  float sigmaSq_rng_ = math_util::Sq(0.07); // var of range measurements [meters]
  double gamma_ = 1.0/50.0; // number in [1/n,1], 1/n=>perfect correlation, 1=>uncorrelated (larger gamma=more meas confidence)
  float dmin_   = -4.0; // p(z|x)=1 if z-zhat is less than this
  float dmax_   =  1.0; // p(z|x)=0 if z-zhat is greater than this
  float dshort_ = -0.2; // p(z|x)=ramp if z-zhat is less than this
  float dlong_  =  0.3; // p(z|x)=const if z-zhat is greater than this
  std::vector<float> obs_lh_fx_;
  int num_LL_ = 100000;
  float stepSize_ll_;
  float dist_since_update_ = 0.0; // initialize to big number so update step happens on first iteration
  float ang_since_update_ = 0.0; // initialize to big number so update step happens on first iteration
  float dist_reqd_between_updates_ = 0.2; // [meters]
  float ang_reqd_between_updates_ = math_util::DegToRad(1.0); // [rad]
  int num_updates_since_resample_ = 0;
  double KLD_thold_ = 5.0; // Resample when KL-div of log(weights) is greater than this
  Eigen::Vector2f ka_ = Eigen::Vector2f(0.4 , 1.0); // (dist,ang)
  Eigen::Vector2f kx_ = Eigen::Vector2f(0.4 , 0.02); // (dist,ang)
  Eigen::Vector2f ky_ = Eigen::Vector2f(0.2 , 0.1); // (dist,ang)
  // Recommendation from Dr Biswas in Piazza:
  //   Angular error per unit rotation (radians / radian):  0.05 - 0.2
  //   Angular error per unit translation (radians / meter): 0.02 - 0.1
  //   Translation error per unit rotation (meters / radian): 0.01
  //   Translation error per unit translation (meters / meter): 0.1 - 0.2
};
}  // namespace slam

#endif   // SRC_PARTICLE_FILTER_H_
