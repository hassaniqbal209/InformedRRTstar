
#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/line2d.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "config_reader/config_reader.h"
#include "particle_filter.h"

#include "vector_map/vector_map.h"

using geometry::line2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using Eigen::Vector2f;
using Eigen::Vector2i;
using Eigen::Rotation2Df;
using vector_map::VectorMap;
using math_util::DegToRad;
using math_util::RadToDeg;
using math_util::AngleMod;
using math_util::AngleDiff;
using math_util::Sq;
using math_util::Clamp;
using math_util::Ramp;

DEFINE_double(num_particles, 100, "Number of particles");

namespace particle_filter {

config_reader::ConfigReader config_reader_({"config/particle_filter.lua"});

ParticleFilter::ParticleFilter() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {//FLAGS_num_particles = 1000
		//current_num = FLAG....
		}

void ParticleFilter::GetParticles(vector<Particle>* particles) const {
  *particles = particles_;
}

void ParticleFilter::GetPredictedRng(const Vector2f& loc,
									const float angle,
									int num_ranges,
									float range_min,
									float range_max,
									float angle_min,
									float angle_max,
									vector<bool>* valid_ptr,
									vector<float>* scan_ptr) {
  if (!odom_initialized_) { return; }
  vector<float>& scan = *scan_ptr;
  vector<bool>& isValid = *valid_ptr;
  //printf("GetPredictedPointCloud() Executing ... isCartesian=%i\n", isCartesian);
  // Compute what the predicted point cloud would be, if the car was at the pose
  // loc, angle, with the sensor characteristics defined by the provided
  // parameters.
  // This is NOT the motion model predict step: it is the prediction of the
  // expected observations, to be used for the update step.

  // Note: The returned values must be set using the `scan` variable:
  scan.resize(num_ranges);
  // Fill in the entries of scan using array writes, e.g. scan[i] = ...
  float x_orig, y_orig, x_end, y_end, totBng, rng2int, scanBng;
  float ang_stepSize = (angle_max-angle_min) / (num_ranges-1);
  Vector2f sensorLoc(loc.x()+0.2*cosf(angle) , loc.y()+0.2*sinf(angle));
  Vector2f intersection_point;
  bool intersects;
  int N = 0;
  for (int k = 0; k<num_ranges ; k+=scan_decimation_) {
	  if (!isValid[k]) { continue; } // there's not a range measurement here
	  isValid[k] = false; // set back to true if there's an intersection
	  scanBng = angle_min + ang_stepSize*k;
	  totBng = AngleMod(angle + scanBng);
	  x_orig = sensorLoc.x() + range_min*cosf(totBng);
	  y_orig = sensorLoc.y() + range_min*sinf(totBng);
	  x_end  = sensorLoc.x() + range_max*cosf(totBng);
	  y_end  = sensorLoc.y() + range_max*sinf(totBng);
	  line2f my_line(x_orig, y_orig, x_end, y_end); // Line segment from (1,2) to (3.4)
	  float closest_meas = range_max;
	  for (size_t i = 0; i < map_.lines.size(); i++) {
		  const line2f map_line = map_.lines[i];
 		  // Simultaneously check for intersection, and return the point of intersection
		  intersects = map_line.Intersection(my_line, &intersection_point);
		  //intersects = map_line.Intersects(my_line);
		  if (intersects) {
			  //printf("Intersection at %f,%f\n", intersection_point.x(), intersection_point.y());
			  rng2int = (sensorLoc-intersection_point).norm();
			  if (rng2int<closest_meas) {
				  closest_meas = rng2int;
  				  isValid[k] = true;
			  }
		  }
	  } // for loop over map_line
      scan[k] = closest_meas;
      if (!isValid[k]) { N++; }
  }
  //printf("%i pred_rng marked as invalid\n", N);
}

void ParticleFilter::IdentifyObstacles(const vector<float>& ranges,
									   vector<bool>* valid_ptr,
									   vector<float>& pred_rng) {
    vector<bool>& isValid = *valid_ptr;
    int n = 0;
    vector<float> delta(ranges.size()/scan_decimation_);
    vector<int> k_index(ranges.size()/scan_decimation_);
    float max_delta = 0.0;
	float trusted_delta = 0.0;
	for (size_t k=0 ; k<ranges.size() ; k+=scan_decimation_) {
		if (!isValid[k]) { continue; }
		delta[n] = (ranges[k] - pred_rng[k]); // neg values could be obstacles
		k_index[n] = k;
		if (delta[n]>max_delta) { max_delta = delta[n]; }
		//printf("delta[%zu] = %f  %i\n", k, delta[n], int(isValid[k]));
		n++;
	}
	if (n>0) {
		delta.resize(n);
		nth_element(delta.begin(), delta.begin()+n/2, delta.end());
		trusted_delta = 5.0 * (delta[n/2] - max_delta); // negative value
		//float val = *min_element(delta.begin(), delta.end());
		//float trusted = (delta[n/2]-max_delta);
		//printf("val=%f , trusted=%f , numTrusted=%f\n", val, trusted, val / trusted);
	}
	// check for sequential untrusted scans
	bool prev_was_untrusted = false;
	int N = 0;
	for (int m=0 ; m<n ; m++) {
		if (delta[m] < trusted_delta) {
			if (prev_was_untrusted) {
				printf("Obstacle Detected ... Distrusting Measurement\n");
				printf("delta=%f , trusted=%f , max_delta=%f\n", delta[m], trusted_delta, max_delta);
				isValid[k_index[m-1]] = false;
				isValid[k_index[m]] = false;
				N++;
			}
			prev_was_untrusted = true;
		}
		else { prev_was_untrusted = false; }
	}
	//printf("%i untrusted as obstacles\n", N+1);
}

void ParticleFilter::Update(const vector<float>& ranges,
                            float range_min,
                            float range_max,
                            float angle_min,
                            float angle_max,
                            vector<bool> valid_rng,
                            Particle* p_ptr) {
  // Implement the update step of the particle filter here.
  // You will have to use the `GetPredictedPointCloud` to predict the expected
  // observations for each particle, and assign weights to the particles based
  // on the observation likelihood computed by relating the observation to the
  // predicted point cloud.
  Particle& particle = *p_ptr;
  //vector<Vector2f> pred_rng;
  //GetPredictedPointCloud(particle.loc, particle.angle, ranges.size(), range_min, range_max, angle_min, angle_max, false, &pred_rng);
  vector<float> pred_rng;
  vector<bool> isValid = valid_rng; // just to ensure valid_rng changes in GetPredictedRng aren't shared between particles
  GetPredictedRng(particle.loc, particle.angle, ranges.size(), range_min, range_max, angle_min, angle_max, &isValid, &pred_rng);
  
  IdentifyObstacles(ranges, &valid_rng, pred_rng);
  
  double log_likelihood = 0.0;
  float z;
  int ndx;
  int N=0;
  for (size_t k=0 ; k<pred_rng.size() ; k+=scan_decimation_) {
	  if (!isValid[k]) { continue; }
	  z = ranges[k] - pred_rng[k];
	  if (z<dmin_) { continue; }
	  if (z>dmax_) {
		  printf("Bad Observation!!!\n");
		  printf("range[%zu]=%f , pred=%f , z=%f\n", k, ranges[k], pred_rng[k], z);
		  log_likelihood += 10000.0; // exp(-1000) is approx. 0
	  }
	  else {
		  z = Clamp(ranges[k]-pred_rng[k], dshort_, dlong_);
		  ndx = static_cast<int>(std::round((z-dmin_) / stepSize_ll_));
		  log_likelihood += obs_lh_fx_[ndx];
	  }
	  N++;
  }
  //printf("%i meas contributed to this particle\n", N);
  particle.weight += gamma_ * log_likelihood; // log(w[i]) = log(w[i]) + log(p(z|x))
}

void ParticleFilter::ObserveLaser(const vector<float>& ranges,
                                  float range_min,
                                  float range_max,
                                  float angle_min,
                                  float angle_max) {
  // A new laser scan observation is available (in the laser frame)
  // Call the Update and Resample steps as necessary.
  //printf("ObserveLaser called rng min=%f , max=%f\n", range_min, range_max);
  if (!odom_initialized_) { return; }
  
  vector<bool> valid_rng(ranges.size(), false);
  int N = 0;
  for (size_t k=0 ; k<ranges.size() ; k+=scan_decimation_) {
	  //printf("range[%zu]=%f\n", k, ranges[k]);
	  if (ranges[k]<range_max) { valid_rng[k] = true; N++; }
  }
  //printf("There are %i valid ranges in this obs\n", N);
  
  if ((dist_since_update_>=dist_reqd_between_updates_) | (ang_since_update_>=ang_reqd_between_updates_)) {
	  double minNegLogW = 10000.0; // weights are all positive
	  for (Particle& p : particles_) {
		  Update(ranges, range_min, range_max, angle_min, angle_max, valid_rng, &p);
		  if (p.weight < minNegLogW) { minNegLogW = p.weight; }
	  }
	  printf("MinNegLogWeight = %f\n", minNegLogW);
	  double sumW = 0.0;
	  int k=0;
	  for (Particle& p : particles_) {
	      p.weight = p.weight - minNegLogW;
   		  sumW += p.weight;
	      printf("Log Weight[%i] = %f\n", k, p.weight);
	      k++;
	  }
	  dist_since_update_ = 0.0;
	  ang_since_update_ = 0.0;
	  num_updates_since_resample_++; // increment counter
	  
	  // If KL-Div of weights is large enough: resample
	  //    Q(x)~Uniform => sumW/num_particles
	  double w_offset = 1.0 / particles_.size();
	  double D = 0.0; // KL-divergence
	  for (Particle& p : particles_) { D += p.weight * log(p.weight+w_offset); }
	  //printf("---- D=%f , sumW=%f\n", D, sumW);
	  D /= sumW;// + log((double)particles_.size());
	  printf("KLd=%f after %i updates\n", D, num_updates_since_resample_);
	  if ((D > KLD_thold_) | (num_updates_since_resample_ >= 4)) { 
		  Resample();
		  num_updates_since_resample_ = 0;
	  }
  }
}

void ParticleFilter::Resample() {
  // Resample the particles, proportional to their weights.
  // The current particles are in the `particles_` variable. 
  // Create a variable to store the new particles, and when done, replace the
  // old set of particles:
  // vector<Particle> new_particles';
  // During resampling: 
  //    new_particles.push_back(...)
  // After resampling:
  // particles_ = new_particles;
  float M = particles_.size(); // number of samples to be drawn

  //new particle set
  vector<Particle> new_particles;
  vector<double> weights;
  weights.resize(M);
  
  //normalize the weights
  double sum_weight = 0.0;
  int k = 0;
  for (Particle p : particles_) {
    weights[k] = exp(-p.weight);
    sum_weight += weights[k];
    k++;
  }
  double c = weights[0] / sum_weight;
  
  //generate random number from 0 to 1/M
  float r = rng_.UniformRandom(0.0, 1.0/M);
  
  int i = 0;
  float u = r;
  float oneOverM = 1.0 / M;
  for (int m=0 ; m<M ; m++) {
    //u = r + m/M;
    while (u > c) {
      i++;
      c += weights[i] / sum_weight;
    }
    printf("Choosing particle[%i] as %i\n", m, i);
    particles_[i].weight = 0.0; // reset all particle log(weights)
    new_particles.push_back(particles_[i]);
    u += oneOverM;
  }
  // save re-sampled particles
  particles_ = new_particles;
}

void ParticleFilter::ObserveOdometry(const Vector2f& odom_loc,
                                     const float odom_angle) {
  // A new odometry value is available (in the odom frame)
  // Implement the motion model predict step here, to propagate the particles
  // forward based on odometry.

  // You will need to use the Gaussian random number generator provided. For
  // example, to generate a random number from a Gaussian with mean 0, and
  // standard deviation 2:
  //float x = rng_.Gaussian(0.0, 2.0);
  //printf("Random number drawn from Gaussian distribution with 0 mean and "
  //       "standard deviation of 2 : %f\n", x);
  
  if (!part_initialized_) {
	  printf("Particles not initialized yet\n");
	  return;
  }
  if (!odom_initialized_) {
	  //printf("Odom Initializing: (%f,%f,%f)\n", odom_loc.x(), odom_loc.y(), RadToDeg(odom_angle));
	  prev_odom_loc_ = odom_loc;
	  prev_odom_angle_ = odom_angle;
	  odom_initialized_ = true;
	  return;
  }
  
  Vector2f delta_odom = odom_loc - prev_odom_loc_;
  //printf("Prev odom=(%f,%f,%f)\n", prev_odom_loc_.x(), prev_odom_loc_.y(), RadToDeg(prev_odom_angle_));
  //printf("Curr odom=(%f,%f,%f)\n", odom_loc.x(), odom_loc.y(), RadToDeg(odom_angle));
  //sleep(1);
  float dist = (delta_odom).norm();
  if (dist<0.0001) { return; } // don't do anything if we're stopped
  float delta_ang = AngleDiff(odom_angle, prev_odom_angle_);
  
  Vector2f dist_dang(dist , abs(delta_ang));
  float sigma_x = kx_.dot(dist_dang); // (k1x * dist) + (k2x * abs(delta_ang));
  float sigma_y = ky_.dot(dist_dang); // (k1y * dist) + (k2y * abs(delta_ang));
  float sigma_a = ka_.dot(dist_dang); // (k3  * dist) + (k4  * abs(delta_ang));
  for (Particle& p : particles_) {
	  /*
	  // Thrun Method
	  float drot1 = atan2f(delta_odom.y() , delta_odom.x()) - prev_odom_angle_;
	  float drot2 = odom_angle - prev_odom_angle_ - drot1;
	  float dhrot1 = drot1 - rng_.Gaussian(0.0, ka_.dot(Vector2f(dist,drot1)));
	  float dhtransx = dist - rng_.Gaussian(0.0, kx_.dot(Vector2f(dist,drot1+drot2)));
	  float dhtransy = dist - rng_.Gaussian(0.0, ky_.dot(Vector2f(dist,drot1+drot2)));
	  float dhrot2 = drot2 - rng_.Gaussian(0.0, ka_.dot(Vector2f(dist,drot2)));
	  p.loc.x() += dhtransx*cosf(p.angle+dhrot1);
	  p.loc.y() += dhtransy*sinf(p.angle+dhrot1);
	  p.angle = AngleMod(p.angle + dhrot1 + dhrot2);
	  */
	  
	  // Loc - Lecture method 2
	  Vector2f noise_xy(rng_.Gaussian(0.0, sigma_x) , rng_.Gaussian(0.0, sigma_y)); // indep x,y noise in base_link
	  Vector2f noise_map = Rotation2Df(p.angle+delta_ang) * noise_xy; // x,y noise in map frame at curr time
	  p.loc += (Rotation2Df(p.angle-prev_odom_angle_) * delta_odom) + noise_map;
	  // Angle
	  p.angle = AngleMod(p.angle + delta_ang + rng_.Gaussian(0.0, sigma_a));
	  
  }
  
  dist_since_update_ += dist;
  ang_since_update_ += abs(delta_ang);
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

void ParticleFilter::CalcObsLogLikelihoodFx() {
	obs_lh_fx_.resize(num_LL_);
	stepSize_ll_ = (dmax_-dmin_) / num_LL_;
	float val_at_dshort = Sq(dshort_) / sigmaSq_rng_;
	float z;
	for (int k=0 ; k<num_LL_ ; k++) {
		z = dmin_ + k*stepSize_ll_;
		if (z<dshort_) { obs_lh_fx_[k] = Ramp(z, dmin_, dshort_, float(0.0), val_at_dshort); }
		else {
			z = std::min(dlong_, z);
			obs_lh_fx_[k] = Sq(z) / sigmaSq_rng_;
		}
	}
}

void ParticleFilter::Initialize(const string& map_file,
                                const Vector2f& loc,
                                const float angle) {
  // The "set_pose" button on the GUI was clicked, or an initialization message
  // was received from the log. Initialize the particles accordingly, e.g. with
  // some distribution around the provided location and angle.
  map_.Load("maps/" + map_file + ".txt");
  particles_.resize(FLAGS_num_particles);
  int n=0;
  for (Particle& p : particles_) {
	  p.loc.x() = loc.x() + rng_.Gaussian(0.0, sigma_init_loc_);
	  p.loc.y() = loc.y() + rng_.Gaussian(0.0, sigma_init_loc_);
	  p.angle = angle + rng_.Gaussian(0.0, sigma_init_heading_);
	  p.weight = 0.0; // this is log(w)
	  n++;
  }
  printf("Initializing %i particles!\n", n);
  part_initialized_ = true;
  odom_initialized_ = false;
  CalcObsLogLikelihoodFx();
}

void ParticleFilter::GetLocation(Eigen::Vector2f* loc_ptr, 
                                 float* angle_ptr) const {
  Vector2f& loc = *loc_ptr;
  float& angle = *angle_ptr;
  // Compute the best estimate of the robot's location based on the current set
  // of particles. The computed values must be set to the `loc` and `angle`
  // variables to return them. Modify the following assignments:
  if (!odom_initialized_) {
	  printf("Odom not initialized yet\n");
	  return;
  }
  if (particles_.size()<1) {
	  printf("Particles not initialized ... Loc is (%f,%f,%f)\n", loc.x(), loc.y(), RadToDeg(angle));
	  return;
  }
  /*
  // Set vehicle pose to particle with largest weight
  int maxNdx = 0;
  int k = 0;
  double maxW = particles_[0].weight;
  for (Particle p : particles_) {
	  if (p.weight > maxW) {
		  maxNdx = k;
		  maxW = p.weight;
	  }
	  k++;
  }
  loc = particles_[maxNdx].loc;
  angle = particles_[maxNdx].angle;
  */
  double expW;
  double sumW = 0.0;
  loc = Vector2f(0.0 , 0.0);
  float sin_ang = 0.0;
  float cos_ang = 0.0;
  for (Particle p : particles_) {
	  expW = exp(-p.weight);
	  sumW += expW;
	  loc += expW * p.loc;
	  sin_ang += expW * sinf(p.angle);
	  cos_ang += expW * cosf(p.angle);
  }
  loc /= sumW;
  angle = atan2f(sin_ang , cos_ang);
  
  //printf("Location set as (%f,%f,%f)\n", loc.x(), loc.y(), RadToDeg(angle));
}


}  // namespace particle_filter
