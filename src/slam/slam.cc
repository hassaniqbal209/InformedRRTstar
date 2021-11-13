
#include <algorithm>
#include <cmath>
#include <iostream>
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "shared/math/geometry.h"
#include "shared/math/math_util.h"
#include "shared/util/timer.h"

#include "slam.h"

#include "vector_map/vector_map.h"

using namespace math_util;
using Eigen::Affine2f;
using Eigen::Rotation2Df;
using Eigen::Translation2f;
using Eigen::Vector2f;
using Eigen::Vector2i;
using Eigen::Vector3f;
using Eigen::Matrix2f;
using std::cout;
using std::endl;
using std::string;
using std::swap;
using std::vector;
using vector_map::VectorMap;
using math_util::AngleDiff;

namespace slam {

SLAM::SLAM() :
    prev_odom_loc_(0, 0),
    prev_odom_angle_(0),
    odom_initialized_(false) {}

void SLAM::GetPose(Eigen::Vector2f* loc, float* angle) const {
  // Return the latest pose estimate of the robot.
  if (!odom_initialized_) { return; }
  Vector3f xyt = poses_.back();
  *loc = Rotation2Df(xyt.z())*dxdy_since_update_ + Vector2f(xyt.x(), xyt.y());
  *angle = xyt.z() + ang_since_update_;
}

void SLAM::GetScanXY(const vector<float>& ranges,
					 float range_max,
					 float angle_min,
					 float angle_max,
					 float buffer,
					 vector<Vector2f>* scan_ptr) {

	// This function returns the (x,y) position of laser scans in base_link ref frame
	//    Only the points inside the square of side-length max_range_ are included
	vector<Vector2f>& scanXY = *scan_ptr;
	
	float sensorLoc = 0.2; // x-coord of sensor location in base_link frame
    float ang_stepSize = (angle_max-angle_min) / (ranges.size()-1);
	float scanBng, x, y;
	for (size_t k = 0; k<ranges.size() ; k++) {
		if (ranges[k]<range_max) {
			scanBng = angle_min + ang_stepSize*k;
			x = ranges[k] * cosf(scanBng) + sensorLoc;
			if (abs(x)>max_range_+buffer) { continue; }
			y = ranges[k] * sinf(scanBng);
			if (abs(y)>max_range_+buffer) { continue; }
			scanXY.push_back(Vector2f(x,y));
		}
	}
}

void SLAM::GetScanXY(const vector<float>& ranges,
					 float range_max,
					 float angle_min,
					 float angle_max,
					 Vector2f ref_loc,
					 float ref_ang,
					 vector<Vector2f>* scan_ptr) {

	// This function returns the (x,y) position of laser scans in base_link ref frame
	//    Only the points inside the square of side-length max_range_ are included
	vector<Vector2f>& scanXY = *scan_ptr;
	
	float sensorLoc = 0.2; // x-coord of sensor location in base_link frame
    float ang_stepSize = (angle_max-angle_min) / (ranges.size()-1);
	float scanBng, x, y;
	for (size_t k = 0; k<ranges.size() ; k+=scan_decimation_) {
		if (ranges[k]<range_max) {
			scanBng = angle_min + ang_stepSize*k + ref_ang;
			x = ranges[k] * cosf(scanBng) + sensorLoc + ref_loc.x();
			if (abs(x)>max_range_) { continue; }
			y = ranges[k] * sinf(scanBng) + ref_loc.y();
			if (abs(y)>max_range_) { continue; }
			scanXY.push_back(Vector2f(x,y));
		}
	}
}

void SLAM::AugmentMap(const vector<Vector2f>& scan_xy) {
	printf("Augmenting Map!\n");
	Vector3f X = poses_.back();
	int Nskip = 0;
	for (Vector2f pt : scan_xy) {
		if (Nskip<NskipInMap_) { Nskip++; continue; }
		Nskip = 0;
		Vector2f rot_pt(Rotation2Df(X.z()) * pt);
		Vector2f trans_xy(X.x(), X.y());
		map_.push_back(rot_pt + trans_xy);
	}
}

void SLAM::InitializeLUT() {
	float range = max_range_ + buffer_;
	LUT_Ny_ = (int) ceil(2.0 * range / resolution_);
	LUT_Nx_ = (int) ceil(2.0 * range / resolution_); // lidar has 270 deg FOV
	printf("Init LUT[%i][%i]\n", LUT_Nx_, LUT_Ny_);
	LUT_x_start_ = -range;
	LUT_y_start_ = -range;
	vector<float> tmp;
	for (int n=0 ; n<=LUT_Ny_ ; n++) { // iterate over y (rows)
		tmp.clear();
		for (int m=0 ; m<=LUT_Nx_ ; m++) { // iterate over x (cols)
			tmp.push_back(0.0);
		}
		LUT_.push_back(tmp);
	}
}

void SLAM::CreateLookupTable(const vector<Vector2f>& scanXY) {
	printf("Creating LUT\n");
	for (int n=0 ; n<=LUT_Ny_ ; n++) { // iterate over y (rows)
		for (int m=0 ; m<=LUT_Nx_ ; m++) { // iterate over x (cols)
			LUT_[n][m] = 1e12;
		}
	}
	
	Vector2f grid_loc(LUT_x_start_ , LUT_y_start_);
	//Vector2f best_grid_loc(0.0 , 0.0);
	float D = 0.0;
	for (Vector2f pt : scanXY) {
		float rng_dep = 1.0 + pt.squaredNorm();
		grid_loc.y() = LUT_y_start_;
		for (int ndx_y=0 ; ndx_y<=LUT_Ny_ ; ndx_y++) { // iterate over y (rows)
			grid_loc.x() = LUT_x_start_;
			for (int ndx_x=0 ; ndx_x<=LUT_Nx_ ; ndx_x++) { // iterate over x (cols)
				//D = (grid_loc-pt).squaredNorm() / sigmaSq_rng_;
				D = (grid_loc-pt).squaredNorm() / (rng_dep*sigmaSq_rng_);
				LUT_[ndx_y][ndx_x] = std::min(LUT_[ndx_y][ndx_x], D);
				//LUT_[ndx_y][ndx_x] += D;
				grid_loc.x() += resolution_;
			} // loop over ndx_x
			grid_loc.y() += resolution_;
		} // loop over ndx_y
		//printf("best_grid_loc=(%f,%f) , pt=(%f,%f) , LUT=%f\n", best_grid_loc.x(), best_grid_loc.y(), pt.x(), pt.y(),
		//					minD);
		//sleep(1);
	} // loop over scanXY
}

float SLAM::IndexLookupTable(const vector<Vector2f>& points, Vector2f dxdy) {
	float LH = 0.0;
	for (Vector2f pt : points) {
		int ndx_x = Clamp(static_cast<int>(roundf((pt.x()+dxdy.x() - LUT_x_start_) / resolution_)), 0, LUT_Nx_);
		int ndx_y = Clamp(static_cast<int>(roundf((pt.y()+dxdy.y() - LUT_y_start_) / resolution_)), 0, LUT_Ny_);
		//int ndx_x = static_cast<int>(roundf((pt.x()+dxdy.x() - LUT_x_start_) / resolution_));
		//int ndx_y = static_cast<int>(roundf((pt.y()+dxdy.y() - LUT_y_start_) / resolution_));
		
		//printf("pt (%f,%f) indexes into [%i][%i] -> LUT=%f\n", pt.x()+dxdy.x(), pt.y()+dxdy.y(), ndx_x, ndx_y, LUT_[ndx_x][ndx_y]);
		
		LH += LUT_[ndx_y][ndx_x];
	}

	//printf("Mean abs delta = (%f,%f)\n", mean_abs_delta.x()/points.size(), mean_abs_delta.y()/points.size());
	//printf("Used %i of %zu points\n", Nused, points.size());
	
	return LH;
}

void SLAM::ObserveLaser(const vector<float>& ranges,
                        float range_min,
                        float range_max,
                        float angle_min,
                        float angle_max) {
  // A new laser scan has been observed. Decide whether to add it as a pose
  // for SLAM. If decided to add, align it to the scan from the last saved pose,
  // and save both the scan and the optimized pose.
  if (!odom_initialized_) { return; }
  
  if (!LUT_initialized_) {
	  printf("Initializing LUT\n");
	  InitializeLUT();
	  vector<Vector2f> scanXY;
	  GetScanXY(ranges, range_max, angle_min, angle_max, buffer_, &scanXY);
	  CreateLookupTable(scanXY);
	  LUT_initialized_ = true;
	  //poses_.push_back(Vector3f(prev_odom_loc_.x() , prev_odom_loc_.y() , prev_odom_angle_));
	  poses_.push_back(Vector3f(0.0 , 0.0 , 0.0));
	  AugmentMap(scanXY);
	  //printf("Init poses at (%f,%f,%f)\n", prev_odom_loc_.x() , prev_odom_loc_.y() , RadToDeg(prev_odom_angle_));
	  
	  for (size_t k=0 ; k<scanXY.size() ; k++) {
		  printf("scanXY = (%f,%f)\n", scanXY[k].x(), scanXY[k].y());
	  }
	  printf("DONE\n");
  }
  
  if ((dist_since_update_>=d_reqd_) || (abs_ang_since_update_>=ang_reqd_)) {
  	  printf("Delta pose since update (%f,%f,%f)\n", dxdy_since_update_.x(), dxdy_since_update_.y(), RadToDeg(ang_since_update_));
	  Vector2f dist_dang(dist_since_update_ , abs_ang_since_update_);
	  float sigma_x = kx_.dot(dist_dang); // (k1x * dist) + (k2x * abs(delta_ang));
	  float sigma_y = ky_.dot(dist_dang); // (k1y * dist) + (k2y * abs(delta_ang));
	  float sigma_a = ka_.dot(dist_dang); // (k3  * dist) + (k4  * abs(delta_ang));
	  
	  float dx_rng = roundf(std::min(d_reqd_, 3.0f * sigma_x)/dx_step_) * dx_step_;
	  float dy_rng = roundf(std::min(d_reqd_, 3.0f * sigma_y)/dy_step_) * dy_step_;
	  float dtheta_rng = roundf(std::min(ang_reqd_, 3.0f * sigma_a)/dtheta_step_) * dtheta_step_;
	  printf("SLAM Ranges = (%f,%f,%f)\n", dx_rng, dy_rng, RadToDeg(dtheta_rng));
	  
      // Inverse Covar from Motion Model
	  Matrix2f E;
	  E << 1.0/Sq(sigma_x), 0,
              0,    1.0/Sq(sigma_y);
      float dxdy_ang = atanf(dxdy_since_update_.y() / dxdy_since_update_.x());
      Matrix2f E_inv = (Rotation2Df(dxdy_ang)*E).transpose(); // inverse covar for (x,y)
      
      //vector<Vector2f> points(scanXY.size());
	  vector<Vector2f> points;
	  Vector2f dxdy(0.0,0.0);
	  Vector2f best_xy_offset(0.0,0.0);
	  float minLH = 1e12;
	  float logLH, thetaLH, motionLH, obsLH;
	  float best_motionLH = 0.0;
	  float best_obsLH = 0.0;
	  float best_ang_offset = 0.0;
	  for (float dtheta=-dtheta_rng ; dtheta<=dtheta_rng ; dtheta+=dtheta_step_) {
		  points.clear();
		  GetScanXY(ranges, range_max, angle_min, angle_max, dxdy_since_update_, ang_since_update_+dtheta, &points);
		  //for (size_t k=0 ; k<scanXY.size() ; k++) {
			//  points[k] = (Rotation2Df(ang_since_update_ + dtheta) * scanXY[k]) + dxdy_since_update_;
		  //}
		  thetaLH = Sq(dtheta) / Sq(sigma_a);

		  /*
		  if (abs(dtheta-DegToRad(0.0))<DegToRad(0.1)) {
			  printf("There are %zu points\n", points.size());
			  int Nskip = 0;
			  for (Vector2f pt : points) {
				  if (Nskip<10) { Nskip++; continue; }
				  Nskip = 0;
				  printf("point = (%f,%f, 'r.')\n", pt.x(), pt.y());
			  }
			  printf("DONE -- Check Now!!\n");
		  }
		  */
		  for (float dx=-dx_rng ; dx<=dx_rng ;  dx+=dx_step_) {
			  dxdy.x() = dx;
			  for (float dy=-dy_rng ; dy<=dy_rng ; dy+=dy_step_) {
				  dxdy.y() = dy;
				  motionLH = (dxdy.transpose() * E_inv * dxdy).sum() + thetaLH;
				  obsLH = IndexLookupTable(points, dxdy);
				  //if (abs(dtheta)<DegToRad(0.1)) {
					//  printf("LH for (%f,%f,%f) = %f : %f\n", dxdy.x(), dxdy.y(), RadToDeg(dtheta), motionLH, obsLH);
				  //}
				  logLH = motionLH + obsLH;
				  if (logLH<minLH) {
					  minLH = logLH;
					  best_xy_offset  = dxdy;
					  best_ang_offset = dtheta;
					  best_motionLH = motionLH;
					  best_obsLH = obsLH;
				  } // if largest LH
			  } // dy loop
		  } // dx loop
	  } // dtheta loop
	  
	  printf("Best offset is (%f,%f,%f) : p(x)=%f : p(z)=%f\n", best_xy_offset.x(), best_xy_offset.y(), RadToDeg(best_ang_offset), 
														best_motionLH, best_obsLH);
	  Vector3f prev_pose = poses_.back();
	  Vector2f new_xy = Rotation2Df(prev_pose.z()) * (dxdy_since_update_+best_xy_offset) + Vector2f(prev_pose.x(), prev_pose.y());
	  poses_.push_back(Vector3f(new_xy.x(), new_xy.y(), AngleMod(ang_since_update_+best_ang_offset+prev_pose.z())));
	  //printf("Previous pose was (%f,%f,%f)\n", prev_pose.x(), prev_pose.y(), RadToDeg(prev_pose.z()));
	  printf("Pose estimated as (%f,%f,%f)\n", new_xy.x(), new_xy.y(), RadToDeg(ang_since_update_+best_ang_offset+prev_pose.z()));
	  
	  //scanXY.clear();
	  vector<Vector2f> new_scanXY; // (x,y) meas in base_link frame
	  GetScanXY(ranges, range_max, angle_min, angle_max, buffer_, &new_scanXY);
	  CreateLookupTable(new_scanXY);
	  
	  vector<Vector2f> scanXY; // (x,y) meas in base_link frame
	  GetScanXY(ranges, range_max, angle_min, angle_max, 0.0, &scanXY);
	  AugmentMap(scanXY);
	  /*
	  printf("There are %zu scanXY\n", scanXY.size());
	  int Nskip = 0;
	  for (Vector2f pt : new_scanXY) {
		  if (Nskip<10) { Nskip++; continue; }
		  Nskip = 0;
		  printf("scanXY = (%f,%f, 'k.')\n", pt.x(), pt.y());
	  }
	  */
	  dxdy_since_update_ = Vector2f(0.0, 0.0);
	  dist_since_update_ = 0.0;
	  ang_since_update_ = 0.0;
	  abs_ang_since_update_ = 0.0;
  } // if moved enough
}

void SLAM::ObserveOdometry(const Vector2f& odom_loc, const float odom_angle) {
  if (!odom_initialized_) {
	printf("Odom getting initialized\n");
    prev_odom_angle_ = odom_angle;
    prev_odom_loc_ = odom_loc;
    odom_initialized_ = true;
    return;
  }
	
  //printf("Odom observed as (%f,%f,%f)\n", odom_loc.x(), odom_loc.y(), RadToDeg(odom_angle));
  // Keep track of odometry to estimate how far the robot has moved between poses
  Vector2f delta_odom = odom_loc - prev_odom_loc_;
  float dist = (delta_odom).norm();
  float delta_ang = AngleDiff(odom_angle, prev_odom_angle_);
  
  dxdy_since_update_ += Rotation2Df(-prev_odom_angle_+ang_since_update_) * delta_odom; // base_link frame
  dist_since_update_ += dist;
  ang_since_update_ += delta_ang; // base_link frame
  abs_ang_since_update_ += abs(delta_ang);
  prev_odom_loc_ = odom_loc;
  prev_odom_angle_ = odom_angle;
}

vector<Vector2f> SLAM::GetMap() {
  printf("GetMap Executing!\n");
  //vector<Vector2f> map = map_;
  // Reconstruct the map as a single aligned point cloud from all saved poses
  // and their respective scans.
  return map_;
}

}  // namespace slam
