// This is an experimental validation of the theoretical work:
//
// Agrawal, A., Barratt, S., Boyd, S., & Stellato, B. (2020, July). 
// Learning convex optimization control policies. 
// In Learning for Dynamics and Control (pp. 361-373).


#include <ros/ros.h>

#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <geometry_msgs/Pose2D.h>
#include <std_msgs/Float64.h>
#include <tf/transform_datatypes.h>

 // wheel RPM constant
#define RAD_SEC_TO_RPM 30.0/3.14159265358979323846

extern "C"
{
#include "cvxgen/solver.h"
}

// Global variables required by CVXGEN
Vars vars;
Params params;
Workspace work;
Settings settings;
// End of global variables for CVXGEN


// Configuration
// Vehicle parameters
double cfg_axisLength_m = 0.408;
double cfg_wheelRadius_m = 0.079652424;
double L_car = 0.57;

// Message to publish
geometry_msgs::Pose2D pose2d;


// Trajectory parameters 
unsigned int last_idx = 0;
unsigned int lower_idx = 0;
unsigned int upper_idx = 0;
unsigned int window_search = 40;

// vectors trajectory -> data points that characterize the discretized trajectory
std::vector<float> traj_points_x;
std::vector<float> traj_points_y;
std::vector<float> vel_profile;
std::vector<float> traj_curvature;

// Lanekeeping parameters
float look_ahead = 0;
bool stop_vehicle = false;
float lat_error = 0;
float yaw_ref = 0;
float yaw_error = 0;


// Measurements and filters (ACCELERATION)
float ax_decay_rate = 1;
float ax_measured = 0;

// Measurements and filters (YAW RATE)
float r_decay_rate = 1;
float r_measured = 0;

// Measurements and filters (LINEAR VEL)
float linear_vel_decay_rate = 1;
float linear_vel_measured = 0;
float linear_vel = 0.4;
float linear_vel_des = 0.0;

// Discretization step
float h_step = 0.05;

// Constant to convert lat. error to desired curvature
float e_to_k = 0.0;

bool use_optimum = true;

// Times
ros::Time now, last;

// Control Commands
float cmd_a = 0;
float cmd_curvature = 0;
float delta_prev = 0;
float steering_ticks_to_rad = 571.4286;
float curvature = 0.0;
float current_curvature = 0.0;
float curvature_prev = 0.0;


// Compute trajectory errors
void calcErrorTrajectory(){

  if (stop_vehicle){
    lat_error = 0;
    yaw_ref = 0;
    return;
  }

  // Current vehicle coordinates
  float po_x = pose2d.x; 
  float po_y = pose2d.y;
  float theta = -pose2d.theta+3.141592653/2;

  // Look ahead projection
  float pi_x = sin(theta)*look_ahead + po_x;
  float pi_y = cos(theta)*look_ahead + po_y;

  // Limit the search considering  a fixed size Window search
  int lower_idx = last_idx - window_search/2;
  int upper_idx = last_idx + window_search/2;

  // Validates upper and lower limits
  if (lower_idx < 0){ // LOWEST value can only be ZERO
    lower_idx = 0;
  } 
  if (upper_idx> (int) traj_points_x.size()){ // HIGHEST value can only be the VECTOR size
    upper_idx = (int) traj_points_x.size();
  }

  // Nearest point search (search)
  // ROS_INFO("lower_idx, upper_idx: %u, %u", lower_idx,upper_idx);
  unsigned int indx = 0;  
  float shortest_dist = 999;
  for (int k = lower_idx; k < upper_idx; k++){
    float new_dist = sqrt( pow(pi_x-traj_points_x[k],2) + pow(pi_y-traj_points_y[k],2) );
    if (new_dist<shortest_dist){
      indx = k;
      shortest_dist = new_dist;    }
  }
  // Nearest point 
  float pk_x = traj_points_x[indx]; 
  float pk_y = traj_points_y[indx];

  // Compute the error (distance between vehicle and trajectory)
  lat_error = sqrt( pow(pi_x-pk_x,2) + pow(pi_y-pk_y,2) );

  // Determine the sign of the error:
  // Subsequent point
  float pk1_x = traj_points_x[indx+1];
  float pk1_y = traj_points_y[indx+1];

  // Compute orientation which is tangent to the trajectory
  float yaw_local = atan2((pk1_x-pk_x),(pk1_y-pk_y));

  // reference in the robot frame
  yaw_ref = atan2((pk1_y-pk_y),(pk1_x-pk_x));

  // Simply coordinate rotation
  float pi_x_rot = cos(yaw_local)*pi_x - sin(yaw_local)*pi_y;
  float pk_x_rot = cos(yaw_local)*pk_x - sin(yaw_local)*pk_y;

  yaw_local = yaw_local-3.141592653/2;
  // atualização sinal do erro.
  if (pi_x_rot<pk_x_rot){
    lat_error = -lat_error;
  }
  lat_error*= -1; // Change coordinates to the paper's reference (z axle down)
  last_idx = indx;
  // Checks if we are at the end of the trajectory
  if (indx >= traj_points_x.size()-20){
    stop_vehicle = true;
    ROS_INFO("stop_vehicle = true");
  }

  curvature_prev = curvature; // Saves the previous value of curvature

  // New curvature value corresponding to the current segment path
  curvature = traj_curvature[indx];
  curvature = curvature - e_to_k*lat_error;

  // updates the desired velocity
  linear_vel_des = vel_profile[indx];

}

void odomCallBack(const nav_msgs::Odometry& odom){
  linear_vel_measured = odom.twist.twist.linear.x*linear_vel_decay_rate + linear_vel_measured*(1-linear_vel_decay_rate);
}

void imuCallBack(const sensor_msgs::Imu& imu){
  ax_measured = (imu.linear_acceleration.x)*ax_decay_rate + ax_measured*(1-ax_decay_rate);
  r_measured = (imu.angular_velocity.z)*r_decay_rate + r_measured*(1-r_decay_rate);
}

void poseCallBack(const geometry_msgs::PoseStamped pose){
  pose2d.x = pose.pose.position.x;
  pose2d.y = pose.pose.position.y;

  tf::Quaternion q(
    pose.pose.orientation.x,
    pose.pose.orientation.y,
    pose.pose.orientation.z,
    pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  pose2d.theta = yaw;
}

// SOLVER PARAMETERS 
void loadDefaultData(void) {
  params.lam_3[0] = 0.5;
  params.lam_4[0] = 10.0;

  linear_vel = vel_profile[0];

  if (!use_optimum){
    params.P_sqrt[0] = 1.0;    params.P_sqrt[1] = 0.0;    params.P_sqrt[2] = 0.0;    params.P_sqrt[3] = 0.0;
    params.P_sqrt[4] = 0.0;    params.P_sqrt[5] = 1.0;    params.P_sqrt[6] = 0.0;    params.P_sqrt[7] = 0.0;
    params.P_sqrt[8] = 0.0;    params.P_sqrt[9] = 0.0;    params.P_sqrt[10] = 1.0;   params.P_sqrt[11] = 0.0;
    params.P_sqrt[12] = 0.0;   params.P_sqrt[13] = 0.0;   params.P_sqrt[14] = 0.0;   params.P_sqrt[15] = 1.0;
  
    params.q[0] = 0; params.q[1] = 0;
    params.q[2] = 0; params.q[3] = 0;
    ROS_WARN("Untuned policy!!!");
    ROS_WARN("P is identity and q is zero");

  }else{
    // Gains 
    params.P_sqrt[0] = 1.08050000;     params.P_sqrt[1] = 0.82900000;
    params.P_sqrt[2] = -0.07390000;    params.P_sqrt[3] = 0.05500000;
    params.P_sqrt[4] = 1.15660000;     params.P_sqrt[5] = 1.98560000;
    params.P_sqrt[6] = -0.20210000;    params.P_sqrt[7] = 1.51540000;
    params.P_sqrt[8] = -0.70400000;    params.P_sqrt[9] = 0.93570000;
    params.P_sqrt[10] = 5.01940000;    params.P_sqrt[11] = -0.07220000;
    params.P_sqrt[12] = 0.12620000;    params.P_sqrt[13] = 0.96780000;
    params.P_sqrt[14] = 0.06150000;    params.P_sqrt[15] = 1.14350000;

    params.q[0] = 4.3895e-08;     params.q[1] = 1.9753e-01;
    params.q[2] = -4.5382e-01;    params.q[3] = -1.9041e-02;
  }

  params.fx[0] = 0.0;   params.fx[1] = 0.0;
  params.fx[2] = 0.0;   params.fx[3] = 0.0;

  params.B[0] = 0.0;
  params.B[1] = 0.0;
  params.B[2] = h_step;
  params.B[3] = 0.0;
  params.B[4] = 0.0;
  params.B[5] = h_step * linear_vel / L_car; // h*v/L
  params.B[6] = 0.0;
  params.B[7] =  h_step * h_step * linear_vel * linear_vel / L_car; // h*h*v*v/L


  // Saturation
  params.a_max[0] = 2;
  params.L[0] = L_car;
  params.k[0] = 0.0;
  params.tan_d_max[0] = 0.4040;

  params.z_prev[0] = 0;
  params.k_prev[0] = 0;
  params.dtan_max[0] = 0.1225;

}

// Update and solve the control convex policy 
double solveProblem(){
  float pi2 = 3.141592653/2;
  if (yaw_ref<-pi2 && pose2d.theta>pi2){
    yaw_ref=yaw_ref+2*3.141592653;
  }else if (yaw_ref>pi2 && pose2d.theta<-pi2){
    pose2d.theta = pose2d.theta+2*3.141592653;
  }
  yaw_error = yaw_ref-pose2d.theta;

  yaw_error *= -1;  // Change coordinates to the paper's reference (z axle down)
  current_curvature = r_measured/linear_vel;
  
  // update the problem's parameters given current states
  float enext = lat_error + h_step * linear_vel * sin(yaw_error);
  float dpsi_next = yaw_error + h_step * linear_vel * (current_curvature - current_curvature / (1 - lat_error * current_curvature) * cos(yaw_error));
  params.fx[0] = enext;
  params.fx[1] = dpsi_next;
  params.fx[2] = linear_vel - 0.98*linear_vel_des -0.02*0.55;
  params.fx[3] = enext + h_step * linear_vel * sin(dpsi_next);

  params.B[5] = h_step * linear_vel / L_car; // h*v/L
  params.B[7] = h_step * h_step * linear_vel * linear_vel / L_car; // h*h*v*v/L

  params.k[0] = curvature;

  params.z_prev[0] = cmd_curvature;
  params.k_prev[0] = curvature_prev;
  params.dtan_max[0] = (1/ ( cos(delta_prev) * cos(delta_prev) ) ) * 0.1225;

  // Compute required time to solve the policy
  now = ros::Time::now();
  solve();
  last = ros::Time::now();

  // Solver infos
  if (work.converged){
    cmd_a = vars.u[0];
    cmd_curvature = vars.u[1];
  }else{
    ROS_WARN("Solution did not converged."); // In this case, keep previous values
  }

  return last.toSec()-now.toSec();
}


// Get steering and wheel velocity commands
void getCommands(double& steerval, double& leftval, double& rightval){

if (stop_vehicle){
    steerval = 0.0;
    leftval = 0.0;
    rightval = 0.0;
    delta_prev = 0.0;

    yaw_ref = 0;
    current_curvature = 0;
    curvature = 0;

    return;
  }

  // Command update-> speed and steering angle
  linear_vel = linear_vel + h_step * cmd_a; // m/s
  float delta_rad = atan(cmd_curvature + L_car * curvature); // radians
  // Convert to ticks (for the motor reference)
  steerval = delta_rad*steering_ticks_to_rad;

  // Steering angle saturation
  if(fabs(steerval) > 220.0){
    steerval *= fabs(220.0/steerval);
    //last_steerval = steerval;
  }

  // Compute RPM commands
  float increment = 0;
  // vel_deadzone
  if (fabs(linear_vel) > 0.1){
    // calculate proportion between left and right
    float d2ltphi = (tan(delta_rad)*cfg_axisLength_m)/(2*L_car) ;
    float ratio = (1 + d2ltphi)/(1 - d2ltphi);
    // now, supposing linear_vel are equal for both wheels, calculate an increment that will make cmdr = cmdl * ratio;
    increment = -linear_vel*(1-ratio)/(1+ratio);
  }

  rightval  = (linear_vel+increment)*RAD_SEC_TO_RPM / cfg_wheelRadius_m;
  leftval   = (linear_vel-increment)*RAD_SEC_TO_RPM / cfg_wheelRadius_m;

  // Saves the last steering value (radians)
  delta_prev = delta_rad; 
}


int main (int argc, char **argv){
  
  // ROS module.
  ros::init(argc, argv, "COCP"); 
  ros::NodeHandle n;

  ros::AsyncSpinner spinner(2);
  spinner.start();

  // Publishers for steering and wheel velocity commands
  ros::Publisher verde_steer_pub = n.advertise<std_msgs::Float64>("position_controller/command", 1);
  ros::Publisher verde_left_pub = n.advertise<std_msgs::Float64>("velocity_controllerLeft/command", 1);
  ros::Publisher verde_right_pub = n.advertise<std_msgs::Float64>("velocity_controllerRight/command", 1);
  
  // MONITORING MESSAGES
  ros::Publisher pose2D_pub = n.advertise<geometry_msgs::Pose2D>("pose2D", 1);
  ros::Publisher yaw_ref_pub = n.advertise<std_msgs::Float64>("yaw_ref", 1);
  ros::Publisher indx_traj_pub = n.advertise<std_msgs::Float64>("indx_traj", 1);
  ros::Publisher curvature_pub = n.advertise<std_msgs::Float64>("curvature", 1);
  ros::Publisher current_curvature_pub = n.advertise<std_msgs::Float64>("current_curvature", 1);
  ros::Publisher linear_vel_pub = n.advertise<std_msgs::Float64>("linear_vel", 1);
  ros::Publisher linear_vel_des_pub = n.advertise<std_msgs::Float64>("linear_vel_des", 1);
  ros::Publisher lat_error_pub = n.advertise<std_msgs::Float64>("lat_error", 1);
  ros::Publisher yaw_error_pub = n.advertise<std_msgs::Float64>("yaw_error", 1);
  ros::Publisher time_solve_pub = n.advertise<std_msgs::Float64>("time_solve", 1);
  ros::Publisher r_measured_pub = n.advertise<std_msgs::Float64>("r_measured", 1);

  // Control command publishers
  ros::Publisher a_cmd_pub = n.advertise<std_msgs::Float64>("a_cmd", 1);
  ros::Publisher z_cmd_pub = n.advertise<std_msgs::Float64>("z_cmd", 1);


  ros::Subscriber slam_pose = n.subscribe("/slam_out_pose", 100, poseCallBack);
  ros::Subscriber imu_sub = n.subscribe("xsens/imu", 100, imuCallBack); 
  ros::Subscriber odom_sub = n.subscribe("data/odom", 100, odomCallBack); 

  ros::NodeHandle nhPrivate("~");
 
  double rate = 20.0;
  h_step = 1.0/rate;

  // LOAD TRAJECTORY AND TUNNING PARAMETERS
  nhPrivate.getParam("traj_points_x", traj_points_x);
  ROS_INFO("traj_points_x, size: %lu", traj_points_x.size());

  nhPrivate.getParam("traj_points_y", traj_points_y);
  ROS_INFO("traj_points_y, size: %lu", traj_points_y.size());

  nhPrivate.getParam("vel_profile", vel_profile);
  ROS_INFO("vel_profile, size: %lu", vel_profile.size());

  nhPrivate.getParam("traj_curvature", traj_curvature);
  ROS_INFO("traj_curvature, size: %lu", traj_curvature.size());

  nhPrivate.getParam("look_ahead", look_ahead);
  ROS_INFO("look_ahead: %f", look_ahead);

  nhPrivate.getParam("ax_decay_rate", ax_decay_rate);
  ROS_INFO("ax_decay_rate: %f", ax_decay_rate);

  nhPrivate.getParam("linear_vel_decay_rate", linear_vel_decay_rate);
  ROS_INFO("linear_vel_decay_rate: %f", linear_vel_decay_rate);

  nhPrivate.getParam("r_decay_rate", r_decay_rate);
  ROS_INFO("r_decay_rate: %f", r_decay_rate);
  
  nhPrivate.getParam("e_to_k", e_to_k);
  ROS_INFO("e_to_k: %f", e_to_k);

  nhPrivate.getParam("use_optimum", use_optimum);
  ROS_INFO("use_optimum: %i", use_optimum);
   
  ros::Rate cycle(rate);

  // CCVXGEN initialization
  set_defaults();
  setup_indexing();
  loadDefaultData();
  settings.verbose = 0;
  
  // Messages: steering and speed commands
  std_msgs::Float64 steerval_msg; 
  std_msgs::Float64 leftval_msg;
  std_msgs::Float64 rightval_msg;

  // Monitoring message
  std_msgs::Float64 yaw_ref_msg;
  std_msgs::Float64 last_idx_msg;

  // Curvature
  std_msgs::Float64 curvature_msg;
  std_msgs::Float64 current_curvature_msg;

  // lin vel calculated
  std_msgs::Float64 linear_vel_msg;
  std_msgs::Float64 linear_vel_des_msg;

  // Info errors
  std_msgs::Float64 lat_error_msg;
  std_msgs::Float64 yaw_error_msg;
  std_msgs::Float64 r_measured_msg;

  // Variables of the  optimization problem
  std_msgs::Float64 a_cmd_msg;
  std_msgs::Float64 z_cmd_msg;

  std_msgs::Float64 time_solve_msg;

  while(ros::ok()) { 
    
    calcErrorTrajectory();

    time_solve_msg.data = solveProblem();

    getCommands(steerval_msg.data, leftval_msg.data, rightval_msg.data);

    // Publish commands
    verde_steer_pub.publish(steerval_msg);
    verde_left_pub.publish(leftval_msg);
    verde_right_pub.publish(rightval_msg);

    // Publish remaining variables 
    pose2D_pub.publish(pose2d);
    last_idx_msg.data = last_idx;
    indx_traj_pub.publish(last_idx_msg);
    yaw_ref_msg.data = yaw_ref;
    yaw_ref_pub.publish(yaw_ref_msg);

    current_curvature_msg.data = current_curvature;
    curvature_msg.data = curvature;

    current_curvature_pub.publish(current_curvature_msg);
    curvature_pub.publish(curvature_msg);

    linear_vel_msg.data = linear_vel;
    linear_vel_pub.publish(linear_vel_msg);

    linear_vel_des_msg.data = linear_vel_des;
    linear_vel_des_pub.publish(linear_vel_des_msg);

    lat_error_msg.data = lat_error;
    lat_error_pub.publish(lat_error_msg);

    yaw_error_msg.data = yaw_error;
    yaw_error_pub.publish(yaw_error_msg);

    r_measured_msg.data = r_measured;
    r_measured_pub.publish(r_measured_msg);

    a_cmd_msg.data = cmd_a;
    a_cmd_pub.publish(a_cmd_msg);

    z_cmd_msg.data = cmd_curvature;
    z_cmd_pub.publish(z_cmd_msg);

    time_solve_pub.publish(time_solve_msg);

    ros::spinOnce();
    cycle.sleep();
  }
  return 0;
}