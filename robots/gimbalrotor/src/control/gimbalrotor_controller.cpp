#include <gimbalrotor/control/gimbalrotor_controller.h>
#include <algorithm>
#include <cmath>
#include <proxsuite/proxqp/dense/dense.hpp>

using namespace std;

namespace aerial_robot_control
{
GimbalrotorController::GimbalrotorController() : PoseLinearController()
{
}

void GimbalrotorController::initialize(ros::NodeHandle nh, ros::NodeHandle nhp,
                                       boost::shared_ptr<aerial_robot_model::RobotModel> robot_model,
                                       boost::shared_ptr<aerial_robot_estimation::StateEstimator> estimator,
                                       boost::shared_ptr<aerial_robot_navigation::BaseNavigator> navigator,
                                       double ctrl_loop_rate)
{
  PoseLinearController::initialize(nh, nhp, robot_model, estimator, navigator, ctrl_loop_rate);
  gimbalrotor_robot_model_ = boost::dynamic_pointer_cast<GimbalrotorRobotModel>(robot_model);

  GimbalrotorController::rosParamInit();

  rotor_coef_ = gimbal_dof_ + 1;  // number of virtual rotors in each rotor arm

  target_base_thrust_.resize(motor_num_ * rotor_coef_);
  target_full_thrust_.resize(motor_num_);
  target_gimbal_angles_.resize(motor_num_ * gimbal_dof_, 0);

  flight_cmd_pub_ = nh_.advertise<spinal::FourAxisCommand>("four_axes/command", 1);
  gimbal_control_pub_ = nh_.advertise<sensor_msgs::JointState>("gimbals_ctrl", 1);
  gimbal_state_pub_ = nh_.advertise<sensor_msgs::JointState>("joint_states", 1);
  target_vectoring_force_pub_ = nh_.advertise<std_msgs::Float32MultiArray>("debug/target_vectoring_force", 1);
  rpy_gain_pub_ = nh_.advertise<spinal::RollPitchYawTerms>("rpy/gain", 1);
  torque_allocation_matrix_inv_pub_ =
      nh_.advertise<spinal::TorqueAllocationMatrixInv>("torque_allocation_matrix_inv", 1);
  gimbal_dof_pub_ = nh_.advertise<std_msgs::UInt8>("gimbal_dof", 1);
}

void GimbalrotorController::reset()
{
  PoseLinearController::reset();

  setAttitudeGains();
}

void GimbalrotorController::rosParamInit()
{
  ros::NodeHandle control_nh(nh_, "controller");
  getParam<int>(control_nh, "gimbal_dof", gimbal_dof_, 1);
  getParam<bool>(control_nh, "gimbal_calc_in_fc", gimbal_calc_in_fc_, true);
  getParam<bool>(control_nh, "hovering_approximate", hovering_approximate_, false);
  getParam<bool>(control_nh, "underactuate", underactuate_, false);
  //QP
  getParam<bool>(control_nh, "use_ground_qp",
                 use_ground_qp_, false);
  getParam<double>(control_nh, "ground_mu_static",
                   ground_mu_static_, 0.5);
  getParam<double>(control_nh, "ground_mu_kinetic",
                   ground_mu_kinetic_, 0.3);
  getParam<double>(control_nh, "ground_rolling_resistance",
                 ground_rolling_resistance_, 0.0);
  // N_min = ground_normal_force_rate * mg
  getParam<double>(control_nh, "ground_normal_force_rate",
                   ground_normal_force_rate_, 0.1);
  getParam<double>(control_nh, "ground_vel_eps",
                   ground_vel_eps_, 0.03);
  getParam<double>(control_nh, "ground_acc_eps",
                   ground_acc_eps_, 0.3);
}

bool GimbalrotorController::update()
{
  sendGimbalCommand();
  if (gimbal_calc_in_fc_)
  {
    std_msgs::UInt8 msg;
    msg.data = gimbal_dof_;
    gimbal_dof_pub_.publish(msg);
  }

  return PoseLinearController::update();
}

double smoothStep(double x)
{
  x = std::max(0.0, std::min(1.0, x));
  return x * x * (3.0 - 2.0 * x);
}

Eigen::Vector3d calcFstar(
    const Eigen::Vector2d& acc_xy_cmd,
    const Eigen::Vector2d& d_hat,
    double mu,double limit_min,double limit_max)
{
  double g = aerial_robot_estimation::G;
  // A_mu: 2 x 3
  Eigen::Matrix<double, 2, 3> A_mu;
  A_mu << 1.0, 0.0, mu * d_hat.x(),
          0.0, 1.0, mu * d_hat.y();

  // b = m a_xy_cmd + mu m g d_hat
  Eigen::Vector2d b;
  b = acc_xy_cmd + mu * g * d_hat;

  // F* = A^T (A A^T)^-1 b
  Eigen::Matrix2d AAT = A_mu * A_mu.transpose();

  Eigen::Vector2d lambda = AAT.ldlt().solve(b);

  Eigen::Vector3d F_star = A_mu.transpose() * lambda;
  if(F_star.z()>limit_max){
    F_star.z() = limit_max;
    F_star.x() = b.x() - mu * d_hat.x() * F_star.z();
    F_star.y() = b.y() - mu * d_hat.y() * F_star.z();
  }
  else if (F_star.z()<limit_min){
    F_star.z() = limit_min;
    F_star.x() = b.x() - mu * d_hat.x() * F_star.z();
    F_star.y() = b.y() - mu * d_hat.y() * F_star.z();
  }
  return F_star;
}
  
void GimbalrotorController::controlCore()
{
  PoseLinearController::controlCore();
  tf::Matrix3x3 uav_rot = estimator_->getOrientation(Frame::COG, estimate_mode_);
  tf::Vector3 target_acc_w(pid_controllers_.at(X).result(), pid_controllers_.at(Y).result(),
                           pid_controllers_.at(Z).result());
  /*if(navigator_->getNaviState() == aerial_robot_navigation::HOVER_STATE ){
    }*/
    
  tf::Vector3 target_acc_dash = (tf::Matrix3x3(tf::createQuaternionFromYaw(rpy_.z()))).inverse() * target_acc_w;
  tf::Vector3 target_acc_cog = uav_rot.inverse() * target_acc_w;
  Eigen::VectorXd target_wrench_acc_cog = Eigen::VectorXd::Zero(6);

  if (underactuate_)
    target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_dash.x(), target_acc_dash.y(), target_acc_dash.z());
  else
    target_wrench_acc_cog.head(3) = Eigen::Vector3d(target_acc_cog.x(), target_acc_cog.y(), target_acc_cog.z());

  double target_ang_acc_x = pid_controllers_.at(ROLL).result();
  double target_ang_acc_y = pid_controllers_.at(PITCH).result();
  double target_ang_acc_z = pid_controllers_.at(YAW).result();
  Eigen::Matrix3d inertia = gimbalrotor_robot_model_->getInertia<Eigen::Matrix3d>();
  Eigen::Vector3d omega;
  tf::vectorTFToEigen(omega_, omega);
  Eigen::Vector3d gyro = omega.cross(inertia * omega);

  if (gimbal_calc_in_fc_)
    target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z);
  else
    target_wrench_acc_cog.tail(3) = Eigen::Vector3d(target_ang_acc_x, target_ang_acc_y, target_ang_acc_z) + gyro;

  pid_msg_.roll.total.at(0) = target_ang_acc_x;
  pid_msg_.roll.p_term.at(0) = pid_controllers_.at(ROLL).getPTerm();
  pid_msg_.roll.i_term.at(0) = pid_controllers_.at(ROLL).getITerm();
  pid_msg_.roll.d_term.at(0) = pid_controllers_.at(ROLL).getDTerm();
  pid_msg_.roll.target_p = target_rpy_.x();
  pid_msg_.roll.err_p = pid_controllers_.at(ROLL).getErrP();
  pid_msg_.roll.target_d = target_omega_.x();
  pid_msg_.roll.err_d = pid_controllers_.at(ROLL).getErrD();
  pid_msg_.pitch.total.at(0) = target_ang_acc_y;
  pid_msg_.pitch.p_term.at(0) = pid_controllers_.at(PITCH).getPTerm();
  pid_msg_.pitch.i_term.at(0) = pid_controllers_.at(PITCH).getITerm();
  pid_msg_.pitch.d_term.at(0) = pid_controllers_.at(PITCH).getDTerm();
  pid_msg_.pitch.target_p = target_rpy_.y();
  pid_msg_.pitch.err_p = pid_controllers_.at(PITCH).getErrP();
  pid_msg_.pitch.target_d = target_omega_.y();
  pid_msg_.pitch.err_d = pid_controllers_.at(PITCH).getErrD();

  Eigen::MatrixXd full_q_mat = Eigen::MatrixXd::Zero(6, 3 * motor_num_);

  double mass_inv = 1 / gimbalrotor_robot_model_->getMass();

  Eigen::Matrix3d inertia_inv = inertia.inverse();

  std::vector<Eigen::Vector3d> rotors_origin_from_cog =
      gimbalrotor_robot_model_->getRotorsOriginFromCog<Eigen::Vector3d>();
  const auto& rotor_direction = gimbalrotor_robot_model_->getRotorDirection();
  const double m_f_rate = gimbalrotor_robot_model_->getMFRate();
  
  Eigen::MatrixXd wrench_map = Eigen::MatrixXd::Zero(6, 3);
  wrench_map.block(0, 0, 3, 3) = Eigen::MatrixXd::Identity(3, 3);
  int last_col = 0;

  /* calculate normal allocation */
  for (int i = 0; i < motor_num_; i++)
  {
    wrench_map.block(3, 0, 3, 3) = aerial_robot_model::skew(rotors_origin_from_cog.at(i)) +
                                   rotor_direction.at(i + 1) * m_f_rate * Eigen::Matrix3d::Identity();
    full_q_mat.middleCols(last_col, 3) = wrench_map;
    last_col += 3;
  }
  Eigen::MatrixXd full_wrench_map=full_q_mat;//add
  full_q_mat.topRows(3) = mass_inv * full_q_mat.topRows(3);
  full_q_mat.bottomRows(3) = inertia_inv * full_q_mat.bottomRows(3);

  /* calculate masked rotation matrix */
  std::vector<KDL::Rotation> thrust_coords_rot = gimbalrotor_robot_model_->getThrustCoordRot<KDL::Rotation>();
  std::vector<Eigen::MatrixXd> masked_rot;
  for (int i = 0; i < motor_num_; i++)
  {
    tf::Quaternion r;
    tf::quaternionKDLToTF(thrust_coords_rot.at(i), r);
    Eigen::Matrix3d conv_cog_from_thrust;
    tf::matrixTFToEigen(tf::Matrix3x3(r), conv_cog_from_thrust);
    if (gimbal_dof_ == 1)
    {
      Eigen::MatrixXd mask(3, 2);
      mask << 0, 0, 1, 0, 0, 1;
      masked_rot.push_back(conv_cog_from_thrust * mask);
    }
    else if (gimbal_dof_ == 2)
    {
      Eigen::MatrixXd mask = Eigen::Matrix3d::Identity();
      masked_rot.push_back(conv_cog_from_thrust * mask);
    }
  }

  /* mask integrated allocation */
  Eigen::MatrixXd integrated_rot = Eigen::MatrixXd::Zero(3 * motor_num_, rotor_coef_ * motor_num_);
  Eigen::MatrixXd integrated_map = Eigen::MatrixXd::Zero(6, (gimbal_dof_ + 1) * motor_num_);
  for (int i = 0; i < motor_num_; i++)
  {
    integrated_rot.block(3 * i, rotor_coef_ * i, 3, rotor_coef_) = masked_rot[i];
  }
  Eigen::MatrixXd integrated_wrench_map = full_wrench_map * integrated_rot;//add Q'
  integrated_map = full_q_mat * integrated_rot;

  /* extract controlled axis  */
  if (underactuate_)
  {
    target_wrench_acc_cog = target_wrench_acc_cog.tail(4);  // z, roll, pitch, yaw
    integrated_map = integrated_map.bottomRows(4);          // z, roll, pitch, yaw
  }

  /* vectoring force mapping */
  Eigen::MatrixXd integrated_map_inv = aerial_robot_model::pseudoinverse(integrated_map);
  integrated_map_inv_trans_ = integrated_map_inv.leftCols(underactuate_ ? 1 : 3);
  integrated_map_inv_rot_ = integrated_map_inv.rightCols(3);
  if (underactuate_)
    target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog(0);
  else
    target_vectoring_f_trans_ = integrated_map_inv_trans_ * target_wrench_acc_cog.topRows(3);
  target_vectoring_f_rot_ = integrated_map_inv_rot_ * target_wrench_acc_cog.bottomRows(3);  // debug
  last_col = 0;
  //QP
  const bool ground_qp_active =use_ground_qp_ &&!underactuate_;
  if (ground_qp_active)
  {
    const int n_var =rotor_coef_ * motor_num_;
    const int n_eq = 6;//等式制約
    const int n_in = 1;//不等式制約
    /*nominal allocation
    Eigen::VectorXd lambda_nom =target_vectoring_f_trans_+target_vectoring_f_rot_;
     * Q_F, Q_tau
     *
     * W = [F ; tau] = Qbar lambda */
    Eigen::MatrixXd Q_F =
        integrated_wrench_map.topRows(3);

    Eigen::MatrixXd Q_tau =
        integrated_wrench_map.bottomRows(3);

    /*
     * friction direction
     * Q_F is COG frame, therefore d_hat also COG frame
    */

    tf::Vector3 vel_w =estimator_->getVel(Frame::COG,estimate_mode_);
    tf::Vector3 vel_cog =uav_rot.inverse() *vel_w;

    Eigen::Vector2d vel_xy(vel_cog.x(),vel_cog.y());
    Eigen::Vector2d acc_xy_cmd(target_acc_cog.x(),target_acc_cog.y());

    const double vel_norm =vel_xy.norm();
    const double acc_norm =acc_xy_cmd.norm();

    Eigen::Vector2d d_hat = Eigen::Vector2d::Zero();
    double mu = 0.0;

    // 静止摩擦16角形 
    const int friction_edges = 16;
    const int n_in_static = friction_edges + 3;

    const double mass = gimbalrotor_robot_model_->getMass();

    const double gravity = aerial_robot_estimation::G;
    const double mg = mass * gravity;
    
    const double N_min = ground_normal_force_rate_ * mg;

    std::array<Eigen::Vector3d, 8> contact_pos = {
    //4隅（COG基準）
    Eigen::Vector3d( 0.238388,  0.259399, 0.0),
    Eigen::Vector3d(-0.281612,  0.259399, 0.0),
    Eigen::Vector3d(-0.281612, -0.260601, 0.0),
    Eigen::Vector3d( 0.238388, -0.260601, 0.0),

    // rotor_arm直下4点（COG基準）
    Eigen::Vector3d( 0.098798,  0.119809, 0.0),
    Eigen::Vector3d(-0.142902,  0.120899, 0.0),
    Eigen::Vector3d(-0.142902, -0.122101, 0.0),
    Eigen::Vector3d( 0.099678, -0.122101, 0.0)
    };

    Eigen::Vector3d vel_cog_eigen(vel_cog.x(),vel_cog.y(),vel_cog.z());
    Eigen::Vector3d s = Eigen::Vector3d::Zero();
    Eigen::Vector3d h = Eigen::Vector3d::Zero();

    const int contact_num = 8;

    for (int i = 0; i < contact_num; ++i)
      {
	const Eigen::Vector3d& r_i = contact_pos[i];
	// 接地点速度
	Eigen::Vector3d v_i =vel_cog_eigen + omega.cross(r_i);
	Eigen::Vector2d v_i_xy(v_i.x(), v_i.y());
	Eigen::Vector3d d_i = Eigen::Vector3d::Zero();

	if (v_i_xy.norm() > 1.0e-4)
	  {
	    d_i.x() = v_i_xy.x() / v_i_xy.norm();
	    d_i.y() = v_i_xy.y() / v_i_xy.norm();
	  }

	// 1/8 Σ d_i
	s += d_i / static_cast<double>(contact_num);
	
	// 1/8 Σ (r_i × d_i)
	h += r_i.cross(d_i) / static_cast<double>(contact_num);
      }

    double r_eff = 0.0;
    for (int i = 0; i < contact_num; ++i)
      {
	const double rho_i =
        std::sqrt(contact_pos[i].x() * contact_pos[i].x() + contact_pos[i].y() * contact_pos[i].y());
	r_eff += rho_i / static_cast<double>(contact_num);
      }
 
    //desired torque
    Eigen::Vector3d alpha_cmd(target_ang_acc_x,target_ang_acc_y,target_ang_acc_z);
    Eigen::Vector3d tau_cmd =inertia * alpha_cmd + gyro;

    //STATIC FRICTION QP
    Eigen::MatrixXd Aeq_static = Eigen::MatrixXd::Zero(n_eq, n_var);
    Eigen::VectorXd beq_static = Eigen::VectorXd::Zero(n_eq);

    Aeq_static.topRows(2) = Q_F.topRows(2);
    beq_static.head(2) = mass * acc_xy_cmd;
    
    //roll, pitch, yaw
    Aeq_static.bottomRows(3) = Q_tau;
    beq_static.tail(3) = tau_cmd;
    //16角形で近似
    Eigen::MatrixXd C_static = Eigen::MatrixXd::Zero(n_in_static, n_var);
    Eigen::VectorXd l_static = Eigen::VectorXd::Constant(n_in_static, -1.0e20);
    Eigen::VectorXd u_static = Eigen::VectorXd::Zero(n_in_static);

    //cos(theta) fx + sin(theta) fy<= mu N cos(pi/16)
    const double polygon_scale = std::cos(M_PI / friction_edges);
    for (int j = 0; j < friction_edges; ++j)
      {
	const double theta =
	  2.0 * M_PI * j / friction_edges;
	const double nx = std::cos(theta);
	const double ny = std::sin(theta);
	C_static.row(j) =-nx * Q_F.row(0)-ny * Q_F.row(1)+ ground_mu_static_* polygon_scale* Q_F.row(2);
	u_static(j) =ground_mu_static_* polygon_scale * mg;
      }
    C_static.row(friction_edges) =Q_F.row(2);
    l_static(friction_edges) = 0.0;
    u_static(friction_edges) = mg - N_min;

    const int yaw_pos_idx = friction_edges + 1;
    const int yaw_neg_idx = friction_edges + 2;

    C_static.row(yaw_pos_idx) =Q_tau.row(2) + ground_mu_static_ * r_eff * Q_F.row(2);

    u_static(yaw_pos_idx) =ground_mu_static_ * r_eff * mg;
    
    C_static.row(yaw_neg_idx) =-Q_tau.row(2)+ ground_mu_static_ * r_eff * Q_F.row(2);

    u_static(yaw_neg_idx) = ground_mu_static_ * r_eff * mg;

    Eigen::MatrixXd H_static = Eigen::MatrixXd::Identity(n_var, n_var);
    Eigen::VectorXd g_static = Eigen::VectorXd::Zero(n_var);//-lambda_nom;
    //solve
    using namespace proxsuite::proxqp;

    dense::QP<double> qp_static(n_var,n_eq,n_in_static);

    qp_static.settings.verbose = false;
    qp_static.settings.eps_abs = 1.0e-6;
    qp_static.settings.eps_rel = 1.0e-6;

    qp_static.init(H_static,g_static,Aeq_static,beq_static,C_static,l_static,u_static);
    qp_static.solve();
    
    if (false/*vel_norm < ground_vel_eps_ && qp_static.results.info.status == QPSolverOutput::PROXQP_SOLVED*/)
      {
	Eigen::VectorXd lambda_static = qp_static.results.x;

	target_vectoring_f_trans_ = lambda_static;
	target_vectoring_f_rot_ = Eigen::VectorXd::Zero(n_var);
	/* debug */
	Eigen::Vector3d force_actual = Q_F * lambda_static;
	Eigen::Vector2d friction_static =-force_actual.head<2>();

	double Fz = force_actual.z();
	double N  = mg - Fz;
	ROS_INFO_STREAM_THROTTLE(
				 0.1,
				 "STICK"
				 << " F = " << force_actual.transpose()
				 << " fs = " << friction_static.transpose()
				 << " |fs| = " << friction_static.norm()
				 << " muN = " << ground_mu_static_ * N
				 << " N = " << N);
	Eigen::Vector3d tau_actual = Q_tau * lambda_static;
	ROS_INFO_STREAM_THROTTLE(
				 0.1,
				 "yaw alpha_cmd=" << target_ang_acc_z
				 << " tau_z_cmd=" << tau_cmd.z()
				 << " tau_z_actual=" << tau_actual.z()
				 << " omega_z=" << omega.z());
      }
    else
      {
	ROS_INFO_STREAM_THROTTLE(0.1,"Static friction limit exceeded -> SLIP");
    /*if (vel_norm > ground_vel_eps_)
    {
      d_hat =vel_xy / vel_norm;
      mu = ground_mu_kinetic_;
    }
    else if (acc_norm > ground_acc_eps_)
    {
      d_hat = acc_xy_cmd / acc_norm;

      mu = ground_mu_static_;
      }*/;
    mu = ground_mu_kinetic_;
    /* [ QFx + mu dx QFz ] lambda= m ax + mu m g dx
       [ QFy + mu dy QFz ] lambda= m ay + mu m g dy */
    Eigen::MatrixXd Aeq =Eigen::MatrixXd::Zero(n_eq,n_var);
    Eigen::VectorXd beq =Eigen::VectorXd::Zero(n_eq);
    Eigen::Vector2d s_xy(s.x(), s.y());
    Aeq.topRows(2) =Q_F.topRows(2)+mu *s_xy *Q_F.row(2);
    
    beq.head(2) =gimbalrotor_robot_model_->getMass() *acc_xy_cmd + mu *gimbalrotor_robot_model_->getMass() *aerial_robot_estimation::G * s_xy;

    /* yaw 
     tau_rotor =I alpha + omega x I omega
     地面のyaw摩擦トルクは0としている*/

    Eigen::Vector3d alpha_cmd(target_ang_acc_x,target_ang_acc_y,target_ang_acc_z);
    Eigen::Vector3d tau_cmd =inertia *alpha_cmd+gyro;
    Eigen::Vector3d h_yaw = Eigen::Vector3d::Zero();

    const double omega_yaw_eps = 0.03;   // [rad/s]
    const double yaw_cmd__eps = 0.05;   // [rad/s]

    double yaw_dir = 0.0;
    const double yaw_rate_cmd =target_omega_.z();
    if (std::abs(omega.z()) > omega_yaw_eps)
      {
	h_yaw.z() =h.z();
      }
    else if (std::abs(yaw_rate_cmd) > yaw_cmd__eps)
      {
	yaw_dir = (target_ang_acc_z > 0.0) ? 1.0 : -1.0;
	h_yaw.z() = r_eff * yaw_dir;}
    else{
      h_yaw.z() =0.0;
    }
 
    Aeq.bottomRows(3) =Q_tau + mu * h_yaw * Q_F.row(2);;
    beq.tail(3) =tau_cmd + mu * gimbalrotor_robot_model_->getMass() * aerial_robot_estimation::G * h_yaw;;

    /* N = mg - Fz
     N >= N_min
     Fz <= mg - N_min */
    Eigen::MatrixXd C =Eigen::MatrixXd::Zero(n_in,n_var);
    Eigen::VectorXd l =Eigen::VectorXd::Zero(n_in);

    Eigen::VectorXd u =Eigen::VectorXd::Zero(n_in);
    C.row(0) =Q_F.row(2);
    const double N_min =ground_normal_force_rate_*gimbalrotor_robot_model_->getMass()*aerial_robot_estimation::G;
    l(0) = 0;
    u(0) = gimbalrotor_robot_model_->getMass() * aerial_robot_estimation::G - N_min;
    /*Objective
     * 1/2 ||lambda-lambda_nom||^2
     * 1/2 lambda^T lambda
     * - lambda_nom^T lambda
     * + constant
     * H = I
     * g = -lambda_nom
     */

    Eigen::MatrixXd H =Eigen::MatrixXd::Identity(n_var,n_var);
    Eigen::VectorXd g =Eigen::VectorXd::Zero(n_var);//-lambda_nom;
    //ProxQP solve
    using namespace
        proxsuite::proxqp;

    dense::QP<double> qp(n_var,n_eq,n_in);
    qp.settings.verbose = false;
    qp.settings.eps_abs = 1.0e-6;
    qp.settings.eps_rel = 1.0e-6;

    qp.init(H,g,Aeq,beq,C,l,u);

    qp.solve();

    if (qp.results.info.status ==QPSolverOutput::PROXQP_SOLVED)
    {
      Eigen::VectorXd lambda_qp =qp.results.x;
      
      target_vectoring_f_trans_ =lambda_qp;

      target_vectoring_f_rot_ =Eigen::VectorXd::Zero(n_var);

      Eigen::Vector3d tau_actual = Q_tau * lambda_qp;
      Eigen::Vector3d tau_error  = tau_actual - tau_cmd;

      /* ROS_INFO_STREAM_THROTTLE(
			     0.1,
			     "tau_actual = " << tau_actual.transpose()
			     << " tau_cmd = " << tau_cmd.transpose()
			     << " tau_error = " << tau_error.transpose());*/
      Eigen::Vector3d force_actual = Q_F * lambda_qp;

      ROS_INFO_STREAM_THROTTLE(
    0.1,
    "Force = " << force_actual.transpose()
    << "  Fz = " << force_actual.z());

      double mass =gimbalrotor_robot_model_->getMass();

      double Fz = force_actual.z();

      double N =mass * aerial_robot_estimation::G - Fz;

      ROS_INFO_STREAM_THROTTLE(
    0.1,
    "Fz = " << Fz
    << " N = " << N);

      ROS_INFO_STREAM_THROTTLE(
    0.1,
    "vel=" << vel_xy.transpose()
    << " vel_norm=" << vel_norm
    << " acc=" << acc_xy_cmd.transpose()
    << " mu=" << mu
    << " d_hat=" << d_hat.transpose());

    ROS_INFO_STREAM_THROTTLE(
			     0.1,
			     "yaw alpha_cmd=" << target_ang_acc_z
			     << " tau_z_cmd=" << tau_cmd.z()
			     << " tau_z_actual=" << tau_actual.z()
			     << " omega_z=" << omega.z());
    ROS_INFO_STREAM(
    "s = " << s.transpose()
    << " h = " << h.transpose()
    << " muNh_z = " << mu * N * h.z());
    }
    else
    {
      ROS_WARN_THROTTLE(1.0,"Ground ProxQP failed; ""use nominal pseudoinverse allocation");
    }
  }
  }
  
  /* under actuated axis  */
  if (underactuate_)
  {
    if (hovering_approximate_)
    {
      target_roll_ = -target_acc_dash.y() / aerial_robot_estimation::G;
      target_pitch_ = target_acc_dash.x() / aerial_robot_estimation::G;
      navigator_->setTargetRoll(target_roll_);
      navigator_->setTargetPitch(target_pitch_);
    }
    else
    {
      target_roll_ = atan2(-target_acc_dash.y(),
                           sqrt(target_acc_dash.x() * target_acc_dash.x() + target_acc_dash.z() * target_acc_dash.z()));
      target_pitch_ = atan2(target_acc_dash.x(), target_acc_dash.z());
      navigator_->setTargetRoll(target_roll_);
      navigator_->setTargetPitch(target_pitch_);
    }
  }

  /*  calculate target base thrust (considering only translational components)*/
  double max_yaw_scale = 0;  // for reconstruct yaw control term in spinal
  for (int i = 0; i < motor_num_; i++)
  {
    Eigen::VectorXd f_i = target_vectoring_f_trans_.segment(last_col, rotor_coef_);
    if (gimbal_dof_ == 1)
    {
      target_base_thrust_.at(rotor_coef_ * i) = f_i[0];
      target_base_thrust_.at(rotor_coef_ * i + 1) = f_i[1];
    }
    else if (gimbal_dof_ == 2)
    {
      target_base_thrust_.at(rotor_coef_ * i) = f_i[0];
      target_base_thrust_.at(rotor_coef_ * i + 1) = f_i[1];
      target_base_thrust_.at(rotor_coef_ * i + 2) = f_i[2];
    }
    if (integrated_map_inv(i, (underactuate_ ? YAW - 2 : YAW)) > max_yaw_scale)
      max_yaw_scale = integrated_map_inv(i, (underactuate_ ? YAW - 2 : YAW));  // underactuated: yaw col is shifted

    last_col += rotor_coef_;
  }
  //yaw加算阻止
  if (ground_qp_active)
  {
    candidate_yaw_term_ = 0.0;
  }
  else
  {
    candidate_yaw_term_ =pid_controllers_.at(YAW).result()*max_yaw_scale;
  }

  /* calculate target full thrusts and gimbal angles (considering full components)*/
  last_col = 0;
  for (int i = 0; i < motor_num_; i++)
  {
    Eigen::VectorXd f_i_integrated = target_vectoring_f_rot_.segment(last_col, rotor_coef_) +
                                     target_vectoring_f_trans_.segment(last_col, rotor_coef_);
    target_full_thrust_.at(i) = f_i_integrated.norm();
    if (gimbal_dof_ == 1)
    {
      target_gimbal_angles_.at(i) = atan2(-f_i_integrated[0], f_i_integrated[1]);
    }
    else if (gimbal_dof_ == 2)
    {
      if (f_i_integrated[0] == 0 || f_i_integrated[2] == 0)
        continue;

      double gimbal_roll = atan2(-f_i_integrated[1], f_i_integrated[2]);
      double gimbal_pitch =
          atan2(f_i_integrated[0], -f_i_integrated[1] * sin(gimbal_roll) + f_i_integrated[2] * cos(gimbal_roll));
      target_gimbal_angles_.at(2 * i) = gimbal_roll;
      target_gimbal_angles_.at(2 * i + 1) = gimbal_pitch;
    }
    last_col += rotor_coef_;
  }
}

void GimbalrotorController::sendCmd()
{
  PoseLinearController::sendCmd();

  sendFourAxisCommand();

  if (gimbal_calc_in_fc_)
  {
    sendTorqueAllocationMatrixInv();
  }
  else
  {
    sensor_msgs::JointState gimbal_control_msg;
    gimbal_control_msg.header.stamp = ros::Time::now();
    for (int i = 0; i < motor_num_; i++)
    {
      if (gimbal_dof_ == 1)
      {
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(i));
      }
      else if (gimbal_dof_ == 2)
      {
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(2 * i));
        gimbal_control_msg.position.push_back(target_gimbal_angles_.at(2 * i + 1));
      }
    }
    gimbal_control_pub_.publish(gimbal_control_msg);

    std_msgs::Float32MultiArray target_vectoring_force_msg;
    target_vectoring_f_ = target_vectoring_f_trans_ + target_vectoring_f_rot_;
    for (int i = 0; i < target_vectoring_f_.size(); i++)
    {
      target_vectoring_force_msg.data.push_back(target_vectoring_f_(i));
    }
    target_vectoring_force_pub_.publish(target_vectoring_force_msg);
  }
}

void GimbalrotorController::sendFourAxisCommand()
{
  spinal::FourAxisCommand flight_command_data;

  flight_command_data.angles[0] = target_roll_;
  flight_command_data.angles[1] = target_pitch_;

  if (gimbal_calc_in_fc_)
  {
    flight_command_data.base_thrust = target_base_thrust_;
    flight_command_data.angles[2] = candidate_yaw_term_;
  }
  else
  {
    flight_command_data.base_thrust = target_full_thrust_;
  }

  flight_cmd_pub_.publish(flight_command_data);
}

void GimbalrotorController::sendGimbalCommand()
{
  sensor_msgs::JointState gimbal_state_msg;
  gimbal_state_msg.header.stamp = ros::Time::now();
  for (int i = 0; i < motor_num_; i++)
  {
    if (gimbal_dof_ == 1)
    {
      gimbal_state_msg.position.push_back(target_gimbal_angles_.at(i));
      std::string gimbal_name = "gimbal" + std::to_string(i + 1);
      gimbal_state_msg.name.push_back(gimbal_name);
    }
    else if (gimbal_dof_ == 2)
    {
      gimbal_state_msg.position.push_back(target_gimbal_angles_.at(2 * i));
      gimbal_state_msg.position.push_back(target_gimbal_angles_.at(2 * i + 1));
      std::string gimbal_roll_name = "gimbal" + std::to_string(i + 1) + "_roll";
      std::string gimbal_pitch_name = "gimbal" + std::to_string(i + 1) + "_pitch";
      gimbal_state_msg.name.push_back(gimbal_roll_name);
      gimbal_state_msg.name.push_back(gimbal_pitch_name);
    }
  }
  // gimbal_state_pub_.publish(gimbal_state_msg);
}

void GimbalrotorController::sendTorqueAllocationMatrixInv()
{
  spinal::TorqueAllocationMatrixInv torque_allocation_matrix_inv_msg;
  torque_allocation_matrix_inv_msg.rows.resize(motor_num_ * rotor_coef_);
  Eigen::MatrixXd torque_allocation_matrix_inv = integrated_map_inv_rot_;
  if (torque_allocation_matrix_inv.cwiseAbs().maxCoeff() > INT16_MAX * 0.001f)
    ROS_ERROR("Torque Allocation Matrix overflow");
  for (unsigned int i = 0; i < motor_num_ * rotor_coef_; i++)
  {
    torque_allocation_matrix_inv_msg.rows.at(i).x = torque_allocation_matrix_inv(i, 0) * 1000;
    torque_allocation_matrix_inv_msg.rows.at(i).y = torque_allocation_matrix_inv(i, 1) * 1000;
    torque_allocation_matrix_inv_msg.rows.at(i).z = torque_allocation_matrix_inv(i, 2) * 1000;
  }
  torque_allocation_matrix_inv_pub_.publish(torque_allocation_matrix_inv_msg);
}

void GimbalrotorController::setAttitudeGains()
{
  spinal::RollPitchYawTerms rpy_gain_msg;  // for rosserial
  /* to flight controller via rosserial scaling by 1000 */
  rpy_gain_msg.motors.resize(1);
  rpy_gain_msg.motors.at(0).roll_p = pid_controllers_.at(ROLL).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_i = pid_controllers_.at(ROLL).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).roll_d = pid_controllers_.at(ROLL).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_p = pid_controllers_.at(PITCH).getPGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_i = pid_controllers_.at(PITCH).getIGain() * 1000;
  rpy_gain_msg.motors.at(0).pitch_d = pid_controllers_.at(PITCH).getDGain() * 1000;
  rpy_gain_msg.motors.at(0).yaw_d = pid_controllers_.at(YAW).getDGain() * 1000;
  rpy_gain_pub_.publish(rpy_gain_msg);
}
}  // namespace aerial_robot_control

/* plugin registration */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(aerial_robot_control::GimbalrotorController, aerial_robot_control::ControlBase);
