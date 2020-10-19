#include <ros/ros.h>
#include <ros/package.h>
#include <boost/filesystem.hpp>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/MarkerArray.h>
#include <tesseract_msgs/GetMotionPlanAction.h>
#include <tesseract_rosutils/plotting.h>
#include <actionlib/client/simple_action_client.h>
#include <tesseract_rosutils/conversions.h>
#include <tesseract_command_language/command_language.h>
#include <tesseract_command_language/serialize.h>
#include <tesseract_command_language/deserialize.h>
#include <tesseract_visualization/visualization_loader.h>
#include <tesseract_monitoring/tesseract_monitor_interface.h>
#include <tesseract_environment/ofkt/ofkt_state_solver.h>

#include <yaml-cpp/yaml.h>
#include <yaml-cpp/node/iterator.h>
#include <yaml-cpp/node/impl.h>

#include <tf2_eigen/tf2_eigen.h>

using namespace tesseract_planning;

static const std::string TOOLPATH = "twc_toolpath";
static const std::string OUTPUT_PLAN_FILE_PATH = "output_plan.xml";
static const std::vector<std::string> HOME_JOINTS = { "positioner_joint_1", "positioner_joint_2", "robot_joint_1",
                                                      "robot_joint_2",      "robot_joint_3",      "robot_joint_4",
                                                      "robot_joint_5",      "robot_joint_6" };
static const std::vector<double> HOME_POSITION = { 0, 0, 0, -0.349066, 0.349066, 0, 0, 0 };

static geometry_msgs::Pose pose3DtoPoseMsg(const std::array<float, 6>& p)
{
  using namespace Eigen;
  geometry_msgs::Pose pose_msg;
  Eigen::Affine3d eigen_pose = Translation3d(Vector3d(std::get<0>(p), std::get<1>(p), std::get<2>(p))) *
                               AngleAxisd(std::get<3>(p), Vector3d::UnitX()) *
                               AngleAxisd(std::get<4>(p), Vector3d::UnitY()) *
                               AngleAxisd(std::get<5>(p), Vector3d::UnitZ());

  pose_msg = tf2::toMsg(eigen_pose);
  return std::move(pose_msg);
}


visualization_msgs::MarkerArray convertToAxisMarkers(const std::vector<geometry_msgs::PoseArray>& path,
                                          const std::string& frame_id,
                                          const std::string& ns,
                                          const std::size_t& start_id = 1,
                                          const double& axis_scale =  0.001,
                                          const double& axis_length = 0.01,
                                          const std::array<float, 6>& offset =  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 })
{
  using namespace Eigen;

  visualization_msgs::MarkerArray markers;

  auto create_line_marker = [&](const int id,
                                const std::tuple<float, float, float, float>& rgba) -> visualization_msgs::Marker {
    visualization_msgs::Marker line_marker;
    line_marker.action = line_marker.ADD;
    std::tie(line_marker.color.r, line_marker.color.g, line_marker.color.b, line_marker.color.a) = rgba;
    line_marker.header.frame_id = frame_id;
    line_marker.type = line_marker.LINE_LIST;
    line_marker.id = id;
    line_marker.lifetime = ros::Duration(0);
    line_marker.ns = ns;
    std::tie(line_marker.scale.x, line_marker.scale.y, line_marker.scale.z) = std::make_tuple(axis_scale, 0.0, 0.0);
    line_marker.pose = pose3DtoPoseMsg(offset);
    return std::move(line_marker);
  };

  // markers for each axis line
  int marker_id = start_id;
  visualization_msgs::Marker x_axis_marker = create_line_marker(++marker_id, std::make_tuple(1.0, 0.0, 0.0, 1.0));
  visualization_msgs::Marker y_axis_marker = create_line_marker(++marker_id, std::make_tuple(0.0, 1.0, 0.0, 1.0));
  visualization_msgs::Marker z_axis_marker = create_line_marker(++marker_id, std::make_tuple(0.0, 0.0, 1.0, 1.0));

  auto add_axis_line = [](const Isometry3d& eigen_pose,
                          const Vector3d& dir,
                          const geometry_msgs::Point& p1,
                          visualization_msgs::Marker& marker) {
    geometry_msgs::Point p2;
    Eigen::Vector3d line_endpoint;

    // axis endpoint
    line_endpoint = eigen_pose * dir;
    std::tie(p2.x, p2.y, p2.z) = std::make_tuple(line_endpoint.x(), line_endpoint.y(), line_endpoint.z());

    // adding line
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  };

  for (auto& poses : path)
  {
    for (auto& pose : poses.poses)
    {
      Eigen::Isometry3d eigen_pose;
      tf2::fromMsg(pose, eigen_pose);

      geometry_msgs::Point p1;
      std::tie(p1.x, p1.y, p1.z) = std::make_tuple(pose.position.x, pose.position.y, pose.position.z);

      add_axis_line(eigen_pose, Vector3d::UnitX() * axis_length, p1, x_axis_marker);
      add_axis_line(eigen_pose, Vector3d::UnitY() * axis_length, p1, y_axis_marker);
      add_axis_line(eigen_pose, Vector3d::UnitZ() * axis_length, p1, z_axis_marker);
    }
  }

  markers.markers.push_back(x_axis_marker);
  markers.markers.push_back(y_axis_marker);
  markers.markers.push_back(z_axis_marker);
  return std::move(markers);
}

visualization_msgs::MarkerArray convertToAxisMarkers(const std::vector<std::vector<Eigen::Isometry3d>>& path,
                                                     const std::string& frame_id,
                                                     const std::string& ns,
                                                     const std::size_t& start_id = 1,
                                                     const double& axis_scale =  0.001,
                                                     const double& axis_length = 0.01,
                                                     const std::array<float, 6>& offset =  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 })
{
  std::vector<geometry_msgs::PoseArray> path_vec_msg;
  for(const auto&v : path)
  {
    geometry_msgs::PoseArray path_msg;
    for(const Eigen::Isometry3d& p : v)
    {
      path_msg.poses.push_back(tf2::toMsg(p));
    }
    path_vec_msg.push_back(path_msg);
  }
  return convertToAxisMarkers(path_vec_msg, frame_id, ns, start_id, axis_scale, axis_length, offset);
}

///
/// \brief parsePathFromFile Creates a collection of raster strips from a yaml file
/// \param yaml_filepath
/// \return success
///
bool parsePathFromFile(std::vector<std::vector<Eigen::Isometry3d>>& raster_strips,
                       const std::string& yaml_filepath)
{
  YAML::Node full_yaml_node = YAML::LoadFile(yaml_filepath);
  YAML::Node paths = full_yaml_node[0]["paths"];
  std::double_t offset_strip = 0.0;
  for (YAML::const_iterator path_it = paths.begin(); path_it != paths.end(); ++path_it)
  {
    std::vector<Eigen::Isometry3d> temp_poses;
    YAML::Node strip = (*path_it)["poses"];
    for (YAML::const_iterator pose_it = strip.begin(); pose_it != strip.end(); ++pose_it)
    {
      const YAML::Node& pose = *pose_it;
      try
      {
        Eigen::Isometry3d current_pose;

        float x = pose["position"]["x"].as<float>();
        float y = pose["position"]["y"].as<float>();
        float z = pose["position"]["z"].as<float>();

        float qx = pose["orientation"]["x"].as<float>();
        float qy = pose["orientation"]["y"].as<float>();
        float qz = pose["orientation"]["z"].as<float>();
        float qw = pose["orientation"]["w"].as<float>();

        current_pose.translation() = Eigen::Vector3d(x, y, z);
        current_pose.linear() = Eigen::Quaterniond(qw, qx, qy, qz).toRotationMatrix();

        Eigen::Isometry3d offset_pose =
            current_pose * Eigen::Translation3d(0.0, 0.0, offset_strip) * Eigen::Quaterniond(0, 1, 0, 0);

        temp_poses.push_back(offset_pose);
      }
      catch (YAML::InvalidNode& e)
      {
        continue;
      }
    }
    raster_strips.push_back(temp_poses);
  }
  return true;
}

std::vector<std::vector<Eigen::Isometry3d>> filterPath(std::vector<std::vector<Eigen::Isometry3d>>& paths)
{
  std::vector<std::vector<Eigen::Isometry3d>> filter_paths;

  for (std::size_t i = 0; i < paths.size(); ++i)
  {

    if (i < 2)
    {
      filter_paths.push_back(paths[i]);
    }
    else if (i < 5)
    {
      std::vector<Eigen::Isometry3d> path;
      for (std::size_t j = 0; j < paths[i].size(); ++j)
      {
        if (j < 2)
          path.push_back(paths[i][j]);
        else if (j > 7 && j < 12)
          path.push_back(paths[i][j]);
        else if (j > 17 && j < 22)
          path.push_back(paths[i][j]);
        else if (j > paths[i].size() - 3)
          path.push_back(paths[i][j]);
      }

      filter_paths.push_back(path);
    }
    else
    {
      filter_paths.push_back(paths[i]);
    }
  }

  return filter_paths;
}

CompositeInstruction createProgram(const std::vector<std::vector<Eigen::Isometry3d>>& raster_strips,
                                   const std::string& working_frame,
                                   Eigen::Isometry3d tcp,
                                   Eigen::Isometry3d transform = Eigen::Isometry3d::Identity())
{
  ManipulatorInfo manip_info("robot_only");
  manip_info.working_frame = working_frame;
  manip_info.tcp = tcp;
  CompositeInstruction program("raster_program", CompositeInstructionOrder::ORDERED, manip_info);

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "robot_joint_1", "robot_joint_2", "robot_joint_3",
                                           "robot_joint_4", "robot_joint_5", "robot_joint_6" };
  StateWaypoint swp1 = StateWaypoint(joint_names, Eigen::VectorXd::Zero(6));
  PlanInstruction start_instruction(swp1, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  for (std::size_t rs = 0; rs < raster_strips.size(); ++rs)
  {
    if (rs == 0)
    {
      // Define from start composite instruction
      CartesianWaypoint wp1 = transform * raster_strips[rs][0];
      PlanInstruction plan_f0(wp1, PlanInstructionType::FREESPACE, "FREESPACE");
      plan_f0.setDescription("from_start_plan");
      CompositeInstruction from_start;
      from_start.setDescription("from_start");
      from_start.push_back(plan_f0);
      program.push_back(from_start);
    }

    // Define raster
    CompositeInstruction raster_segment;
    raster_segment.setDescription("Raster #" + std::to_string(rs + 1));

    for (std::size_t i = 1; i < raster_strips[rs].size(); ++i)
    {
      CartesianWaypoint wp = transform * raster_strips[rs][i];
      raster_segment.push_back(PlanInstruction(wp, PlanInstructionType::LINEAR, "RASTER"));
    }
    program.push_back(raster_segment);


    if (rs < raster_strips.size() - 1)
    {
      // Add transition
      CartesianWaypoint twp = transform * raster_strips[rs + 1].front();

      PlanInstruction tranisiton_instruction1(twp, PlanInstructionType::FREESPACE, "TRANSITION");
      tranisiton_instruction1.setDescription("transition_from_end_plan");

      CompositeInstruction transition;
      transition.setDescription("transition_from_end");
      transition.push_back(tranisiton_instruction1);

      program.push_back(transition);
    }
    else
    {
      // Add to end instruction
      PlanInstruction plan_f2(swp1, PlanInstructionType::FREESPACE, "FREESPACE");
      plan_f2.setDescription("to_end_plan");
      CompositeInstruction to_end;
      to_end.setDescription("to_end");
      to_end.push_back(plan_f2);
      program.push_back(to_end);
    }
  }

  return program;
}

CompositeInstruction createFreespaceProgram(Eigen::VectorXd jp, Eigen::Isometry3d tcp)
{
  ManipulatorInfo manip_info("robot_only");
  manip_info.tcp = tcp;
  CompositeInstruction program("raster_program", CompositeInstructionOrder::ORDERED, manip_info);

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "robot_joint_1", "robot_joint_2", "robot_joint_3",
                                           "robot_joint_4", "robot_joint_5", "robot_joint_6" };
  StateWaypoint swp1 = StateWaypoint(joint_names, Eigen::VectorXd::Zero(6));
  PlanInstruction start_instruction(swp1, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);


  JointWaypoint jwp = JointWaypoint(joint_names, jp);
  program.push_back(PlanInstruction(jwp, PlanInstructionType::FREESPACE));
  return program;
}

CompositeInstruction createCartesianProgram(Eigen::Isometry3d tcp)
{
  ManipulatorInfo manip_info("robot_only");
  manip_info.tcp = tcp;
  CompositeInstruction program("raster_program", CompositeInstructionOrder::ORDERED, manip_info);

  // Start Joint Position for the program
  std::vector<std::string> joint_names = { "robot_joint_1", "robot_joint_2", "robot_joint_3",
                                           "robot_joint_4", "robot_joint_5", "robot_joint_6" };
  StateWaypoint swp1 = StateWaypoint(joint_names, Eigen::VectorXd::Zero(6));
  PlanInstruction start_instruction(swp1, PlanInstructionType::START);
  program.setStartInstruction(start_instruction);

  CartesianWaypoint cwp1(Eigen::Isometry3d::Identity() * Eigen::Translation3d(-0.2, 0, 1.5) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0)));
  CartesianWaypoint cwp2(Eigen::Isometry3d::Identity() * Eigen::Translation3d(0.2, 0, 1.5) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d(1, 0, 0)));
  program.push_back(PlanInstruction(cwp1, PlanInstructionType::FREESPACE));
  program.push_back(PlanInstruction(cwp2, PlanInstructionType::LINEAR));
  return program;
}


int main(int argc, char** argv)
{
  namespace fs = boost::filesystem;

  ros::init(argc, argv, "application_node");
  ros::NodeHandle nh, pnh("~");

  std::string tool_path = ros::package::getPath("twc_application") + "/config/job_path.yaml";
  std::string seed_plan = "";
  tool_path = pnh.param<std::string>("tool_path", tool_path);
  seed_plan = pnh.param<std::string>("seed_plan", seed_plan);
  ROS_INFO("Using tool path file: %s", tool_path.c_str());

  // Create a tesseract interface
  tesseract_monitoring::TesseractMonitorInterface interface("tesseract_workcell");
  tesseract::Tesseract::Ptr thor = interface.getTesseract<tesseract_environment::OFKTStateSolver>("tesseract_workcell_environment");
  auto current_transforms = thor->getEnvironment()->getCurrentState()->link_transforms;

  // creating marker array publisher for toolpaths
  ros::Publisher toolpath_pub = nh.advertise<visualization_msgs::MarkerArray>("input_toolpath", 1);

  // Dynamically load ignition visualizer if exist
  tesseract_visualization::VisualizationLoader loader;
  auto plotter = loader.get();

  if (plotter != nullptr && thor != nullptr)
  {
    plotter->init(thor);
    plotter->waitForConnection(3);
    plotter->plotEnvironment();
  }

  if((plotter != nullptr && !plotter->isConnected()) || (plotter == nullptr && thor != nullptr))
  {
    plotter = std::make_shared<tesseract_rosutils::ROSPlotting>();
    plotter->init(thor);
    plotter->waitForConnection(3);
  }

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<tesseract_msgs::GetMotionPlanAction> ac("/twc_planning_server/tesseract_get_motion_plan", true);

  // wait for the action server to start
  ROS_INFO("Waiting for action server to start.");
  ac.waitForServer();  // will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  tesseract_msgs::GetMotionPlanGoal goal;

  Eigen::Isometry3d tcp = current_transforms["robot_tool0"].inverse() * current_transforms["st_tool0"];

  Eigen::Isometry3d rot_offset = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.05) * Eigen::AngleAxisd(M_PI_2, Eigen::Vector3d(0, 0, 1));
  Eigen::Isometry3d transform = current_transforms["part_link"];
  std::vector<std::vector<Eigen::Isometry3d>> paths;
  parsePathFromFile(paths, tool_path);
  std::vector<std::vector<Eigen::Isometry3d>> filtered_path = filterPath(paths);

  // publishing path
  visualization_msgs::MarkerArray toolpath_markers = convertToAxisMarkers(filtered_path, "part_link", "twc");
  toolpath_pub.publish(toolpath_markers);

  // create program
  CompositeInstruction program = createProgram(filtered_path, "", tcp * rot_offset, transform);
  goal.request.name = goal.RASTER_G_FT_PLANNER_NAME;

  // loading seed path
  if(!seed_plan.empty())
  {
    if(!fs::exists(fs::path(seed_plan)))
    {
      ROS_ERROR("Seed path file %s does not exists", seed_plan.c_str());
      return -1;
    }
    std::ifstream seed_plan_stream(seed_plan);
    std::stringstream ss;
    ss << seed_plan_stream.rdbuf();
    goal.request.seed = ss.str();
    ROS_INFO("Loaded seed plan file %s",seed_plan.c_str());
  }
  else
  {
    ROS_WARN("Seed plan file not loaded, skipping");
  }

//  Eigen::VectorXd jp = Eigen::VectorXd::Zero(6);
//  jp(0) = M_PI_2;
//  CompositeInstruction program = createFreespaceProgram(jp, tcp * rot_offset);
//  goal.request.name = goal.CARTESIAN_PLANNER_NAME;

//  Eigen::Isometry3d rot_offset = Eigen::Isometry3d::Identity() * Eigen::Translation3d(0, 0, 0.05) * Eigen::AngleAxisd(-M_PI_2, Eigen::Vector3d(0, 0, 1));
//  CompositeInstruction program = createCartesianProgram(tcp * rot_offset);
//  goal.request.name = goal.DESCARTES_PLANNER_NAME;

  if (plotter != nullptr && thor != nullptr)
  {
    plotter->waitForInput();
    plotter->plotToolPath(program);
  }

  goal.request.instructions = toXMLString(program);
  goal.request.num_threads = 1;

  ros::Time start_time = ros::Time::now();
  ac.sendGoal(goal);
  ac.waitForResult();

  actionlib::SimpleClientGoalState state = ac.getState();
  ROS_INFO("Action finished: %s", state.toString().c_str());
  ROS_WARN("Planning time: %f", (ros::Time::now() - start_time).toSec());

  auto result = ac.getResult();
  Instruction program_results = fromXMLString(result->response.results);

  // saving program into file
  std::ofstream output_plan_stream(OUTPUT_PLAN_FILE_PATH);
  output_plan_stream << result->response.results;
  output_plan_stream.close();
  ROS_INFO("Saved output plan file to location %s", OUTPUT_PLAN_FILE_PATH.c_str());

  if (!result->response.successful)
  {
    ROS_ERROR("Get Motion Plan Failed: %s", result->response.status_string.c_str());
  }
  else
  {
    ROS_ERROR("Get Motion Plan Successful!");
  }

  if (plotter != nullptr && thor != nullptr)
  {
    plotter->waitForInput();
    plotter->plotToolPath(program_results);

    plotter->waitForInput();
    plotter->plotTrajectory(program_results);
  }

  ros::spin();

  return 0;
}
