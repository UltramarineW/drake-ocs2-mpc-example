// system
#include <iostream>
#include <gflags/gflags.h>
//drake
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/meshcat_visualizer.h"
#include "drake/geometry/meshcat.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/common/eigen_types.h"

#include "utils.h"
#include "mpc_controller.h"
#include "user_command.h"

DEFINE_double(dt, 0.01, "system control update time");
DEFINE_double(simulation_time, 1000.0, "simulation time");

// drake main process
int DoMain(const std::string exec_path) {
    drake::systems::DiagramBuilder<double> builder;
    auto scene_graph = builder.AddSystem<drake::geometry::SceneGraph<double>>();
    // configure multiplant
    auto plant = builder.AddSystem<drake::multibody::MultibodyPlant>(FLAGS_dt);
    plant->RegisterAsSourceForSceneGraph(scene_graph);
    
    const std::string model_name = GetAbsolutePath("double_integrator.urdf");
    const std::string target_model_name = GetAbsolutePath("target_ball.urdf");
    const auto double_integrator_instance = drake::multibody::Parser(plant).AddModelFromFile(model_name);
    // drake::multibody::Parser(plant).AddModelFromFile(target_model_name);
    
    // set fixed base
    plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("slideBar"));
    // plant->WeldFrames(plant->world_frame(), plant->GetFrameByName("target"));
    plant->Finalize();
    uint32_t nq = plant->num_positions();
    uint32_t nv = plant->num_velocities();
    uint32_t na = plant->num_actuators();
    std::cout << "nq: " << nq << " nv: " << nv << " na: " << na << "\n";

    // geometry settings 
    builder.Connect(plant->get_geometry_poses_output_port(), scene_graph->get_source_pose_port(plant->get_source_id().value()));
    builder.Connect(scene_graph->get_query_output_port(), plant->get_geometry_query_input_port());
    auto meshcat = std::make_shared<drake::geometry::Meshcat>();
    auto& visualizer = drake::geometry::MeshcatVisualizerd::AddToBuilder(&builder, *scene_graph, meshcat);
    meshcat->AddSlider("Desire Target", -10, 10, 0.5, 0);

    // controller constructor setting 
    const std::string taskFile = GetAbsolutePath("task.info");
    const std::string LibraryPath = GetAbsolutePath("auto_generated/"); 
    auto controller = builder.AddSystem<My_MPC_Controller>(plant, taskFile, LibraryPath, false);
    auto user_command_system = builder.AddSystem<UserCommand>(plant, meshcat);
    builder.Connect(plant->get_state_output_port(), controller->get_input_port(1));
    builder.Connect(controller->get_output_port(0), plant->get_actuation_input_port());
    builder.Connect(user_command_system->get_output_port(0), controller->get_input_port(0));
    auto diagram = builder.Build();

    // prepare to start the simulation
    drake::systems::Simulator<double> simulator(*diagram);
    auto& plant_context = diagram->GetMutableSubsystemContext(*plant, &simulator.get_mutable_context());
    auto& controller_context = diagram->GetMutableSubsystemContext(*controller, &simulator.get_mutable_context());

    drake::VectorX<double> q0 = drake::VectorX<double>::Zero(1);
    drake::VectorX<double> v0 = drake::VectorX<double>::Zero(1);
    double desired_position = 4.0;
    q0 << 0.0;
    v0 << 0.0;
    plant->SetPositions(&plant_context, q0);
    plant->SetVelocities(&plant_context, v0);

    // start simulation
    simulator.Initialize();
    simulator.set_target_realtime_rate(1.0);
    visualizer.StartRecording();
    simulator.AdvanceTo(FLAGS_simulation_time);
    visualizer.StopRecording();
    visualizer.PublishRecording();
    return 0;
}

int main(int argc, char **argv) {
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    const std::string exec_path = argv[0];
    return DoMain(exec_path);
}