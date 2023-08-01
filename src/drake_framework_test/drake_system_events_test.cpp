/*
    该文件用与验证drake framework中的各个system更新的顺序以及当某个system中的update函数超过更新频率时间时候的处理操作
    测试结果：
    当Update函数时间大于更新时间时，系统会按照Update的函数时间来进行
    当框架中有两个LeafSystem在更新的时候，会按照最慢系统的Update函数时间进行更新，也就是达不到设定的更新时间
*/
#include <iostream>
#include <unistd.h>
#include <signal.h>
#include <time.h>
#include <fstream>
#include <gflags/gflags.h>
#include "drake/systems/framework/diagram_builder.h"
#include "drake/systems/analysis/simulator.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/multibody/plant/multibody_plant.h"
#include "drake/multibody/parsing/parser.h"
#include "drake/geometry/scene_graph.h"
#include "drake/geometry/drake_visualizer.h"
#include "drake/multibody/tree/revolute_joint.h"
#include "drake/lcm/drake_lcm.h"
#include <drake/systems/lcm/lcm_interface_system.h>
#include "drake/systems/lcm/lcm_publisher_system.h"
#include "drake/systems/lcm/lcm_subscriber_system.h"
#include "drake/systems/controllers/linear_quadratic_regulator.h"
#include "drake/systems/primitives/vector_log_sink.h"
#include "drake/common/eigen_types.h"

#define TIME_DIFF(t0, t1) ((t1.tv_sec + t1.tv_nsec * 1e-9) - (t0.tv_sec + t0.tv_nsec * 1e-9))

// initialize system update rate 2000hz
DEFINE_double(dt, 1, "system update rate");

namespace drake
{
    class MySystem : public systems::LeafSystem<double>
    {
    public:
        DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MySystem);
        MySystem()
        {
            DeclareDiscreteState(1);
            dt_ = FLAGS_dt;
            const double offset = 0.0;
            DeclarePeriodicDiscreteUpdateEvent(dt_, 0.0, &MySystem::Update);
            // DeclarePeriodicUnrestrictedUpdateEvent(dt_, 0.0, &MySystem::Update);
            DeclareVectorOutputPort("num", 1, &MySystem::Output);
        }

    private:
        void Update(const systems::Context<double> &context,
                    systems::DiscreteValues<double> *xd) const
        {
            struct timespec t0, t1;
            // 验证当update函数时间大于更新时间的行为
            clock_gettime(CLOCK_MONOTONIC, &t0);
            std::cout << "Discrete system update start" << std::endl;
            int i = 0;
            const double x_n = context.get_discrete_state()[0];
            (*xd)[0] = x_n + 1;
            std::cout << "update i: " << i << std::endl;
            i++;
            sleep(1);
            std::cout << "update i: " << i << std::endl;
            i++;
            sleep(1);
            std::cout << "update i: " << i << std::endl;
            i++;
            sleep(1);
            clock_gettime(CLOCK_MONOTONIC, &t1);
            std::cout << "Period: " << TIME_DIFF(t0, t1) << std::endl;
            std::cout << "Discrete system update end" << std::endl;
        }
        void Output(const systems::Context<double> &context,
                    systems::BasicVector<double> *results) const
        {
            double x_n = context.get_discrete_state()[0];
            double s_n = 10 * x_n;
            (*results)[0] = s_n;
        }

        double dt_;
    };
} // namespace drake;

namespace drake
{
    class MySystem2 : public systems::LeafSystem<double>
    {
    public:
        DRAKE_NO_COPY_NO_MOVE_NO_ASSIGN(MySystem2);
        MySystem2()
        {
            DeclareDiscreteState(1);
            dt_ = FLAGS_dt;
            const double offset = 0.0;
            DeclarePeriodicDiscreteUpdateEvent(dt_, 0.0, &MySystem2::Update);
            DeclareVectorOutputPort("num", 1, &MySystem2::Output);
        }

    private:
        void Update(const systems::Context<double> &context,
                    systems::DiscreteValues<double> *xd) const
        {
            struct timespec t0, t1;
            // 验证当update函数时间大于更新时间的行为
            clock_gettime(CLOCK_MONOTONIC, &t0);
            std::cout << "Discrete system2 update start" << std::endl;
            int i = 0;
            const double x_n = context.get_discrete_state()[0];
            (*xd)[0] = x_n + 1;
            std::cout << "update i: " << i << std::endl;
            i++;
            clock_gettime(CLOCK_MONOTONIC, &t1);
            std::cout << "Period: " << TIME_DIFF(t0, t1) << std::endl;
            std::cout << "Discrete system2 update end" << std::endl;
        }
        void Output(const systems::Context<double> &context,
                    systems::BasicVector<double> *results) const
        {
            double x_n = context.get_discrete_state()[0];
            double s_n = 10 * x_n;
            (*results)[0] = s_n;
        }

        double dt_;
    }; // class MySystem2
} // namespace drake

namespace drake
{
    int doMain()
    {
        systems::DiagramBuilder<double> builder;
        // multibody::MultibodyPlant<double> *plant = builder.AddSystem<multibody::MultibodyPlant>(FLAGS_dt);
        // geometry::SceneGraph<double> *scene_graph = builder.AddSystem<geometry::SceneGraph>();
        // plant->RegisterAsSourceForSceneGraph(scene_graph);
        auto example = builder.AddSystem<MySystem>();
        auto example2 = builder.AddSystem<MySystem2>();
        auto logger = LogVectorOutput(example->GetOutputPort("num"), &builder, 1.0);
        auto diagram = builder.Build();

        struct timespec system_start_time, system_end_time;
        clock_gettime(CLOCK_MONOTONIC, &system_start_time);
        systems::Simulator<double> simulator(*diagram);
        simulator.Initialize();
        simulator.set_target_realtime_rate(1.0);
        simulator.AdvanceTo(10);
        clock_gettime(CLOCK_MONOTONIC, &system_end_time);
        std::cout << "System simulation time: " << TIME_DIFF(system_start_time, system_end_time) 
        << " rate: " << 10.0 / TIME_DIFF(system_start_time, system_end_time) << std::endl;
        std::cout << "Actual simulation rate: " << simulator.get_actual_realtime_rate() << std::endl;
        const auto &log = logger->FindLog(simulator.get_context());
        for (int n = 0; n < log.sample_times().size(); ++n)
        {
            const double t = log.sample_times()[n];
            std::cout << n << ": " << log.data()(0, n) << " (" << t << ")\n";
        }
        return 0;
    }
} // namespace drake

int main(int argc, char *argv[])
{
    gflags::ParseCommandLineFlags(&argc, &argv, true);
    drake::doMain();
}
