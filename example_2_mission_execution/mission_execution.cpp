#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <carrot_interface/carrot_interface.hpp>

#include "mission_planner.hpp"

#include <future>

using namespace mavsdk;
using std::chrono::seconds;
using std::this_thread::sleep_for;

std::shared_ptr<System> get_system(Mavsdk &mavsdk)
{
    std::cout << "Waiting to discover system...\n";
    auto prom = std::promise<std::shared_ptr<System>>{};
    auto fut = prom.get_future();

    // We wait for new systems to be discovered, once we find one that has an
    // autopilot, we decide to use it.
    mavsdk.subscribe_on_new_system([&mavsdk, &prom]()
                                   {
        auto system = mavsdk.systems().back();

        if (system->has_autopilot()) {
            std::cout << "Discovered autopilot\n";

            // Unsubscribe again as we only want to find one system.
            mavsdk.subscribe_on_new_system(nullptr);
            prom.set_value(system);
        } });

    // We usually receive heartbeats at 1Hz, therefore we should find a
    // system after around 3 seconds max, surely.
    if (fut.wait_for(seconds(3)) == std::future_status::timeout)
    {
        std::cerr << "No autopilot found.\n";
        return {};
    }

    // Get discovered system now.
    return fut.get();
}

int main(int argc, char **argv)
{
    Mavsdk mavsdk;

    uint8_t our_sysid = 1;
    uint8_t our_compid = 200;
    mavsdk.set_configuration(Mavsdk::Configuration{our_sysid, our_compid, false});

    std::string connection_url;
    // connection_url = "tcp://:5790";
    connection_url = "udp://:14540";

    ConnectionResult connection_result = mavsdk.add_any_connection(connection_url);

    if (connection_result != ConnectionResult::Success)
    {
        std::cout << "Connection failed: " << connection_result << std::endl;
        return 1;
    }

    auto system = get_system(mavsdk);
    if (!system)
    {
        return 1;
    }

    int sysid = system->get_system_id();

    std::cout << "MAVSDK System ID: " << sysid << std::endl;

    auto action = Action{system};
    auto offboard = Offboard{system};
    auto telemetry = Telemetry{system};
    auto param = Param{system};

    std::pair<Param::Result, float> _param_nav_acc_rad = param.get_param_float("NAV_ACC_RAD");

    if (_param_nav_acc_rad.first != Param::Result::Success)
    {
        throw std::runtime_error("Unable to get NAV_ACC_RAD");
    }
    else
    {
        std::cout << "NAV_ACC_RAD: " << _param_nav_acc_rad.second << std::endl;
    }

    while (!telemetry.health_all_ok())
    {
        std::cout << "Waiting for system to be ready\n";
        sleep_for(seconds(1));
    }
    std::cout << "System is ready\n";

    while (!telemetry.armed())
    {
        std::cerr << "Waiting to arm... " << '\n';
        sleep_for(std::chrono::seconds(5));
    }

    std::cerr << "Armed" << '\n';

    const auto takeoff_result = action.takeoff();
    if (takeoff_result != Action::Result::Success)
    {
        std::cerr << "Takeoff failed: " << takeoff_result << '\n';
        return 1;
    }

    auto in_air_promise = std::promise<void>{};
    auto in_air_future = in_air_promise.get_future();
    telemetry.subscribe_landed_state([&telemetry, &in_air_promise](Telemetry::LandedState state)
                                     {
            if (state == Telemetry::LandedState::InAir) {
                std::cout << "Taking off has finished\n.";
                telemetry.subscribe_landed_state(nullptr);
                in_air_promise.set_value();
            } });
    in_air_future.wait_for(seconds(10));
    if (in_air_future.wait_for(seconds(120)) == std::future_status::timeout)
    {
        std::cerr << "Takeoff timed out.\n";
        return 1;
    }

    Telemetry::PositionVelocityNed _position_velocity_ned;

    telemetry.subscribe_position_velocity_ned([&](Telemetry::PositionVelocityNed position_velocity_ned)
                                              { _position_velocity_ned = position_velocity_ned; });

    Telemetry::Position _home_position = telemetry.home();

    std::cout << "Home position: " << _home_position.latitude_deg << ", " << _home_position.longitude_deg << ", " << _home_position.absolute_altitude_m << std::endl;

    // std::string plan_file = "example_2_mission_execution/example-mission.plan";

    std::string plan_file = "../example_2_mission_execution/example-mission.plan";

    px4_autonomy::MissionPlanner _mission_planner(plan_file);

    _mission_planner.setHomePosition(_home_position.latitude_deg, _home_position.longitude_deg);

    telemetry.subscribe_flight_mode([&](Telemetry::FlightMode flight_mode)
                                    {
            if (flight_mode == Telemetry::FlightMode::Offboard) {
                std::cout << "In Offboard Mode!" << std::endl;
                telemetry.subscribe_flight_mode(nullptr);
            } });

    std::cout << "Waiting to switch to Offboard Mode..." << std::endl;

    sleep_for(seconds(5));

    CarrotInterface _carrot_interface{system};

    _carrot_interface.enable();

    int current_index = 0;

    Eigen::Vector3f target_local_position_ned{};
    float target_yaw{};

    while (current_index != -1)
    {

        Eigen::Vector3f local_position{_position_velocity_ned.position.north_m, _position_velocity_ned.position.east_m,
                                       _position_velocity_ned.position.down_m};

        _mission_planner.updateCurrentWaypoint(local_position, _param_nav_acc_rad.second, target_local_position_ned);

        Eigen::Vector2f local_position_xy{local_position.x(), local_position.y()};

        Eigen::Vector2f target_local_position_xy{target_local_position_ned.x(), target_local_position_ned.y()};

        Eigen::Vector2f target_vector = target_local_position_xy - local_position_xy;

        _mission_planner.compute_heading_from_2D_vector(target_yaw, target_vector);

        _carrot_interface.flyToPositionYaw(
            target_local_position_ned, target_yaw);

        sleep_for(std::chrono::milliseconds(20)); // 50hz

        current_index = _mission_planner.getCurrentWaypointIndex();
    }

    _carrot_interface.disable();

    sleep_for(seconds(5));

    const auto land_result = action.land();
    if (land_result != Action::Result::Success)
    {
        std::cerr << "Landing failed: " << land_result << '\n';
        return 1;
    }

    // Check if vehicle is still in air
    while (telemetry.in_air())
    {
        std::cout << "Vehicle is landing...\n";
        sleep_for(seconds(5));
    }
    std::cout << "Landed!\n";

    sleep_for(seconds(3));
    std::cout << "Finished...\n";

    return 0;
}