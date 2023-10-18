#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include <mavsdk/mavsdk.h>
#include <mavsdk/plugins/action/action.h>
#include <mavsdk/plugins/offboard/offboard.h>
#include <mavsdk/plugins/telemetry/telemetry.h>

#include <carrot_interface/carrot_interface.hpp>

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

    sleep_for(seconds(5));

    CarrotInterface _carrot_interface{system};

    _carrot_interface.enable();

    int current_index = 0;

    Eigen::Vector3f target_local_position_ned{0.0f, 0.0f, -10.0f};
    float target_yaw{-M_PI};

    while (1)
    {

        _carrot_interface.flyToPositionYaw(
            target_local_position_ned, target_yaw);

        sleep_for(std::chrono::milliseconds(20)); // 50hz

    }

    _carrot_interface.disable();

    return 0;
}