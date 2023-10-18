#include "PositionSmoothing.hpp"


class CarrotInterface
{
public:
    CarrotInterface(std::shared_ptr<mavsdk::System> system)
        : _position_smoothing(system)
    {
    }

    void flyToPositionYaw(const Eigen::Vector3f &target_local_position_ned, float &target_yaw)
    {
        _position_smoothing.fly_to_position_local_ned(target_local_position_ned, target_yaw);
    }

    bool enable()
    {
        return _position_smoothing.startOffboardMode();
    }

    bool disable()
    {
        return _position_smoothing.stopOffboardMode();
    }

private:
    PositionSmoothing _position_smoothing;
};