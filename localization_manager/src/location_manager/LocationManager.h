#ifndef LocationManager_H
#define LocationManager_H

#ifndef UKF_DOUBLE_PRECISION
#define UKF_DOUBLE_PRECISION

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <vector>
#include <unordered_map>
#include "UKF/Types.h"
#include "UKF/Integrator.h"
#include "UKF/StateVector.h"
#include "UKF/MeasurementVector.h"
#include "UKF/Core.h"

/**
 * Defines the robot state field
 */ 
enum RobotState {
    Position,
    Orientation,
    Velocity,
    AngularVelocity
};

/**
 * Defines the robot state vector
 */
using LMStateVector = UKF::StateVector<
    UKF::Field<Position, UKF::Vector<3>>,
    UKF::Field<Orientation, UKF::Quaternion>,
    UKF::Field<Velocity, UKF::Vector<3>>,
    UKF::Field<AngularVelocity, UKF::Vector<3>>
>;


/**
 *  Process model for the robot motion. 
 *  Simple newtonian mechanics.
 * 
 *  TODO: Optimize use of incoming commands
 */ 
template <> template <>
LMStateVector LMStateVector::derivative<>() const {
    LMStateVector temp;

    //Angular stuff
    UKF::Quaternion omega_q;
    omega_q.vec() = get_field<AngularVelocity>() * 0.5;
    omega_q.w() = 0;
    temp.set_field<Orientation>(omega_q.conjugate() * get_field<Orientation>());
    temp.set_field<AngularVelocity>(UKF::Vector<3>(0, 0, 0));
    
    //Translational stuff
    temp.set_field<Position>(temp.get_field<Velocity>());
    temp.set_field<Velocity>(UKF::Vector<3>(0, 0, 0));
    return temp;
}

/**
 * The core localization manager class
 */ 
template<typename MeasurementVector>
class LocalizationManager {
public:
    UKF::SquareRootCore<LMStateVector, MeasurementVector, UKF::IntegratorRK4> filter;
    MeasurementVector last_meas;
    ros::Time last_update;

    LocalizationManager () {
        filter.state.set_field<Position>(UKF::Vector<3>(0, 0, 0));
        filter.state.set_field<Velocity>(UKF::Vector<3>(0, 0, 0));
        filter.state.set_field<Attitude>(UKF::Quaternion(1, 0, 0, 0));
        filter.state.set_field<AngularVelocity>(UKF::Vector<3>(0, 0, 0));
    }

    void requestSensorUpdate (ros::Time timestamp, int sensorName) {
        if(last_update > timestamp) {
            ROS_WARN("Dropping measurement from %s", sensorName);
            return;
        }
    }

    /**
     * Retrieves pose
     */ 
    nav_msgs::Odometry getPoseEstimate() {

    }
};
#endif
#endif