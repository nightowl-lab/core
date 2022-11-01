  | Name                                   | Type                                                     | Description                                           |
  | -------------------------------------- | -------------------------------------------------------- | ----------------------------------------------------- |
  | `/control/command/control_cmd`         | autoware_auto_control_msgs::msg::AckermannControlCommand | lateral and longitudinal control command              |
  | `/control/command/gear_cmd`            | autoware_auto_vehicle_msgs::msg::GearCommand             | gear command                                          |
  | `/control/command/turn_indicators_cmd` | autoware_auto_vehicle_msgs::msg::TurnIndicatorsCommand   | turn indicators command                               |
  | `/vehicle/engage`                      | autoware_auto_vehicle_msgs::msg::Engage                  | engage command                                        |
  | `/control/command/emergency_cmd`       | tier4_vehicle_msgs::msg::VehicleEmergencyStamped         | emergency command                                     |
  | `/vehicle/status/control_mode`           | autoware_auto_vehicle_msgs::msg::ControlModeReport      | control mode                                         |
  | `/vehicle/status/velocity_status`        | autoware_auto_vehicle_msgs::msg::VelocityReport         | velocity                                             |
  | `/vehicle/status/steering_status`        | autoware_auto_vehicle_msgs::msg::SteeringReport         | steering wheel angle                                 |
  | `/vehicle/status/gear_status`            | autoware_auto_vehicle_msgs::msg::GearReport             | gear status                                          |
  | `/vehicle/status/turn_indicators_status` | autoware_auto_vehicle_msgs::msg::TurnIndicatorsReport   | turn indicators status                               |