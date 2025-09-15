package frc.robot.constants.subsystems.drive;

import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import frc.robot.constants.JsonConstants;

public class DriveTrainConstants {
  @JSONExclude
  public static final JSONSync<DriveTrainConstants> synced =
      new JSONSync<DriveTrainConstants>(
          new DriveTrainConstants(),
          "DriveTrainConstants.json",
          JsonConstants.getPathProvider(),
          new JSONSyncConfigBuilder().build());

  public final Double maxLinearSpeed = 1.0; // meters per second
  public final Double maxAngularSpeed = 1.0; // radians per second
  public final Double joystickDeadband = 0.1; // joystick deadband
}
