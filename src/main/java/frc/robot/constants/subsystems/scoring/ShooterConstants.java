package frc.robot.constants.subsystems.scoring;

import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;

public final class ShooterConstants {
  @JSONExclude
  public static final JSONSync<ShooterConstants> synced =
      new JSONSync<ShooterConstants>(
          new ShooterConstants(),
          "ShooterConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  // TODO: Use JSONSync
  /**
   * What fraction of the total angular velocity must the shooter velocity be within in order for the shooter to be considered "ready?"
   * 
   * <p> For example, if this value is 0.05, the shooter wheels must be within +/- 5% of the goal speed to shoot.
   * */
  public final Double shooterVelocityEpsilonFraction = 0.05;
}
