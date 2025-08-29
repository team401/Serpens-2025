package frc.robot.constants;

import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.subsystems.scoring.ShooterConstants;

public class JsonConstants {
  public static void loadConstants() {
    EnvironmentHandler.getEnvironmentHandler(
        Filesystem.getDeployDirectory().toPath().resolve("constants/config.json").toString());

    FeatureFlags.synced.loadData();

    featureFlags = FeatureFlags.synced.getObject();
    shooterConstants = ShooterConstants.synced.getObject();
  }

  public static FeatureFlags featureFlags;
  public static ShooterConstants shooterConstants;
}
