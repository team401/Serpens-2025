package frc.robot.constants;

import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import coppercore.parameter_tools.path_provider.EnvironmentPathProvider;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.subsystems.drive.DriveTrainConstants;

public class JsonConstants {

  private static final EnvironmentHandler ENVIRONMENT_HANDLER =
      EnvironmentHandler.getEnvironmentHandler(
          Filesystem.getDeployDirectory().toPath().resolve("constants/config.json").toString());
  private static final EnvironmentPathProvider ENVIRONMENT_PATH_PROVIDER =
      ENVIRONMENT_HANDLER.getEnvironmentPathProvider();

  public static EnvironmentPathProvider getPathProvider() {
    return ENVIRONMENT_PATH_PROVIDER;
  }

  public static void loadConstants() {
    FeatureFlags.synced.loadData();
    OperatorConstants.synced.loadData();
    DriveTrainConstants.synced.loadData();

    featureFlags = FeatureFlags.synced.getObject();
    operatorConstants = OperatorConstants.synced.getObject();
    drivetrainConstants = DriveTrainConstants.synced.getObject();
  }

  public static FeatureFlags featureFlags;
  public static OperatorConstants operatorConstants;
  public static DriveTrainConstants drivetrainConstants;
}
