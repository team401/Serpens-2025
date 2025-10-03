package frc.robot.constants;

import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.wpilibj.Filesystem;

public class JsonConstants {
  public static void loadConstants() {
    EnvironmentHandler.getEnvironmentHandler(
        Filesystem.getDeployDirectory().toPath().resolve("constants/config.json").toString());

    FeatureFlags.synced.loadData();
    IndexerConstants.synced.loadData();

    featureFlags = FeatureFlags.synced.getObject();
    indexerConstants = IndexerConstants.synced.getObject();
  }

  public static FeatureFlags featureFlags;
  public static IndexerConstants indexerConstants;
  public static IndexerConstants indexerConstantsSim;
}
