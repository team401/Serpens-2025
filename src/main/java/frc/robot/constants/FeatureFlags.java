package frc.robot.constants;

import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;

public final class FeatureFlags {
  @JSONExclude
  public static final JSONSync<FeatureFlags> synced =
      new JSONSync<FeatureFlags>(
          new FeatureFlags(),
          "FeatureFlags.json",
          JsonConstants.getPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final Boolean runDrive = true;
}
