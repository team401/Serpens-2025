package frc.robot.constants.subsystems.scoring;

import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;

/** FeatureFlags for which mechanisms of the scoring subsystem should be run. */
public class ScoringFeatureFlags {
  @JSONExclude
  public static final JSONSync<ScoringFeatureFlags> synced =
      new JSONSync<ScoringFeatureFlags>(
          new ScoringFeatureFlags(),
          "ScoringFeatureFlags.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final Boolean runShooter = true;
  public final Boolean runIndexer =
      false; // This value is false until the indexer is written & merged
}
