package frc.robot.constants.subsystems.scoring;

import static edu.wpi.first.units.Units.Seconds;

import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.units.measure.Time;

/** General scoring constants that don't fit under either mechanism */
public class ScoringConstants {
  @JSONExclude
  public static final JSONSync<ScoringConstants> synced =
      new JSONSync<ScoringConstants>(
          new ScoringConstants(),
          "ScoringConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final Time timeToWaitForScore = Seconds.of(0.5);

  /**
   * How long can the indexer not move before it is determined that it must already have been at the
   * bottom.
   */
  public final Time homingMaxUnmovingTime = Seconds.of(0.3);
}
