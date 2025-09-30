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

  /**
   * The maximum amount of time that homing can take place before it is assumed that the indexer must be at the bottom.
   * 
   * <p>This value exists because, if the indexer is homing for a certain ridiculous period of time, the code should not freeze.
   * 
   * TODO: evaluate whether it would be better to disable motors in this case
   */
  public final Time homingMaxTime = Seconds.of(1.0);
}
