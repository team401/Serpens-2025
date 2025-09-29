package frc.robot.constants;

import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Field Locations for various useful positions to our robot's functioning when on the Red alliance
 *
 * <p>Unless otherwise specified, all values are in meters according to the field coordinate system.
 *
 * <p>Values like these can be determined by moving robot poses to the desired place in PathPlanner
 * and then copying the coordinates out of the PathPlanner UI and into these constants.
 */
public class RedFieldLocations {
  @JSONExclude
  public static final JSONSync<RedFieldLocations> synced =
      new JSONSync<RedFieldLocations>(
          new RedFieldLocations(),
          "RedFieldLocations.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  /**
   * A pair of Translation2ds representing the start and endpoint of the line segment of possible
   * targets for pose-based shooting.
   *
   * <p>These values are determined using PathPlanner to manually pick poses and then copying down
   * their coordinates.
   *
   * <p>These values may have to be tuned to prevent the robot from shooting too close to the ends
   * of the barge.
   */
  public final Pair<Translation2d, Translation2d> bargeLine =
      new Pair<Translation2d, Translation2d>(
          new Translation2d(8.758, 0.190), new Translation2d(8.758, 3.577));
}
