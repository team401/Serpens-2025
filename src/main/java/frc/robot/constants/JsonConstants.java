package frc.robot.constants;

import coppercore.parameter_tools.json.JSONConverter;
import coppercore.parameter_tools.json.adapters.measure.JSONMeasure;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.subsystems.scoring.ScoringFeatureFlags;
import frc.robot.constants.subsystems.scoring.ShooterConstants;

public class JsonConstants {
  public static void loadConstants() {
    EnvironmentHandler.getEnvironmentHandler(
        Filesystem.getDeployDirectory().toPath().resolve("constants/config.json").toString());

    // Add type adapters for AngularVelocity and Frequency, since these don't exist in coppercore
    // yet.
    JSONConverter.jsonMap.put(AngularVelocity.class, JSONMeasure.class);
    JSONConverter.jsonMap.put(Frequency.class, JSONMeasure.class);

    FeatureFlags.synced.loadData();
    CANConstants.synced.loadData();
    RedFieldLocations.synced.loadData();
    BlueFieldLocations.synced.loadData();
    ScoringFeatureFlags.synced.loadData();
    ShooterConstants.synced.loadData();
    ShooterConstants.Sim.synced.loadData();

    featureFlags = FeatureFlags.synced.getObject();
    canConstants = CANConstants.synced.getObject();
    redFieldLocations = RedFieldLocations.synced.getObject();
    blueFieldLocations = BlueFieldLocations.synced.getObject();
    scoringFeatureFlags = ScoringFeatureFlags.synced.getObject();
    shooterConstants = ShooterConstants.synced.getObject();
    shooterConstantsSim = ShooterConstants.Sim.synced.getObject();

    shooterConstants.initializeDistanceToRPMMaps();
  }

  public static FeatureFlags featureFlags;
  public static CANConstants canConstants;
  public static RedFieldLocations redFieldLocations;
  public static BlueFieldLocations blueFieldLocations;
  public static ScoringFeatureFlags scoringFeatureFlags;
  public static ShooterConstants shooterConstants;
  public static ShooterConstants.Sim shooterConstantsSim;
}
