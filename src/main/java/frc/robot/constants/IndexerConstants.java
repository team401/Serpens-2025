package frc.robot.constants;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.Filesystem;

public class IndexerConstants {

  @JSONExclude
  public static final JSONSync<IndexerConstants> synced =
      new JSONSync<IndexerConstants>(
          new IndexerConstants(),
          "IndexerConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().build());

  public final Integer indexerMotorId = 0;
  public final Integer indexerCANcoderId = 0;

  public final Double sensorToMechanismRatio = 0.0;

  public final Double indexerReduction = 0.0;

  @JSONExclude public final Double rotorToSensorRatio = indexerReduction;

  public final InvertedValue indexerMotorInvertedValue = InvertedValue.Clockwise_Positive;

  public final NeutralModeValue indexerNeutralModeValue = NeutralModeValue.Brake;

  public final Current indexerSupplyCurrentLimit = Amps.of(0.0);
  public final Current indexerStatorCurrentLimit = Amps.of(0.0);

  public final Current peakFOCCurrent = Amps.of(0.0);

  public final Double indexerKG = 0.0;
  public final Double indexerKS = 0.0;
  public final Double indexerKV = 0.0;
  public final Double indexerKA = 0.0;

  public final Double indexerKP = 0.0;
  public final Double indexerKI = 0.0;
  public final Double indexerKD = 0.0;

  public final Double indexerMotionMagicCruiseVelocityRotationsPerSecond = 0.0;

  public final Double indexerMotionMagicExpo_kA = 0.0;
  public final Double indexerMotionMagicExpo_kV = 0.0;

  public final Angle indexerCANcoderAbsoluteSensorDiscontinuityPoint = Rotations.of(0.0);
  public final Angle indexerCANcoderMagnetOffset = Rotations.of(0.0);
  public final SensorDirectionValue indexerCANcoderSensorDirection =
      SensorDirectionValue.Clockwise_Positive;

  // These clamps are the default clamps for the indexer, as well as limiting the moving clamps of
  // the
  // indexer themselves.
  public final Angle indexerMinMinAngle = Radians.of(0);
  public final Angle indexerMaxMaxAngle = Radians.of(0);

  /**
   * The minimum angle the indexer can be at while the elevator is down and not hit the parts of the
   * robot below.
   */
  public final Angle minElevatorDownSafeAngle = Radians.of(0);

  /**
   * The minimum angle the indexer can be at without hitting the reef when very close to the reef
   */
  public final Angle minReefSafeAngle = Rotations.of(0.0);

  /**
   * When less than this distance from the center of the reef, the claw can collide with it the reef
   */
  public final Distance closeToReefThreshold = Meters.of(0);

  /** The indexer can be this far away from the goal and considered "at the setpoint" */
  public final Angle indexerSetpointEpsilon = Degrees.of(0.0);

  /**
   * How slow the indexer must be moving before it is considered to be stable at its goal position
   *
   * <p>This value is a Double because it can't be serialized with JSONSync
   */
  public final Double maxIndexerSetpointVelocityRotationsPerSecond = 0.0;

  public final Double indexerStableDebounceTimeSeconds = 0.0;

  public final Angle algaeUnderCrossbarAngle = Radians.of(0);

  /** The angle at which the indexer will collide with the crossbar when rotating downward */
  public final Angle crossbarTopCollisionAngle = Rotations.of(0);

  /** The angle at which the indexer will collide with the crossbar when rotating upward */
  public final Angle crossbarBottomCollisionAngle = Rotations.of(0);

  /** How close the indexer must be to its goal to spin the rollers in net shot */
  public final Angle netShotRollerIndexerEpsilon = Rotations.of(0.0);

  public final Angle maxReefBaseIndexerDownCollisionAngle = Rotations.of(0.0);

  // These should be unit-safe, not Object

  public final Angle indexerMaxAngle = Radians.of(0);
  public MomentOfInertia indexerMomentOfInertia = KilogramSquareMeters.of(0.0);
  public Distance indexerArmLength = Meters.of(0.0);
  public Angle indexerMinAngle = Radians.of(0.0);
  public Angle indexerStartingAngle = Rotations.of(0.0);

  public static final class Sim {
    @JSONExclude
    public static final JSONSync<IndexerConstants.Sim> synced =
        new JSONSync<IndexerConstants.Sim>(
            new IndexerConstants.Sim(),
            Filesystem.getDeployDirectory() // Don't use environment handler for sim constants
                .toPath()
                .resolve("constants/IndexerConstants.Sim.json")
                .toString(),
            new JSONSyncConfigBuilder().build());

    // This value is a Double because MomentOfInertia units don't serialize properly with JSONSync
    public final Double indexerMomentOfInertiaKgM2 = 0.0;

    @JSONExclude
    public final MomentOfInertia indexerMomentOfInertia =
        KilogramSquareMeters.of(indexerMomentOfInertiaKgM2);

    public final Distance indexerArmLength = Meters.of(0.0);

    public final Angle indexerMinAngle = Radians.of(0);
    public final Angle indexerMaxAngle = Radians.of(0);

    public final Angle indexerStartingAngle = Rotations.of(0.0);
  }
}
