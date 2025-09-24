package frc.robot.subsystems.scoring.indexer; // NOTE: This should be changed if you keep your
// constants in a separate package from your code

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.Filesystem;

public final class IndexerConstants {
  @JSONExclude
  public static final JSONSync<IndexerConstants> synced =
      new JSONSync<IndexerConstants>(
          new IndexerConstants(),
          "IndexerConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final Double indexerKP = 0.0;
  public final Double indexerKI = 0.0;
  public final Double indexerKD = 0.0;

  public final Double indexerKS = 0.0;
  public final Double indexerKV = 0.0;
  public final Double indexerKA = 0.0;
  public final Double indexerKG = 0.0;

  /** This is a Double until coppercore JSONSync supports RotationsPerSecond */
  public final Double indexerAngularCruiseVelocityRotationsPerSecond = 1.0;

  /*
   * The Motion Magic Expo kV, measured in Volts per Radian per Second, but represented as a double so it can be synced by JSONSync
   *
   * <p> This kV is used by Motion Magic Expo to generate a motion profile. Dividing the supply voltage by
   * kV results in the maximum velocity of the system. Therefore, a higher profile kV results in a
   * lower profile velocity.
   */
  public final Double indexerMotionMagicExpo_kV = 0.0;

  /*
   * The Motion Magic Expo kA, measured in Volts per Radian per Second Squared, but represented as a double so it can be synced by JSONSync
   */
  public final Double indexerMotionMagicExpo_kA = 0.0;

  public final Current indexerStatorCurrentLimit =
      Amps.of(80.0); // TODO: Replace placeholder current limit

  public final Double indexerReduction = 1.0; // TODO: Replace placeholder reduction

  public final Angle indexerMinMinAngle =
      Rotations.of(0.0); // TODO: Replace placeholder constraints
  public final Angle indexerMaxMaxAngle = Rotations.of(1.0);

  public static final class Sim {
    @JSONExclude
    public static final JSONSync<IndexerConstants.Sim> synced =
        new JSONSync<IndexerConstants.Sim>(
            new IndexerConstants.Sim(),
            Filesystem.getDeployDirectory()
                .toPath()
                .resolve("constants/IndexerConstants.Sim.json")
                .toString(),
            new JSONSyncConfigBuilder().build());

    /** Standard deviation passed to sim for the position measurement */
    public final Double positionStdDev = 0.0;

    /** Standard deviation passed to sim for the velocity measurement */
    public final Double velocityStdDev = 0.0;

    @JSONExclude
    public final MomentOfInertia indexerMomentOfInertia =
        KilogramSquareMeters.of(0.05); // TODO: Replace placeholder moment of inertia

    public final Distance indexerArmLength = Meters.of(1.0); // TODO: Replace placeholder arm length
    public final Angle indexerMinAngle =
        Radians.of(0.0); // TODO: Update placeholder min & max angles
    public final Angle indexerMaxAngle = Rotations.of(0.5);

    public final Angle indexerStartingAngle = Radians.of(0.0);
  }
}
