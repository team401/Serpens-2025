package frc.robot.constants.subsystems.intake; // NOTE: This should be changed if you keep your constants in a
// separate package from your code

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

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

public final class IntakeConstants {
  @JSONExclude
  public static final JSONSync<IntakeConstants> synced =
      new JSONSync<IntakeConstants>(
          new IntakeConstants(),
          "IntakeConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final Integer intakeArmMotorId = 1; // TODO: Replace placeholder CAN ID
  public final Integer intakeRollerMotorId = 2; // TODO: Replace placeholder CAN ID

  /**
   * What point in the sensor's range the discontinuity occurs. Results in a range of [1-x, x). For
   * example, a value of 1 gives a range of [0.0, 1).
   */
  public final Double intakeArmEncoderDiscontinuityPoint = 1.0;

  public final Angle intakeArmEncoderMagnetOffset = Radians.of(0.0);

  public final Integer intakeArmEncoderID = 2; // TODO: Replace placeholder CAN ID

  public final SensorDirectionValue intakeArmEncoderDirection =
      SensorDirectionValue.Clockwise_Positive;

  /*
   * The intakeArmEncoder is represented as the mechanism in our Phoenix configs.
   * This means that we are controlling to a goal in terms of large CANCoder angle.
   */
  @JSONExclude public final double intakeArmEncoderToMechanismRatio = 1.0;

  @JSONExclude
  public final double rotorToIntakeArmEncoderRatio = 1.0; // TODO: Replace placeholder value

  public final Double intakeArmKP = 0.0;
  public final Double intakeArmKI = 0.0;
  public final Double intakeArmKD = 0.0;

  public final Double intakeArmKS = 0.0;
  public final Double intakeArmKV = 0.0;
  public final Double intakeArmKA = 0.0;
  public final Double intakeArmKG = 0.0;

  /** This is a Double until coppercore JSONSync supports RotationsPerSecond */
  public final Double intakeArmAngularCruiseVelocityRotationsPerSecond = 1.0;

  /*
   * The Motion Magic Expo kV, measured in Volts per Radian per Second, but represented as a double so it can be synced by JSONSync
   *
   * <p> This kV is used by Motion Magic Expo to generate a motion profile. Dividing the supply voltage by
   * kV results in the maximum velocity of the system. Therefore, a higher profile kV results in a
   * lower profile velocity.
   */
  public final Double intakeArmMotionMagicExpo_kV = 0.0;

  /*
   * The Motion Magic Expo kA, measured in Volts per Radian per Second Squared, but represented as a double so it can be synced by JSONSync
   */
  public final Double intakeArmMotionMagicExpo_kA = 0.0;

  public final Current intakeArmStatorCurrentLimit =
      Amps.of(80.0); // TODO: Replace placeholder current limit

  public final Double intakeArmReduction = 1.0; // TODO: Replace placeholder reduction

  public final Angle intakeArmMinMinAngle =
      Rotations.of(0.0); // TODO: Replace placeholder constraints
  public final Angle intakeArmMaxMaxAngle = Rotations.of(1.0);

  public static final class Sim {
    @JSONExclude
    public static final JSONSync<IntakeConstants.Sim> synced =
        new JSONSync<IntakeConstants.Sim>(
            new IntakeConstants.Sim(),
            Filesystem.getDeployDirectory()
                .toPath()
                .resolve("constants/IntakeArmConstants.Sim.json")
                .toString(),
            new JSONSyncConfigBuilder().build());

    /** Standard deviation passed to sim for the position measurement */
    public final Double positionStdDev = 0.0;

    /** Standard deviation passed to sim for the velocity measurement */
    public final Double velocityStdDev = 0.0;

    @JSONExclude
    public final MomentOfInertia intakeArmMomentOfInertia =
        KilogramSquareMeters.of(0.05); // TODO: Replace placeholder moment of inertia

    public final Distance intakeArmArmLength =
        Meters.of(1.0); // TODO: Replace placeholder arm length
    public final Angle intakeArmMinAngle =
        Radians.of(0.0); // TODO: Update placeholder min & max angles
    public final Angle intakeArmMaxAngle = Radians.of(1.0);

    public final Angle intakeArmStartingAngle = Radians.of(0.0);
  }
}
