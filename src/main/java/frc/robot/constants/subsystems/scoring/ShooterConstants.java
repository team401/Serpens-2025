package frc.robot.constants.subsystems.scoring;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.ClosedLoopGeneralConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.units.measure.MomentOfInertia;
import frc.robot.subsystems.scoring.shooter.ShooterMechanism.ShooterSpeeds;

public final class ShooterConstants {
  @JSONExclude
  public static final JSONSync<ShooterConstants> synced =
      new JSONSync<ShooterConstants>(
          new ShooterConstants(),
          "ShooterConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final String CANBusName = "canivore";

  /**
   * What fraction of the total angular velocity must the shooter velocity be within in order for
   * the shooter to be considered "ready?"
   *
   * <p>For example, if this value is 0.05, the shooter wheels must be within +/- 5% of the goal
   * speed to shoot.
   */
  public final Double shooterVelocityEpsilonFraction = 0.05;

  /**
   * Base TalonFX configs that will be modified by ShooterIOTalonFX before being applied to the
   * motors
   *
   * <p>Fields that will be updated outside of this object:
   *
   * <ul>
   *   <li>PID & Feed Forward Gains - Updated in tuning modes
   *   <li>Motor inverts - Read at load time and updated in config right before being applied in IO
   * </ul>
   */
  public final TalonFXConfiguration baseTalonFXConfigs =
      new TalonFXConfiguration()
          .withCurrentLimits(
              new CurrentLimitsConfigs()
                  .withSupplyCurrentLimit(Amps.of(40.0))
                  .withSupplyCurrentLimitEnable(true)
                  .withStatorCurrentLimit(Amps.of(40.0))
                  .withStatorCurrentLimitEnable(true))
          .withClosedLoopGeneral(new ClosedLoopGeneralConfigs().withContinuousWrap(true))
          .withSlot0(
              new Slot0Configs()
                  .withKP(100.0) // TODO: Tune gains in real life
                  .withKI(0.0)
                  .withKD(0.0)
                  .withKS(0.0)
                  .withKG(0.0)
                  .withKV(0.01)
                  .withKA(10.0))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(80)));

  public final InvertedValue leftMotorInverted = InvertedValue.Clockwise_Positive;
  public final InvertedValue rightMotorInverted = InvertedValue.CounterClockwise_Positive;

  /** Maximum number of times to try re-applying TalonFX configs if applying configs fails */
  public final Integer maxConfigApplyAttempts = 5;

  /** How long to wait on each attempt to apply configs before timing out */
  public final Double configApplyTimeoutSeconds = 0.25;

  /**
   * The reduction of rotor to drum, as a ratio of output to input
   *
   * <p>To get a ratio of output : input, we take input gear teeth : output gear teeth
   */
  public final Double gearing = 1.0;
  //   public final Double gearing = 22.0 / 60.0;
  // As of 2025-9-13T13:38-4:00, I have isolated the sim issues to gearing-related problems
  // Therefore, the gearing is set to 1.0 to eliminate variables. After more experimentation is
  // done, and after Design releases the final gear ratio, this constant can be updated and the sim
  // can be re-tuned.

  // JSON Excluded for now since JSONSync doesn't support RPM yet
  @JSONExclude
  public final ShooterSpeeds defaultShot =
      new ShooterSpeeds(RPM.of(100), RPM.of(50)); // TODO: Tune this in real life!

  /** The set of distances for the mapping of distance to shooter speeds */
  public final double[] shooterMapDistances = {2.0, 10.0};
  /**
   * The set of speeds for the close-side of the shooter in the mapping of distance to shooter
   * speeds
   *
   * <p>For example, if the robot is shooting left, this will become the left speed
   */
  public final double[] shooterMapCloseSpeedsRPM = {100.0, 2000.0};
  /**
   * The set of speeds for the far-side of the shooter in the mapping of distance to shooter speeds
   *
   * <p>For example, if the robot is shooting left, this will become the right speed
   */
  public final double[] shooterMapFarSpeedsRPM = {150.0, 3000.0};

  /**
   * Mapping of shot distance to RPM of the close motor.
   *
   * <p>`initializeSpeedMaps()` MUST be called AFTER `synced.loadData()` but BEFORE this map is
   * read.
   */
  @JSONExclude
  public final InterpolatingDoubleTreeMap distanceToCloseRPM = new InterpolatingDoubleTreeMap();

  /**
   * The furthest distance in the shooter map.
   *
   * <p>`initializeSpeedMaps()` MUST be called AFTER `synced.loadData()` but BEFORE this value is
   * read.
   */
  @JSONExclude public Double maxShotDistance = 0.0;

  /**
   * The closest that the robot may be to the barge line while still shooting.
   *
   * <p>This value should be tuned such that the robot won't shoot straight up and hit the underside
   * of the barge.
   */
  public final Double minShotDistance = 1.0;

  /**
   * Mapping of shot distance to RPM of the far motor.
   *
   * <p>`initializeSpeedMaps()` MUST be called AFTER `synced.loadData()` but BEFORE this map is
   * read.
   */
  @JSONExclude
  public final InterpolatingDoubleTreeMap distanceToFarRPM = new InterpolatingDoubleTreeMap();

  /**
   * Propagate the distanceToCloseRPM and distanceToFarRPM maps with the values from the double
   * arrays loaded from JSON.
   *
   * <p>This method MUST be called AFTER `synced.loadData()` but BEFORE the maps or max distance are
   * read.
   */
  public void initializeSpeedMaps() {
    if (shooterMapDistances.length != shooterMapCloseSpeedsRPM.length
        || shooterMapDistances.length != shooterMapFarSpeedsRPM.length) {
      throw new Error("Shooter map arrays had differing lengths");
    }

    for (int i = 0; i < shooterMapDistances.length; i++) {
      double distance = shooterMapDistances[i];
      double closeRPM = shooterMapCloseSpeedsRPM[i];
      double farRPM = shooterMapFarSpeedsRPM[i];

      distanceToCloseRPM.put(distance, closeRPM);
      distanceToFarRPM.put(distance, farRPM);

      if (distance > maxShotDistance) {
        maxShotDistance = distance;
      }
    }
  }

  public static class Sim {
    @JSONExclude
    public static final JSONSync<ShooterConstants.Sim> synced =
        new JSONSync<ShooterConstants.Sim>(
            new ShooterConstants.Sim(),
            "ShooterConstants.Sim.json",
            EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
            new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

    // 12.181 LbIn^2 = 0.00356 KgM^2
    @JSONExclude public final MomentOfInertia momentOfInertia = KilogramSquareMeters.of(0.00356);
  }
}
