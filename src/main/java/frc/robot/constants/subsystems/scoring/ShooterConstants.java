package frc.robot.constants.subsystems.scoring;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.units.measure.MomentOfInertia;

public final class ShooterConstants {
  @JSONExclude
  public static final JSONSync<ShooterConstants> synced =
      new JSONSync<ShooterConstants>(
          new ShooterConstants(),
          "ShooterConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final String CANBusName = "canivore";

  // TODO: Use JSONSync
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
          .withSlot0(
              new Slot0Configs()
                  .withKP(0.0) // TODO: Tune gains in sim and real life
                  .withKI(0.0)
                  .withKD(0.0)
                  .withKS(0.0)
                  .withKG(0.0)
                  .withKV(0.0)
                  .withKA(0.0))
          .withMotionMagic(
              new MotionMagicConfigs()
                  .withMotionMagicAcceleration(RotationsPerSecondPerSecond.of(10)));

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
  public final Double gearing = 22.0 / 60.0;

  public static class Sim {
    @JSONExclude
    public static final JSONSync<ShooterConstants.Sim> synced =
        new JSONSync<ShooterConstants.Sim>(
            new ShooterConstants.Sim(),
            "ShooterConstants.Sim.json",
            EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
            new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

    // 12.181 LbIn^2 = 0.00356 KgM^2
    public final MomentOfInertia momentOfInertia = KilogramSquareMeters.of(0.00356);
  }
}
