package frc.robot.subsystems.scoring.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.CustomUnits.RotationsPerMinute;

import coppercore.parameter_tools.LoggedTunableNumber;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.shooter.ShooterIO.ShooterInputs;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class ShooterMechanism {
  public record ShooterSpeeds(AngularVelocity leftSpeed, AngularVelocity rightSpeed) {}

  private static final ShooterSpeeds ZERO_SPEEDS =
      new ShooterSpeeds(RotationsPerSecond.zero(), RotationsPerSecond.zero());

  private final ShooterIO io;
  private ShooterInputsAutoLogged inputs = new ShooterInputsAutoLogged();

  /** The last set shooter speeds */
  @AutoLogOutput(key = "scoring/shooter/goalSpeeds")
  private ShooterSpeeds goalSpeeds = ZERO_SPEEDS;

  private enum ShooterOutputMode {
    ClosedLoop,
    Voltage,
    Current,
    Stop,
  }

  @AutoLogOutput(key = "scoring/shooter/outputMode")
  private ShooterOutputMode outputMode = ShooterOutputMode.ClosedLoop;

  // Tunables for Test Mode
  // Tunable gains
  private LoggedTunableNumber shooterKP =
      new LoggedTunableNumber(
          "ShooterTunables/KP", JsonConstants.shooterConstants.baseTalonFXConfigs.Slot0.kP);
  private LoggedTunableNumber shooterKI =
      new LoggedTunableNumber(
          "ShooterTunables/KI", JsonConstants.shooterConstants.baseTalonFXConfigs.Slot0.kI);
  private LoggedTunableNumber shooterKD =
      new LoggedTunableNumber(
          "ShooterTunables/KD", JsonConstants.shooterConstants.baseTalonFXConfigs.Slot0.kD);

  private LoggedTunableNumber shooterKS =
      new LoggedTunableNumber(
          "ShooterTunables/KS", JsonConstants.shooterConstants.baseTalonFXConfigs.Slot0.kS);
  private LoggedTunableNumber shooterKV =
      new LoggedTunableNumber(
          "ShooterTunables/KV", JsonConstants.shooterConstants.baseTalonFXConfigs.Slot0.kV);
  private LoggedTunableNumber shooterKA =
      new LoggedTunableNumber(
          "ShooterTunables/KA", JsonConstants.shooterConstants.baseTalonFXConfigs.Slot0.kA);

  // Tunable outputs
  private LoggedTunableNumber shooterLeftManualVolts =
      new LoggedTunableNumber("ShooterTunables/LeftManualVolts", 0.0);
  private LoggedTunableNumber shooterRightManualVolts =
      new LoggedTunableNumber("ShooterTunables/RightManualVolts", 0.0);

  private LoggedTunableNumber shooterLeftManualAmps =
      new LoggedTunableNumber("ShooterTunables/LeftManualAmps", 0.0);
  private LoggedTunableNumber shooterRightManualAmps =
      new LoggedTunableNumber("ShooterTunables/RightManualAmps", 0.0);

  private LoggedTunableNumber shooterLeftTargetRPM =
      new LoggedTunableNumber("ShooterTunables/LeftTargetRPM", 0.0);
  private LoggedTunableNumber shooterRightTargetRPM =
      new LoggedTunableNumber("ShooterTunables/RightTargetRPM", 0.0);

  public ShooterMechanism(ShooterIO shooterIO) {
    this.io = shooterIO;
  }

  /**
   * This method should be called in each periodic loop by the ScoringSubsystem. It will NOT run
   * automatically.
   */
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("scoring/shooter/inputs", inputs);
  }

  /**
   * This method should be called in each periodic loop by the ScoringSubsystem when the robot is in
   * test mode. It will NOT run automatically.
   */
  public void testPeriodic() {
    switch (TestModeManager.getTestMode()) {
      case ShooterCurrentTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (currents) -> {
              io.runOpenLoop(Amps.of(currents[0]), Amps.of(currents[1]));
              outputMode = ShooterOutputMode.Current;
            },
            shooterLeftManualAmps,
            shooterRightManualAmps);
      }

      case ShooterVoltageTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (voltages) -> {
              io.runOpenLoop(Volts.of(voltages[0]), Volts.of(voltages[1]));
              outputMode = ShooterOutputMode.Voltage;
            },
            shooterLeftManualVolts,
            shooterRightManualVolts);
      }

      case ShooterClosedLoopTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid) -> {
              io.setPID(pid[0], pid[1], pid[2]);
            },
            shooterKP,
            shooterKI,
            shooterKD);
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (ff) -> {
              io.setFFSVA(ff[0], ff[1], ff[2]);
            },
            shooterKS,
            shooterKV,
            shooterKA);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (speeds) -> {
              runSpeeds(
                  new ShooterSpeeds(
                      RotationsPerMinute.of(speeds[0]), RotationsPerMinute.of(speeds[1])));
            },
            shooterLeftTargetRPM,
            shooterRightTargetRPM);
      }

      default -> {}
    }
  }

  /**
   * Run the shooter wheels at a certain set of speeds.
   *
   * <p>This also updates the goal speeds of the shooter, for reference in {@link
   * ShooterMechanism#shooterReady()}
   *
   * @param speeds The set of speeds to run the shooter at
   */
  public void runSpeeds(ShooterSpeeds speeds) {
    io.runSpeeds(speeds);
    outputMode = ShooterOutputMode.ClosedLoop;
  }

  /** Stop the shooter wheels, setting their goal speeds to zero */
  public void stop() {
    io.stop();
    outputMode = ShooterOutputMode.Stop;
  }

  /**
   * Checks whether the shooter currently within the error margin of its goal speeds.
   *
   * @return Whether the shooter is currently within the error margin of its goal speeds
   */
  public boolean shooterReady() {
    if (outputMode != ShooterOutputMode.ClosedLoop) {
      return true;
    }

    boolean leftReady =
        goalSpeeds.leftSpeed.isNear(
            inputs.leftMotorVelocity,
            JsonConstants.shooterConstants.shooterVelocityEpsilonFraction);
    boolean rightReady =
        goalSpeeds.rightSpeed.isNear(
            inputs.rightMotorVelocity,
            JsonConstants.shooterConstants.shooterVelocityEpsilonFraction);

    Logger.recordOutput("scoring/shooter/leftReady", leftReady);
    Logger.recordOutput("scoring/shooter/rightReady", rightReady);

    boolean shooterReady = leftReady && rightReady;

    Logger.recordOutput("scoring/shooter/shooterReady", shooterReady);

    return shooterReady;
  }

  public final ShooterInputs getInputs() {
    return inputs;
  }
}
