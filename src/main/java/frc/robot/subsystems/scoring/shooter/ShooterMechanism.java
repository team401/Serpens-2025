package frc.robot.subsystems.scoring.shooter;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.shooter.ShooterIO.ShooterInputs;

public class ShooterMechanism {
  public record ShooterSpeeds(AngularVelocity leftSpeed, AngularVelocity rightSpeed) {}

  private static final ShooterSpeeds ZERO_SPEEDS =
      new ShooterSpeeds(RotationsPerSecond.zero(), RotationsPerSecond.zero());

  private final ShooterIO io;
  private ShooterInputsAutoLogged inputs;

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

  public ShooterMechanism(ShooterIO shooterIO) {
    this.io = shooterIO;
  }

  /**
   * This method should be called in each periodic loop by the ScoringSubsystem. It will NOT run
   * automatically.
   */
  public void periodic() {
    // TODO: Implement periodic
    Logger.processInputs("scoring/shooter/inputs", inputs);
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

    boolean leftReady = goalSpeeds.leftSpeed.isNear(inputs.leftMotorVelocity, JsonConstants.shooterConstants.shooterVelocityEpsilonFraction);
    boolean rightReady = goalSpeeds.rightSpeed.isNear(inputs.rightMotorVelocity, JsonConstants.shooterConstants.shooterVelocityEpsilonFraction);

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
