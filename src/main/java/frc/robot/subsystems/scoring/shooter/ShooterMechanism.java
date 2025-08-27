package frc.robot.subsystems.scoring.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.units.measure.AngularVelocity;
import org.littletonrobotics.junction.Logger;

public class ShooterMechanism {
  public record ShooterSpeeds(AngularVelocity leftSpeed, AngularVelocity rightSpeed) {}

  private static final ShooterSpeeds ZERO_SPEEDS =
      new ShooterSpeeds(RotationsPerSecond.zero(), RotationsPerSecond.zero());

  private final ShooterIO shooterIO;
  private ShooterInputsAutoLogged shooterInputs;

  public ShooterMechanism(ShooterIO shooterIO) {
    this.shooterIO = shooterIO;
  }

  /**
   * This method should be called in each periodic loop by the ScoringSubsystem. It will NOT run
   * automatically.
   */
  public void periodic() {
    // TODO: Implement periodic
    Logger.processInputs("scoring/shooter/inputs", shooterInputs);
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
    // TODO: Implement shooter warmup
  }

  /** Stop the shooter wheels, setting their goal speeds to zero */
  public void stop() {
    // TODO: Implement stopping
    runSpeeds(ZERO_SPEEDS);
  }

  /**
   * Checks whether the shooter currently within the error margin of its goal speeds.
   *
   * @return Whether the shooter is currently within the error margin of its goal speeds
   */
  public boolean shooterReady() {
    // TODO: Implement checking if the shooter is ready
    return true;
  }
}
