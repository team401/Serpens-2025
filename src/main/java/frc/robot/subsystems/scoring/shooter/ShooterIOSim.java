package frc.robot.subsystems.scoring.shooter;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.JsonConstants;

public class ShooterIOSim extends ShooterIOTalonFX {
  private TalonFXSimState leftMotorSimState;
  private TalonFXSimState rightMotorSimState;

  private FlywheelSim leftSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60Foc(1),
              JsonConstants.shooterConstantsSim.momentOfInertia.in(KilogramSquareMeters),
              JsonConstants.shooterConstants.gearing),
          DCMotor.getKrakenX60Foc(1));
  private FlywheelSim rightSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60Foc(1),
              JsonConstants.shooterConstantsSim.momentOfInertia.in(KilogramSquareMeters),
              JsonConstants.shooterConstants.gearing),
          DCMotor.getKrakenX60Foc(1));

  private Timer deltaTimer = new Timer();

  public ShooterIOSim() {
    leftMotorSimState = leftMotor.getSimState();
    rightMotorSimState = rightMotor.getSimState();

    deltaTimer.restart();
  }

  /**
   * Use 1 mutable angular velocity to do all calculations, preventing the creation & freeing of
   * multiple RotationsPerSecond objects per cycle
   */
  private MutAngularVelocity velocityCache = RotationsPerSecond.mutable(0.0);

  private MutAngularAcceleration accelCache = RadiansPerSecondPerSecond.mutable(0.0);

  private void updateSimState() {
    double deltaTime = deltaTimer.get();
    deltaTimer.restart();

    // Update left sim
    var leftMotorVoltage = leftMotorSimState.getMotorVoltage();
    leftSim.setInput(leftMotorVoltage);
    leftSim.update(deltaTime);

    // Gearing = Output : Input
    // Flywheel Speed = Motor Speed * Gearing, therefore:
    // Motor Speed = Flywheel Speed / Gearing
    velocityCache.mut_replace(
        leftSim.getAngularVelocityRadPerSec() / JsonConstants.shooterConstants.gearing,
        RadiansPerSecond);
    leftMotorSimState.setRotorVelocity(velocityCache);

    accelCache.mut_replace(
        leftSim.getAngularAccelerationRadPerSecSq(),
        RadiansPerSecondPerSecond); // Get the value as a double instead of as an
    // AngularAcceleration to avoid creating a new measure every
    // cycle
    leftMotorSimState.setRotorAcceleration(accelCache);

    // Update right sim
    var rightMotorVoltage = rightMotorSimState.getMotorVoltage();
    rightSim.setInput(rightMotorVoltage);
    rightSim.update(deltaTime);

    // Gearing = Output : Input
    // Flywheel Speed = Motor Speed * Gearing, therefore:
    // Motor Speed = Flywheel Speed / Gearing
    velocityCache.mut_replace(
        rightSim.getAngularVelocityRadPerSec() / JsonConstants.shooterConstants.gearing,
        RadiansPerSecond);
    rightMotorSimState.setRotorVelocity(velocityCache);

    accelCache.mut_replace(
        rightSim.getAngularAccelerationRadPerSecSq(),
        RadiansPerSecondPerSecond); // Get the value as a double instead of as an
    // AngularAcceleration to avoid creating a new measure every
    // cycle
    rightMotorSimState.setRotorAcceleration(accelCache);
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    updateSimState();

    super.updateInputs(inputs);
  }
}
