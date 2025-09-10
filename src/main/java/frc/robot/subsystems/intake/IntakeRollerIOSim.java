package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecondPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.JsonConstants;

public class IntakeRollerIOSim extends IntakeRollerIOTalonFX {
  private TalonFXSimState motorSimState;

  private FlywheelSim leftSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60Foc(1),
              JsonConstants.intakeConstantsSim.intakeRollerMomentOfInertia.in(KilogramSquareMeters),
              JsonConstants.intakeConstants.gearing),
          DCMotor.getKrakenX60Foc(1));

  private Timer deltaTimer = new Timer();

  public IntakeRollerIOSim() {

    motorSimState = motor.getSimState();

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

    // Update sim
    var leftMotorVoltage = motorSimState.getMotorVoltage();
    leftSim.setInput(leftMotorVoltage);
    leftSim.update(deltaTime);

    // Gearing = Output : Input
    // Flywheel Speed = Motor Speed * Gearing, therefore:
    // Motor Speed = Flywheel Speed / Gearing
    velocityCache.mut_replace(
        leftSim.getAngularVelocityRadPerSec() / JsonConstants.intakeConstants.gearing,
        RadiansPerSecond);

    // Get the value as a double instead of as an
    // AngularAcceleration to avoid creating a new measure every
    // cycle
    accelCache.mut_replace(
        leftSim.getAngularAccelerationRadPerSecSq() / JsonConstants.intakeConstants.gearing,
        RadiansPerSecondPerSecond);

    if (JsonConstants.intakeConstants.rollerInverted == InvertedValue.Clockwise_Positive) {
      velocityCache.mut_times(-1.0);
      accelCache.mut_times(-1.0);
    }

    motorSimState.setRotorVelocity(velocityCache.in(RotationsPerSecond));
    motorSimState.setRotorAcceleration(accelCache.in(RotationsPerSecondPerSecond));
  }

  @Override
  public void updateInputs(IntakeRollerInputs inputs) {
    updateSimState();

    super.updateInputs(inputs);
  }
}
