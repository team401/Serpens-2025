package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IntakeRollerIO {
  /**
   * A value to indicate to a shooterIO what side of the shooter it's on.
   *
   * <p>This is necessary because it allows the same IO to be reused between the left and right
   * sides and removes a ton of duplicate code.
   */
  @AutoLog
  public static class IntakeRollerInputs {
    public boolean motorConnected = false;
    public MutAngularVelocity motorVelocity = RotationsPerSecond.mutable(0.0);
    public MutAngularAcceleration motorAcceleration = RotationsPerSecondPerSecond.mutable(0.0);
    public MutVoltage motorAppliedVolts = Volts.mutable(0.0);
    public MutCurrent motorSupplyCurrent = Amps.mutable(0.0);
    public MutCurrent motorStatorCurrent = Amps.mutable(0.0);
  }

  /**
   * Refresh and read all status signals from motors, updating a ShooterInputs object with new
   * values
   *
   * @param inputs The ShooterInputs object to update
   */
  public default void updateInputs(IntakeRollerInputs inputs) {}

  /**
   * Run the Shooter flywheels with a certain torque current applied to each motor
   *
   * @param torqueCurrent Torque current to apply to the left motor
   */
  public default void runOpenLoop(Current torqueCurrent) {}

  /**
   * Run the roller flywheels with a certain voltage applied to each motor
   *
   * @param voltage Voltage to apply to the left motor
   */
  public default void runOpenLoop(Voltage voltage) {}

  /** Stop both roller flywheels */
  public default void stop() {}

  /**
   * Run the roller flywheel at a certain speed using Motion Magic Velocity (TorqueCurrentFOC)
   *
   * @param speed The AngularVelocity to target
   */
  public default void runSpeed(AngularVelocity speed) {}

  /**
   * Set the PID gains used for closed-loop control
   *
   * @param kP Proportional gain
   * @param kI Integrated gain
   * @param kD Derivative gain
   */
  public default void setPID(double kP, double kI, double kD) {}

  /**
   * Set the feed-forward gains used for closed-loop control
   *
   * <p>This method name includes SVA to indicate that it expects the arguments kS, then kV, then
   * kA, in that order
   *
   * @param kS Static friction gain: output needed to overcome static friction
   * @param kV Velocity gain: output required to hold a certain velocity
   * @param kA Acceleration gain: output required to induce a certain acceleration from the wheels
   */
  public default void setFFSVA(double kS, double kV, double kA) {}

  /**
   * Set whether the shooter rollers should coast or brake when no output is applied
   *
   * @param brakeEnabled True if brake mode should be enabled, false if motors should coast
   */
  public default void setBrakeMode(boolean brakeEnabled) {}
}
