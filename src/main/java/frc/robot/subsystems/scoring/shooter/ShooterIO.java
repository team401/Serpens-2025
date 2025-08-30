package frc.robot.subsystems.scoring.shooter;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngularAcceleration;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutTemperature;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.scoring.shooter.ShooterMechanism.ShooterSpeeds;
import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterInputs {
    public boolean leftMotorConnected = false;
    public MutAngularVelocity leftMotorVelocity = RotationsPerSecond.mutable(0.0);
    public MutAngularAcceleration leftMotorAcceleration = RotationsPerSecondPerSecond.mutable(0.0);
    public MutVoltage leftMotorAppliedVolts = Volts.mutable(0.0);
    public double leftMotorClosedLoopOutput = 0.0;
    public MutCurrent leftMotorSupplyCurrent = Amps.mutable(0.0);
    public MutCurrent leftMotorStatorCurrent = Amps.mutable(0.0);
    public MutTemperature leftMotorTemp = Celsius.mutable(0.0);

    public boolean rightMotorConnected = false;
    public MutAngularVelocity rightMotorVelocity = RotationsPerSecond.mutable(0.0);
    public MutAngularAcceleration rightMotorAcceleration = RotationsPerSecondPerSecond.mutable(0.0);
    public MutVoltage rightMotorAppliedVolts = Volts.mutable(0.0);
    public double rightMotorClosedLoopOutput = 0.0;
    public MutCurrent rightMotorSupplyCurrent = Amps.mutable(0.0);
    public MutCurrent rightMotorStatorCurrent = Amps.mutable(0.0);
    public MutTemperature rightMotorTemp = Celsius.mutable(0.0);
  }

  /**
   * Refresh and read all status signals from motors, updating a ShooterInputs object with new
   * values
   *
   * @param inputs The ShooterInputs object to update
   */
  public default void updateInputs(ShooterInputs inputs) {}

  /**
   * Run the Shooter flywheels with a certain torque current applied to each motor
   *
   * @param leftTorqueCurrent Torque current to apply to the left motor
   * @param rightTorqueCurrent Torque current to apply to the right motor
   */
  public default void runOpenLoop(Current leftTorqueCurrent, Current rightTorqueCurrent) {}

  /**
   * Run the Shooter flywheels with a certain voltage applied to each motor
   *
   * @param leftVoltage Voltage to apply to the left motor
   * @param rightVoltage Voltage to apply to the right motor
   */
  public default void runOpenLoop(Voltage leftVoltage, Voltage rightVoltage) {}

  /** Stop both Shooter flywheels */
  public default void stop() {}

  /**
   * Run the shooter flywheels at a certain set of speeds using Motion Magic Velocity
   * (TorqueCurrentFOC)
   *
   * @param speeds The ShooterSpeeds to target
   */
  public default void runSpeeds(ShooterSpeeds speeds) {}

  /**
   * Set the PID gains used for closed-loop control
   *
   * @param kP Proportional gain
   * @param kI Integrated gain
   * @param kD Derivative gain
   */
  public default void setPID(double kP, double kI, double kD) {}

  /**
   * Set the Feedforward gains used for closed-loop control
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
