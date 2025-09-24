package frc.robot.subsystems.scoring.indexer;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  enum IndexerOutputMode {
    ClosedLoop, // Not overriding, it should be closed loop
    Current, // Overriding, manually applying a current
    Voltage // Overriding, manually applying a voltage
  }

  @AutoLog
  public static class IndexerInputs {
    public boolean indexerMotorConnected = false;

    /** Stator current of the indexerMotor */
    public MutCurrent indexerMotorStatorCurrent = Amps.mutable(0.0);

    /** Supply current of the indexerMotor */
    public MutCurrent indexerMotorSupplyCurrent = Amps.mutable(0.0);

    public boolean indexerEncoderConnected = false;

    /**
     * Current position of the indexerEncoder. This measures total rotation since power on, not
     * absolute position
     */
    public MutAngle indexerEncoderPos = Rotations.mutable(0.0);

    /** Current velocity reported by the indexerEncoder */
    public MutAngularVelocity indexerEncoderVel = RotationsPerSecond.mutable(0.0);

    /** The current closed-loop goal position of the name, in terms of the indexerEncoder */
    public MutAngle indexerMotorGoalPos = Rotations.mutable(0.0);

    /** Profile setpoint goal position of the name, in terms of the indexerEncoder */
    public MutAngle indexerEncoderSetpointPos = Rotations.mutable(0.0);

    /**
     * Current closed-loop error (distance from setpoint position) as reported by the indexerMotor
     * TalonFX, in rotations.
     */
    public double motionMagicError = 0.0;

    /** Velocity of the Indexer mechanism, as reported by the indexerMotor TalonFX */
    public MutAngularVelocity indexerVelocity = RotationsPerSecond.mutable(0.0);
  }

  @AutoLog
  public static class IndexerOutputs {
    /** Are the motors currently disabled in software? */
    public boolean motorsDisabled = false;

    /** The current output mode of the Indexer */
    public IndexerOutputMode outputMode = IndexerOutputMode.ClosedLoop;

    /** The voltage currently applied to the motors */
    public MutVoltage indexerAppliedVolts = Volts.mutable(0.0);

    /** The current closed-loop output from Motion Magic */
    public double indexerClosedLoopOutput = 0.0;

    /** Contribution of the p-term to motor output */
    public MutVoltage pContrib = Volts.mutable(0.0);

    /** Contribution of the i-term to motor output */
    public MutVoltage iContrib = Volts.mutable(0.0);

    /** Contribution of the d-term to motor output */
    public MutVoltage dContrib = Volts.mutable(0.0);
  }

  /**
   * Updates an IndexerInputs with the current information from sensors readings and from the
   * motors.
   *
   * @param inputs IndexerInputs object to update with latest information
   */
  public default void updateInputs(IndexerInputs inputs) {}
  ;

  /**
   * Applies requests to motors and updates an IndexerOutputs object with information about motor
   * output.
   *
   * @param outputs IndexerOutputs object to update with latest applied outputs
   */
  public default void applyOutputs(IndexerOutputs outputs) {}
  ;

  /**
   * Set the goal position of indexerEncoder which the Indexer will control to when it is not in
   * override mode
   */
  public default void setIndexerMotorGoalAngle(Angle goalPos) {}

  /**
   * Set the position of the indexerEncoder. This position is separate from absolute position and
   * can track multiple rotations.
   */
  public default void setIndexerPosition(Angle newAngle) {}

  /**
   * Set the override voltage for the Indexer when in Voltage output mode
   *
   * @param volts The voltage to apply
   */
  public default void setOverrideVoltage(Voltage volts) {}

  /**
   * Set the static current (because of FOC) that will be applied when the Indexer is in Current
   * output mode.
   */
  public default void setOverrideCurrent(Current current) {}

  /**
   * Set whether the Indexer should use ClosedLoop control (default), voltage override, or current
   * override
   */
  public default void setOutputMode(IndexerOutputMode mode) {}

  /** Update PID gains for the Indexer */
  public default void setPID(double p, double i, double d) {}

  /** Set profile constraints to be sent to Motion Magic Expo */
  public default void setMaxProfile(
      AngularVelocity maxVelocity,
      Per<VoltageUnit, AngularAccelerationUnit> expo_kA,
      Per<VoltageUnit, AngularVelocityUnit> expo_kV) {}

  /** Set feedforward gains for closed-loop control */
  public default void setFF(double kS, double kV, double kA, double kG) {}

  /** Set whether or not the motors should brake while idle */
  public default void setBrakeMode(boolean brakeMode) {}

  /** Set the stator current limit for the Indexer motor */
  public default void setStatorCurrentLimit(Current currentLimit) {}

  /** Set whether or not the motor on the Indexer should be disabled. */
  public default void setMotorsDisabled(boolean disabled) {}
}
