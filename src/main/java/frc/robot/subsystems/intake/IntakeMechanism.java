package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.UnitUtils;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import frc.robot.constants.subsystems.intake.IntakeConstants;
import frc.robot.subsystems.intake.IntakeArmIO.IntakeArmOutputMode;
import org.littletonrobotics.junction.Logger;

/**
 * A Mechanism to manage the IntakeArm
 *
 * <ul>
 *   <li>Uses closed-loop TorqueCurrentFOC control
 */
public class IntakeMechanism {
  IntakeArmIO io;
  IntakeArmInputsAutoLogged inputs = new IntakeArmInputsAutoLogged();
  IntakeArmOutputsAutoLogged outputs = new IntakeArmOutputsAutoLogged();

  MutAngle goalAngle = Rotations.mutable(0.0);
  MutAngle clampedGoalAngle = Rotations.mutable(0.0);

  MutAngle minAngle = IntakeConstants.synced.getObject().intakeArmMinMinAngle.mutableCopy();
  MutAngle maxAngle = IntakeConstants.synced.getObject().intakeArmMaxMaxAngle.mutableCopy();

  LoggedTunableNumber intakeArmkP;
  LoggedTunableNumber intakeArmkI;
  LoggedTunableNumber intakeArmkD;

  LoggedTunableNumber intakeArmkS;
  LoggedTunableNumber intakeArmkV;
  LoggedTunableNumber intakeArmkA;
  LoggedTunableNumber intakeArmkG;

  LoggedTunableNumber intakeArmCruiseVelocity;
  LoggedTunableNumber intakeArmExpokV;
  LoggedTunableNumber intakeArmExpokA;

  LoggedTunableNumber intakeArmTuningSetpointRotations;
  LoggedTunableNumber intakeArmTuningOverrideVolts;

  public IntakeMechanism(IntakeArmIO io) {
    intakeArmkP =
        new LoggedTunableNumber(
            "IntakeArmTunables/intakeArmkP", IntakeConstants.synced.getObject().intakeArmKP);
    intakeArmkI =
        new LoggedTunableNumber(
            "IntakeArmTunables/intakeArmkI", IntakeConstants.synced.getObject().intakeArmKI);
    intakeArmkD =
        new LoggedTunableNumber(
            "IntakeArmTunables/intakeArmkD", IntakeConstants.synced.getObject().intakeArmKD);

    intakeArmkS =
        new LoggedTunableNumber(
            "IntakeArmTunables/intakeArmkS", IntakeConstants.synced.getObject().intakeArmKS);
    intakeArmkV =
        new LoggedTunableNumber(
            "IntakeArmTunables/intakeArmkV", IntakeConstants.synced.getObject().intakeArmKV);
    intakeArmkA =
        new LoggedTunableNumber(
            "IntakeArmTunables/intakeArmkA", IntakeConstants.synced.getObject().intakeArmKA);
    intakeArmkG =
        new LoggedTunableNumber(
            "IntakeArmTunables/intakeArmkG", IntakeConstants.synced.getObject().intakeArmKG);

    intakeArmCruiseVelocity =
        new LoggedTunableNumber(
            "IntakeArmTunables/intakeArmCruiseVelocity",
            IntakeConstants.synced.getObject().intakeArmAngularCruiseVelocityRotationsPerSecond);
    intakeArmExpokV =
        new LoggedTunableNumber(
            "IntakeArmTunables/intakeArmExpokV",
            IntakeConstants.synced.getObject().intakeArmMotionMagicExpo_kV);
    intakeArmExpokA =
        new LoggedTunableNumber(
            "IntakeArmTunables/intakeArmExpokA",
            IntakeConstants.synced.getObject().intakeArmMotionMagicExpo_kA);

    intakeArmTuningSetpointRotations =
        new LoggedTunableNumber("IntakeArmTunables/intakeArmTuningSetpointRotations", 0.0);
    intakeArmTuningOverrideVolts =
        new LoggedTunableNumber("IntakeArmTunables/intakeArmTuningOverrideVolts", 0.0);

    this.io = io;
  }

  /**
   * Runs periodically when the robot is enabled
   *
   * <p>Does NOT run automatically! Must be called by the subsystem
   */
  public void periodic() {
    sendGoalAngleToIO();

    io.updateInputs(inputs);
    io.applyOutputs(outputs);

    Logger.processInputs("IntakeArm/inputs", inputs);
    Logger.processInputs("IntakeArm/outputs", outputs);
  }

  public void setBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
  }

  /** This method must be called from the subsystem's test periodic! */
  public void testPeriodic() {
    if (false) { // TODO: Replace placeholder test if IntakeArmTuning mode is active
      // switch (TestModeManager.getTestMode()) {
      // case IntakeArmClosedLoopTuning:
      io.setOutputMode(IntakeArmOutputMode.ClosedLoop);
      LoggedTunableNumber.ifChanged(
          hashCode(),
          (pid) -> {
            io.setPID(pid[0], pid[1], pid[2]);
          },
          intakeArmkP,
          intakeArmkI,
          intakeArmkD);

      LoggedTunableNumber.ifChanged(
          hashCode(),
          (ff) -> {
            io.setFF(ff[0], ff[1], ff[2], ff[3]);
          },
          intakeArmkS,
          intakeArmkV,
          intakeArmkA,
          intakeArmkG);

      LoggedTunableNumber.ifChanged(
          hashCode(),
          (maxProfile) -> {
            io.setMaxProfile(
                RadiansPerSecond.of(0.0),
                VoltsPerRadianPerSecondSquared.ofNative(maxProfile[0]),
                VoltsPerRadianPerSecond.ofNative(maxProfile[1]));
          },
          intakeArmExpokA,
          intakeArmExpokV);

      LoggedTunableNumber.ifChanged(
          hashCode(),
          (setpoint) -> {
            setGoalAngle(Rotations.of(setpoint[0]));
          },
          intakeArmTuningSetpointRotations);
      /*  case IntakeArmVoltageTuning:
        LoggedTunableNumber.ifChanged(
          hashCode(),
          (setpoint) -> {
            io.setOverrideVoltage(Volts.of(setpoint[0]));
          },
          intakeArmTuningOverrideVolts);
        io.setOverrideMode(true);
        break;
      }
      */
    }
  }

  public void sendGoalAngleToIO() {
    updateClampedGoalAngle();
    io.setIntakeArmEncoderGoalPos(clampedGoalAngle);
  }

  /**
   * Based on the bounds previously set, clamp the last set goal angle to be between the bounds.
   *
   * <p>If the goal height is outside of the bounds and the bounds are expanded, this function will
   * still behave as expected, as the mechanism remembers its unclamped goal height and will attempt
   * to get there once it is allowed.
   */
  private void updateClampedGoalAngle() {
    clampedGoalAngle.mut_replace(UnitUtils.clampMeasure(goalAngle, minAngle, maxAngle));

    Logger.recordOutput("IntakeArm/clampedGoalAngle", clampedGoalAngle);
  }

  /**
   * Set the goal angle the intakeArm will to control to.
   *
   * <p>This goal angle will be clamped by the allowed range of motion
   *
   * @param goalAngle The new goal angle
   */
  public void setGoalAngle(Angle goalAngle) {
    this.goalAngle.mut_replace(goalAngle);

    Logger.recordOutput("IntakeArm/goalAngle", goalAngle);
  }
  /**
   * Sets the minimum and maximum allowed angles that the intakeArm may target.
   *
   * <p>When not in override mode, the goal angle of the intakeArm will be clamped to be between
   * these values before it is sent to the IO. When these clamps change, the original goal angle is
   * clamped to be within the new bounds.
   *
   * @param minAngle The minimum angle, which will be clamped between intakeArmMinMinAngle and
   *     intakeArmMaxMaxAngle before being applied
   * @param maxAngle The maximum angle, which will be clamped between intakeArmMinMinAngle and
   *     intakeArmMaxMaxAngle before being applied
   */
  public void setAllowedRangeOfMotion(Angle minAngle, Angle maxAngle) {
    setMinAngle(minAngle);
    setMaxAngle(maxAngle);
  }

  /**
   * Sets the minimum allowed angle that the intakeArm may target.
   *
   * <p>When not in override mode, the goal angle of the intakeArm will be clamped to be between
   * these values before it is sent to the IO. When these clamps change, the original goal angle is
   * clamped to be within the new bounds.
   *
   * @param minAngle The minimum angle, which will be clamped between intakeArmMinMinAngle and
   *     intakeArmMaxMaxAngle before being applied
   */
  public void setMinAngle(Angle minAngle) {
    this.minAngle.mut_replace(
        UnitUtils.clampMeasure(
            minAngle,
            IntakeConstants.synced.getObject().intakeArmMinMinAngle,
            IntakeConstants.synced.getObject().intakeArmMaxMaxAngle));

    Logger.recordOutput("IntakeArm/minAngle", minAngle);
  }

  /**
   * Sets the maximum allowed angle that the intakeArm may target.
   *
   * <p>When not in override mode, the goal angle of the intakeArm will be clamped to be between
   * these values before it is sent to the IO. When these clamps change, the original goal angle is
   * clamped to be within the new bounds.
   *
   * @param maxAngle The maximum angle, which will be clamped between intakeArmMinMinAngle and
   *     intakeArmMaxMaxAngle before being applied
   */
  public void setMaxAngle(Angle maxAngle) {
    this.maxAngle.mut_replace(
        UnitUtils.clampMeasure(
            maxAngle,
            IntakeConstants.synced.getObject().intakeArmMaxMaxAngle,
            IntakeConstants.synced.getObject().intakeArmMaxMaxAngle));

    Logger.recordOutput("IntakeArm/maxAngle", maxAngle);
  }

  /**
   * Get the current angle of the intakeArm
   *
   * @return
   */
  public Angle getIntakeArmAngle() {
    return inputs.intakeArmEncoderPos;
  }

  /**
   * Get the current velocity of the intakeArm
   *
   * @return The current velocity of the intakeArm, according to the intakeArmEncoder
   */
  public AngularVelocity getIntakeArmVelocity() {
    return inputs.intakeArmEncoderVel;
  }

  /**
   * Check whether or not the intakeArmEncoder is currently connected.
   *
   * <p>"Connected" means that last time the position and velocity status signals were refreshed,
   * the status code was OK
   *
   * @return True if connected, false if disconnected
   */
  public boolean isIntakeArmEncoderConnected() {
    return inputs.intakeArmEncoderConnected;
  }

  /**
   * Get a reference to the intakeArm's IO. This should be used to update PID, motion profile, and
   * feed forward gains, and to set brake mode/disable motors. This method exists to avoid the need
   * to duplicate all of these functions between the mechanism and the IO.
   *
   * @return the intakeArm mechanism's IO
   */
  public IntakeArmIO getIO() {
    return io;
  }

  /** Set whether or not the motor on the intakeArm should be disabled */
  public void setMotorsDisabled(boolean disabled) {
    io.setMotorsDisabled(disabled);
  }

  /** Get the current unclamped goal angle of the intakeArm */
  public Angle getGoalAngle() {
    return goalAngle;
  }
}
