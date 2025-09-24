package frc.robot.subsystems.scoring.indexer;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecond;
import static edu.wpi.first.units.Units.VoltsPerRadianPerSecondSquared;

import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.wpilib_interface.UnitUtils;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import frc.robot.TestModeManager;
import frc.robot.subsystems.scoring.indexer.IndexerIO.IndexerOutputMode;
import org.littletonrobotics.junction.Logger;

/**
 * A Mechanism to manage the Indexer
 *
 * <ul>
 *   <li>Uses closed-loop TorqueCurrentFOC control
 */
public class IndexerMechanism {
  IndexerIO io;
  IndexerInputsAutoLogged inputs = new IndexerInputsAutoLogged();
  IndexerOutputsAutoLogged outputs = new IndexerOutputsAutoLogged();

  MutAngle goalAngle = Rotations.mutable(0.0);
  MutAngle clampedGoalAngle = Rotations.mutable(0.0);

  MutAngle minAngle = IndexerConstants.synced.getObject().indexerMinMinAngle.mutableCopy();
  MutAngle maxAngle = IndexerConstants.synced.getObject().indexerMaxMaxAngle.mutableCopy();

  LoggedTunableNumber indexerkP;
  LoggedTunableNumber indexerkI;
  LoggedTunableNumber indexerkD;

  LoggedTunableNumber indexerkS;
  LoggedTunableNumber indexerkV;
  LoggedTunableNumber indexerkA;
  LoggedTunableNumber indexerkG;

  LoggedTunableNumber indexerCruiseVelocity;
  LoggedTunableNumber indexerExpokV;
  LoggedTunableNumber indexerExpokA;

  LoggedTunableNumber indexerTuningSetpointRotations;
  LoggedTunableNumber indexerTuningOverrideVolts;

  public IndexerMechanism(IndexerIO io) {
    indexerkP =
        new LoggedTunableNumber(
            "IndexerTunables/indexerkP", IndexerConstants.synced.getObject().indexerKP);
    indexerkI =
        new LoggedTunableNumber(
            "IndexerTunables/indexerkI", IndexerConstants.synced.getObject().indexerKI);
    indexerkD =
        new LoggedTunableNumber(
            "IndexerTunables/indexerkD", IndexerConstants.synced.getObject().indexerKD);

    indexerkS =
        new LoggedTunableNumber(
            "IndexerTunables/indexerkS", IndexerConstants.synced.getObject().indexerKS);
    indexerkV =
        new LoggedTunableNumber(
            "IndexerTunables/indexerkV", IndexerConstants.synced.getObject().indexerKV);
    indexerkA =
        new LoggedTunableNumber(
            "IndexerTunables/indexerkA", IndexerConstants.synced.getObject().indexerKA);
    indexerkG =
        new LoggedTunableNumber(
            "IndexerTunables/indexerkG", IndexerConstants.synced.getObject().indexerKG);

    indexerCruiseVelocity =
        new LoggedTunableNumber(
            "IndexerTunables/indexerCruiseVelocity",
            IndexerConstants.synced.getObject().indexerAngularCruiseVelocityRotationsPerSecond);
    indexerExpokV =
        new LoggedTunableNumber(
            "IndexerTunables/indexerExpokV",
            IndexerConstants.synced.getObject().indexerMotionMagicExpo_kV);
    indexerExpokA =
        new LoggedTunableNumber(
            "IndexerTunables/indexerExpokA",
            IndexerConstants.synced.getObject().indexerMotionMagicExpo_kA);

    indexerTuningSetpointRotations =
        new LoggedTunableNumber("IndexerTunables/indexerTuningSetpointRotations", 0.0);
    indexerTuningOverrideVolts =
        new LoggedTunableNumber("IndexerTunables/indexerTuningOverrideVolts", 0.0);

    this.io = io;

    // TODO: Stop assuming that the indexer is at 0 on init.
    // THIS IS TERRIBLE AWFUL CODE
    io.setIndexerPosition(Rotations.of(0.0));
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

    Logger.processInputs("Indexer/inputs", inputs);
    Logger.processInputs("Indexer/outputs", outputs);
  }

  public void setBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
  }

  /** This method must be called from the subsystem's test periodic! */
  public void testPeriodic() {
    switch (TestModeManager.getTestMode()) {
      case IndexerClosedLoopTuning:
        io.setOutputMode(IndexerOutputMode.ClosedLoop);
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid) -> {
              io.setPID(pid[0], pid[1], pid[2]);
            },
            indexerkP,
            indexerkI,
            indexerkD);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (ff) -> {
              io.setFF(ff[0], ff[1], ff[2], ff[3]);
            },
            indexerkS,
            indexerkV,
            indexerkA,
            indexerkG);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (maxProfile) -> {
              io.setMaxProfile(
                  RadiansPerSecond.of(0.0),
                  VoltsPerRadianPerSecondSquared.ofNative(maxProfile[0]),
                  VoltsPerRadianPerSecond.ofNative(maxProfile[1]));
            },
            indexerExpokA,
            indexerExpokV);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (setpoint) -> {
              setGoalAngle(Rotations.of(setpoint[0]));
            },
            indexerTuningSetpointRotations);
        break;
      case IndexerVoltageTuning:
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (setpoint) -> {
              io.setOverrideVoltage(Volts.of(setpoint[0]));
            },
            indexerTuningOverrideVolts);
        io.setOutputMode(IndexerOutputMode.Voltage);
        break;
    }
  }

  public void sendGoalAngleToIO() {
    updateClampedGoalAngle();
    io.setIndexerMotorGoalAngle(clampedGoalAngle);
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

    Logger.recordOutput("Indexer/clampedGoalAngle", clampedGoalAngle);
  }

  /**
   * Set the goal angle the indexer will to control to.
   *
   * <p>This goal angle will be clamped by the allowed range of motion
   *
   * @param goalAngle The new goal angle
   */
  public void setGoalAngle(Angle goalAngle) {
    this.goalAngle.mut_replace(goalAngle);

    Logger.recordOutput("Indexer/goalAngle", goalAngle);
  }
  /**
   * Get the current angle of the indexer
   *
   * @return
   */
  public Angle getIndexerAngle() {
    return inputs.indexerEncoderPos;
  }

  /**
   * Get the current velocity of the indexer
   *
   * @return The current velocity of the indexer, according to the indexerEncoder
   */
  public AngularVelocity getIndexerVelocity() {
    return inputs.indexerEncoderVel;
  }

  /**
   * Check whether or not the indexerEncoder is currently connected.
   *
   * <p>"Connected" means that last time the position and velocity status signals were refreshed,
   * the status code was OK
   *
   * @return True if connected, false if disconnected
   */
  public boolean isIndexerEncoderConnected() {
    return inputs.indexerEncoderConnected;
  }

  /**
   * Get a reference to the indexer's IO. This should be used to update PID, motion profile, and
   * feed forward gains, and to set brake mode/disable motors. This method exists to avoid the need
   * to duplicate all of these functions between the mechanism and the IO.
   *
   * @return the indexer mechanism's IO
   */
  public IndexerIO getIO() {
    return io;
  }

  /** Set whether or not the motor on the indexer should be disabled */
  public void setMotorsDisabled(boolean disabled) {
    io.setMotorsDisabled(disabled);
  }

  /** Get the current unclamped goal angle of the indexer */
  public Angle getGoalAngle() {
    return goalAngle;
  }

  public void indexIntoShooter() {
    // TODO: Implement indexing into shooter
  }

  public void stopIndexing() {
    // TODO: Implement returning indexer to zero
  }
}
