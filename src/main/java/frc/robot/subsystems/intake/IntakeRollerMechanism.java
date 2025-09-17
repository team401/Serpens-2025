package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import coppercore.parameter_tools.LoggedTunableNumber;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.intake.IntakeRollerIO.IntakeRollerInputs;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

/**
 * A Mechanism to manage the IntakeArm
 *
 * <ul>
 *   <li>Uses closed-loop TorqueCurrentFOC control
 */
public class IntakeRollerMechanism {
  IntakeRollerIO io;
  IntakeRollerInputsAutoLogged inputs = new IntakeRollerInputsAutoLogged();

  public record RollerSpeeds(AngularVelocity rollerSpeeds) {}

  private static final RollerSpeeds ZERO_SPEEDS = new RollerSpeeds(RotationsPerSecond.zero());

  @AutoLogOutput(key = "Intake/Roller/goalSpeeds")
  private RollerSpeeds goalSpeeds = ZERO_SPEEDS;

  private enum RollerOutputMode {
    ClosedLoop,
    Voltage,
    Current,
    Stop,
  }

  @AutoLogOutput(key = "Intake/Roller/outputMode")
  private RollerOutputMode outputMode = RollerOutputMode.ClosedLoop;
  // tuneable for test modes
  // tuneable gains
  private LoggedTunableNumber intakeRollerKP =
      new LoggedTunableNumber(
          "IntakeRollerTunables/KP", JsonConstants.intakeConstants.baseTalonFXConfigs.Slot0.kP);
  private LoggedTunableNumber intakeRollerKI =
      new LoggedTunableNumber(
          "IntakeRollerTunables/KI", JsonConstants.intakeConstants.baseTalonFXConfigs.Slot0.kI);
  private LoggedTunableNumber intakeRollerKD =
      new LoggedTunableNumber(
          "IntakeRollerTunables/KD", JsonConstants.intakeConstants.baseTalonFXConfigs.Slot0.kD);

  private LoggedTunableNumber intakeRollerKS =
      new LoggedTunableNumber(
          "IntakeRollerTunables/KS", JsonConstants.intakeConstants.baseTalonFXConfigs.Slot0.kS);
  private LoggedTunableNumber intakeRollerKV =
      new LoggedTunableNumber(
          "IntakeRollerTunables/KV", JsonConstants.intakeConstants.baseTalonFXConfigs.Slot0.kV);
  private LoggedTunableNumber intakeRollerKA =
      new LoggedTunableNumber(
          "IntakeRollerTunables/KA", JsonConstants.intakeConstants.baseTalonFXConfigs.Slot0.kA);

  // tunable outputs
  private LoggedTunableNumber intakeRollerManualVolts =
      new LoggedTunableNumber("IntakeRollerTunables/ManualVolts", 0.0);

  private LoggedTunableNumber intakeRollerManualAmps =
      new LoggedTunableNumber("IntakeRollerTunables/ManualAmps", 0.0);

  private LoggedTunableNumber intakeRollerTargetRPM =
      new LoggedTunableNumber("IntakeRollerTunables/TargetRPM", 0.0);

  public IntakeRollerMechanism(IntakeRollerIO io) {
    this.io = io;
  }

  public void testPeriodic() {
    switch (TestModeManager.getTestMode()) {
      case IntakeRollerCurrentTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (currents) -> {
              io.runOpenLoop(Amps.of(currents[0]));

              outputMode = RollerOutputMode.Current;
            },
            intakeRollerManualAmps);
      }

      case IntakeRollerVoltageTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (voltages) -> {
              io.runOpenLoop(Volts.of(voltages[0]));

              outputMode = RollerOutputMode.Voltage;
            },
            intakeRollerManualVolts);
      }

      case IntakeRollerClosedLoopTuning -> {
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid) -> {
              io.setPID(pid[0], pid[1], pid[2]);
            },
            intakeRollerKP,
            intakeRollerKI,
            intakeRollerKD);
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (ff) -> {
              io.setFFSVA(ff[0], ff[1], ff[2]);
            },
            intakeRollerKS,
            intakeRollerKV,
            intakeRollerKA);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (speeds) -> {
              runSpeeds(new RollerSpeeds(RotationsPerSecond.of(speeds[0])));
            },
            intakeRollerTargetRPM);
      }

      default -> {}
    }
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Rollers/rollerInputs", inputs);
  }

  public void runSpeeds(RollerSpeeds speeds) {
    io.runSpeed(speeds.rollerSpeeds);

    outputMode = RollerOutputMode.ClosedLoop;
  }

  /** Stop the intake rollers, setting their goal speeds to zero */
  public void stop() {
    io.stop();

    outputMode = RollerOutputMode.Stop;
  }

  public final IntakeRollerInputs getLeftInputs() {
    return inputs;
  }
}
