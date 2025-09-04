package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.ChassisReference;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.SimConstants;
import frc.robot.constants.subsystems.intake.IntakeConstants;

import org.littletonrobotics.junction.Logger;

public class IntakeArmIOSim extends IntakeArmIOTalonFX {
  CANcoderSimState intakeArmEncoderSimState = intakeArmEncoder.getSimState();

  TalonFXSimState intakeArmMotorSimState = intakeArmMotor.getSimState();

  private final SingleJointedArmSim intakeArmSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          IntakeConstants.synced.getObject().intakeArmReduction,
          IntakeConstants.Sim.synced
              .getObject()
              .intakeArmMomentOfInertia
              .in(KilogramSquareMeters),
          IntakeConstants.Sim.synced.getObject().intakeArmArmLength.in(Meters),
          IntakeConstants.Sim.synced.getObject().intakeArmMinAngle.in(Radians),
          IntakeConstants.Sim.synced.getObject().intakeArmMaxAngle.in(Radians),
          true,
          IntakeConstants.Sim.synced.getObject().intakeArmStartingAngle.in(Radians));

  MutAngle lastIntakeArmAngle = Radians.mutable(0.0);

  public IntakeArmIOSim() {
    super();

    intakeArmEncoderSimState.Orientation = ChassisReference.Clockwise_Positive;

    // Initialize sim state so that the first periodic runs with accurate data
    updateSimState();
  }

  private void updateSimState() {
    Angle intakeArmAngle = Radians.of(intakeArmSim.getAngleRads());
    AngularVelocity intakeArmVelocity = RadiansPerSecond.of(intakeArmSim.getVelocityRadPerSec());

    Angle diffAngle = intakeArmAngle.minus(lastIntakeArmAngle);
    lastIntakeArmAngle.mut_replace(intakeArmAngle);

    // 1:1 ratio of IntakeArm to CANcoder makes this math very easy
    intakeArmEncoderSimState.setRawPosition(
        intakeArmAngle.minus(
            IntakeConstants.synced.getObject()
                .intakeArmEncoderMagnetOffset)); // Subtract the magnet offset since it's 0 in sim
    intakeArmEncoderSimState.setVelocity(intakeArmVelocity);

    Angle rotorDiffAngle =
        diffAngle.times(IntakeConstants.synced.getObject().intakeArmReduction);
    AngularVelocity rotorVelocity =
        intakeArmVelocity.times(IntakeConstants.synced.getObject().intakeArmReduction);
    intakeArmMotorSimState.addRotorPosition(rotorDiffAngle);
    intakeArmMotorSimState.setRotorVelocity(rotorVelocity);
    intakeArmMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    intakeArmSim.setInputVoltage(intakeArmMotorSimState.getMotorVoltage());

    Logger.recordOutput("intakeArmSim/position", intakeArmAngle.in(Radians));

    intakeArmSim.update(SimConstants.simDeltaTime.in(Seconds));
  }

  @Override
  public void updateInputs(IntakeArmInputs inputs) {
    updateSimState();

    super.updateInputs(inputs);
  }
}
