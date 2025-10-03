package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.JsonConstants;
import org.littletonrobotics.junction.Logger;

public class IndexerIOSim extends IndexerIOTalonFX {

  TalonFXSimState indexerMotorSimState = indexerMotor.getSimState();

  private final SingleJointedArmSim indexerSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          JsonConstants.indexerConstants.indexerReduction,
          JsonConstants.indexerConstantsSim.indexerMomentOfInertia.in(KilogramSquareMeters),
          JsonConstants.indexerConstantsSim.indexerArmLength.in(Meters),
          JsonConstants.indexerConstantsSim.indexerMinAngle.in(Radians),
          JsonConstants.indexerConstantsSim.indexerMaxAngle.in(Radians),
          true,
          JsonConstants.indexerConstantsSim.indexerStartingAngle.in(Radians));

  public IndexerIOSim(TalonFXSimState indexerMotorSimState) {
    this.indexerMotorSimState = indexerMotorSimState;
  }

  public IndexerIOSim(MutAngle lastIndexerAngle) {
    this.lastIndexerAngle = lastIndexerAngle;
  }

  public SingleJointedArmSim getIndexerSim() {
    return indexerSim;
  }

  public IndexerIOSim(TalonFXSimState indexerMotorSimState, MutAngle lastIndexerAngle) {
    this.indexerMotorSimState = indexerMotorSimState;
    this.lastIndexerAngle = lastIndexerAngle;
  }

  public IndexerIOSim() {
    super();

    // Initialize sim state so that the first periodic runs with accurate data
    updateSimState();
  }

  MutAngle lastIndexerAngle = Radians.mutable(0.0);

  private void updateSimState() {
    Angle indexerAngle = Radians.of(indexerSim.getAngleRads());
    AngularVelocity indexerVelocity = RadiansPerSecond.of(indexerSim.getVelocityRadPerSec());

    Angle diffAngle = indexerAngle.minus(lastIndexerAngle);
    lastIndexerAngle.mut_replace(indexerAngle);

    Angle rotorDiffAngle = diffAngle.times(JsonConstants.indexerConstants.indexerReduction);
    AngularVelocity rotorVelocity =
        indexerVelocity.times(JsonConstants.indexerConstants.indexerReduction);

    indexerMotorSimState.addRotorPosition(rotorDiffAngle);
    indexerMotorSimState.setRotorVelocity(rotorVelocity);
    indexerMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    indexerSim.setInputVoltage(indexerMotorSimState.getMotorVoltage());

    Logger.recordOutput("indexerSim/position", indexerAngle.in(Radians));
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    updateSimState();

    super.updateInputs(inputs);
  }
}
