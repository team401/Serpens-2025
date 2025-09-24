package frc.robot.subsystems.scoring.indexer;

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
import org.littletonrobotics.junction.Logger;

public class IndexerIOSim extends IndexerIOTalonFX {
  TalonFXSimState indexerMotorSimState = indexerMotor.getSimState();

  private final SingleJointedArmSim indexerSim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60Foc(1),
          IndexerConstants.synced.getObject().indexerReduction,
          IndexerConstants.Sim.synced.getObject().indexerMomentOfInertia.in(KilogramSquareMeters),
          IndexerConstants.Sim.synced.getObject().indexerArmLength.in(Meters),
          IndexerConstants.Sim.synced.getObject().indexerMinAngle.in(Radians),
          IndexerConstants.Sim.synced.getObject().indexerMaxAngle.in(Radians),
          true,
          IndexerConstants.Sim.synced.getObject().indexerStartingAngle.in(Radians));

  MutAngle lastIndexerAngle = Radians.mutable(0.0);

  public IndexerIOSim() {
    super();

    // Initialize sim state so that the first periodic runs with accurate data
    updateSimState();
  }

  private void updateSimState() {
    Angle indexerAngle = Radians.of(indexerSim.getAngleRads());
    AngularVelocity indexerVelocity = RadiansPerSecond.of(indexerSim.getVelocityRadPerSec());

    Angle diffAngle = indexerAngle.minus(lastIndexerAngle);
    lastIndexerAngle.mut_replace(indexerAngle);

    Angle rotorDiffAngle = diffAngle.times(IndexerConstants.synced.getObject().indexerReduction);
    AngularVelocity rotorVelocity =
        indexerVelocity.times(IndexerConstants.synced.getObject().indexerReduction);
    indexerMotorSimState.addRotorPosition(rotorDiffAngle);
    indexerMotorSimState.setRotorVelocity(rotorVelocity);
    indexerMotorSimState.setSupplyVoltage(RobotController.getBatteryVoltage());

    indexerSim.setInputVoltage(indexerMotorSimState.getMotorVoltage());

    Logger.recordOutput("indexerSim/position", indexerAngle.in(Radians));

    indexerSim.update(0.02);
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    updateSimState();

    super.updateInputs(inputs);
  }
}
