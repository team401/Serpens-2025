package frc.robot.subsystems.scoring;

import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.scoring.IndexerIO.IndexerInputs;

public class IndexerIOSim implements IndexerIO {

  private double positionRot = 0.0;
  private double velocityRps = 0.0;
  private double appliedVoltage = 0.0;

  private boolean brakeMode = true;
  private boolean motorsDisabled = false;
  private boolean overrideMode = false;

  // crude simulation constants
  private static final double kSimInertia = 0.0; // "flywheel" inertia
  private static final double kSimKv = 0.0; // RPS per volt
  private static final double kSimKa = 0.0; // acceleration per volt
  private static final double kSimDt = 0.0; // 20ms loop

  @Override
  public void updateInputs(IndexerInputs inputs) {
    // integrate physics ??
    if (!motorsDisabled) {
      double accel = (appliedVoltage * kSimKa) - (velocityRps * 0.0);
      velocityRps += accel * kSimDt;
      positionRot += velocityRps * kSimDt;
    } else {
      if (brakeMode) velocityRps = 0.0;
    }

    // populate inputs
    inputs.isIndexerEncoderConnected = true;
    inputs.indexerPosition.mut_replace(positionRot, Units.Rotations);
    inputs.indexerVelocity.mut_replace(velocityRps, Units.RotationsPerSecond);
    inputs.indexerSupplyCurrent.mut_replace(Math.abs(appliedVoltage) * 0.0, Units.Amps);
    inputs.indexerStatorCurrent.mut_replace(Math.abs(appliedVoltage) * 0.0, Units.Amps);
    inputs.indexerInput = appliedVoltage;
  }

  @Override
  public void setIndexerGoalPos(Angle goalPos) {
    // crude "motion magic" style: just directly set position
    double error = goalPos.in(Units.Rotations) - positionRot;
    velocityRps = error * 0.0; // pretend proportional controller
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    // no-op in sim
  }

  @Override
  public void setMaxProfile(
      AngularVelocity maxVelocity,
      Per<VoltageUnit, AngularAccelerationUnit> expo_kA,
      Per<VoltageUnit, AngularVelocityUnit> expo_kV) {
    // no-op in sim
  }

  @Override
  public void setFF(double kS, double kV, double kA, double kG) {
    // no-op in sim
  }

  @Override
  public void setBrakeMode(boolean brakeMode) {
    this.brakeMode = brakeMode;
  }

  @Override
  public void setCurrentLimits(com.ctre.phoenix6.configs.CurrentLimitsConfigs limits) {
    // no-op in sim
  }

  @Override
  public void setMotorsDisabled(boolean disabled) {
    this.motorsDisabled = disabled;
  }

  @Override
  public void setOverrideMode(boolean override) {
    this.overrideMode = override;
  }

  @Override
  public void setOverrideVoltage(Voltage voltage) {
    appliedVoltage = overrideMode ? voltage.in(Units.Volts) : 0.0;
  }
}
