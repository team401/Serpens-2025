package frc.robot.subsystems.scoring.indexer;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.JsonConstants;
import org.littletonrobotics.junction.Logger;

public class IndexerIOTalonFX implements IndexerIO {
  MutAngle indexerGoalAngle = Rotations.mutable(0.0);
  MutAngle indexerSetpointPosition = Rotations.mutable(0.0);

  Current overrideCurrent;
  Voltage overrideVoltage;

  IndexerOutputMode outputMode = IndexerOutputMode.ClosedLoop;
  TalonFX indexerMotor;

  // Reuse the same talonFXConfiguration instead of making a new one each time.
  TalonFXConfiguration talonFXConfigs;

  boolean motorDisabled = false;

  private StatusSignal<Angle> indexerMotorPosition;
  private StatusSignal<AngularVelocity> indexerMotorVelocity;
  private StatusSignal<Current> indexerMotorSupplyCurrent;
  private StatusSignal<Current> indexerMotorStatorCurrent;

  // Reuse the same motion magic request to avoid garbage collector having to clean them up.
  MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueCurrentFOC =
      new MotionMagicExpoTorqueCurrentFOC(0.0);
  VoltageOut voltageOut = new VoltageOut(0.0);
  TorqueCurrentFOC currentOut = new TorqueCurrentFOC(0.0);

  public IndexerIOTalonFX() {
    // Initialize TalonFXs  and CANcoders with their correct IDs
    indexerMotor = new TalonFX(JsonConstants.canConstants.indexerMotorID, "canivore");

    // Cache status signals and refresh them when used
    indexerMotorPosition = indexerMotor.getPosition();
    indexerMotorVelocity = indexerMotor.getVelocity();

    indexerMotorSupplyCurrent = indexerMotor.getSupplyCurrent();
    indexerMotorStatorCurrent = indexerMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        indexerMotorSupplyCurrent,
        indexerMotorStatorCurrent,
        indexerMotorPosition,
        indexerMotorVelocity);

    // Initialize talonFXConfigs to use FusedCANCoder and Motion Magic Expo and have correct PID
    // gains and current limits.
    talonFXConfigs =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(
                        IndexerConstants.synced.getObject().indexerStatorCurrentLimit))
            .withSlot0(
                new Slot0Configs()
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withKS(IndexerConstants.synced.getObject().indexerKS)
                    .withKV(IndexerConstants.synced.getObject().indexerKV)
                    .withKA(IndexerConstants.synced.getObject().indexerKA)
                    .withKG(IndexerConstants.synced.getObject().indexerKG)
                    .withKP(IndexerConstants.synced.getObject().indexerKP)
                    .withKI(IndexerConstants.synced.getObject().indexerKI)
                    .withKD(IndexerConstants.synced.getObject().indexerKD))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(
                        IndexerConstants.synced.getObject()
                            .indexerAngularCruiseVelocityRotationsPerSecond)
                    .withMotionMagicExpo_kA(
                        IndexerConstants.synced.getObject().indexerMotionMagicExpo_kA)
                    .withMotionMagicExpo_kV(
                        IndexerConstants.synced.getObject().indexerMotionMagicExpo_kV));

    // Apply talonFX config to motor
    indexerMotor.getConfigurator().apply(talonFXConfigs);

    // Make follower motor permanently follow lead motor.
  }

  @Override
  public void updateInputs(IndexerInputs inputs) {
    StatusCode refreshStatus =
        BaseStatusSignal.refreshAll(indexerMotorPosition, indexerMotorVelocity);

    inputs.indexerEncoderPos.mut_replace(indexerMotorPosition.getValue());
    inputs.indexerEncoderVel.mut_replace(indexerMotorVelocity.getValue());
    inputs.indexerEncoderConnected = refreshStatus.isOK();

    refreshStatus =
        BaseStatusSignal.refreshAll(indexerMotorSupplyCurrent, indexerMotorStatorCurrent);

    inputs.indexerMotorSupplyCurrent.mut_replace(indexerMotorSupplyCurrent.getValue());
    inputs.indexerMotorStatorCurrent.mut_replace(indexerMotorStatorCurrent.getValue());
    inputs.indexerMotorConnected = indexerMotor.isConnected();

    inputs.indexerMotorGoalPos.mut_replace(indexerGoalAngle);
    inputs.indexerEncoderSetpointPos.mut_replace(indexerSetpointPosition);

    inputs.motionMagicError = indexerMotor.getClosedLoopError().getValueAsDouble();

    inputs.indexerVelocity.mut_replace(indexerMotorVelocity.getValue());
  }

  @Override
  public void applyOutputs(IndexerOutputs outputs) {
    outputs.motorsDisabled = motorDisabled;
    outputs.outputMode = outputMode;

    motionMagicExpoTorqueCurrentFOC.withPosition(indexerGoalAngle);

    if (motorDisabled) {
      indexerMotor.setControl(voltageOut.withOutput(0.0));
      outputs.indexerAppliedVolts.mut_replace(Volts.of(0.0));
    } else {
      switch (outputMode) {
        case ClosedLoop:
          indexerMotor.setControl(motionMagicExpoTorqueCurrentFOC);

          indexerSetpointPosition.mut_setMagnitude(
              (indexerMotor.getClosedLoopReference().getValue()));

          Logger.recordOutput(
              "indexer/referenceSlope",
              indexerMotor.getClosedLoopReferenceSlope().getValueAsDouble());
          outputs.indexerAppliedVolts.mut_replace(indexerMotor.getMotorVoltage().getValue());
          outputs.indexerClosedLoopOutput = indexerMotor.getClosedLoopOutput().getValueAsDouble();
          outputs.pContrib.mut_replace(
              Volts.of(indexerMotor.getClosedLoopProportionalOutput().getValueAsDouble()));
          outputs.iContrib.mut_replace(
              Volts.of(indexerMotor.getClosedLoopIntegratedOutput().getValueAsDouble()));
          outputs.dContrib.mut_replace(
              Volts.of(indexerMotor.getClosedLoopDerivativeOutput().getValueAsDouble()));
          break;
        case Voltage:
          indexerMotor.setControl(new VoltageOut(overrideVoltage));
          outputs.indexerAppliedVolts.mut_replace(overrideVoltage);
          break;
        case Current:
          indexerMotor.setControl(currentOut.withOutput(overrideCurrent));
          outputs.indexerAppliedVolts.mut_replace(indexerMotor.getMotorVoltage().getValue());
          break;
      }
    }
  }

  @Override
  public void setIndexerMotorGoalAngle(Angle goalPos) {
    indexerGoalAngle.mut_replace(goalPos);
  }

  @Override
  public void setIndexerPosition(Angle newAngle) {
    indexerMotor.setPosition(newAngle);
  }

  @Override
  public void setOutputMode(IndexerOutputMode outputMode) {
    this.outputMode = outputMode;
  }

  @Override
  public void setOverrideVoltage(Voltage volts) {
    overrideVoltage = volts;
  }

  @Override
  public void setOverrideCurrent(Current current) {
    overrideCurrent = current;
  }

  @Override
  public void setPID(double p, double i, double d) {
    Slot0Configs configs = talonFXConfigs.Slot0;

    configs.kP = p;
    configs.kI = i;
    configs.kD = d;
    indexerMotor.getConfigurator().apply(configs);
  }

  @Override
  public void setMaxProfile(
      AngularVelocity maxVelocity,
      Per<VoltageUnit, AngularAccelerationUnit> expo_kA,
      Per<VoltageUnit, AngularVelocityUnit> expo_kV) {
    MotionMagicConfigs configs =
        talonFXConfigs
            .MotionMagic
            // .withMotionMagicCruiseVelocity(maxVelocity)
            .withMotionMagicExpo_kA(expo_kA)
            .withMotionMagicExpo_kV(expo_kV);
    indexerMotor.getConfigurator().apply(configs);
  }

  @Override
  public void setFF(double kS, double kV, double kA, double kG) {
    Slot0Configs configs = talonFXConfigs.Slot0;

    configs.kS = kS;
    configs.kV = kV;
    configs.kA = kA;
    configs.kG = kG;
    indexerMotor.getConfigurator().apply(configs);
  }

  @Override
  public void setBrakeMode(boolean brakeMode) {
    indexerMotor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setStatorCurrentLimit(Current currentLimit) {
    talonFXConfigs.CurrentLimits.withStatorCurrentLimit(currentLimit);

    // Only apply current limit configs to avoid overwriting PID and FF values from tuning
    indexerMotor.getConfigurator().apply(talonFXConfigs.CurrentLimits);
  }

  @Override
  public void setMotorsDisabled(boolean disabled) {
    motorDisabled = disabled;
  }
}
