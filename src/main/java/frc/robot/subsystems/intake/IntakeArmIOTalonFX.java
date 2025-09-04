package frc.robot.subsystems.intake;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
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
import frc.robot.constants.subsystems.intake.IntakeConstants;

import org.littletonrobotics.junction.Logger;

public class IntakeArmIOTalonFX implements IntakeArmIO {
  MutAngle intakeArmEncoderGoalAngle = Rotations.mutable(0.0);
  MutAngle intakeArmEncoderSetpointPosition = Rotations.mutable(0.0);

  Current overrideCurrent;
  Voltage overrideVoltage;

  IntakeArmOutputMode outputMode = IntakeArmOutputMode.ClosedLoop;
  TalonFX intakeArmMotor;

  CANcoder intakeArmEncoder;

  // Reuse the same talonFXConfiguration instead of making a new one each time.
  TalonFXConfiguration talonFXConfigs;

  boolean motorDisabled = false;

  private StatusSignal<Angle> intakeArmEncoderPosition;
  private StatusSignal<AngularVelocity> intakeArmEncoderVelocity;
  private StatusSignal<Current> intakeArmMotorSupplyCurrent;
  private StatusSignal<Current> intakeArmMotorStatorCurrent;

  // Reuse the same motion magic request to avoid garbage collector having to clean them up.
  MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueCurrentFOC =
      new MotionMagicExpoTorqueCurrentFOC(0.0);
  VoltageOut voltageOut = new VoltageOut(0.0);
  TorqueCurrentFOC currentOut = new TorqueCurrentFOC(0.0);

  public IntakeArmIOTalonFX() {
    // Initialize TalonFXs  and CANcoders with their correct IDs
    intakeArmMotor =
        new TalonFX(IntakeConstants.synced.getObject().intakeArmMotorId, "canivore");

    intakeArmEncoder =
        new CANcoder(IntakeConstants.synced.getObject().intakeArmEncoderID, "canivore");

    CANcoderConfiguration cancoderConfiguration = new CANcoderConfiguration();
    cancoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        IntakeConstants.synced.getObject().intakeArmEncoderDiscontinuityPoint;

    // Update with large CANcoder direction and apply
    cancoderConfiguration.MagnetSensor.SensorDirection =
        IntakeConstants.synced.getObject().intakeArmEncoderDirection;
    cancoderConfiguration.MagnetSensor.MagnetOffset =
        IntakeConstants.synced.getObject().intakeArmEncoderMagnetOffset.in(Rotations);
    intakeArmEncoder.getConfigurator().apply(cancoderConfiguration);

    // Cache status signals and refresh them when used
    intakeArmEncoderPosition = intakeArmEncoder.getPosition();
    intakeArmEncoderVelocity = intakeArmEncoder.getVelocity();

    intakeArmMotorSupplyCurrent = intakeArmMotor.getSupplyCurrent();
    intakeArmMotorStatorCurrent = intakeArmMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        intakeArmMotorSupplyCurrent,
        intakeArmMotorStatorCurrent,
        intakeArmEncoderPosition,
        intakeArmEncoderVelocity);

    // Initialize talonFXConfigs to use FusedCANCoder and Motion Magic Expo and have correct PID
    // gains and current limits.
    talonFXConfigs =
        new TalonFXConfiguration()
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackRemoteSensorID(intakeArmEncoder.getDeviceID())
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withSensorToMechanismRatio(
                        IntakeConstants.synced.getObject().intakeArmEncoderToMechanismRatio)
                    .withRotorToSensorRatio(
                        IntakeConstants.synced.getObject().rotorToIntakeArmEncoderRatio))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(
                        IntakeConstants.synced.getObject().intakeArmStatorCurrentLimit))
            .withSlot0(
                new Slot0Configs()
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withKS(IntakeConstants.synced.getObject().intakeArmKS)
                    .withKV(IntakeConstants.synced.getObject().intakeArmKV)
                    .withKA(IntakeConstants.synced.getObject().intakeArmKA)
                    .withKG(IntakeConstants.synced.getObject().intakeArmKG)
                    .withKP(IntakeConstants.synced.getObject().intakeArmKP)
                    .withKI(IntakeConstants.synced.getObject().intakeArmKI)
                    .withKD(IntakeConstants.synced.getObject().intakeArmKD))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(
                        IntakeConstants.synced.getObject()
                            .intakeArmAngularCruiseVelocityRotationsPerSecond)
                    .withMotionMagicExpo_kA(
                        IntakeConstants.synced.getObject().intakeArmMotionMagicExpo_kA)
                    .withMotionMagicExpo_kV(
                        IntakeConstants.synced.getObject().intakeArmMotionMagicExpo_kV));

    // Apply talonFX config to motor
    intakeArmMotor.getConfigurator().apply(talonFXConfigs);

    // Make follower motor permanently follow lead motor.
  }

  @Override
  public void updateInputs(IntakeArmInputs inputs) {
    StatusCode refreshStatus =
        BaseStatusSignal.refreshAll(intakeArmEncoderPosition, intakeArmEncoderVelocity);

    inputs.intakeArmEncoderPos.mut_replace(intakeArmEncoderPosition.getValue());
    inputs.intakeArmEncoderVel.mut_replace(intakeArmEncoderVelocity.getValue());
    inputs.intakeArmEncoderConnected = refreshStatus.isOK();

    refreshStatus =
        BaseStatusSignal.refreshAll(intakeArmMotorSupplyCurrent, intakeArmMotorStatorCurrent);

    inputs.intakeArmMotorSupplyCurrent.mut_replace(intakeArmMotorSupplyCurrent.getValue());
    inputs.intakeArmMotorStatorCurrent.mut_replace(intakeArmMotorStatorCurrent.getValue());
    inputs.intakeArmMotorConnected = intakeArmMotor.isConnected();

    inputs.intakeArmEncoderGoalPos.mut_replace(intakeArmEncoderGoalAngle);
    inputs.intakeArmEncoderSetpointPos.mut_replace(intakeArmEncoderSetpointPosition);

    inputs.motionMagicError = intakeArmMotor.getClosedLoopError().getValueAsDouble();

    inputs.intakeArmVelocity.mut_replace(intakeArmEncoder.getVelocity().getValue());
  }

  @Override
  public void applyOutputs(IntakeArmOutputs outputs) {
    outputs.motorsDisabled = motorDisabled;
    outputs.outputMode = outputMode;

    motionMagicExpoTorqueCurrentFOC.withPosition(intakeArmEncoderGoalAngle);

    if (motorDisabled) {
      intakeArmMotor.setControl(voltageOut.withOutput(0.0));
      outputs.intakeArmAppliedVolts.mut_replace(Volts.of(0.0));
    } else {
      switch (outputMode) {
        case ClosedLoop:
          intakeArmMotor.setControl(motionMagicExpoTorqueCurrentFOC);

          intakeArmEncoderSetpointPosition.mut_setMagnitude(
              (intakeArmMotor.getClosedLoopReference().getValue()));

          Logger.recordOutput(
              "intakeArm/referenceSlope",
              intakeArmMotor.getClosedLoopReferenceSlope().getValueAsDouble());
          outputs.intakeArmAppliedVolts.mut_replace(intakeArmMotor.getMotorVoltage().getValue());
          outputs.intakeArmClosedLoopOutput =
              intakeArmMotor.getClosedLoopOutput().getValueAsDouble();
          outputs.pContrib.mut_replace(
              Volts.of(intakeArmMotor.getClosedLoopProportionalOutput().getValueAsDouble()));
          outputs.iContrib.mut_replace(
              Volts.of(intakeArmMotor.getClosedLoopIntegratedOutput().getValueAsDouble()));
          outputs.dContrib.mut_replace(
              Volts.of(intakeArmMotor.getClosedLoopDerivativeOutput().getValueAsDouble()));
          break;
        case Voltage:
          intakeArmMotor.setControl(new VoltageOut(overrideVoltage));
          outputs.intakeArmAppliedVolts.mut_replace(overrideVoltage);
          break;
        case Current:
          intakeArmMotor.setControl(currentOut.withOutput(overrideCurrent));
          outputs.intakeArmAppliedVolts.mut_replace(intakeArmMotor.getMotorVoltage().getValue());
          break;
      }
    }
  }

  @Override
  public void setIntakeArmEncoderGoalPos(Angle goalPos) {
    intakeArmEncoderGoalAngle.mut_replace(goalPos);
  }

  @Override
  public void setIntakeArmEncoderPosition(Angle newAngle) {
    intakeArmEncoder.setPosition(newAngle);
  }

  @Override
  public void setOutputMode(IntakeArmOutputMode outputMode) {
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
    intakeArmMotor.getConfigurator().apply(configs);
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
    intakeArmMotor.getConfigurator().apply(configs);
  }

  @Override
  public void setFF(double kS, double kV, double kA, double kG) {
    Slot0Configs configs = talonFXConfigs.Slot0;

    configs.kS = kS;
    configs.kV = kV;
    configs.kA = kA;
    configs.kG = kG;
    intakeArmMotor.getConfigurator().apply(configs);
  }

  @Override
  public void setBrakeMode(boolean brakeMode) {
    intakeArmMotor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setStatorCurrentLimit(Current currentLimit) {
    talonFXConfigs.CurrentLimits.withStatorCurrentLimit(currentLimit);

    // Only apply current limit configs to avoid overwriting PID and FF values from tuning
    intakeArmMotor.getConfigurator().apply(talonFXConfigs.CurrentLimits);
  }

  @Override
  public void setMotorsDisabled(boolean disabled) {
    motorDisabled = disabled;
  }
}
