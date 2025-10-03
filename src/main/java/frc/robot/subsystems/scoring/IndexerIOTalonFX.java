package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.JsonConstants;

public class IndexerIOTalonFX implements IndexerIO {
  TalonFX indexerMotor = new TalonFX(JsonConstants.indexerConstants.indexerMotorId, "canivore");

  // Keep track of talonFX configs and only update FF/PID when necessary to avoid unnecessary object
  // creation
  private TalonFXConfiguration talonFXConfigs;

  private MutAngle indexerGoalPosition = Rotations.mutable(0.2);

  private MotionMagicExpoVoltage request =
      new MotionMagicExpoVoltage(indexerGoalPosition).withEnableFOC(true);
  private VoltageOut overrideRequest = new VoltageOut(0.0);

  private boolean motorsDisabled = false;

  private boolean isOverriding = false;
  private MutVoltage overrideVoltage = Volts.mutable(0.0);

  public IndexerIOTalonFX() {
    talonFXConfigs =
        new TalonFXConfiguration()
            .withFeedback(
                new FeedbackConfigs()
                    .withSensorToMechanismRatio(
                        JsonConstants.indexerConstants.sensorToMechanismRatio))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(
                        JsonConstants.indexerConstants.indexerSupplyCurrentLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(
                        JsonConstants.indexerConstants.indexerStatorCurrentLimit))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(JsonConstants.indexerConstants.peakFOCCurrent)
                    .withPeakReverseTorqueCurrent(JsonConstants.indexerConstants.peakFOCCurrent))
            .withSlot0(
                new Slot0Configs()
                    .withKG(JsonConstants.indexerConstants.indexerKG)
                    .withKS(JsonConstants.indexerConstants.indexerKS)
                    .withKV(JsonConstants.indexerConstants.indexerKV)
                    .withKA(JsonConstants.indexerConstants.indexerKA)
                    .withKP(JsonConstants.indexerConstants.indexerKP)
                    .withKI(JsonConstants.indexerConstants.indexerKI)
                    .withKD(JsonConstants.indexerConstants.indexerKD)
                    .withGravityType(GravityTypeValue.Arm_Cosine))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(
                        JsonConstants.indexerConstants
                            .indexerMotionMagicCruiseVelocityRotationsPerSecond)
                    .withMotionMagicExpo_kA(
                        JsonConstants.indexerConstants.indexerMotionMagicExpo_kA)
                    .withMotionMagicExpo_kV(
                        JsonConstants.indexerConstants.indexerMotionMagicExpo_kV));

    indexerMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void updateInputs(IndexerInputs inputs) {
    inputs.indexerGoalPosition.mut_replace(indexerGoalPosition);
    inputs.indexerSetpointPosition.mut_replace(
        Rotations.of(indexerMotor.getClosedLoopReference().getValue()));

    if (isOverriding) {
      indexerMotor.setControl(overrideRequest.withOutput(overrideVoltage));
    }

    indexerMotor.setControl(request.withPosition(indexerGoalPosition));
  }

  public void setIndexerGoalPos(Angle goalPos) {
    indexerGoalPosition.mut_replace(goalPos);
  }

  public void setPID(double kP, double kI, double kD) {
    talonFXConfigs.Slot0.kP = kP;
    talonFXConfigs.Slot0.kI = kI;
    talonFXConfigs.Slot0.kD = kD;

    indexerMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void setMaxProfile(
      AngularVelocity maxVelocity,
      Per<VoltageUnit, AngularAccelerationUnit> expo_kA,
      Per<VoltageUnit, AngularVelocityUnit> expo_kV) {
    talonFXConfigs.withMotionMagic(
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(maxVelocity)
            .withMotionMagicExpo_kA(expo_kA)
            .withMotionMagicExpo_kV(expo_kV));

    indexerMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void setFF(double kS, double kV, double kA, double kG) {
    talonFXConfigs.Slot0.kS = kS;
    talonFXConfigs.Slot0.kV = kV;
    talonFXConfigs.Slot0.kA = kA;
    talonFXConfigs.Slot0.kG = kG;

    indexerMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void setBrakeMode(boolean brakeMode) {
    if (brakeMode) {
      talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    } else {
      talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }

    indexerMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void setCurrentLimits(CurrentLimitsConfigs limits) {
    talonFXConfigs.withCurrentLimits(limits);

    indexerMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void setMotorsDisabled(boolean disabled) {
    motorsDisabled = disabled;
  }

  public void setOverrideMode(boolean override) {
    isOverriding = override;
  }

  public void setOverrideVoltage(Voltage volts) {
    overrideVoltage.mut_replace(volts);
  }
}
