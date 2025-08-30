package frc.robot.subsystems.scoring.shooter;

import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.shooter.ShooterMechanism.ShooterSpeeds;
import frc.robot.util.PhoenixUtil;

public class ShooterIOTalonFX implements ShooterIO {
  protected TalonFX leftMotor;
  protected TalonFX rightMotor;

  private Debouncer leftConnectedDebouncer =
      new Debouncer(JsonConstants.canConstants.deviceConnectedDebounceTime.in(Seconds));
  private Debouncer rightConnectedDebouncer =
      new Debouncer(JsonConstants.canConstants.deviceConnectedDebounceTime.in(Seconds));

  // Store TalonFX configs to avoid creating a new one when updating PID/FF gains in tuning mode
  private TalonFXConfiguration talonFXConfigs;

  private final StatusSignal<AngularVelocity> leftMotorVelocity;
  private final StatusSignal<AngularAcceleration> leftMotorAcceleration;
  private final StatusSignal<Voltage> leftMotorVoltage;
  private final StatusSignal<Double> leftMotorClosedLoopOutput;
  private final StatusSignal<Current> leftMotorSupplyCurrent;
  private final StatusSignal<Current> leftMotorStatorCurrent;
  private final StatusSignal<Temperature> leftMotorTemperature;

  private final StatusSignal<AngularVelocity> rightMotorVelocity;
  private final StatusSignal<AngularAcceleration> rightMotorAcceleration;
  private final StatusSignal<Voltage> rightMotorVoltage;
  private final StatusSignal<Double> rightMotorClosedLoopOutput;
  private final StatusSignal<Current> rightMotorSupplyCurrent;
  private final StatusSignal<Current> rightMotorStatorCurrent;
  private final StatusSignal<Temperature> rightMotorTemperature;

  // Store control requests to avoid creating new ones every cycle
  private final TorqueCurrentFOC leftFOCRequest = new TorqueCurrentFOC(0.0);
  private final VoltageOut leftVoltageRequest = new VoltageOut(0.0);
  private final MotionMagicVelocityTorqueCurrentFOC leftClosedLoopRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);

  private final TorqueCurrentFOC rightFOCRequest = new TorqueCurrentFOC(0.0);
  private final VoltageOut rightVoltageRequest = new VoltageOut(0.0);
  private final MotionMagicVelocityTorqueCurrentFOC rightClosedLoopRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);

  public ShooterIOTalonFX() {
    // Configure motors
    leftMotor =
        new TalonFX(
            JsonConstants.canConstants.shooterLeftMotorID,
            JsonConstants.shooterConstants.CANBusName);
    rightMotor =
        new TalonFX(
            JsonConstants.canConstants.shooterRightMotorID,
            JsonConstants.shooterConstants.CANBusName);

    talonFXConfigs = JsonConstants.shooterConstants.baseTalonFXConfigs;
    applyConfigsToMotors();

    // Create left status signals
    leftMotorVelocity = leftMotor.getRotorVelocity();
    leftMotorAcceleration = leftMotor.getAcceleration();
    leftMotorVoltage = leftMotor.getMotorVoltage();
    leftMotorClosedLoopOutput = leftMotor.getClosedLoopOutput();
    leftMotorSupplyCurrent = leftMotor.getSupplyCurrent();
    leftMotorStatorCurrent = leftMotor.getStatorCurrent();
    leftMotorTemperature = leftMotor.getDeviceTemp();

    // Create right status signals
    rightMotorVelocity = rightMotor.getRotorVelocity();
    rightMotorAcceleration = rightMotor.getAcceleration();
    rightMotorVoltage = rightMotor.getMotorVoltage();
    rightMotorClosedLoopOutput = rightMotor.getClosedLoopOutput();
    rightMotorSupplyCurrent = rightMotor.getSupplyCurrent();
    rightMotorStatorCurrent = rightMotor.getStatorCurrent();
    rightMotorTemperature = rightMotor.getDeviceTemp();

    // Configure status signal updates
    BaseStatusSignal.setUpdateFrequencyForAll(
        JsonConstants.canConstants.updateFrequency,
        leftMotorVelocity,
        leftMotorAcceleration,
        leftMotorVoltage,
        leftMotorClosedLoopOutput,
        leftMotorSupplyCurrent,
        leftMotorStatorCurrent,
        leftMotorTemperature,
        rightMotorVelocity,
        rightMotorAcceleration,
        rightMotorVoltage,
        rightMotorClosedLoopOutput,
        rightMotorSupplyCurrent,
        rightMotorStatorCurrent,
        rightMotorTemperature);

    // Only update the signals configured above, and reduce all frequencies to the configured values
    ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor);
  }

  /**
   * Apply the current talonFXConfigs to the motors, updating the motor inverts individually before
   * applying
   */
  private void applyConfigsToMotors() {
    talonFXConfigs.MotorOutput.withInverted(JsonConstants.shooterConstants.leftMotorInverted);
    PhoenixUtil.tryUntilOk(
        JsonConstants.shooterConstants.maxConfigApplyAttempts,
        () ->
            leftMotor
                .getConfigurator()
                .apply(talonFXConfigs, JsonConstants.shooterConstants.configApplyTimeoutSeconds));

    talonFXConfigs.MotorOutput.withInverted(JsonConstants.shooterConstants.rightMotorInverted);
    PhoenixUtil.tryUntilOk(
        JsonConstants.shooterConstants.maxConfigApplyAttempts,
        () ->
            rightMotor
                .getConfigurator()
                .apply(talonFXConfigs, JsonConstants.shooterConstants.configApplyTimeoutSeconds));
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    var leftStatus =
        BaseStatusSignal.refreshAll(
            leftMotorVelocity,
            leftMotorAcceleration,
            leftMotorVoltage,
            leftMotorClosedLoopOutput,
            leftMotorSupplyCurrent,
            leftMotorStatorCurrent,
            leftMotorTemperature);

    var rightStatus =
        BaseStatusSignal.refreshAll(
            rightMotorVelocity,
            rightMotorAcceleration,
            rightMotorVoltage,
            rightMotorClosedLoopOutput,
            rightMotorSupplyCurrent,
            rightMotorStatorCurrent,
            rightMotorTemperature);

    // Update left inputs
    inputs.leftMotorConnected = leftConnectedDebouncer.calculate(leftStatus.isOK());
    inputs.leftMotorVelocity.mut_replace(leftMotorVelocity.getValue());
    inputs.leftMotorAcceleration.mut_replace(leftMotorAcceleration.getValue());
    inputs.leftMotorAppliedVolts.mut_replace(leftMotorVoltage.getValue());
    inputs.leftMotorClosedLoopOutput = leftMotorClosedLoopOutput.getValueAsDouble();
    inputs.leftMotorSupplyCurrent.mut_replace(leftMotorSupplyCurrent.getValue());
    inputs.leftMotorStatorCurrent.mut_replace(leftMotorStatorCurrent.getValue());
    inputs.leftMotorTemp.mut_replace(leftMotorTemperature.getValue());

    // Update right inputs
    inputs.rightMotorConnected = rightConnectedDebouncer.calculate(rightStatus.isOK());
    inputs.rightMotorVelocity.mut_replace(rightMotorVelocity.getValue());
    inputs.rightMotorAcceleration.mut_replace(rightMotorAcceleration.getValue());
    inputs.rightMotorAppliedVolts.mut_replace(rightMotorVoltage.getValue());
    inputs.rightMotorClosedLoopOutput = rightMotorClosedLoopOutput.getValueAsDouble();
    inputs.rightMotorSupplyCurrent.mut_replace(rightMotorSupplyCurrent.getValue());
    inputs.rightMotorStatorCurrent.mut_replace(rightMotorStatorCurrent.getValue());
    inputs.rightMotorTemp.mut_replace(rightMotorTemperature.getValue());
  }

  @Override
  public void runOpenLoop(Current leftTorqueCurrent, Current rightTorqueCurrent) {
    leftMotor.setControl(leftFOCRequest.withOutput(leftTorqueCurrent));
    rightMotor.setControl(rightFOCRequest.withOutput(rightTorqueCurrent));
  }

  @Override
  public void runOpenLoop(Voltage leftVoltage, Voltage rightVoltage) {
    leftMotor.setControl(leftVoltageRequest.withOutput(leftVoltage));
    rightMotor.setControl(rightVoltageRequest.withOutput(rightVoltage));
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
  }

  @Override
  public void runSpeeds(ShooterSpeeds speeds) {
    leftMotor.setControl(leftClosedLoopRequest.withVelocity(speeds.leftSpeed()));
    rightMotor.setControl(rightClosedLoopRequest.withVelocity(speeds.rightSpeed()));
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    talonFXConfigs.Slot0.kP = kP;
    talonFXConfigs.Slot0.kI = kI;
    talonFXConfigs.Slot0.kD = kD;

    applyConfigsToMotors();
  }

  @Override
  public void setFF(double kS, double kV, double kA) {
    talonFXConfigs.Slot0.kS = kS;
    talonFXConfigs.Slot0.kV = kV;
    talonFXConfigs.Slot0.kA = kA;

    applyConfigsToMotors();
  }

  @Override
  public void setBrakeMode(boolean brakeEnabled) {
    talonFXConfigs.MotorOutput.NeutralMode =
        brakeEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    applyConfigsToMotors();
  }
}
