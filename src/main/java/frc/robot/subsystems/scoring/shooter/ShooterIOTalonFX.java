package frc.robot.subsystems.scoring.shooter;

import static edu.wpi.first.units.Units.RotationsPerSecondPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import frc.robot.constants.JsonConstants;
import frc.robot.util.PhoenixUtil;
import org.littletonrobotics.junction.Logger;

public class ShooterIOTalonFX implements ShooterIO {
  protected final ShooterSide side;

  protected TalonFX motor;

  private Debouncer connectedDebouncer =
      new Debouncer(JsonConstants.canConstants.deviceConnectedDebounceTime.in(Seconds));

  // Store TalonFX configs to avoid creating a new one when updating PID/FF gains in tuning mode
  private TalonFXConfiguration talonFXConfigs;

  private final StatusSignal<AngularVelocity> motorVelocity;
  private final StatusSignal<AngularAcceleration> motorAcceleration;
  private final StatusSignal<Voltage> motorVoltage;
  private final StatusSignal<Double> motorClosedLoopOutput;
  private final StatusSignal<Double> motorClosedLoopReference;
  private final StatusSignal<Current> motorSupplyCurrent;
  private final StatusSignal<Current> motorStatorCurrent;

  // Store control requests to avoid creating new ones every cycle
  private final TorqueCurrentFOC focRequest = new TorqueCurrentFOC(0.0);
  private final VoltageOut voltageRequest = new VoltageOut(0.0);
  private final MotionMagicVelocityTorqueCurrentFOC closedLoopRequest =
      new MotionMagicVelocityTorqueCurrentFOC(0.0);

  // Store an alert to publish if configs fail to apply
  private final Alert configFailedToApplyAlert;

  public ShooterIOTalonFX(ShooterSide side) {
    this.side = side;

    final int motorID =
        side == ShooterSide.Left
            ? JsonConstants.canConstants.shooterLeftMotorID
            : JsonConstants.canConstants.shooterRightMotorID;

    // Configure motors
    motor = new TalonFX(motorID, JsonConstants.shooterConstants.CANBusName);

    talonFXConfigs = JsonConstants.shooterConstants.baseTalonFXConfigs;

    InvertedValue motorInvert =
        side == ShooterSide.Left
            ? JsonConstants.shooterConstants.leftMotorInverted
            : JsonConstants.shooterConstants.rightMotorInverted;

    talonFXConfigs.MotorOutput.Inverted = motorInvert;

    applyMotorConfig();

    // Create status signals
    motorVelocity = motor.getRotorVelocity();
    motorAcceleration = motor.getAcceleration();
    motorVoltage = motor.getMotorVoltage();
    motorClosedLoopOutput = motor.getClosedLoopOutput();
    motorClosedLoopReference = motor.getClosedLoopReference();
    motorSupplyCurrent = motor.getSupplyCurrent();
    motorStatorCurrent = motor.getStatorCurrent();

    // Configure status signal updates
    BaseStatusSignal.setUpdateFrequencyForAll(
        JsonConstants.canConstants.updateFrequency,
        motorVelocity,
        motorAcceleration,
        motorVoltage,
        motorClosedLoopOutput,
        motorClosedLoopReference,
        motorSupplyCurrent,
        motorStatorCurrent);

    // Only update the signals configured above, and reduce all frequencies to the configured values
    ParentDevice.optimizeBusUtilizationForAll(motor);

    // Initialize alert(s) with proper name
    configFailedToApplyAlert =
        new Alert(side.name() + " shooter IO failed to apply configs.", AlertType.kError);
    configFailedToApplyAlert.set(false);
  }

  /**
   * Apply the current talonFXConfigs to the motor, trying to re-apply until it succeeds
   *
   * <p>If the config fails to apply after all attempts, an alert will be shown and an error will be
   * printed.
   */
  private void applyMotorConfig() {
    boolean configApplySucceeded =
        PhoenixUtil.tryUntilOk(
            JsonConstants.shooterConstants.maxConfigApplyAttempts,
            () ->
                motor
                    .getConfigurator()
                    .apply(
                        talonFXConfigs, JsonConstants.shooterConstants.configApplyTimeoutSeconds));

    if (!configApplySucceeded) {
      configFailedToApplyAlert.set(true);

      new Exception(configFailedToApplyAlert.getText()).printStackTrace();
    }
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    StatusCode status =
        BaseStatusSignal.refreshAll(
            motorVelocity,
            motorAcceleration,
            motorVoltage,
            motorClosedLoopOutput,
            motorClosedLoopReference,
            motorSupplyCurrent,
            motorStatorCurrent);

    // Update inputs
    inputs.motorConnected = connectedDebouncer.calculate(status.isOK());
    inputs.motorVelocity.mut_replace(motorVelocity.getValue());
    inputs.motorAcceleration.mut_replace(motorAcceleration.getValue());
    inputs.motorAppliedVolts.mut_replace(motorVoltage.getValue());
    inputs.motorSupplyCurrent.mut_replace(motorSupplyCurrent.getValue());
    inputs.motorStatorCurrent.mut_replace(motorStatorCurrent.getValue());

    // Do extra logging
    Logger.recordOutput(
        "shooter/closedLoopOutput" + side.name(), motorClosedLoopOutput.getValueAsDouble());
    Logger.recordOutput(
        "shooter/closedLoopReference" + side.name(), motorClosedLoopReference.getValueAsDouble());

    if (!status.isOK()) {
      System.err.println(side.name() + " shooter motor had bad status: " + status);
    }

    // Update output status logging
    Logger.recordOutput(
        "scoring/shooter/outputStatus" + side.name(), motor.getMotorOutputStatus().getValue());
  }

  @Override
  public void runOpenLoop(Current torqueCurrent) {
    motor.setControl(focRequest.withOutput(torqueCurrent));
    // TODO: Decide if this level of logging is good or necessary
    Logger.recordOutput("scoring/shooter/lastOutputCommand" + side.name(), "TorqueCurrent");
  }

  @Override
  public void runOpenLoop(Voltage voltage) {
    motor.setControl(voltageRequest.withOutput(voltage));
    Logger.recordOutput("scoring/shooter/lastOutputCommand" + side.name(), "Voltage");
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public void runSpeed(AngularVelocity speed) {
    motor.setControl(closedLoopRequest.withVelocity(speed));

    Logger.recordOutput("scoring/shooter/lastOutputCommand" + side.name(), "Speed");
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    talonFXConfigs.Slot0.kP = kP;
    talonFXConfigs.Slot0.kI = kI;
    talonFXConfigs.Slot0.kD = kD;

    applyMotorConfig();
  }

  @Override
  public void setFFSVA(double kS, double kV, double kA) {
    talonFXConfigs.Slot0.kS = kS;
    talonFXConfigs.Slot0.kV = kV;
    talonFXConfigs.Slot0.kA = kA;

    applyMotorConfig();
  }

  @Override
  public void setMaxProfileAcceleration(AngularAcceleration maxAcceleration) {
    talonFXConfigs.MotionMagic.MotionMagicAcceleration =
        maxAcceleration.in(RotationsPerSecondPerSecond);

    applyMotorConfig();
  }

  @Override
  public void setBrakeMode(boolean brakeEnabled) {
    talonFXConfigs.MotorOutput.NeutralMode =
        brakeEnabled ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    applyMotorConfig();
  }
}
