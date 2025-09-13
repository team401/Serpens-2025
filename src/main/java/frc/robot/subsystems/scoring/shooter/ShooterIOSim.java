package frc.robot.subsystems.scoring.shooter;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Seconds;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.sim.TalonFXSimState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.constants.JsonConstants;
import org.littletonrobotics.junction.Logger;

public class ShooterIOSim extends ShooterIOTalonFX {
  private TalonFXSimState motorSimState;

  private FlywheelSim flywheelSim =
      new FlywheelSim(
          LinearSystemId.createFlywheelSystem(
              DCMotor.getKrakenX60Foc(1),
              JsonConstants.shooterConstantsSim.momentOfInertia.in(KilogramSquareMeters),
              JsonConstants.shooterConstants.gearing),
          DCMotor.getKrakenX60Foc(1));

  private Timer deltaTimer = new Timer();

  // Store invert factor and gearing to
  private final int invertFactor;
  private final double gearing;

  public ShooterIOSim(ShooterSide side) {
    super(side);

    motorSimState = motor.getSimState();

    deltaTimer.restart();

    InvertedValue motorInvert =
        side == ShooterSide.Left
            ? JsonConstants.shooterConstants.leftMotorInverted
            : JsonConstants.shooterConstants.rightMotorInverted;

    if (motorInvert == InvertedValue.CounterClockwise_Positive) {
      invertFactor = 1;
    } else {
      invertFactor = -1;
    }

    gearing = JsonConstants.shooterConstants.gearing;
  }

  private void updateSimState() {
    double deltaTime = deltaTimer.get();
    deltaTimer.restart();

    // Update sim
    var motorVoltage = motorSimState.getMotorVoltage();
    Logger.recordOutput("scoring/shooter/sim/motorVoltage", motorVoltage);

    flywheelSim.setInput(motorVoltage);
    flywheelSim.update(deltaTime);

    Logger.recordOutput("scoring/shooter/sim/deltaTime", deltaTime);

    // Gearing = Output : Input
    // Flywheel Speed = Motor Speed * Gearing, therefore:
    // Motor Speed = Flywheel Speed / Gearing
    // The same math holds for position 
    motorSimState.addRotorPosition(flywheelSim.getAngularVelocity().times(Seconds.of(deltaTime)));
    motorSimState.setRotorVelocity(flywheelSim.getAngularVelocity().times(invertFactor / gearing));
    motorSimState.setRotorAcceleration(
        flywheelSim.getAngularAcceleration().times(invertFactor / gearing));
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    updateSimState();

    super.updateInputs(inputs);
  }
}
