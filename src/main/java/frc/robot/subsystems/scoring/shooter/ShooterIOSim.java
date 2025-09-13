package frc.robot.subsystems.scoring.shooter;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Seconds;

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

  // Store gearing to make it easier to type
  private final double gearing;

  public ShooterIOSim(ShooterSide side) {
    super(side);

    motorSimState = motor.getSimState();

    deltaTimer.restart();

    gearing = JsonConstants.shooterConstants.gearing;
  }

  private void updateSimState() {
    double deltaTime = deltaTimer.get();
    deltaTimer.restart();

    // Update sim
    var motorVoltage = motorSimState.getMotorVoltage();
    // TODO: Figure out why this method returns 0 on 1 of every 2 of the calls
    // Logger.recordOutput("scoring/shooter/sim/motorVoltage", motorVoltage);
    // System.out.println("scoring/shooter/sim/motorVoltage: " + motorVoltage);

    flywheelSim.setInput(motorVoltage);
    flywheelSim.update(deltaTime);

    Logger.recordOutput("scoring/shooter/sim/deltaTime", deltaTime);

    // Gearing = Output : Input
    // Flywheel Speed = Motor Speed * Gearing, therefore:
    // Motor Speed = Flywheel Speed / Gearing
    // The same math holds for position
    motorSimState.addRotorPosition(
        flywheelSim.getAngularVelocity().times(Seconds.of(deltaTime)).div(gearing));
    motorSimState.setRotorVelocity(flywheelSim.getAngularVelocity().div(gearing));
    motorSimState.setRotorAcceleration(flywheelSim.getAngularAcceleration().div(gearing));
  }

  @Override
  public void updateInputs(ShooterInputs inputs) {
    updateSimState();

    super.updateInputs(inputs);
  }
}
