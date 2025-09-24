package frc.robot.subsystems.intake;

import coppercore.wpilib_interface.MonitoredSubsystem;
import java.util.Optional;

public class IntakeSubsystem extends MonitoredSubsystem {
  private static Optional<IntakeSubsystem> instance = Optional.empty();
  private IntakeArmMechanism arm;
  private IntakeRollerMechanism roller;

  public IntakeSubsystem(IntakeArmMechanism arm, IntakeRollerMechanism roller) {
    this.arm = arm;
    this.roller = roller;
  }

  public static IntakeSubsystem create(IntakeArmMechanism arm, IntakeRollerMechanism roller) {
    if (instance.isPresent()) {
      throw new Error("IntakeSubsystem was created more than once.");
    }
    IntakeSubsystem createdInstance = new IntakeSubsystem(arm, roller);
    instance = Optional.of(createdInstance);

    return createdInstance;
  }

  public static Optional<IntakeSubsystem> getInstance() {
    return instance;
  }

  @Override
  public void periodic() {
    arm.periodic();
    roller.periodic();
  }

  public void startIntaking() {
    arm.goToIntakePos();
    roller.startSpinning();
  }

  public void stopIntaking() {
    arm.goToStartPos();
    roller.stop();
  }

  @Override
  public void monitoredPeriodic() {}
}
