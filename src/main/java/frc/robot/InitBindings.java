package frc.robot;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.scoring.ScoringSubsystem;

/** Methods to initialize bindings for each subsystems */
public abstract class InitBindings {
  /**
   * Initialize bindings that require only the scoring subsystem
   *
   * <p>This method will assume controller and scoring subsystem are not null.
   */
  public static void initScoringBindings(
      CommandXboxController controller, ScoringSubsystem scoring) {
    controller
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  scoring.tempWarmup();
                }))
        .onFalse(
            new InstantCommand(
                () -> {
                  scoring.tempStopShooter();
                }));
  }
}
