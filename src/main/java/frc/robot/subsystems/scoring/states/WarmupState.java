package frc.robot.subsystems.scoring.states;

import coppercore.controls.state_machine.transition.Transition;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;
import java.util.Optional;
import java.util.function.Supplier;

public class WarmupState extends BaseScoringState {
  public WarmupState(Supplier<Optional<ScoringSubsystem>> scoringSupplier) {
    super(scoringSupplier);
  }

  public void onEntry(Transition transition, ScoringSubsystem scoring) {}

  public void periodic(ScoringSubsystem scoring) {
    scoring.stopIndexing();
    scoring.warmupShooter();

    // If the shooter is ready, we can fire WarmupReady if:
    // - Pose-based shooting is enabled and the shot is attainable
    // - Pose-based shooting is not enabled
    if (scoring.isShooterReady()
        && ((scoring.isPoseBasedShootingEnabled() && scoring.isShotAttainable())
            || (!scoring.isPoseBasedShootingEnabled()))) {
      scoring.fireTrigger(ScoringTrigger.WarmupReady);
    }
  }

  public void onExit(Transition transition, ScoringSubsystem scoring) {}
}
