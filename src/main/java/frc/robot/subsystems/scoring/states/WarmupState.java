package frc.robot.subsystems.scoring.states;

import coppercore.controls.state_machine.transition.Transition;
import frc.robot.subsystems.scoring.ScoringSubsystem;
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
  }

  public void onExit(Transition transition, ScoringSubsystem scoring) {}
}
