package frc.robot.subsystems.scoring.states;

import coppercore.controls.state_machine.transition.Transition;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import java.util.Optional;
import java.util.function.Supplier;

public class IdleState extends BaseScoringState {
  public IdleState(Supplier<Optional<ScoringSubsystem>> scoringSupplier) {
    super(scoringSupplier);
  }

  public void onEntry(Transition transition, ScoringSubsystem scoring) {}

  public void periodic(ScoringSubsystem scoring) {
    scoring.stopIndexing();
    scoring.stopShooter();
  }

  public void onExit(Transition transition, ScoringSubsystem scoring) {}
}
