package frc.robot.subsystems.scoring.states;

import coppercore.controls.state_machine.transition.Transition;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;

public class KickState extends BaseScoringState {
  public void onEntry(Transition transition, ScoringSubsystem scoring) {}

  @Override
  public void periodic(ScoringSubsystem scoring) {
    scoring.indexIntoShooter();
    scoring.warmupShooter();

    if (scoring.hasIndexerKicked()) {
      scoring.fireTrigger(ScoringTrigger.IndexerDoneKicking);
    }
  }

  public void onExit(Transition transition, ScoringSubsystem scoring) {}
}
