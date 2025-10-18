package frc.robot.subsystems.scoring.states;

import coppercore.controls.state_machine.transition.Transition;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;

/**
 * A state to warm up (get ready to shoot)
 *
 * <p>Holds the indexer in its bottom/idle position while warming up the shooter.
 *
 * <p>Once the shooter is at its goal speeds, transitions to Kick by firing WarmupReady
 */
public class WarmupState extends BaseScoringState {
  public void onEntry(Transition transition, ScoringSubsystem scoring) {}

  public void periodic(ScoringSubsystem scoring) {
    scoring.stopIndexing();
    scoring.warmupShooter();

    if (scoring.isShooterReady()) {
      scoring.fireTrigger(ScoringTrigger.WarmupReady);
    }
  }

  public void onExit(Transition transition, ScoringSubsystem scoring) {}
}
