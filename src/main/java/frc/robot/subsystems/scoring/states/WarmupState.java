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
    // Indexer should already be at 0 from IdleState, but keep commanding it down here for
    // safety/robustness
    scoring.stopIndexing();

    // Start spinning up the shooter wheels.
    scoring.warmupShooter();

    // Since the indexer should already have been at the bottom, shooter RPM is the only thing that
    // needs to be checked here. If the indexer did somehow inexplicably rise, it would effectively
    // have entered kick state early and thus this transition would still be somewhat valid
    // representation of what was happening.
    if (scoring.isShooterReady()) {
      scoring.fireTrigger(ScoringTrigger.WarmupReady);
    }
  }

  public void onExit(Transition transition, ScoringSubsystem scoring) {}
}
