package frc.robot.subsystems.scoring.states;

import coppercore.controls.state_machine.transition.Transition;
import frc.robot.subsystems.scoring.ScoringSubsystem;

/**
 * The default state of the Scoring subsystem. Holds the indexer at the bottom of its range of
 * motion and stops the shooter.
 */
public class IdleState extends BaseScoringState {
  public void onEntry(Transition transition, ScoringSubsystem scoring) {}

  public void periodic(ScoringSubsystem scoring) {
    scoring.stopIndexing();
    scoring.stopShooter();
  }

  public void onExit(Transition transition, ScoringSubsystem scoring) {}
}
