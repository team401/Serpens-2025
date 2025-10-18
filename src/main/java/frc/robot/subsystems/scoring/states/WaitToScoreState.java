package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.Seconds;

import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;

/**
 * Keeps the flywheels warming up and the indexer at top/indexing position for a certain delay,
 * configurable in {@link ScoringConstants}.
 *
 * @see frc.robot.constants.subsystems.scoring.ScoringConstants#timeToWaitForScore
 */
public class WaitToScoreState extends BaseScoringState {
  private Timer scoreTimer = new Timer();

  public void onEntry(Transition transition, ScoringSubsystem scoring) {
    scoreTimer.restart();
  }

  public void periodic(ScoringSubsystem scoring) {
    if (scoreTimer.hasElapsed(JsonConstants.scoringConstants.timeToWaitForScore.in(Seconds))) {
      scoring.fireTrigger(ScoringTrigger.WaitToScoreTimeExpired);
    }

    scoring.indexIntoShooter();
    scoring.warmupShooter();
  }

  public void onExit(Transition transition, ScoringSubsystem scoring) {}
}
