package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.Seconds;

import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;

/**
 * A state to handle the initial portion of homing of the Indexer to the bottom of its range of
 * motion.
 *
 * <ul>
 *   <li>Drives the indexer downward until it moves or until a certain unmoving timeout has expired.
 *   <li>Transitions out of the state after the indexer is homed.
 */
public class StartInitState extends BaseScoringState {
  /**
   * Timer to record the maximum duration that the indexer can remain stationary before it is
   * determined that the system is at the bottom of its range of motion.
   */
  private Timer homingTimer = new Timer();

  public void onEntry(Transition transition, ScoringSubsystem scoring) {
    scoring.stopShooter();
    homingTimer.restart();
    scoring.startHomingIndexer();
  }

  public void periodic(ScoringSubsystem scoring) {
    scoring.startHomingIndexer();

    if (!DriverStation.isEnabled()) {
      homingTimer.restart();
      return;
    }

    // This state should never be entered but we may as well check and make sure
    if (scoring.hasIndexerSeeded()) {
      scoring.fireTrigger(ScoringTrigger.HomingFinished);
    }

    if (scoring.isIndexerMoving()) {
      scoring.fireTrigger(ScoringTrigger.MovedDuringHoming);
    } else if (homingTimer.hasElapsed(
        JsonConstants.scoringConstants.homingMaxUnmovingTime.in(Seconds))) {
      System.out.println("INDEXER: Homed by never moving");
      seedAtBottomAndExit(scoring);
    }

    if (homingTimer.hasElapsed(JsonConstants.scoringConstants.homingMaxTime.in(Seconds))) {
      System.out.println("INDEXER: Homed by giving up");
      seedAtBottomAndExit(scoring);
    }
  }

  public void onExit(Transition transition, ScoringSubsystem scoring) {
    // Puts indexer back into closed-loop mode, targets idle position
    scoring.stopIndexing();
  }

  /**
   * Seed the indexer at the bottom of its range of motion and fire the {@code Homed} trigger to
   * exit the state.
   */
  private void seedAtBottomAndExit(ScoringSubsystem scoring) {
    scoring.seedIndexerAtBottom();
    scoring.fireTrigger(ScoringTrigger.HomingFinished);
  }
}
