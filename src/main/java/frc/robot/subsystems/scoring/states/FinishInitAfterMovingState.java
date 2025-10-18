package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.Seconds;

import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;

/**
 * A state to handle finishing homing of the Indexer to the bottom of its range of motion after it
 * has already moved.
 *
 * <ul>
 *   <li>Drives the indexer downward until it is "homed." This is detected when:
 *       <ul>
 *         <li>The indexer stops moving/its velocity drops below a certain threshold determined to
 *             ignore encoder noise/jitter, since it must have been moving to enter this state.
 *         <li>A certain period of time has elapsed. This is a "giving up" last resort to avoid code
 *             hanging indefinitely in this state. This likely indicates that the velocity threshold
 *             needs to be increased to avoid encoder noise.
 *       </ul>
 *   <li>Transitions out of the state after the indexer is homed.
 */
public class FinishInitAfterMovingState extends BaseScoringState {
  /**
   * Timer to record the maximum duration that the indexer can be homing before it is determined
   * that the system is at the bottom of its range of motion.
   */
  private Timer homingTimer = new Timer();

  public void onEntry(Transition transition, ScoringSubsystem scoring) {
    scoring.stopShooter();
    homingTimer.restart();
    scoring.startHomingIndexer();
  }

  public void periodic(ScoringSubsystem scoring) {
    if (!DriverStation.isEnabled()) {
      homingTimer.restart();
      return;
    }

    // This state should never be entered but we may as well check and make sure
    if (scoring.hasIndexerSeeded()) {
      scoring.fireTrigger(ScoringTrigger.HomingFinished);
    }

    if (!scoring.isIndexerMoving()) {
      // If indexer has moved and is now not moving, it has moved and has come to rest, and
      // therefore is at the bottom
      System.out.println("INDEXER: Homed by moving and then stopping");
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
