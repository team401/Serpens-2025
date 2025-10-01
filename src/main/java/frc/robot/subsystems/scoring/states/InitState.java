package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.Seconds;

import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * A state to handle homing of the Indexer to the bottom of its range of motion.
 *
 * <ul>
 *   <li>Drives the indexer downward until it is "homed." This is detected when:
 *       <ul>
 *         <li>Doesn't move for a certain period of time
 *         <li>Moves at a certain velocity and then stops
 *       </ul>
 *   <li>Transitions out of the state after the indexer is homed.
 */
public class InitState extends BaseScoringState {
  public InitState(Supplier<Optional<ScoringSubsystem>> scoringSupplier) {
    super(scoringSupplier);
  }

  /**
   * Timer to record the maximum duration that the indexer can remain stationary before it is
   * determined that the system is at the bottom of its range of motion.
   */
  private Timer homingTimer = new Timer();
  /** Keep track of whether or not the indexer has moved yet */
  private boolean hasMoved = false;

  public void onEntry(Transition transition, ScoringSubsystem scoring) {
    scoring.stopShooter();
    homingTimer.restart();
    scoring.startHomingIndexer();

    hasMoved = false;
  }

  public void periodic(ScoringSubsystem scoring) {
    if (!DriverStation.isEnabled()) {
      homingTimer.restart();
      return;
    }

    // This state should never be entered but we may as well check and make sure
    if (scoring.hasIndexerSeeded()) {
      scoring.fireTrigger(ScoringTrigger.Homed);
    }

    if (scoring.isIndexerMoving()) {
      hasMoved = true;
    } else {
      // If the indexer is not moving
      if (hasMoved) {
        // If indexer has moved and is now not moving, it has moved and has come to rest, and
        // therefore is at the bottom
        System.out.println("INDEXER: Homed by moving and then stopping");
        seedAtBottomAndExit(scoring);
      } else if (homingTimer.hasElapsed(
          JsonConstants.scoringConstants.homingMaxUnmovingTime.in(Seconds))) {
        System.out.println("INDEXER: Homed by never moving");
        seedAtBottomAndExit(scoring);
      }
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
    scoring.fireTrigger(ScoringTrigger.Homed);
  }
}
