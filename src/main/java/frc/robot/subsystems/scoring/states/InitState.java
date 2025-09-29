package frc.robot.subsystems.scoring.states;

import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.scoring.ScoringSubsystem;
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
  Timer homingTimer = new Timer();

  public void onEntry(Transition transition, ScoringSubsystem scoring) {
    homingTimer.reset();
    homingTimer.start();
  }

  public void periodic(ScoringSubsystem scoring) {}

  public void onExit(Transition transition, ScoringSubsystem scoring) {}
}
