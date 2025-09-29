package frc.robot.subsystems.scoring.states;

import coppercore.controls.state_machine.transition.Transition;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * A state that prevents the scoring state machine from taking any action while in test mode.
 *
 * <p>This state should be entered whenever the robot enters a scoring test mode by firing the
 * EnterTestMode trigger
 *
 * <p>This state will automatically exit when the robot leaves a Scoring test mode
 */
public class TestModeState extends BaseScoringState {
  public TestModeState(Supplier<Optional<ScoringSubsystem>> scoringSupplier) {
    super(scoringSupplier);
  }

  public void onEntry(Transition transition, ScoringSubsystem scoring) {}

  public void periodic(ScoringSubsystem scoring) {}

  public void onExit(Transition transition, ScoringSubsystem scoring) {}
}
