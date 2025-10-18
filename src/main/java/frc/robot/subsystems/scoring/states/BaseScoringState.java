package frc.robot.subsystems.scoring.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import frc.robot.subsystems.scoring.ScoringSubsystem;

/**
 * A base state for the ScoringSubsystem which wraps calls to onEntry, periodic, and onExit with
 * checks to ensure that the ScoringSubsystem exists before unwrapping the Optional and passing the
 * ScoringSubsystem object to child implementations of these methods.
 *
 * <p>This means that classes which extend BaseScoringState don't need to perform optional or null
 * checks on the ScoringSubsystem and can assume that their onEntry, periodic, and onExit methods
 * will only run if the ScoringSubsystem is present.
 */
public abstract class BaseScoringState implements PeriodicStateInterface {
  @Override
  public final void onEntry(Transition transition) {
    ScoringSubsystem.getInstance().ifPresent(scoring -> onEntry(transition, scoring));
  }

  /**
   * Method called on state entry if the scoring instance isn't null.
   *
   * @param transition the Transition object representing the transition into the state
   * @param scoring A ScoringSubsystem instance, which cannot be null
   */
  protected abstract void onEntry(Transition transition, ScoringSubsystem scoring);

  @Override
  public final void periodic() {
    ScoringSubsystem.getInstance().ifPresent(scoring -> periodic(scoring));
  }

  /**
   * Method called in the periodic loop if the scoring instance isn't null.
   *
   * @param scoring A scoring subsystem instance, which cannot be null
   */
  protected abstract void periodic(ScoringSubsystem scoring);

  @Override
  public final void onExit(Transition transition) {
    ScoringSubsystem.getInstance().ifPresent(scoring -> onExit(transition, scoring));
  }

  /**
   * Method called on state exit if the scoring instance isn't null.
   *
   * @param transition the Transition object representing the transition out of the state
   * @param scoring A ScoringSubsystem instance, which cannot be null
   */
  protected abstract void onExit(Transition transition, ScoringSubsystem scoring);
}
