package frc.robot.subsystems.scoring;

import coppercore.controls.state_machine.StateMachine;
import coppercore.controls.state_machine.StateMachineConfiguration;
import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.state.StateContainer;
import coppercore.wpilib_interface.MonitoredSubsystem;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.TestModeManager;
import frc.robot.TestModeManager.TestMode;
import frc.robot.subsystems.scoring.shooter.ShooterMechanism;
import frc.robot.subsystems.scoring.states.IdleState;
import frc.robot.subsystems.scoring.states.InitState;
import frc.robot.subsystems.scoring.states.KickState;
import frc.robot.subsystems.scoring.states.TestModeState;
import frc.robot.subsystems.scoring.states.WaitToScoreState;
import frc.robot.subsystems.scoring.states.WarmupState;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public class ScoringSubsystem extends MonitoredSubsystem {
  private static Optional<ScoringSubsystem> instance = Optional.empty();

  // == MECHANISMS ==

  /** The indexer mechanism, which may or may not be enabled */
  private final Optional<IndexerMechanism> indexer;
  /** The shooter mechanism, which may or may not be enabled */
  private final Optional<ShooterMechanism> shooter;

  // == STATE MACHINE ==
  private enum ScoringState implements StateContainer {
    Init(new InitState(ScoringSubsystem::getInstance)),
    TestMode(new TestModeState(ScoringSubsystem::getInstance)),
    Idle(new IdleState(ScoringSubsystem::getInstance)),
    Warmup(new WarmupState(ScoringSubsystem::getInstance)),
    Kick(new KickState(ScoringSubsystem::getInstance)),
    WaitToScore(new WaitToScoreState(ScoringSubsystem::getInstance));

    private final PeriodicStateInterface state;

    ScoringState(PeriodicStateInterface state) {
      this.state = state;
    }

    @Override
    public PeriodicStateInterface getState() {
      return state;
    }
  }

  /** Triggers for the {@link ScoringSubsystem}'s {@link team401.coppercore.StateMachine} */
  public enum ScoringTrigger {
    /** Fired by the InitState after the indexer is homed successfully */
    Homed,
    /**
     * Fired by the ScoringSubsystem in periodic when in a Scoring tuning mode but not in
     * TestModeState
     */
    ScoringTestModeEntered,
    /** Fired by the TestModeState when no longer in a Scoring test mode */
    ScoringTestModeExited,
    /** Fired by a button binding when the warmup button is pressed */
    WarmupPressed,
    /** Fired by a button binding when the warmup button is released */
    WarmupReleased,
    /**
     * Fired by the WarmupState when the shooter is ready (both motors are at their goal RPM)
     *
     * <p>This should cause a transition to "Kick" state when:
     *
     * <ul>
     *   <li>The manual score button is pressed
     *   <li>Pose-based shooting is enabled and the shot is attainable
     */
    WarmupReady,
    /** Fired by the KickState when the indexer has moved to the top of its range of motion */
    Kicked,
    /** Fired by the WaitToScoreState when the time to wait to score has elapsed */
    WaitedToScore,
  }

  private final StateMachineConfiguration<ScoringState, ScoringTrigger> stateMachineConfiguration;

  private final StateMachine<ScoringState, ScoringTrigger> stateMachine;

  // == SUPPLIERS ==
  private BooleanSupplier shootPressedSupplier = () -> false;

  /**
   * Construct a new ScoringSubsystem
   *
   * <p>This constructor is private so that users are forced to use {@link ScoringSubsystem#create}
   *
   * @param indexer The IndexerMechanism instance to use
   * @param shooter The ShooterMechanism instance to use
   */
  private ScoringSubsystem(Optional<IndexerMechanism> indexer, Optional<ShooterMechanism> shooter) {
    this.indexer = indexer;
    this.shooter = shooter;

    stateMachineConfiguration = new StateMachineConfiguration<>();

    stateMachineConfiguration
        .configure(ScoringState.Init)
        .permitIf(ScoringTrigger.Homed, ScoringState.TestMode, ScoringSubsystem::inScoringTestMode)
        .permit(ScoringTrigger.Homed, ScoringState.Idle);

    stateMachineConfiguration
        .configure(ScoringState.TestMode)
        .permitIf(
            ScoringTrigger.ScoringTestModeExited,
            ScoringState.Idle,
            () -> !ScoringSubsystem.inScoringTestMode());

    stateMachineConfiguration
        .configure(ScoringState.Idle)
        .permit(ScoringTrigger.WarmupPressed, ScoringState.Warmup)
        .permit(ScoringTrigger.ScoringTestModeEntered, ScoringState.TestMode);

    stateMachineConfiguration
        .configure(ScoringState.Warmup)
        .permit(ScoringTrigger.WarmupReleased, ScoringState.Idle)
        // If the score button is pressed, allow transitioning to "Kick"
        .permitIf(ScoringTrigger.WarmupReady, ScoringState.Kick, shootPressedSupplier::getAsBoolean)
        // If pose-based-shooting is enabled, allow transitioning to "Kick"
        .permitIf(
            ScoringTrigger.WarmupReady,
            ScoringState.Kick,
            () -> isPoseBasedShootingEnabled() && isShotAttainable());

    stateMachineConfiguration
        .configure(ScoringState.Kick)
        .permit(ScoringTrigger.Kicked, ScoringState.WaitToScore);

    stateMachineConfiguration
        .configure(ScoringState.WaitToScore)
        .permit(ScoringTrigger.WaitedToScore, ScoringState.Idle);

    // Create the scoring state machine, starting in Init state
    // This does not automatically call InitState.onEntry, therefore it must be called in the create
    // method, since calling it here would be leaking `this` in the constructor.
    stateMachine = new StateMachine<>(stateMachineConfiguration, ScoringState.Init);
  }

  // Create method architecture suggested by OpenAI ChatGPT, although no generated code has been
  // used here.
  /**
   * Create a new ScoringSubsystem, returning the instance created and updating the static instance
   * field.
   *
   * <p>This method exists because updating the instance field with a {@link java.util.Optional}
   * created from `this` would leak a partially initialized object in the constructor. Instead, this
   * method creates the object and <i>then</i> updates the instance by creating an Optional.
   *
   * @param indexer The IndexerMechanism instance to use
   * @param shooter The ShooterMechanism instance to use
   * @return The newly created ScoringSubsystem instance
   */
  public static ScoringSubsystem create(
      Optional<IndexerMechanism> indexer, Optional<ShooterMechanism> shooter) {
    if (instance.isPresent()) {
      throw new Error("ScoringSubsystem was created more than once.");
    }

    ScoringSubsystem createdInstance = new ScoringSubsystem(indexer, shooter);

    instance = Optional.of(createdInstance);

    createdInstance.stateMachine.getCurrentState().state.onEntry(null);

    return createdInstance;
  }

  /**
   * Initialize the Shooter mechanism's drive pose supplier to use for pose-based shots
   *
   * <p>If shooter isn't run, this method is a no-op
   *
   * @param newPoseSupplier A Supplier for a Pose2d that supplies the drivetrain's current odometry
   *     pose
   */
  public void initializeShooterPoseSupplier(Supplier<Pose2d> newPoseSupplier) {
    shooter.ifPresent(shooter -> shooter.initializePoseSupplier(newPoseSupplier));
  }

  /**
   * Initialize the supplier for whether or not the shoot button is pressed.
   *
   * @param newShootPressedSupplier A BooleanSupplier that returns true when shoot is pressed and
   *     false when it is not.
   */
  public void initializeShootPressedSupplier(BooleanSupplier newShootPressedSupplier) {
    shootPressedSupplier = newShootPressedSupplier;
  }

  /**
   * Get an Optional containing the current ScoringSubsystem instance.
   *
   * <p>If the ScoringSubsystem has not been instantiated, this will be an empty optional
   *
   * @return An Optional that will contain the ScoringSubsystem if it has been instantiated
   */
  public static Optional<ScoringSubsystem> getInstance() {
    return instance;
  }

  @Override
  public void monitoredPeriodic() {
    if (inScoringTestMode() && stateMachine.getCurrentState() != ScoringState.TestMode) {
      fireTrigger(ScoringTrigger.ScoringTestModeEntered);
    }

    indexer.ifPresent(indexer -> indexer.periodic());
    shooter.ifPresent(shooter -> shooter.periodic());

    stateMachine.periodic();
  }

  public void testPeriodic() {
    shooter.ifPresent(shooter -> shooter.testPeriodic());
  }

  /**
   * Fire a trigger for the ScoringSubsystem and its state machine
   *
   * @param trigger The trigger to fire
   */
  public void fireTrigger(ScoringTrigger trigger) {
    stateMachine.fire(trigger);
  }

  /**
   * Check whether or not the robot is currently in a Scoring test mode.
   *
   * <p>If the robot is not in test mode, or test mode chooser hasn't been initialized, this method
   * will return false.
   *
   * <p>The list of test modes that qualify as scoring test modes is as followed:
   *
   * <ul>
   *   <li>Shooter Closed Loop Tuning
   *   <li>Shooter Current Open-Loop Tuning
   *   <li>Shooter Voltage Open-Loop Tuning
   * </ul>
   *
   * @return True if the robot is enabled in test mode and a scoring test mode is selected, false if
   *     not
   */
  public static boolean inScoringTestMode() {
    return TestModeManager.getTestMode() == TestMode.ShooterClosedLoopTuning
        || TestModeManager.getTestMode() == TestMode.ShooterCurrentTuning
        || TestModeManager.getTestMode() == TestMode.ShooterVoltageTuning;
  }

  // == SHOOTER PASSTHROUGH ==
  /**
   * Warm up the shooter
   *
   * <p>If the shooter isn't enabled in ScoringFeatureFlags, this is a no-op
   */
  public void warmupShooter() {
    shooter.ifPresent(shooter -> shooter.warmUp());
  }

  /**
   * Stop the shooter
   *
   * <p>If the shooter isn't enabled in ScoringFeatureFlags, this is a no-op
   */
  public void stopShooter() {
    shooter.ifPresent(shooter -> shooter.stop());
  }

  /**
   * Can the state machine automatically transition from Warmup to Kick when ready?
   *
   * <p>This is true when pose-based-shooting is enabled in the shooter.
   *
   * <p>If the shooter isn't enabled in ScoringFeatureFlags, this will return false.
   *
   * @return True if pose-based-shooting is enabled, false if not (or if the shooter doesn't exist).
   */
  public boolean canAutoKick() {
    return shooter.map(shooter -> shooter.isPoseBasedShootingEnabled()).orElse(false);
  }

  /**
   * Check if the shooter flywheels are at their goal speeds.
   *
   * <p>This returns true if the shooter is disabled in ScoringFeatureFlags
   *
   * @return
   */
  public boolean isShooterReady() {
    return shooter.map(shooter -> shooter.atGoalSpeeds()).orElse(true);
  }

  /**
   * Check if pose-based shooting is enabled in the shooter mechanism
   *
   * <p>This defaults to false if the shooter is disabled in ScoringFeatureFlags
   *
   * @return True if pose-based shooting is enabled, false if it's disabled or the shooter doesn't
   *     exist
   */
  public boolean isPoseBasedShootingEnabled() {
    return shooter.map(shooter -> shooter.isPoseBasedShootingEnabled()).orElse(false);
  }

  /**
   * Is the shooter shot currently attainable?
   *
   * <p>Defaults to false if the shooter is disabled in ScoringFeatureFlags.
   *
   * @see frc.robot.subsystems.scoring.shooter.ShooterMechanism#isShotAttainable
   * @return
   */
  private boolean isShotAttainable() {
    return shooter.map(shooter -> shooter.isShotAttainable()).orElse(false);
  }

  // == INDEXER PASSTHROUGH ==
  /**
   * Commands the indexer to control to its idle position
   *
   * <p>If the indexer isn't enabled in ScoringFeatureFlags, this is a no-op
   *
   * <p>This method should ONLY be called by scoring states or test modes
   */
  public void stopIndexing() {
    indexer.ifPresent(indexer -> indexer.stopIndexing());
  }

  /**
   * Commands the indexer to control to its top/indexing position
   *
   * <p>If the indexer isn't enabled in ScoringFeatureFlags, this is a no-op
   *
   * <p>This method should ONLY be called by scoring states or test modes
   */
  public void indexIntoShooter() {
    indexer.ifPresent(indexer -> indexer.indexIntoShooter());
  }

  /**
   * Commands the indexer to drive downward to home into the bottom hardstop
   *
   * <p>If the indexer isn't enabled in ScoringFeatureFlags, this is a no-op
   *
   * <p>This method should ONLY be called by scoring states or test modes
   */
  public void startHomingIndexer() {
    indexer.ifPresent(indexer -> indexer.startHoming());
  }

  /**
   * Checks whether or not the indexer is currently moving.
   *
   * @return True if the indexer is moving, false if the indexer isn't moving or doesn't exist
   */
  public boolean isIndexerMoving() {
    return indexer.map(indexer -> indexer.isMoving()).orElse(false);
  }

  /**
   * Seed the indexer's position measurement at the bottom of its range of motion
   *
   * <p>This method should be called by InitState when it is sure that the indexer is touching the
   * bottom hardstop.
   *
   * <p>If the indexer isn't enabled in ScoringFeatureFlags, this is a no-op
   *
   * <p>This method should ONLY be called by scoring states or test modes
   */
  public void seedIndexerAtBottom() {
    indexer.ifPresent(indexer -> indexer.seedAtBottom());
  }

  /**
   * Checks whether the indexer's position measurement has been seeded/initialized
   *
   * <p>If this value is false, the indexer's position is unknown and closed-loop control cannot
   * safely be used.
   *
   * <p>If the Indexer is disabled in ScoringFeatureFlags, this will default to true since no homing
   * action needs to occur.
   *
   * @return True if the indexer has been seeded or doesn't exist, false if the indexer has not been
   *     seeded.
   */
  public boolean hasIndexerSeeded() {
    return indexer.map(indexer -> indexer.hasBeenSeeded()).orElse(true);
  }

  /**
   * Check whether the indexer is at the top of its range of motion.
   *
   * <p>When the indexer is disabled in ScoringFeatureFlags, this value defaults to true.
   *
   * @see frc.robot.subsystems.scoring.IndexerMechanism#isAtTop()
   * @return True if the indexer is at the top of its range of motion or is disabled, false if the
   *     indexer is not at the top of its range of motion.
   */
  public boolean hasIndexerKicked() {
    return indexer.map(indexer -> indexer.isAtTop()).orElse(true);
  }
}
