package frc.robot.subsystems.scoring;

import coppercore.wpilib_interface.MonitoredSubsystem;
import java.util.Optional;

public class ScoringSubsystem extends MonitoredSubsystem {
  private static Optional<ScoringSubsystem> instance = Optional.empty();

  private IndexerMechanism indexer;
  private ShooterMechanism shooter;

  /**
   * Construct a new ScoringSubsystem
   *
   * <p>This constructor is private so that users are forced to use {@link ScoringSubsystem#create}
   *
   * @param indexer The IndexerMechanism instance to use
   * @param shooter The ShooterMechanism instance to use
   */
  private ScoringSubsystem(IndexerMechanism indexer, ShooterMechanism shooter) {
    this.indexer = indexer;
    this.shooter = shooter;
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
  public static ScoringSubsystem create(IndexerMechanism indexer, ShooterMechanism shooter) {
    if (instance.isPresent()) {
      throw new Error("ScoringSubsystem was created more than once.");
    }

    ScoringSubsystem createdInstance = new ScoringSubsystem(indexer, shooter);

    instance = Optional.of(createdInstance);

    return createdInstance;
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
  public void monitoredPeriodic() {}
}
