package frc.robot.subsystems.scoring;

public class IndexerMechanism {
  /**
   * This method should be called in each periodic loop by the ScoringSubsystem. It will NOT run
   * automatically.
   */
  public void periodic() {
    // TODO: Implement periodic
  }

  public void indexIntoShooter() {
    // TODO: Implement indexing into shooter
  }

  public void stopIndexing() {
    // TODO: Implement returning indexer to zero
  }

  public void startHoming() {}

  /**
   * Check whether or not the indexer is moving.
   * 
   * <p> The velocity threshold for movement will be defined in IndexerConstants
   * @return True if the indexer's velocity is above the threshold, false if not
   */
  public boolean isMoving() {
    // TODO: Implement isMoving
    return false;
  }

  public void seedAtBottom() {
    // TODO Implement seedAtBottom
    throw new UnsupportedOperationException("Unimplemented method 'seedAtBottom'");
  }

  public boolean hasBeenSeeded() {
    // TODO: Implement hasBeenSeeded
    return false;
  }
}
