package frc.robot.subsystems.scoring; 

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations; 
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import edu.wpi.first.units.AngularAccelerationUnit; 
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutAngularVelocity;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog 
  public static class IndexerInputs {
  /**
    * Whether or not the indexer CANcoder is connected
    *
    *<p> This value is determined by whether or not refreshing the Position status signal returns an OK signal
    */
    
  public boolean isIndexerEncoderConnected = false; 

  /** Current indexer position as reported by the CANcoder */
  public MutAngle indexerPosition = Rotations.mutable(0.0);

  /** Current velocity of the indexer as reported by the CANcoder */
  public MutAngularVelocity indexerVelocity = RotationsPerSecond.mutable(0.0);

  /** Goal angle of the indexer, as seen by the indexer CANcoder */
  public MutAngle indexerGoalPosition = Rotations.mutable(0.0);
  
