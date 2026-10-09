package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog 
    public class IndexerIOInputs {
        public double indexerVelocity = 0.0;

        public double indexerVolts = 0.0;

        public double indexerAmps = 0.0;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void setIndexerVelocity(double velocity) {}

    public default void setIndexerVolts(double volts) {
    }

}
