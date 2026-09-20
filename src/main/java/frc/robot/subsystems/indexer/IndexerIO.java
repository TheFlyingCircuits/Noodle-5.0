package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
    @AutoLog 
    public class IndexerIOInputs {
        public double indexerFLVelocity = 0.0;
        public double indexerFRVelocity = 0.0;
        public double indexerBLVelocity = 0.0;
        public double indexerBRVelocity = 0.0;

        public double indexerFLVolts = 0.0;
        public double indexerFRVolts = 0.0;
        public double indexerBLVolts = 0.0;
        public double indexerBRVolts = 0.0;

        public double indexerFLAmps = 0.0;
        public double indexerFRAmps = 0.0;
        public double indexerBLAmps = 0.0;
        public double indexerBRAmps = 0.0;

        public double indexerRangeDistanceMeters = 0.0;
    }

    public default void updateInputs(IndexerIOInputs inputs) {}

    public default void setIndexerVelocity(double velocity) {}

    public default void setIndexerVolts(double volts) {
    }

}
