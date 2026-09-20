package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.VendorWrappers.Neo;

public class IndexerIOSim implements IndexerIO {
    private static final double MAX_VOLTAGE = 12.0;
    private static final double FREE_SPEED_RPM = 5820.0;

    private double commandedVolts;
    private double velocityRPM;
    private double rangeDistanceMeters = 1.0;
    private double lastUpdateTimestamp = Timer.getFPGATimestamp();

    @Override
    public void setIndexerVolts(double volts) {
        commandedVolts = Math.max(-MAX_VOLTAGE, Math.min(MAX_VOLTAGE, volts));
    }

    @Override
    public void updateInputs(IndexerIOInputs inputs) {
        double currentTimestamp = Timer.getFPGATimestamp();
        double deltaTimeSeconds = Math.max(0.0, currentTimestamp - lastUpdateTimestamp);
        lastUpdateTimestamp = currentTimestamp;

        double targetVelocityRPM = commandedVolts / MAX_VOLTAGE * FREE_SPEED_RPM;
        double velocityChange = targetVelocityRPM - velocityRPM;
        double responseFactor = 1.0 - Math.exp(-deltaTimeSeconds / Constants.IndexerConstants.simVelocityTimeConstantSeconds);
        velocityRPM += velocityChange * responseFactor;

        double appliedVolts = commandedVolts;
        double currentAmps = Math.abs(appliedVolts) / MAX_VOLTAGE * Neo.freeCurrent;

        inputs.indexerFLVelocity = velocityRPM;
        inputs.indexerFRVelocity = velocityRPM;
        inputs.indexerBLVelocity = velocityRPM;
        inputs.indexerBRVelocity = velocityRPM;

        inputs.indexerFLVolts = appliedVolts;
        inputs.indexerFRVolts = appliedVolts;
        inputs.indexerBLVolts = appliedVolts;
        inputs.indexerBRVolts = appliedVolts;

        inputs.indexerFLAmps = currentAmps;
        inputs.indexerFRAmps = currentAmps;
        inputs.indexerBLAmps = currentAmps;
        inputs.indexerBRAmps = currentAmps;

        inputs.indexerRangeDistanceMeters = rangeDistanceMeters;
    }

    public void setRangeDistanceMeters(double distanceMeters) {
        rangeDistanceMeters = distanceMeters;
    }
}
