package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;
import frc.robot.VendorWrappers.Neo;

public class IndexerIOSim implements IndexerIO {
    private static final double MAX_VOLTAGE = 12.0;
    private static final double FREE_SPEED_RPS = 5800.0 / 60.0;

    private double commandedVolts;
    private double velocityRPS;
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

        double targetVelocityRPS = commandedVolts / MAX_VOLTAGE * FREE_SPEED_RPS;
        double velocityChange = targetVelocityRPS - velocityRPS;
        double responseFactor = 1.0 - Math.exp(-deltaTimeSeconds / Constants.IndexerConstants.simVelocityTimeConstantSeconds);
        velocityRPS += velocityChange * responseFactor;

        double appliedVolts = commandedVolts;
        double currentAmps = Math.abs(appliedVolts) / MAX_VOLTAGE * Neo.freeCurrent;

        inputs.indexerVelocity = velocityRPS;

        inputs.indexerVolts = appliedVolts;

        inputs.indexerAmps = currentAmps;
    }

    public void setRangeDistanceMeters(double distanceMeters) {
        rangeDistanceMeters = distanceMeters;
    }
}
