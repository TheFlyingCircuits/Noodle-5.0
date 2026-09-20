package frc.robot.subsystems.indexer;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.indexer.IndexerIO.IndexerIOInputs;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputs inputs = new IndexerIOInputs();

    private final PIDController velocityPID = new PIDController(
    Constants.IndexerConstants.velocityKpVoltsPerRPM,
    Constants.IndexerConstants.velocityKiVoltsPerRPMSecond,
    Constants.IndexerConstants.velocityKdVoltsPerRPMPerSecond);

private final SimpleMotorFeedforward velocityFeedforward =
    new SimpleMotorFeedforward(
        Constants.IndexerConstants.velocityKsVolts,
        Constants.IndexerConstants.velocityKvVoltsPerRPM);

    public Indexer(IndexerIO io) {
        this.io = io;
    }

    // gets periodically called every 20 ms
    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command setIndexerVolts(double volts) {
        return this.run(() -> io.setIndexerVolts(volts));
    }

    /**
     * Uses feed forward and PID to calculate voltage from input target velocity
     *
     * @param targetRPM The desired RPM of the indexer motors
     * @return A runnable command that runs the velocity
     */
    public Command setIndexerVelocity(double targetRPM) {
    return this.run(
        () -> {
            double measuredRPM = inputs.indexerFLVelocity;

            double feedforwardVolts = velocityFeedforward.calculate(targetRPM);
            double feedbackVolts = velocityPID.calculate(measuredRPM, targetRPM);

            double outputVolts = MathUtil.clamp(
                feedforwardVolts + feedbackVolts,
                -8.0,
                8.0);

            io.setIndexerVolts(outputVolts);
        });
    }

    /**
     * default indexer command that periodically checks if the can range sensor is within tolerance and if so our hopper is full enougth to fill our feeder with fuel.
     * If the fuel is not within tolerance of the can range sensor then the indexer voltage will be set to 0.
     *
     * @param targetRPM The desired RPM of the indexer motors
     * @return A runnable command that will fill out feeder with fuel if in tolerance
     */
    public Command setIndexerVelocityWhenLoaded(double targetRPM) {
        return this.run(
            () -> setIndexerVelocity(
                inputs.indexerRangeDistanceMeters < Constants.IndexerConstants.indexerRangeThresholdMeters
                    ? targetRPM
                    : 0.0));
    }
    


}