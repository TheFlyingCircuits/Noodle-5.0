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
    Constants.IndexerConstants.velocityKpVoltsPerRPS,
    Constants.IndexerConstants.velocityKiVoltsPerRPSSecond,
    Constants.IndexerConstants.velocityKdVoltsPerRPSPerSecond);

    private final SimpleMotorFeedforward velocityFeedforward =
        new SimpleMotorFeedforward(
            Constants.IndexerConstants.velocityKsVolts,
            Constants.IndexerConstants.velocityKvVoltsPerRPS);

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
     * @param targetRPS The desired RPS of the indexer motors
     */
    public void setIndexerVelocity(double targetRPS) {
        double measuredRPS = inputs.indexerVelocity;
        double feedforwardVolts = velocityFeedforward.calculate(targetRPS);
        double feedbackVolts = velocityPID.calculate(measuredRPS, targetRPS);

        double outputVolts = MathUtil.clamp(feedforwardVolts + feedbackVolts, -8.0, 8.0);
        io.setIndexerVolts(outputVolts);
    }

    public void stopIndexing() {
        io.setIndexerVolts(0.0);
    }

    /**
     * Uses feed forward and PID to calculate voltage from input target velocity
     *
     * @param targetRPS The desired RPS of the indexer motors
     * @return A runnable command that runs the velocity
     */
    public Command setIndexerVelocityCommand(double targetRPS) {
        return this.run(
            () -> setIndexerVelocity(targetRPS));
    }


}