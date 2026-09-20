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

    @Override
    public void periodic() {
        io.updateInputs(inputs);
    }

    public Command setIndexerVolts(double volts) {
        return this.run(() -> io.setIndexerVolts(volts));
    }

    public Command setIndexerVelocity(double targetRPM) {
    return this.runEnd(
        () -> {
            double measuredRPM = inputs.indexerFLVelocity;

            double feedforwardVolts = velocityFeedforward.calculate(targetRPM);
            double feedbackVolts = velocityPID.calculate(measuredRPM, targetRPM);

            double outputVolts = MathUtil.clamp(
                feedforwardVolts + feedbackVolts,
                -12.0,
                12.0);

            io.setIndexerVolts(outputVolts);
        },
        () -> {
            io.setIndexerVolts(0.0);
            velocityPID.reset();
        });
}

    public Command setIndexerVelocityWhenLoaded(double targetRPM) {
        return this.runEnd(
            () -> setIndexerVelocity(
                inputs.indexerRangeDistanceMeters < Constants.IndexerConstants.indexerRangeThresholdMeters
                    ? targetRPM
                    : 0.0),
            () -> io.setIndexerVolts(0.0));
    }
    


}