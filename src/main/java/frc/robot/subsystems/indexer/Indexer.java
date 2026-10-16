package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Indexer extends SubsystemBase {
    private final IndexerIO io;
    private final IndexerIOInputsAutoLogged inputs;

    private boolean isLoadingFuel = false;

    private Timer indexerTimer;

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
        inputs = new IndexerIOInputsAutoLogged();

        indexerTimer = new Timer();
        indexerTimer.start();
    }

    // gets periodically called every 20 ms
    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("indexerInputs", inputs);
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

        // // bang bang
        // double feedbackVolts = 0;
        // if((targetRPS - measuredRPS) > 0.0) {
        //     feedbackVolts = 999;
        // }

        double outputVolts = MathUtil.clamp(feedforwardVolts + feedbackVolts, -9, 9);
        io.setIndexerVolts(outputVolts);
    }

    public void periodicallyLoadFuel(double timeInterval) {
        if((indexerTimer.get() > timeInterval) || isLoadingFuel) {
            // at the start of the loading sequence restart the timer so the time interval for loading starts at 0 sec
            if(!isLoadingFuel) {
                indexerTimer.restart();
                isLoadingFuel = true;
            }

            double currentTime = indexerTimer.get();
            stopIndexing();

            if(currentTime <= 0.25) {
                // setIndexerVelocity(10);
            } else if (currentTime <= 0.28) {
                // setIndexerVelocity(-8);
            } else {
                // this will stop the loading sequence
                indexerTimer.restart();
                isLoadingFuel = false;
            }
        } else {
            stopIndexing();
        }
        Logger.recordOutput("Loading test", isLoadingFuel);
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

    public Command setIndexerVoltsCommand(double volts) {
        return this.run(
            () -> io.setIndexerVolts(volts));
    }

    public Command periodicallyLoadFuelCommand(double period) {
        return this.run(() -> periodicallyLoadFuel(period));
    }


}