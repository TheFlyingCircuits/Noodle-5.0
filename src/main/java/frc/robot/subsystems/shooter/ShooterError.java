package frc.robot.subsystems.shooter;

/**
 * This record is used to store the error for the shooter components.
 * @param velocityErrorRPS The error of the drum shooter velocity in RPS from target-current.
 * @param angleErrorDeg The error of the pivot in degrees from target-current.
 */
public record ShooterError(double velocityErrorRPS, double angleErrorDeg) {};