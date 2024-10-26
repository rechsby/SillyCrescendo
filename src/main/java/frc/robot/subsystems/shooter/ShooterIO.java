package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double topMotorVelocityRotPerMin = 0.0;
    public double bottomMotorVelocityRotPerMin = 0.0;
    public double topMotorAppliedVolts = 0.0;
    public double bottomMotorAppliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public boolean isAtSetpoint = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Run open loop at the specified voltage. */
  public default void setTopMotorVoltage(double volts) {}

  public default void setBottomMotorVoltage(double volts) {}

  /** Run closed loop at the specified velocity. */
  public default void setTopMotorVelocity(double velocityRotPerMin) {}

  public default void setBottomMotorVelocity(double velocityRotPerMin) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants. */
  public default void configureTopMotorPID(
      double kP, double kI, double kD, double setpointDeadband) {}

  public default void configureBottomMotorPID(
      double kP, double kI, double kD, double setpointDeadband) {}
}
