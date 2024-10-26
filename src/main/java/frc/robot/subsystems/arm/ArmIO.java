package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import frc.robot.Constants.ArmConstants;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean isAtSetpoint = false;
    public double currentAngle = ArmConstants.MIN_ANGLE.in(Degrees);
    public double appliedVolts = 0.0;
    public double[] currentAmps = new double[] {};
    public double setpoint = 0.0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setSetpoint(Measure<Angle> setpoint) {}

  public default void configurePID(double kP, double kI, double kD, double setpointDeadband) {}
}
