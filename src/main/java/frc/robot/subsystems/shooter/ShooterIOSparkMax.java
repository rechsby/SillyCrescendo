package frc.robot.subsystems.shooter;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;

public class ShooterIOSparkMax implements ShooterIO {
  private CANSparkMax topMotor = new CANSparkMax(28, MotorType.kBrushless);
  private CANSparkMax bottomMotor = new CANSparkMax(29, MotorType.kBrushless);
  private PIDController topMotorPID = new PIDController(0.0, 0.0, 0.0);
  private PIDController bottomMotorPID = new PIDController(0.0, 0.0, 0.0);
  private double topMotorAppliedVolts = 0.0;
  private double bottomMotorAppliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.topMotorVelocityRotPerMin = topMotor.getEncoder().getVelocity();
    inputs.bottomMotorVelocityRotPerMin = bottomMotor.getEncoder().getVelocity();

    topMotorAppliedVolts =
        MathUtil.clamp(topMotorPID.calculate(inputs.topMotorAppliedVolts), -12, 12);
    bottomMotorAppliedVolts =
        MathUtil.clamp(bottomMotorPID.calculate(inputs.bottomMotorAppliedVolts), -12, 12);

    inputs.topMotorAppliedVolts = topMotorAppliedVolts;
    inputs.bottomMotorAppliedVolts = bottomMotorAppliedVolts;

    setTopMotorVoltage(bottomMotorAppliedVolts);
    setBottomMotorVoltage(bottomMotorAppliedVolts);

    inputs.currentAmps = new double[] {topMotor.getOutputCurrent(), bottomMotor.getOutputCurrent()};
  }

  @Override
  public void setTopMotorVoltage(double volts) {
    topMotor.setVoltage(volts);
  }

  @Override
  public void setBottomMotorVoltage(double volts) {
    bottomMotor.setVoltage(volts);
  }

  @Override
  public void stop() {
    topMotor.stopMotor();
    bottomMotor.stopMotor();
  }

  @Override
  public void setTopMotorVelocity(double velocityRotPerMin) {
    topMotorPID.setSetpoint(velocityRotPerMin);
  }

  @Override
  public void setBottomMotorVelocity(double velocityRotPerMin) {
    bottomMotorPID.setSetpoint(velocityRotPerMin);
  }
}
