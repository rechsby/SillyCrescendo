package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class ArmIOSparkMax implements ArmIO {
  private CANSparkMax leftMotor = new CANSparkMax(30, MotorType.kBrushless);
  private CANSparkMax rightMotor = new CANSparkMax(31, MotorType.kBrushless);
  private DutyCycleEncoder absEncoder = new DutyCycleEncoder(3);
  private PIDController controller = new PIDController(0, 0, 0);
  private double appliedVolts = 0.0;

  public ArmIOSparkMax() {
    for (CANSparkMax motor : new CANSparkMax[] {leftMotor, rightMotor}) {
      motor.restoreFactoryDefaults();
      motor.setInverted(true);
      motor.setIdleMode(IdleMode.kBrake);
    }

    leftMotor.follow(rightMotor);

    absEncoder.setDistancePerRotation(360);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {

    appliedVolts = controller.calculate(getAngle());
    rightMotor.setVoltage(appliedVolts);

    inputs.isAtSetpoint = controller.atSetpoint();
    inputs.currentAngle = getAngle();
    inputs.appliedVolts = appliedVolts;
    inputs.currentAmps = new double[] {leftMotor.getOutputCurrent(), rightMotor.getOutputCurrent()};
    inputs.setpoint = controller.getSetpoint();
  }

  private double getAngle() {
    return absEncoder.getDistance() - 281.6;
  }

  @Override
  public void setSetpoint(Measure<Angle> setpoint) {
    controller.setSetpoint(setpoint.in(Degrees));
  }

  @Override
  public void configurePID(double kP, double kI, double kD, double setpointDeadband) {
    controller.setPID(kP, kI, kD);
    controller.setTolerance(setpointDeadband);
  }
}
