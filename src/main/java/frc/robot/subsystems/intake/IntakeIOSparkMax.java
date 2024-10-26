package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

public class IntakeIOSparkMax implements IntakeIO {

  private static final double GEAR_RATIO = 1;
  private boolean frontMotorInvert = true;
  private boolean rearMotorInvert = true;

  private final CANSparkMax frontMotor;
  private final CANSparkMax rearMotor;

  public IntakeIOSparkMax() {
    frontMotor = new CANSparkMax(21, MotorType.kBrushless);
    rearMotor = new CANSparkMax(20, MotorType.kBrushless);

    frontMotor.setInverted(frontMotorInvert);
    rearMotor.setInverted(rearMotorInvert);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.frontMotorVelocityRotationsPerMinute =
        frontMotor.getEncoder().getVelocity() / GEAR_RATIO;

    inputs.rearMotorVelocityRotationsPerMinute = rearMotor.getEncoder().getVelocity() / GEAR_RATIO;

    inputs.frontMotorAppliedVolts = frontMotor.getAppliedOutput() * frontMotor.getBusVoltage();
    inputs.rearMotorAppliedVolts = frontMotor.getAppliedOutput() * frontMotor.getBusVoltage();
    inputs.currentAmps = new double[] {frontMotor.getOutputCurrent(), rearMotor.getOutputCurrent()};
  }

  @Override
  public void setVoltage(double volts) {
    frontMotor.setVoltage(volts);
    rearMotor.setVoltage(volts);
  }
}
