package frc.robot.subsystems.arm;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  public ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  public Arm(ArmIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
        io.configurePID(0.8, 0, 0, 2);
        break;
      case SIM:
        io.configurePID(0.7, 0, 0.5, 2);
        break;
      default:
        io.configurePID(0.8, 0, 0, 2);
    }
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public void setSetpoint(Measure<Angle> setpoint) {
    io.setSetpoint(setpoint);
  }

  public boolean isAtSetpoint() {
    return inputs.isAtSetpoint;
  }

  public double getAngle() {
    return inputs.currentAngle;
  }
}
