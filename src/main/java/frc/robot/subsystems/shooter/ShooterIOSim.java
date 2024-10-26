// Copyright 2021-2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;

public class ShooterIOSim implements ShooterIO {
  private FlywheelSim bottomMotorSim = new FlywheelSim(DCMotor.getNEO(1), 1.5, 0.004);
  private FlywheelSim topMotorSim = new FlywheelSim(DCMotor.getNEO(1), 1.5, 0.004);
  private PIDController topMotorPID = new PIDController(0.0, 0.0, 0.0);
  private PIDController bottomMotorPID = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double topMotorAppliedVolts = 0.0;
  private double bottomMotorAppliedVolts = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (closedLoop) {
      topMotorAppliedVolts =
          MathUtil.clamp(topMotorPID.calculate(topMotorSim.getAngularVelocityRPM()), -12.0, 12.0);
      topMotorSim.setInputVoltage(topMotorAppliedVolts);

      bottomMotorAppliedVolts =
          MathUtil.clamp(
              bottomMotorPID.calculate(bottomMotorSim.getAngularVelocityRPM()), -12.0, 12.0);
      bottomMotorSim.setInputVoltage(bottomMotorAppliedVolts);
    }
    topMotorSim.update(0.02);
    bottomMotorSim.update(0.02);

    inputs.topMotorVelocityRotPerMin = topMotorSim.getAngularVelocityRPM();
    inputs.bottomMotorVelocityRotPerMin = bottomMotorSim.getAngularVelocityRPM();
    inputs.topMotorAppliedVolts = topMotorAppliedVolts;
    inputs.bottomMotorAppliedVolts = bottomMotorAppliedVolts;
    inputs.currentAmps =
        new double[] {topMotorSim.getCurrentDrawAmps(), bottomMotorSim.getCurrentDrawAmps()};
    inputs.isAtSetpoint = topMotorPID.atSetpoint() && bottomMotorPID.atSetpoint();
  }

  @Override
  public void setTopMotorVoltage(double volts) {
    closedLoop = false;
    topMotorAppliedVolts = volts;
    topMotorSim.setInputVoltage(volts);
  }

  @Override
  public void setBottomMotorVoltage(double volts) {
    closedLoop = false;
    bottomMotorAppliedVolts = volts;
    bottomMotorSim.setInputVoltage(volts);
  }

  @Override
  public void setTopMotorVelocity(double velocityRotPerMin) {
    closedLoop = true;
    topMotorPID.setSetpoint(velocityRotPerMin);
  }

  @Override
  public void setBottomMotorVelocity(double velocityRotPerMin) {
    closedLoop = true;
    bottomMotorPID.setSetpoint(velocityRotPerMin);
  }

  @Override
  public void stop() {
    setTopMotorVoltage(0);
    setBottomMotorVoltage(0);
  }

  @Override
  public void configureTopMotorPID(double kP, double kI, double kD, double setpointDeadband) {
    topMotorPID.setPID(kP, kI, kD);
    topMotorPID.setTolerance(setpointDeadband);
  }

  @Override
  public void configureBottomMotorPID(double kP, double kI, double kD, double setpointDeadband) {
    bottomMotorPID.setPID(kP, kI, kD);
    bottomMotorPID.setTolerance(setpointDeadband);
  }
}
