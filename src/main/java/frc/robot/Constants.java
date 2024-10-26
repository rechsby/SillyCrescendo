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

package frc.robot;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final Mode currentMode = Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public static class IntakeConstants {
    public static final double FRONT_MOTOR_GEAR_RATIO = 1;
  }

  public static class TransportConstants {
    public static final double TRANSPORT_SPEED = 100; // -100% - 100%
    public static final double GEAR_RATIO = 1.5;
  }

  public static class ShooterConstants {
    public static final double RPM_DEADBAND = 200;
  }

  public static class ArmConstants {

    public static final Measure<Angle> MIN_ANGLE = Degrees.of(26);
    public static final Measure<Angle> MAX_ANGLE = Degrees.of(63);
  }
}
