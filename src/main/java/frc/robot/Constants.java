// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class ArmConstants {
        public final static int shoulderMotorPort = 6;
        public final static int wristMotorPort = 7;

        public final static int shoulderCancoderPort = 8;
        public final static int wristCancoderPort = 9;

        // I for arm is 6.5 kg m^2

        public final static double shoulderKv = 0.1;
        public final static double shoulderKa = 0.01;
        public final static double wristKv = 0.1;
        public final static double wristKa = 0.01;

        public final static double shoulderI = 100.0;
        public final static double shoulderGearRatio = 100.0;
        public final static double shoulderMomentOfIntertia = 6.5; // kg m^2
        public final static double wristI = 20.0;
        public final static double wristGearRatio = 100.0;
        public final static double wristMomentOfIntertia = 0; // kg m^2

    /*
     * shoulder i = 
     */

        public final static String shoulderName = "Shoulder";
        public final static String wristName = "Wrist";

        /* These 2 values were genreated from the "zero" positions of the motors. */
        public final static double shoulderOffset = -0.1650390625;
        public final static double wristOffset = -0.5107421875;
    }

    public static final class TalonFXConstants {
        public final static double nominalVoltageVolts = 12; // DC Volts
        public final static double stallTorqueNewtonMeters = 4.69; // Nm
        public final static double stallCurrentAmps = 257; // Amps
        public final static double freeCurrentAmps = 1.5; // Amps
        public final static double freeSpeedRadPerSec = 6380 * 2 * Math.PI / 60; // RPM * 2pi / 60 = Rad per second

        public final static DCMotor TalonFXDCMotor = new DCMotor(nominalVoltageVolts, stallTorqueNewtonMeters, stallCurrentAmps, freeCurrentAmps, freeSpeedRadPerSec, 1);
    }
}
