// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import frc.robot.stateSpace.StateSpaceConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class StateSpaceConstants {
        public static final double k_dt = 0.02;
        public static final double k_maxVoltage = 5.0;
    }

    public static final class ShoulderConstants {
        public final static int shoulderMotorPort = 6;

        public final static int shoulderCancoderPort = 8;

        public final static double shoulderGearRatio = 100.0;
        public final static double shoulderMomentOfInertia = 6.5; // kg m^2

        public final static Vector<N2> k_stateStdDevs = VecBuilder.fill(0.2, 0.4);

        public final static Vector<N2> k_qelms = VecBuilder.fill(0.05, 0.1);
        public final static Vector<N1> k_relms = VecBuilder.fill(5.0);

        public final static String k_shoulderName = "Shoulder";

        public final static StateSpaceConfig<N2, N1, N1> k_shoulderControllerConfig = new StateSpaceConfig<N2, N1, N1>(
            LinearSystemId.createSingleJointedArmSystem(TalonFXConstants.TalonFXDCMotor, shoulderMomentOfInertia, shoulderGearRatio),
            k_stateStdDevs,
            TalonFXConstants.k_outputStdDevs,
            k_qelms,
            k_relms,
            Nat.N2(),
            Nat.N1(),
            k_shoulderName
        );

        /* These 2 values were genreated from the "zero" positions of the motors. */
        // public final static double shoulderOffset = -0.1650390625;
    }

    public static final class WristConstants {
        public final static int wristMotorPort = 7;
        public final static int wristCancoderPort = 9;

        public final static double wristGearRatio = 100.0;
        public final static double wristMomentOfInertia = 0.088126; // kg m^2

        public final static Vector<N2> k_stateStdDevs = VecBuilder.fill(0.2, 0.4);

        public final static Vector<N2> k_qelms = VecBuilder.fill(0.05, 0.1);
        public final static Vector<N1> k_relms = VecBuilder.fill(8.0);

        public final static String k_wristName = "Wrist";

        public final static StateSpaceConfig<N2, N1, N1> k_wristControllerConfig = new StateSpaceConfig<N2, N1, N1>(
            LinearSystemId.createSingleJointedArmSystem(TalonFXConstants.TalonFXDCMotor, wristMomentOfInertia, wristGearRatio),
            k_stateStdDevs,
            TalonFXConstants.k_outputStdDevs,
            k_qelms,
            k_relms,
            Nat.N2(),
            Nat.N1(),
            k_wristName
        );

        public final static double wristOffset = -0.5107421875;

    }

    public static final class TalonFXConstants {
        public final static double nominalVoltageVolts = 12.0; // DC Volts
        public final static double stallTorqueNewtonMeters = 4.69; // Nm
        public final static double stallCurrentAmps = 257.0; // Amps
        public final static double freeCurrentAmps = 1.5; // Amps
        public final static double freeSpeedRadPerSec = 6380.0 * 2.0 * Math.PI / 60.0; // RPM * 2pi / 60 = Rad per second

        public final static double positionStdDevs = 1.0 / 2048.0; // rotations
        public final static double velocityStdDevs = 2.0 / 2048.0; // rotations

        public final static DCMotor TalonFXDCMotor = new DCMotor(nominalVoltageVolts, stallTorqueNewtonMeters, stallCurrentAmps, freeCurrentAmps, freeSpeedRadPerSec, 1);

        public final static Vector<N1> k_outputStdDevs = VecBuilder.fill(TalonFXConstants.positionStdDevs);

    }
}
