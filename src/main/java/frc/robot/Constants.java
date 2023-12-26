// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = 0.08;
    public static final double kCubic = 0.95;
    public static final double kLinear = 0.05;
  }

  public static final class CurrentLimit {
    public static final int kDrive = 60;
    public static final int kAzimuth = 20;
  }

  public static final class ModuleConstants {
    public static final class Drive {
      public static final double kGearRatio = (36.0 / 14.0) * (18.0 / 24.0) * (45.0 / 15.0);
      public static final double kWheelDiameter = 0.096706557;
      public static final double kPositionFactor = (1.0 / kGearRatio) * kWheelDiameter * Math.PI;
      public static final double kVelocityFactor = kPositionFactor / 60.0;
      public static final double kNEOMaxSpeed = 5820.0;
      public static final double kp = 0.15;
      public static final double ks = 0.01;
      public static final double kv = 1.0 / (kNEOMaxSpeed * kVelocityFactor);
    }

    public static final class Aziumth {
      public static final double kGearRatio = (50.0 / 14.0) * (72.0 / 14.0);
      public static final double kPositionFactor = 1 / (kGearRatio) * 2 * Math.PI;
      public static final double kVelocityFactor = kPositionFactor / 60.0;
      public static final double kp = 3.00;
    }

  }

  public static final class DriveConstants {

    public static final double kWheelBaseWidth = 0.4826;
    public static final double kWheelBaseLength = 0.4826;

    public static final class FrontLeft {
      public static final int kDrive = 1;
      public static final int kAziumth = 2;
      public static final int kAbsEnc = 1;
      public static final double kOffset = -2.9630;
      public static final Translation2d kLocation = new Translation2d(kWheelBaseLength / 2, kWheelBaseWidth / 2);
    }

    public static final class FrontRight {
      public static final int kDrive = 3;
      public static final int kAziumth = 4;
      public static final int kAbsEnc = 2;
      public static final double kOffset = 1.2626;
      public static final Translation2d kLocation = new Translation2d(kWheelBaseLength / 2, -kWheelBaseWidth / 2);
    }

    public static final class RearLeft {
      public static final int kDrive = 7;
      public static final int kAziumth = 8;
      public static final int kAbsEnc = 0;
      public static final double kOffset = -1.582;
      public static final Translation2d kLocation = new Translation2d(-kWheelBaseLength / 2, kWheelBaseWidth / 2);
    }

    public static final class RearRight {
      public static final int kDrive = 5;
      public static final int kAziumth = 6;
      public static final int kAbsEnc = 3;
      public static final double kOffset = -2.986;
      public static final Translation2d kLocation = new Translation2d(-kWheelBaseLength / 2, -kWheelBaseWidth / 2);
    }

    public static final double kTransSlewRate = 14.0;
    public static final double kRotSlewRate = 16.0;

    public static final double kMaxSpeedMetersPerSecond = 4.9;
    public static final double kMaxAngularSpeed = 2 * Math.PI;
    public static final double kMaxAngularAccel = 1.5 * Math.PI;

    public static final class KeepAngle {
      public static final double kp = 0.30;
      public static final double ki = 0.0;
      public static final double kd = 0.0;
    }

    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(FrontLeft.kLocation,
        FrontRight.kLocation, RearLeft.kLocation, RearRight.kLocation);

    public static final double kRotTransFactor = 0.045;
  }

  public static final class GlobalConstants {
    public static final double kVoltCompensation = 12.6; // Sets a voltage compensation value ideally 12.6V
    public static final int PCHID = 20;
    public static final int PDHID = 24;
    public static final double kLoopTime = 0.020;
  }
}
