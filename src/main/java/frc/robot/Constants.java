// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import swervelib.math.Matter;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean constants. This
 * class should not be used for any other purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants
{





//Controller Buttons//Controller buttons
  public static final         CommandXboxController driverXbox = new CommandXboxController(1);

public static final int A_Button = 1;
public static final int B_Button = 2;
public static final int X_Button = 3;
public static final int Y_Button = 4;
public static final int leftBumper = 5;
public static final int rightBumper = 6;






//added down

public static final class CoralSubsystemConstants {
  public static final int kElevatorMotorCanId = 31;
  public static final int kArmMotorCanId = 34;
  public static final int kIntakeMotorCanId = 35;

  public static final class ElevatorSetpoints {
    public static final int kFeederStation = 0;
    public static final int kLevel1 = 0;
    public static final int kLevel2 = 0;
    public static final int kLevel3 = 100;
    public static final int kLevel4 = 150;
  }

  public static final class ArmSetpoints {
    public static final double kFeederStation = -33;
    public static final double kLevel1 = 0;
    public static final double kLevel2 = -2;
    public static final double kLevel3 = -2;
    public static final double kLevel4 = -19;
  }

  public static final class IntakeSetpoints {
    public static final double kForward = 0.5;
    public static final double kReverse = -0.5;
  }
}

public static final class AlgaeSubsystemConstants {
  public static final int kIntakeMotorCanId = 33;
  public static final int kPivotMotorCanId = 32;

  public static final class ArmSetpoints {
    public static final double kStow = -4.178;
    public static final double kHold = -6.203;
    public static final double kDown = 0;
  }

  public static final class IntakeSetpoints {
    public static final double kForward = 0.5;
    public static final double kReverse = -0.5;
    public static final double kHold = 0.25;
  }
}

//added up


public static final double ROBOT_MASS = (148 - 20.3) * 0.453592; // 32lbs * kg per pound
public static final Matter CHASSIS    = new Matter(new Translation3d(0, 0, Units.inchesToMeters(8)), ROBOT_MASS);
public static final double LOOP_TIME  = 0.13; //s, 20ms + 110ms sprk max velocity lag
public static final double MAX_SPEED  = Units.feetToMeters(14.5);
// Maximum speed of the robot in meters per second, used to limit acceleration.

//  public static final class AutonConstants
//  {
//
//    public static final PIDConstants TRANSLATION_PID = new PIDConstants(0.7, 0, 0);
//    public static final PIDConstants ANGLE_PID       = new PIDConstants(0.4, 0, 0.01);
//  }


public static final class DrivebaseConstants
{

  // Hold time on motor brakes when disabled
  public static final double WHEEL_LOCK_TIME = 10; // seconds
}

public static class OperatorConstants
{

  // Joystick Deadband
  public static final double DEADBAND        = 0.1;
  public static final double LEFT_Y_DEADBAND = 0.1;
  public static final double RIGHT_X_DEADBAND = 0.1;
  public static final double TURN_CONSTANT    = 6;
}


//more added down

public static final class SimulationRobotConstants {
  public static final double kPixelsPerMeter = 20;

  public static final double kElevatorGearing = 25; // 25:1
  public static final double kCarriageMass =
      4.3 + 3.15 + 0.151; // Kg, arm + elevator stage + chain
  public static final double kElevatorDrumRadius = 0.0328 / 2.0; // m
  public static final double kMinElevatorHeightMeters = 0.922; // m
  public static final double kMaxElevatorHeightMeters = 1.62; // m

  public static final double kArmReduction = 60; // 60:1
  public static final double kArmLength = 0.433; // m
  public static final double kArmMass = 4.3; // Kg
  public static final double kMinAngleRads =
      Units.degreesToRadians(-50.1); // -50.1 deg from horiz
  public static final double kMaxAngleRads =
      Units.degreesToRadians(40.9 + 180); // 40.9 deg from horiz

  public static final double kIntakeReduction = 135; // 135:1
  public static final double kIntakeLength = 0.4032262; // m
  public static final double kIntakeMass = 5.8738; // Kg
  public static final double kIntakeMinAngleRads = Units.degreesToRadians(80);
  public static final double kIntakeMaxAngleRads = Units.degreesToRadians(180);
  public static final double kIntakeShortBarLength = 0.1524;
  public static final double kIntakeLongBarLength = 0.3048;
  public static final double kIntakeBarAngleRads = Units.degreesToRadians(-60);
}


public static final class ModuleConstants {
  // The MAXSwerve module can be configured with one of three pinion gears: 12T,
  // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
  // more teeth will result in a robot that drives faster).
  public static final int kDrivingMotorPinionTeeth = 14;

  // Calculations required for driving motor conversion factors and feed forward
  public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
  public static final double kWheelDiameterMeters = 0.0762;
  public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
  // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
  // teeth on the bevel pinion
  public static final double kDrivingMotorReduction =
      (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
  public static final double kDriveWheelFreeSpeedRps =
      (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters) / kDrivingMotorReduction;
}
public static final class NeoMotorConstants {
  public static final double kFreeSpeedRpm = 5676;
}

public static final class OIConstants {
  public static final int kDriverControllerPort = 0;
  public static final double kDriveDeadband = 0.1;
  public static final double kTriggerButtonThreshold = 0.2;
}

//added up

}