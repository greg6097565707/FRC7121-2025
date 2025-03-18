// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.LimelightHelpers.LimelightTarget_Retro;
import frc.robot.IntakeIR;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.GeometryUtil;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.hardware.TalonFX;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import com.studica.frc.AHRS.BoardYawAxis;
import com.studica.frc.AHRS.BoardAxis;

// import com.kauailabs.navx.frc.AHRS;




public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
 
private BooleanSupplier hasTag() {
  if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0) != -1) {
    return () -> true;
  } else {
    return () -> false;
  }
}
// private BooleanSupplier hasTagAndIsNotAlignedLeft() {
//   if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0)!=-1 && RobotContainer.isNotAlignedLeft().getAsBoolean()) {
//     return (BooleanSupplier) () -> true;
//   } else {
//     return (BooleanSupplier) () -> false;
//   }
// }
// private BooleanSupplier hasTagAndIsNotAlignedMiddle() {
//   if (NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0)!=-1 && RobotContainer.isNotAlignedMiddle().getAsBoolean()) {
//     return (BooleanSupplier) () -> false;
//   } else {
//     return (BooleanSupplier) () -> true;
//   }
// }

private IntakeIR iir;

private void announcedone() {
  SmartDashboard.putBoolean("DoneAligning", true);
}

public Command AutoAlignRight() {
  return run(this::autoAlignDriveRight).until(()->RobotContainer.isAlignedRight().getAsBoolean()).onlyWhile(hasTag());
}
public Command AutoAlignLeft() {
  return run(this::autoAlignDriveLeft).until(()->RobotContainer.isAlignedLeft().getAsBoolean()).onlyWhile(hasTag());
}
public Command AutoAlignMiddle(){
  return run(this::autoAlignDriveMiddle).until(()->RobotContainer.isAlignedMiddle().getAsBoolean()).onlyWhile(hasTag());
  
}
//.onlyWhile(hasTagAndIsNotAlignedMiddle()).withTimeout(2).finallyDo(this::autoAlignStop);



// public Command autoAlignStop() {
//   return run(this::autoAlignStop);
// }

public void autoAlignDriveRight() {
  this.drive(limelightXSpeedAlign(), 
  limelightYSpeedAlignRight(), 
  limelightRotateToTarget(), 
  false);
}
public void autoAlignDriveLeft() {
  this.drive(limelightXSpeedAlign(), 
  limelightYSpeedAlignLeft(), 
  0, 
  false);
}

public void autoAlignDriveMiddle() {
  this.drive(limelightXSpeedAlign(), 
  limelightYSpeedAlignmiddle(), 
  0, 
  false);
}

public void autoAlignDriveFar() {
  this.drive(limelightXSpeedFar(), 
  limelightYSpeedAlignmiddle(), 
  limelightRotateToTarget(), 
  false);
}

public void autoAlignStop() {
  this.drive(0, 
  0, 
  0, 
  false);
}
  public final TalonFX talonFrontLeft = new TalonFX(DriveConstants.kFrontLeftDrivingCanId);
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
    talonFrontLeft,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

      public final TalonFX talonFrontRight = new TalonFX(DriveConstants.kFrontRightDrivingCanId);
    private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
        talonFrontRight, 
        DriveConstants.kFrontRightTurningCanId, 
        DriveConstants.kFrontRightChassisAngularOffset
    );
    public final TalonFX talonRearLeft = new TalonFX(DriveConstants.kRearLeftDrivingCanId);
    private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
        talonRearLeft, 
        DriveConstants.kRearLeftTurningCanId, 
        DriveConstants.kBackLeftChassisAngularOffset
    );
    public final TalonFX talonRearRight = new TalonFX(DriveConstants.kRearRightDrivingCanId);
    private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
        talonRearRight, 
        DriveConstants.kRearRightTurningCanId, 
        DriveConstants.kBackRightChassisAngularOffset
    );

  // The gyro sensor
  // private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  // private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(this.getTrueHeading()),
      // Rotation2d.fromDegrees(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(IntakeIR iir) {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    RobotConfig config;
    try{
      config = RobotConfig.fromGUISettings();
    

      // Configure AutoBuilder last
      AutoBuilder.configure(
              this::getPose, // Robot pose supplier
              this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
              this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
              // this::driveRobotRelative(getSpeeds()),
              new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                      new PIDConstants(7.0, 0.0, 0.0), // Translation PID constants
                      new PIDConstants(8.0, 0.0, 0.0) // Rotation PID constants
              ),
              config, // The robot configuration
              () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
              },
              this // Reference to this subsystem to set requirements
      );
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
       
    
   
    this.iir = iir;

    SmartDashboard.putData("Swerve",
        builder -> {
          builder.setSmartDashboardType("SwerveDrive");

          builder.addDoubleProperty(
              "Front Left Angle", () -> m_frontLeft.getPosition().angle.getDegrees(), null);
          builder.addDoubleProperty(
              "Front Left Velocity", () -> m_frontLeft.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Front Right Angle", () -> m_frontRight.getPosition().angle.getDegrees(), null);
          builder.addDoubleProperty(
              "Front Right Velocity", () ->m_frontRight.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Back Left Angle", () -> m_rearLeft.getPosition().angle.getDegrees(), null);
          builder.addDoubleProperty(
              "Back Left Velocity", () -> m_rearLeft.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Back Right Angle", () -> m_rearRight.getPosition().angle.getDegrees(), null);
          builder.addDoubleProperty(
              "Back Right Velocity", () -> m_rearRight.getState().speedMetersPerSecond, null);

          builder.addDoubleProperty(
              "Robot Angle", () -> getTrueHeading(), null);
        });


  }

  

  private double rotationValue;
  private void updateRotationValue(){
   double TAGID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0); 
   if (TAGID == 7 || TAGID == 18)
    {
      rotationValue = 0;
    }
    else if (TAGID == 8 || TAGID == 17)
    {
      rotationValue = -60;
    }
    else if (TAGID == 9 || TAGID == 22)
    {
      rotationValue = -120;
    }
    else if (TAGID == 10 || TAGID == 21)
    {
      rotationValue = 180;
    }
    else if (TAGID == 11 || TAGID == 20)
    {
      rotationValue = 0;
    }
    else if (TAGID == 6 || TAGID == 19)
    {
      rotationValue = 0;//60
    }
  }
  

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    updateRotationValue();
    m_odometry.update(
        Rotation2d.fromDegrees(this.getTrueHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });

    // SmartDashboard.putNumber("x speed", limelightYSpeedAlignRight());
    SmartDashboard.putNumber("TagId", NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0));
    SmartDashboard.putString("pose", getPose().toString());
    SmartDashboard.putNumber("True Heading", this.getTrueHeading());
    SmartDashboard.putNumber("Rotation Value", (rotationValue));
    SmartDashboard.putBoolean("intakeIR", this.iir.supplier.getAsBoolean());
    SmartDashboard.putNumber("xspeed",limelightYSpeedAlignmiddle());
    SmartDashboard.putNumber("yspeed",limelightXSpeedAlign());
    SmartDashboard.putBoolean("alignRight", RobotContainer.isAlignedRight().getAsBoolean());
    SmartDashboard.putBoolean("alignLeft", RobotContainer.isAlignedLeft().getAsBoolean());
    SmartDashboard.putBoolean("alignMiddle", RobotContainer.isAlignedMiddle().getAsBoolean());
    SmartDashboard.putNumber("rotorSpeed", talonFrontLeft.getRotorVelocity().getValueAsDouble());
    SmartDashboard.putBoolean("isClose", isCloseEnough().getAsBoolean());

   
    
    // in DriveSubsystem.java
    

  }

  ChassisSpeeds getRobotRelativeSpeeds() {
    return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }
  public double getTrueHeading() {
    return Math.IEEEremainder(-m_gyro.getAngle(), 360.0d);  
}

  SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {
      m_frontLeft.getState(),
      m_frontRight.getState(),
      m_rearLeft.getState(),
      m_rearRight.getState()
    };
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, AutoConstants.kSwerveDiscreteTimestep);
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    setModuleStates(swerveModuleStates);
  }


  /**
   * Returns the currently-estimated pose of the robot.
   *
   * @return The pose.
   */
  public Pose2d getPose() {
    return m_odometry.getPoseMeters();
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        Rotation2d.fromDegrees(this.getTrueHeading()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered,
                Rotation2d.fromDegrees(-m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Sets the wheels into an X formation to prevent movement.
   */
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  /** Zeroes the heading of the robot. */
  // public void zeroHeading() {
  //   m_gyro.reset();
  // }

  /**
   * Returns the heading of the robot.
   *
   * @return the robot's heading in degrees, from -180 to 180
   */
  public double getHeading() {
    return Rotation2d.fromDegrees(m_gyro.getAngle()).getDegrees();
  }

  /**
   * Returns the turn rate of the robot.
   *
   * @return The turn rate of the robot, in degrees per second
   */
  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }
   
  public void zeroHeading() {
    m_gyro.zeroYaw();
  }

  
   // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  
  
  // "tx" value from the Limelight.


  public double limelightRotateToTarget()
  {    
    // // kP (constant of proportionality)
    // // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // // if it is too high, the robot will oscillate around.
    // // if it is too low, the robot will never reach its targget
    SmartDashboard.putNumber("rotation", getTrueHeading());

    return ((getTrueHeading() + rotationValue)%180) * (-0.003);

    // return ((((getTrueHeading() - rotationValue) + 180) % 360 + 360) % 360 - 180) * 0.008;
  }

  // alignment Y speed Right
  public static final double autoAlignYoffsetRight = -0.03;
  public static double limelightYSpeedAlignRight()
  {
    double kP = -.3;
    Pose3d targetingYSpeed = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
    return (targetingYSpeed.getX()+autoAlignYoffsetRight) * kP;
  }

  // alignment spped Y left
  public static final double autoAlignYoffsetLeft = .29;
  public static double limelightYSpeedAlignLeft()
  {
    double kP = -.3;
    Pose3d targetingYSpeed = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
    return (targetingYSpeed.getX()+autoAlignYoffsetLeft) * kP;
  }
  //align middle
  public static final double autoAlignYoffsetMiddle = .11;
  public static double limelightYSpeedAlignmiddle()
  {
    double kP = -0.3;
    Pose3d targetingYSpeed = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
    return (targetingYSpeed.getX()+autoAlignYoffsetMiddle) * kP;
  }
  //xalign
  public static final double autoAlignXoffset = .59;
  public static double limelightXSpeedAlign()
  {
    double kP = .3;
    Pose3d targetingYSpeed = LimelightHelpers.getBotPose3d_TargetSpace("limelight");
    return (targetingYSpeed.getZ()+autoAlignXoffset) * kP;
  }

  public static BooleanSupplier isCloseEnough()
  {
    Pose3d targetingYSpeed = LimelightHelpers.getBotPose3d_TargetSpace("limelight");

    return () -> {
      return targetingYSpeed.getZ() >= -0.9;
    };
  }



  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // public static final double autoAlignXoffset = -4.5;// if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  // public double limelightXSpeed()
  // {    
  //   double kP = .001;
  //   double targetingForwardSpeed = (LimelightHelpers.getTY("limelight")+autoAlignXoffset) * kP;
  //   targetingForwardSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
  //   targetingForwardSpeed *= -1.0;
  //   return targetingForwardSpeed;
  // }

  public static final double autoAlignXoffsetFar = 0;// if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  public double limelightXSpeedFar()
  {    
    double kP = .001;
    double targetingForwardSpeed = (LimelightHelpers.getTY("limelight")+autoAlignXoffsetFar) * kP;
    targetingForwardSpeed *= DriveConstants.kMaxSpeedMetersPerSecond;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  
}
