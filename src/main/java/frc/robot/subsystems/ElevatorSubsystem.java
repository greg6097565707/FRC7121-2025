package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.RobotContainer;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class ElevatorSubsystem extends SubsystemBase {

    private TalonFX leader;
    private TalonFX follower;
    private SparkClosedLoopController closedLoopController;
    private RelativeEncoder encoder;
    private int targetPosition;
    private MotionMagicExpoVoltage m_request;

    public ElevatorSubsystem() {

        leader = new TalonFX(Constants.ElevatorConstants.leaderID, "rio");
        follower = new TalonFX(Constants.ElevatorConstants.followerID, "rio");

        //.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        targetPosition = 0;

        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

        // in init function
        talonFXConfigs = new TalonFXConfiguration();

        // set slot 0 gains
        var slot0Configs = talonFXConfigs.Slot0;
        slot0Configs.kS = 0.4; 
        slot0Configs.kV = 0.001; 
        slot0Configs.kA = 0;
        slot0Configs.kG = 0.5; 
        slot0Configs.kP = 1.2;
        slot0Configs.kI = 0; 
        slot0Configs.kD = 0;

        // set Motion Magic Velocity settings
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 100; // Target acceleration of 100 rps/s
        motionMagicConfigs.MotionMagicJerk = 100; // Target jerk of 100 rps/s/s
        motionMagicConfigs.MotionMagicCruiseVelocity = 100;
        motionMagicConfigs.MotionMagicExpo_kV = 0.12; // kV is around 0.12 V/rps
        motionMagicConfigs.MotionMagicExpo_kA = 0; // Use a slower kA of 0.1 V/(rps/s)
        

        leader.getConfigurator().apply(talonFXConfigs);

        m_request = new MotionMagicExpoVoltage(0);

        // set target position to 100 rotations
        
        follower.setControl(new Follower(leader.getDeviceID(), true));
        
        

        
    }
    public void resetEncoderElevator() {
        leader.setPosition(0);
      }

    public BooleanSupplier isAtTarget(){
        return (BooleanSupplier) () -> { return this.leader.getPosition().getValueAsDouble() < 40;
        };
         
    }

    // public Command RasiseWhileAligning(){
    // return
    // run(null).onlyWhile(RobotContainer.isNotAligned()).finallyDo(this::raiseElevatorTop);
    // }


    //FIGURE THIS OUT LATER ONCE WE KNOW THE ENCODER VALUES
    // public BooleanSupplier L4AchieveHorizontalElevatorClearance() {
    //     return (BooleanSupplier) () -> {
    //         return this.leader.getEncoder().getPosition() < 40;
    //     };
    // }

    // public BooleanSupplier L3AchieveHorizontalElevatorClearance() {
    //     return (BooleanSupplier) () -> {
    //         return this.leader.getEncoder().getPosition() < 23;
    //     };
    // }

    // public BooleanSupplier L2AchieveHorizontalElevatorClearance() {
    //     return (BooleanSupplier) () -> {
    //         return this.leader.getEncoder().getPosition() < 8;
    //     };
    // }

    public Command tempRaiseElevator()
    {
        return this.runOnce(
            () -> leader.setControl(m_request.withPosition(12))
        );
    }

    public Command tempLowerElevator()
    {
        return this.runOnce(
            () -> leader.setControl(m_request.withPosition(5))
        );
    }

    // public Command raiseElevatorIntake() {
    //     return this.runOnce(
    //             () -> closedLoopController.setReference(9.5, ControlType.kMAXMotionPositionControl, // 50
    //                     ClosedLoopSlot.kSlot0));
    // }

    // public Command raiseElevatorL4() {
    //     return this.runOnce(
    //             () -> closedLoopController.setReference(49, ControlType.kMAXMotionPositionControl, // 50
    //                     ClosedLoopSlot.kSlot0));
    // }

    // public Command raiseElevatorL3() {
    //     return this.runOnce(
    //             () -> closedLoopController.setReference(28.5, ControlType.kMAXMotionPositionControl,
    //                     ClosedLoopSlot.kSlot0));
    // }

    // public Command raiseElevatorL2() {
    //     return this.runOnce(
    //             () -> closedLoopController.setReference(15.5, ControlType.kMAXMotionPositionControl,
    //                     ClosedLoopSlot.kSlot0));
    // }

    // public Command lowerElevator() {
    //     return this.runOnce(
    //             () -> closedLoopController.setReference(0, ControlType.kMAXMotionPositionControl,
    //                     ClosedLoopSlot.kSlot1));
    // }
    // public Command raiseElevatorHighAlgae() {
    //     return this.runOnce(
    //             () -> closedLoopController.setReference(32, ControlType.kMAXMotionPositionControl,
    //                     ClosedLoopSlot.kSlot0));
    // }
    // public Command raiseElevatorLowAlgae() {
    //     return this.runOnce(
    //             () -> closedLoopController.setReference(20, ControlType.kMAXMotionPositionControl,
    //                     ClosedLoopSlot.kSlot0));
    // }

    // public int getTargetPosition() {
    //     return targetPosition;
    // }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("elevator encoder", leader.getPosition().getValueAsDouble());

        // Logger.getInstance().recordOutput("Elevator", leader.getPosition().getValueAsDouble());

    }
}
