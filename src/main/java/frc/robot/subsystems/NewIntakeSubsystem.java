package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.IntakeIR;
import frc.robot.RobotContainer;

public class NewIntakeSubsystem extends SubsystemBase {
    
    private SparkMax leader;
    private int consecutive_algae = 0;
    // private SparkMax leftMotor;
    DigitalInput irSensor;


    public NewIntakeSubsystem()
    {
        leader = new SparkMax(Constants.IntakeConstants.rightMotorID , MotorType.kBrushless);
        // irSensor = new DigitalInput(0);

        SparkMaxConfig globalConfig = new SparkMaxConfig();
        SparkMaxConfig leaderConfig = new SparkMaxConfig();

        /*
        * Set parameters that will apply to all SPARKs. We will also use this as
        * the left leader config.
        */
        globalConfig
            .smartCurrentLimit(50)
            .idleMode(IdleMode.kBrake);

        // Apply the global config and invert since it is on the opposite side
        leaderConfig
            .apply(globalConfig);


        /*
        * Apply the configuration to the SPARKs.
        *
        * kResetSafeParameters is used to get the SPARK MAX to a known state. This
        * is useful in case the SPARK MAX is replaced.
        *
        * kPersistParameters is used to ensure the configuration is not lost when
        * the SPARK MAX loses power. This is useful for power cycles that may occur
        * mid-operation.
        */
        leader.configure(leaderConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }
    public BooleanSupplier isAlgaeGripped(){
        return (BooleanSupplier) () -> {
            if (this.leader.getOutputCurrent() > 45)
                if (this.consecutive_algae > 10)
                    return true;
                else {
                    consecutive_algae += 1;
                    return false;
                }
            else {
                consecutive_algae = 0;
                return false;
            }
        };
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("IR Sensor", RobotContainer.D_INTAKE_IR.supplier.getAsBoolean());
        SmartDashboard.putNumber("intake AMPS", leader.getOutputCurrent());
        SmartDashboard.putBoolean("algaeIn", this.isAlgaeGripped().getAsBoolean());
    }

    public Command IntakeCoralSubstation(){
        return startEnd(this::intake, this::stop).until(RobotContainer.D_INTAKE_IR.supplier);
        //.alongWith(startEnd(RobotContainer.controller::rumble, RobotContainer.controller::stopRumble))
    }
    public Command ScoreIntakeCoral(){
        return startEnd(this::intake, this::stop).until(() -> {
            return !RobotContainer.D_INTAKE_IR.supplier.getAsBoolean();
        });
    }
    public Command runIntake(){
        return startEnd(this::intake, this::stop).withTimeout(0.7);
    }
    public Command intakeAlgae(){
        return 
        startEnd(this::outTake, this::outTake).until(RobotContainer.newIntakeSubsystem.isAlgaeGripped());
    }
    public void intake()
    {
        leader.set(0.5);
    }

    public void outTake()
    {
         leader.set(-0.5);
    }

    public Command IntakeManual()
    {
        return this.run(
            () -> leader.set(0.5));
    }
    public Command stopManual()
    {
        return this.run(
            () -> leader.set(0));
    }

    public void stop()
    {
        leader.set(0);
    }

    public boolean isPresent()
    {
        return irSensor.get();
    }


    public void rumble()
    {
        RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 1);

    }

    public void stopRumble()
    {
            RobotContainer.m_driverController.setRumble(RumbleType.kBothRumble, 0);
    }

    public double getVoltage()
    {
        return leader.getBusVoltage();
    }

    public BooleanSupplier hasBall()
    {
        return () -> {
            return getVoltage() < 11;
        };
    }
}
