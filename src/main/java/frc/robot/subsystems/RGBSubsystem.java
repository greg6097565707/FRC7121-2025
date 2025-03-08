package frc.robot.subsystems;

// import org.w3c.dom.css.RGBColor;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.motorcontrol.Spark;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.IntakeIR;
import frc.robot.RobotContainer;
import frc.robot.controllers.RGBController;

import java.util.HashMap;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.RunCommand;





public class RGBSubsystem extends SubsystemBase {
    private final RGBController m_blinkinDriver = new RGBController();



    private String mode = "Algae";

    

    public RGBSubsystem() {
        this.idle();
    }
    public void idle() {
        m_blinkinDriver.set(.15);
    }
    public void GamePieceIN() {
        //red
        setColor("red");
    }
    public void ScoringCoral() {
        //yellow
        m_blinkinDriver.set(.15);
    }
    public void GrabbingAlgae() {
        //green
        setColor("green");
    }
    public void GamePieceOut() {
        m_blinkinDriver.set(.61);
    
    }


    public void setColor(String color) {
        if (color.equals("green")) {
            m_blinkinDriver.set(.35);
        }
        if (color.equals("red")) {
            m_blinkinDriver.set(.77);
        }
    }


    public void setAlgaeMode() {
        mode = "Algae";
    }
    public void setCoralMode() {
        mode = "Coral";
    }
    
     //commands
    public Command GamePieceINCommand(){
        return run(this::GamePieceIN);
    }
    public Command GamePieceOutCommand(){
        return run(this::GamePieceOut);
    }
    public Command ScoringCoralCommand(){
        return run(this::ScoringCoral);
    }
    public Command ScoringAlgaeCommand(){
        return run(this::GrabbingAlgae);
    }

    public Command setAlgaeModeCommand() {
        return run(this::setAlgaeMode);
    }
    public Command setCoralModeCommand() {
        return run(this::setCoralMode);
    }

    @Override
    public void periodic() {
        

        if (RobotContainer.controller.isInCoralMode) {
            mode = "Coral";
            if(RobotContainer.D_INTAKE_IR.supplier.getAsBoolean() || RobotContainer.newIntakeSubsystem.isAlgaeGripped().getAsBoolean())
                this.GamePieceIN();
            else 
                this.GamePieceOut();
        }
        else {
            mode = "Algae";
            if(RobotContainer.D_INTAKE_IR.supplier.getAsBoolean() || RobotContainer.newIntakeSubsystem.isAlgaeGripped().getAsBoolean())
                this.GamePieceIN();
            else 
                this.GamePieceOut();
        }

        SmartDashboard.putString("Robot Mode", mode);
        
    }
}