package frc.robot.subsystems;

import org.opencv.core.RotatedRect;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SuperStructure extends SubsystemBase{
    
    public enum SuperStructureState{
        CONE_HIGH,
        CONE_MID,
        CONE_HYRBID,

        CUBE_HIGH,
        CUBE_MID,
        CUBE_HYBRID,

        INTAKE_CONE_FALLEN,
        INTAKE_CONE_UPRIGHT,
        INTAKE_CONE_SINGLE,
        INTAKE_CONE_DOUBLE,

        INTAKE_CUBE,
        INTAKE_CUBE_SINGLE,
        INTAKE_CUBE_DOUBLE,

        STOW
    }

    private static SuperStructure instance;

    SuperStructureState state = SuperStructureState.STOW;
    
    private final Claw claw = Claw.getInstance();
    private final Extender extender = Extender.getInstance();
    private final Wrist wrist = Wrist.getInstance();

    public SuperStructure(){

    }

    public static SuperStructure getInstance(){
        if (instance == null){
            instance = new SuperStructure();
        }
        return instance;
    }

    public void goToPosition(Rotation2d wristAngle, double extenderSetpoint){
        wrist.setSetpoint(wristAngle);
        extender.setSetpoint(extenderSetpoint);
    }

    public void intake(Rotation2d wristAngle, double extenderSetpoint){
        wrist.setSetpoint(wristAngle);
        extender.setSetpoint(extenderSetpoint);
        claw.open();
    }

    public void openClaw(){
        claw.open();
    }

    public void closeClaw(){
        claw.close();
    }

    public void setState(SuperStructureState newState){
        state = newState;

        switch(state){
            case CONE_HIGH:
                goToPosition(Rotation2d.fromDegrees(40), 10);
                break;
            case CONE_MID:
                goToPosition(Rotation2d.fromDegrees(40), 10);
                break;
            case CONE_HYRBID:
                goToPosition(Rotation2d.fromDegrees(40), 10);
                break;
            
            case CUBE_HIGH:
                goToPosition(Rotation2d.fromDegrees(40), 10);
                break;
            case CUBE_MID:
                goToPosition(Rotation2d.fromDegrees(40), 10);
                break;
            case CUBE_HYBRID:
                goToPosition(Rotation2d.fromDegrees(40), 10);
                break;
            
            case INTAKE_CONE_FALLEN:
                intake(Rotation2d.fromDegrees(40), 10);
                break;
            case INTAKE_CONE_UPRIGHT:
                intake(Rotation2d.fromDegrees(40), 10);
                break;
            case INTAKE_CONE_SINGLE:
                intake(Rotation2d.fromDegrees(40), 10);
                break;
            case INTAKE_CONE_DOUBLE:
                intake(Rotation2d.fromDegrees(40), 10);
                break;
            
            case INTAKE_CUBE:
                intake(Rotation2d.fromDegrees(40), 10);
                break;
            case INTAKE_CUBE_SINGLE:
                intake(Rotation2d.fromDegrees(40), 10);
                break;
            case INTAKE_CUBE_DOUBLE:
                intake(Rotation2d.fromDegrees(40), 10);
                break;

            case STOW:
                goToPosition(Rotation2d.fromDegrees(40), 10);
                break;
        }
    }

    public boolean atSetpoint(){
        return wrist.atSetpoint() && extender.atSetpoint();
    }

    public SuperStructureState getState(){
        return state;
    }
}
