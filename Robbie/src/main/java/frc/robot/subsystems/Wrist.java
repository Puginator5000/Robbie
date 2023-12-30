package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Wrist extends SubsystemBase{

    //OFF = wrist is off
    //JOG = motor power ; feed forward = the "boost" to get the arm to 90% of the way so PID only has to do 10% of the work
    //POSITION = the position
    //ZERO = the default positon
    public enum ArmState{
        OFF,
        JOG, //need diff jog values bc differences forces are working against different mechanisms
        POSITION,
        ZERO
    }
    
    //motors
    //slave follows master
    CANSparkMax armMaster = new CANSparkMax(ArmConstants.DEVICE_ID_ARM_MASTER, MotorType.kBrushless);
    CANSparkMax armSlave  = new CANSparkMax(ArmConstants.DEVICE_ID_ARM_SLAVE,  MotorType.kBrushless);

    ArmState state = ArmState.OFF;

    double jogValue = 0;
    Rotation2d setpoint = new Rotation2d(); //the desired value, through rotation2d
    //does math for us

    //static = don't need to create an object to call a method, call straight from the class
    private static Wrist instance = new Wrist();     //so we can call this instance across many files instead of having to make instances of this class in others


    public Wrist(){
        configMotors();
    }

    public static Wrist getInstance(){
        return instance;
    }

    @Override
    public void periodic() { //constantly running
        logData(); //displays log values
        
        //checks the state, if state true, do what state is
        switch(state){ //forever if loop
            case OFF:
                set(0);
                break; //makes it so if one is true it doesn't go through the rest in the list
            case JOG:
                set(jogValue);
                break;
            case POSITION:
                goToSetpoint();
                break;
            case ZERO:
                zero();
                break;
        }   
        
    }

    //gets angle of wrist
    public Rotation2d getAngle(){
        return Rotation2d.fromRotations(armMaster.getEncoder().getPosition() / ArmConstants.ARM_GEAR_RATIO);
    }                                          //number of turns of a motor divided by gear ratio = angle
                                               //motor turns cancel out = rotations --> angle bc of .fromRotations
    //GEAR RATIO: the number of motor turns over one rotation
    //rotations x gear ratios = amount of motor turns

    public void zero(){
      if(!zero indicator)
        jog(0.1); //whatever speed is comfortable
      else{
        zeroEncoders();
        jog(0);
        setState(SubsystemState.STOW);
      }
    }

    public void set(double value){ //set speed of motor
        armMaster.set(value);
    }
    
    public void setJog(double jog){
        jogValue = jog;
    }

    private void goToSetpoint(){
        armMaster.getPIDController().setReference(setpoint.getRotations() * ArmConstants.ARM_GEAR_RATIO, ControlType.kSmartMotion);
    }                                 //takes setpoint and gets us to that point
                                      //our setpoint, takes in amount of rotations and control type
                                      //GEAR RATIO: the number of motor turns over one rotation
                                      //rotations x gear ratios = amount of motor turns

    public void setSetpoint(Rotation2d setpoint){
        setState(ArmState.POSITION);
        this.setpoint = setpoint;
    }

    public Rotation2d getSetpoint(){
        return setpoint;
    }

    public void setState(ArmState state){
        this.state = state;
    }

    public ArmState getState(){
        return state; 
    }

    public boolean atSetpoint(){
        return Math.abs(setpoint.minus(getAngle()).getRotations()) < ArmConstants.TOLERANCE.getRotations();
    }                                                               //if the arm isn't within the set tolerance then bad
                        //setpoint minus the this angle from this rotation is less than the tolerance
    public void logData(){
        SmartDashboard.putNumber("Wrist Position", getAngle().getDegrees());
        SmartDashboard.putBoolean("Wrist At Setpoint", atSetpoint());
        SmartDashboard.putString("Wrist State", getState().toString());
        SmartDashboard.putNumber("Wrist Setpoint", getSetpoint().getDegrees());
    }

    //slave follow masters
    public void configMotors(){
        armMaster.restoreFactoryDefaults(); //resets the motor controller settings
        armSlave.restoreFactoryDefaults();

        armMaster.setInverted(false); //inverts motors so the motors are turning same way

        armMaster.setIdleMode(IdleMode.kBrake); //the robot is off / not moving
        armSlave.setIdleMode(armMaster.getIdleMode());

        armMaster.setSmartCurrentLimit(40, 40); //free = motor moving without resistance - max limit on amperage when the motor is running freely
        armSlave.setSmartCurrentLimit(40, 40); //stall = limits motor from overdoing itself in working to do something

        SparkMaxPIDController armController = armMaster.getPIDController(); //pid = tuner
        armController.setP(ArmConstants.ARM_kP); //sets the P of PID
        armController.setD(ArmConstants.ARM_kD); //sets the D of PID

        armController.setSmartMotionAccelStrategy(AccelStrategy.kSCurve, 0);
        armController.setSmartMotionMaxAccel(ArmConstants.MAX_ACCELERATION, 0);
        armController.setSmartMotionMaxVelocity(ArmConstants.MAX_VELOCITY, 0);

        armSlave.follow(armMaster); //slave follows master
    }
}
