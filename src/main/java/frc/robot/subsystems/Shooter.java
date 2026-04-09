
package frc.robot.subsystems;


import java.time.Duration;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.*;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.LimelightHelpers;
import frc.robot.Telemetry;


public class Shooter {
    private static TalonFX shooter1;
    private TalonFX shooter2;
    private static TalonFX preshooter;
    private static TalonFX feeder;
    private CommandSwerveDrivetrain drive;
    private TalonSRX hoodSrx1;
    private TalonSRX hoodSrx2;
    private int hoodMode = 0; // 0 = min, 1 = mid, 2 = max
    private double curPosition = 0;
    private AnalogInput m_analog = new AnalogInput(1);
    private AnalogInput m2_analog = new AnalogInput(2);
    final PositionVoltage m_request = new PositionVoltage(0).withSlot(0);
    final MotionMagicVelocityTorqueCurrentFOC velocityTorqueCurrentFOC = new MotionMagicVelocityTorqueCurrentFOC(0);
    private Telemetry telemetry;
    private InterpolatingDoubleTreeMap shooterMap = new InterpolatingDoubleTreeMap();

    private double lastKnownDistance = 0.0;



    double targetOffsetAngle_Vertical = LimelightHelpers.getTY("bigboy");
    double angleToGoalRadians = Math.toRadians(25  + targetOffsetAngle_Vertical);
    double tx = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-bigboy").getX();
    double tz = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-bigboy").getZ();
    double distance = Math.sqrt((tx* tx) +  (tz * tz)) * 3.28;
    double g = 32.2;                      // ft/s^2
    double h0 = 29.0 / 12.0;              // shooter height (ft)
    double ht = 6.0;                      // hoop height (ft)
    double deltaH = ht - h0;              // height difference (ft)
    double theta = Math.toRadians(55);    // hood angle (radians)
    double r = 4.0 / 12.0;                // wheel radius (ft)
    double speed = 50;
    boolean feeding = false; 
    


    public Shooter(int CanID1, int CanID2, int CanID3, int CanID4, int CanID5, int CanID6, Telemetry t, CommandSwerveDrivetrain d){
        //canbus = new CANBus("rio");
        telemetry = t;
        drive = d;
        TalonFX s1 = new TalonFX(CanID1);
        TalonFX s2 = new TalonFX(CanID2);
        TalonFX f = new TalonFX(CanID3);
        TalonFX ps = new TalonFX(CanID4);
        TalonSRX s5 = new TalonSRX(CanID5);
        TalonSRX s6 = new TalonSRX(CanID6);
        shooter1 = s1;
        shooter2 = s2;
        shooter1.setNeutralMode(NeutralModeValue.Coast);
        shooter2.setNeutralMode(NeutralModeValue.Coast);
        feeder = f;
        preshooter = ps;
        preshooter.setNeutralMode(NeutralModeValue.Coast);
        hoodSrx1 = s5;
        hoodSrx2 = s6;
        var talonFXConfigs = new TalonFXConfiguration();

        /*talonFXConfigs.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 1;
        talonFXConfigs.withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(40).withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(40).withStatorCurrentLimitEnable(true));


         */
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonFXConfigs.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        talonFXConfigs.Slot0 = new Slot0Configs().withKS(0).withKV(0).withKP(0).withKI(0).withKD(0);
        talonFXConfigs.Feedback.SensorToMechanismRatio = 1;
        talonFXConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 80.0;
        talonFXConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -80.0;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 80.0;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;

        var slot0Configs = talonFXConfigs.Slot0;
        //DO NOT CHANGE tuned constants help to adjust and ensure consistency
        slot0Configs.kG = 0; //0
        slot0Configs.kS = 0; //0
        slot0Configs.kV = .1; //0.122, .8
        slot0Configs.kA = 0; //0.1
        slot0Configs.kP = 10; //.6 , 8
        slot0Configs.kI = 0; //0
        slot0Configs.kD = 0.0; //1.5
        
        var motionMagicConfigs = talonFXConfigs.MotionMagic;
        motionMagicConfigs.MotionMagicExpo_kV = 0.8; //.12 , .5
        motionMagicConfigs.MotionMagicExpo_kA = 0.6; //.1
        motionMagicConfigs.MotionMagicAcceleration = 40; //40

        shooter1.getConfigurator().apply(talonFXConfigs);
        shooter2.getConfigurator().apply(talonFXConfigs);
        //shooter2.setControl(new Follower(shooter1.getDeviceID(), MotorAlignmentValue.Opposed));
        
        shooterMap.put(4.0, 58.48);
        shooterMap.put(5.0, 63.31); 
        shooterMap.put(6.0, 66.29);
        shooterMap.put(7.0, 68.55);
        shooterMap.put(8.0, 72.46);
        shooterMap.put(9.0, 77.90);
        shooterMap.put(10.0, 80.0);

    }
    



    public double getRPS() {
        return shooterMap.get(getDistance());
    }

    public void sSetSpeed(double RPS){
        //setting the motor to reach a speed (rps) from imap
        //NOTE: Imap MUST be in RPS to retrieve proper values
        shooter1.setControl(velocityTorqueCurrentFOC.withVelocity(RPS));
        shooter2.setControl(velocityTorqueCurrentFOC.withVelocity(-RPS));
    }
    
    public double getSpeed(){
        return speed;
    }
    
    public double getRightSpeed(){
        //this returns the speed of the motor in rotations per second
        return shooter1.getVelocity().getValueAsDouble();
        
    }

    public double getLeftSpeed(){
        //this returns the speed of the motor in rotations per second
        return shooter1.getVelocity().getValueAsDouble();
        
    }
    public double getPSpeed(){
        //this returns the speed of the motor in rotations per second
        return preshooter.getVelocity().getValueAsDouble();
        
    }
    public void stopShooter(){
        shooter1.setControl(new NeutralOut());
        shooter2.setControl(new NeutralOut());
        readyCounter =0;
        isFiring = false;
        feeding = false;
    }
    public void stopPreshooter(){
        preshooter.setControl(new NeutralOut());
    }

    double readyCounter =0;
    private boolean isFiring = false;

    public double getRobotSpeed(){
        return telemetry.getXVelocityMotorRPS(drive.getState());
    }
    public void loadUp(){
        sSetSpeed(50);
    }
    public void shoot(){
        double targetVelocity = getRPS();
        double compensateVelocity = targetVelocity - telemetry.getXVelocityMotorRPS(drive.getState());
        sSetSpeed(compensateVelocity);
        if ( Math.abs( getRightSpeed() - targetVelocity ) >1 ||  Math.abs( getLeftSpeed() - targetVelocity ) >1 ){
            pSetSpeed(0.0);
            fSetSpeed(0.0);          
        }else{
            pSetSpeed(.9);
            fSetSpeed(.6);
        }
    }

    public void dynamicShoot() {
        double targetVelocity = SmartDashboard.getNumber("Dynamic Shooter Speed", 0);
        sSetSpeed(targetVelocity);
        if ( Math.abs( getRightSpeed() - targetVelocity ) >1 ||  Math.abs( getLeftSpeed() - targetVelocity ) >1 ){
            pSetSpeed(0.0);
            fSetSpeed(0.0);          
        }else{
            pSetSpeed(.9);
            fSetSpeed(.6);
        }
    }
    

    public void fSetSpeed(double percent){
        feeder.set(-percent);
    }
    public void pSetSpeed(double percent){
        preshooter.set(percent);
    }
    public double getDistance(){
        double tx = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-bigboy").getX();
        double tz = LimelightHelpers.getTargetPose3d_CameraSpace("limelight-bigboy").getZ();
        double distance = Math.sqrt((tx* tx) +  (tz * tz)) * 3.28;
        if(distance < 0.10)  {
            distance = lastKnownDistance;
        } else {
        lastKnownDistance = distance;
        }
        return distance;

    }

    public double getVoltage1(){
        return (Math.round(m_analog.getVoltage()*10))/10.0;
    }
    public double getVoltage2(){
        return (Math.round(m2_analog.getVoltage()*10))/10.0;
    }
    public void stopHood(){
        hoodSrx1.set(ControlMode.PercentOutput, 0);
    }

    public void moveHood( double speed){
            hoodSrx1.set(ControlMode.PercentOutput, speed);
            hoodSrx2.set(ControlMode.PercentOutput, -speed);
    }

    
}