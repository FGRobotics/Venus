package org.firstinspires.ftc.teamcode.boilerplate.outake;


import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.LimitSwitch;
import org.firstinspires.ftc.teamcode.boilerplate.BoilerChassis;
import org.firstinspires.ftc.teamcode.boilerplate.helpers.Control;

import java.util.concurrent.atomic.AtomicBoolean;

public class BoilerOuttake {
    public DcMotorEx oSlideRight, oSlideLeft;
    public TouchSensor left, right;
    public LimitSwitch oLSLeft, oLSRight;
    public boolean hasObject = false;
    public boolean outtakeDocked = true;
    public double ampThreshold = 7000;
    public boolean busy = false;
    public AtomicBoolean extending = new AtomicBoolean(false);
    public ElapsedTime extentTimer = new ElapsedTime();
    ElapsedTime homeTimer = new ElapsedTime();
    BoilerOutArmAndBox delivery;
    BoilerChassis chassis;
    RevBlinkinLedDriver LED;
    public BoilerOuttake(HardwareMap hardwareMap, BoilerOutArmAndBox delivery, RevBlinkinLedDriver LED){
        oSlideRight = hardwareMap.get(DcMotorEx.class, "slidesRight" );
        oSlideLeft = hardwareMap.get(DcMotorEx.class, "slides_l");
        oSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        oSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        oSlideLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        left = hardwareMap.get(TouchSensor.class, "left");
        right = hardwareMap.get(TouchSensor.class, "right");
        oLSLeft = new LimitSwitch(left);
        oLSRight = new LimitSwitch(right);
        this.delivery = delivery;
        this.LED = LED;
    }
    public boolean slidesLimiterOuttake(double power, int upperBound, boolean override) {
        double adjPower = 0;
        boolean obstructed = false;
        ampThresholdAdjuster();
        if(override){
            adjPower = power;
        }else if((-oSlideRight.getCurrentPosition() > upperBound && power > 0)){
            adjPower = 0;
        }else if(outtakeDocked() && power < 0){
            adjPower = 0;
        }else if((oSlideRight.getCurrent(CurrentUnit.MILLIAMPS) > ampThreshold)){
            adjPower = 0;
            obstructed = true;
        }else{
            adjPower = power;
        }
        if(adjPower != 0){
            outtakeDocked = false;
        }
        oSlideLeft.setPower(-adjPower);
        oSlideRight.setPower(-adjPower);
        return obstructed;
    }
    public boolean outtakeDocked(){
        if((oLSRight.isPressed() && oLSLeft.isPressed()) || -oSlideRight.getCurrentPosition() <= 5){
            outtakeDocked = true;
            return true;
        }else{
            return false;
        }
    }
    public boolean homeOuttake(){
        if(!busy){
            homeTimer.reset();
            busy = true;
            slidesLimiterOuttake(-1, 1000, false);
        }
        if(outtakeDocked){
            busy = false;
            slidesLimiterOuttake(0, 1000, false);
            resetOuttake();
        }else if (homeTimer.seconds() > 1.3) {
            busy = false;
            outtakeDocked = true;
            slidesLimiterOuttake(0, 1000, false);
            resetOuttake();

        }else{
            slidesLimiterOuttake(-1, 1000, false);
            return false;
        }
        return true;
    }
    public boolean setSlides(int target){
        if(delivery.hasObject()){
            hasObject = true;
        }
        if(!hasObject){
            extending.set(false);
            slidesLimiterOuttake(0, target, false);
            return true;
        }
        if(!extending.get()){
            extentTimer.reset();
            extending.set(true);
        }
        if(-oSlideRight.getCurrentPosition() <= target && extentTimer.seconds() < 2){
            slidesLimiterOuttake(1, target, false);
            return false;
        }
        slidesLimiterOuttake(0, target, false);
        extending.set(false);
        return true;

    }
    public boolean setSlides(int target, boolean override){

        if(!extending.get()){
            extentTimer.reset();
            extending.set(true);
        }
        if(-oSlideRight.getCurrentPosition() <= target && extentTimer.seconds() < 2){
            slidesLimiterOuttake(1, target, false);
            return false;
        }
        slidesLimiterOuttake(0, target, false);
        extending.set(false);
        return true;

    }
    public boolean setSpeci(){
        if(!extending.get()){
            extentTimer.reset();
            extending.set(true);
        }
        //need to make sure slides are moving before we check velo
        boolean velo_bool = (oSlideRight.getCurrent(CurrentUnit.MILLIAMPS) > 0 && extentTimer.seconds() > 0.2) ? Math.abs(oSlideRight.getVelocity()) > 1000 : true;
        if(-oSlideRight.getCurrentPosition() <= 500 &&  extentTimer.seconds() < 1 && velo_bool){
            slidesLimiterOuttake(1, 1000, false);
            return false;
        }
        slidesLimiterOuttake(0, 1000, false);
        extending.set(false);
        return true;

    }
    public boolean setSpeci(boolean ovverride){
        if(delivery.hasObject()){
            hasObject = true;
        }

        if(!hasObject){
            extending.set(false);
            slidesLimiterOuttake(0, 1000, false);
            return true;
        }

        if(!extending.get()){
            extentTimer.reset();
            extending.set(true);
        }
        //need to make sure slides are moving before we check velo
        boolean velo_bool = (oSlideRight.getCurrent(CurrentUnit.MILLIAMPS) > 0 && extentTimer.seconds() > 0.2) ? Math.abs(oSlideRight.getVelocity()) > 1000 : true;
        if(-oSlideRight.getCurrentPosition() <= 500 &&  extentTimer.seconds() < 1 && velo_bool){
            slidesLimiterOuttake(1, 1000, false);
            return false;
        }
        slidesLimiterOuttake(0, 1000, false);
        extending.set(false);
        return true;

    }

    public void resetOuttake(){
        oSlideRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        oSlideRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        oSlideLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
    public void initializeAmpThreshold(BoilerChassis chassis){
        this.chassis = chassis;
    }
    public void ampThresholdAdjuster(){
        //lower amp threshold while driving
        if(Math.pow(chassis.otos.otos.getVelocity().x,2) + Math.pow(chassis.otos.otos.getVelocity().y, 2) > 2){
            ampThreshold = 7000;
        }else{
            ampThreshold = 6500;
        }
    }
}
