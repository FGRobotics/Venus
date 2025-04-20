package org.firstinspires.ftc.teamcode.boilerplate.intake;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.LimitSwitch;
public class BoilerIntake {
    public DcMotorEx islide;
    public TouchSensor intakeLeft, intakeRight;
    public LimitSwitch iLSLeft,iLSRight;
    double intakeReversed = 1;
    double ampThreshold = 100;
    ElapsedTime homeTimer = new ElapsedTime();
    ElapsedTime extentTimer = new ElapsedTime();
    public boolean intakeDocked = true;
    boolean extending = false;
    boolean busy = false;
    public BoilerIntake(HardwareMap hardwareMap){
        intakeLeft = hardwareMap.get(TouchSensor.class, "inLeft");
        intakeRight = hardwareMap.get(TouchSensor.class, "inRight");
        islide = hardwareMap.get(DcMotorEx.class, "horSlide" );
        islide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        islide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        islide.setDirection(DcMotorSimple.Direction.REVERSE);
        iLSLeft = new LimitSwitch(intakeLeft);
        iLSRight = new LimitSwitch(intakeRight);
    }
    public void slidesLimiterIntake(double power, int upperBound, boolean override){
        double adjPower = 0;
        if(override){
            adjPower = intakeReversed*power;
        }else if(intakeReversed*islide.getCurrentPosition() > upperBound && power > 0){
            adjPower = 0;
        }else if(intakeDocked() && power < 0){
            adjPower = 0;
        }/*else if(islide.getCurrent(CurrentUnit.MILLIAMPS) > ampThreshold){
            adjPower = 0;
        }*/else{
            adjPower = intakeReversed*power;
        }
        if(adjPower != 0){
            intakeDocked = false;
        }
        islide.setPower(adjPower);
    }
    public void slidesLimiterIntake(double power, int upperBound, boolean override, double slides_mult){
        double adjPower = 0;
        if(override){
            adjPower = intakeReversed*power;
        }else if(intakeReversed*islide.getCurrentPosition() > upperBound && power > 0){
            adjPower = 0;
        }else if(intakeDocked() && power < 0){
            adjPower = 0;
        }/*else if(islide.getCurrent(CurrentUnit.MILLIAMPS) > ampThreshold){
            adjPower = 0;
        }*/else{
            adjPower = intakeReversed*power*slides_mult;
        }
        if(adjPower != 0){
            intakeDocked = false;
        }
        islide.setPower(adjPower);
    }
    public boolean intakeDocked(){
        if((iLSLeft.isPressed() && iLSRight.isPressed()) && intakeReversed*islide.getCurrentPosition() <= 5){
            intakeDocked = true;
            return true;
        }else{
            return false;
        }
    }
    public boolean homeIntake() {
        if(!busy && !intakeDocked){
            homeTimer.reset();
            islide.setPower(intakeReversed * -1);
            busy = true;
        }
        if (((Math.abs(islide.getVelocity()) < 50) && homeTimer.seconds() > 0.1) || homeTimer.seconds() > 1.6) {
            islide.setPower(0);
            intakeDocked = true;
            busy = false;
        }
        return intakeDocked;

    }
    public boolean setSlides(int target){
        if(!extending){
            extending = true;
            extentTimer.reset();
        }
        if(intakeReversed* islide.getCurrentPosition() <= target && extentTimer.seconds() < 0.5){
            slidesLimiterIntake(1, 1300, false);
            return false;
        }
        slidesLimiterIntake(0, 1300, false);
        extending = false;
        return true;
    }
    public void reset(){
        islide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        islide.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
