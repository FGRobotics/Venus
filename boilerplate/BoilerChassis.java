package org.firstinspires.ftc.teamcode.boilerplate;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.boilerplate.helpers.Control;
import org.firstinspires.ftc.teamcode.boilerplate.outake.BoilerOuttake;
import org.firstinspires.ftc.teamcode.boilerplate.sensorclasses.BoilerDeadwheel;
import org.firstinspires.ftc.teamcode.boilerplate.sensorclasses.SparkIMU;

import java.lang.annotation.Target;

public class BoilerChassis {
    public BoilerDeadwheel leftX, middle, rightX;
    public DcMotorEx fl,fr,br,bl, enc_left;
    public SparkIMU otos;
    public Limelight3A limelight = null;
    public BoilerChassis(HardwareMap hardwareMap, BoilerOuttake outtake, Telemetry telemetry){
        fl = hardwareMap.get(DcMotorEx.class, "FL");
        bl = hardwareMap.get(DcMotorEx.class, "BL");
        br = hardwareMap.get(DcMotorEx.class, "BR");
        fr = hardwareMap.get(DcMotorEx.class, "FR");
        otos = new SparkIMU(hardwareMap);


        try{

            limelight = hardwareMap.get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
            limelight.pipelineSwitch(2); // Switch to pipeline number 2
            limelight.start(); // This tells Limelight to start looking
            telemetry.addData("running: ", limelight.isRunning());

            LLResult results = limelight.getLatestResult();
            telemetry.addData("pl: ", results.getPipelineIndex());
            telemetry.update();

        }catch (Exception e){
            telemetry.addData("exception: ", e.toString());
            telemetry.update();
        }
        br.setDirection(DcMotorSimple.Direction.REVERSE);
        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        enc_left = hardwareMap.get(DcMotorEx.class, "enc_left");
        leftX = new BoilerDeadwheel(enc_left);
        rightX = new BoilerDeadwheel(br);
        middle = new BoilerDeadwheel(outtake.oSlideLeft);
    }
    public void teleDrive(double x, double y, double r, double right_trigger, double rotationalMult, double latMult, boolean reversed,
                          boolean strafe_left, boolean strafe_right){
        double reverse = reversed ? -1 : 1;
        double speedmult = right_trigger > 0 ? 0.4 : 2;
        double rotational = r * rotationalMult * reverse;
        boolean fieldCentric = true;
        if(strafe_left){
            x = -0.5 * reverse;
            y = 0;
            rotational = 0;
            fieldCentric = false;
        }else if(strafe_right){
            x = 0.5 * reverse;
            y = 0;
            rotational = 0;
            fieldCentric = false;
        }else{
            x = -reverse * (x) * speedmult * latMult;
            y = -reverse * (y) * speedmult;
        }
        double mag = Math.pow(x, 2) + Math.pow(y, 2);
        mag = Math.sqrt(mag);
        double rads = Math.atan2(y, x);
        rads = (rads >= 0 && rads < Math.toRadians(270)) ? (-1 * rads) + Math.toRadians(90) : (-1 * rads) + Math.toRadians(450);
        rads = (rads < 0) ? Math.toRadians(360) + rads : rads;
        drive(mag,rads,rotational,fieldCentric);
    }
    public void drive(double mag, double direction,double rotational, boolean fieldCentric){
        if(fieldCentric){
            double rangle = otos.otos.getPosition().h;
            rangle = (rangle < 0) ? Math.toRadians(360) + rangle : rangle;

            double turn = (direction < rangle) ? (Math.toRadians(360) - rangle) + (Math.abs(0 - direction)) : direction - rangle;
            //double turn = rads;
            double equationone = (Math.sin(turn + (Math.PI / 4)) * mag);
            double equationtwo = -(Math.sin(turn - (Math.PI / 4)) * mag);
            fr.setPower((equationone + rotational));
            bl.setPower((equationone - rotational));
            br.setPower((equationtwo + rotational));
            fl.setPower((equationtwo - rotational));
        }else{
            //double turn = rads;
            double equationone = (Math.sin(direction + (Math.PI / 4)) * mag);
            double equationtwo = -(Math.sin(direction - (Math.PI / 4)) * mag);
            fr.setPower((equationone + rotational));
            bl.setPower((equationone - rotational));
            br.setPower((equationtwo + rotational));
            fl.setPower((equationtwo - rotational));
        }
    }
    public void strafeTilStop(double power){
        ElapsedTime runTimer = new ElapsedTime();
        runTimer.reset();
        while (!middle.isStopped()){
            fr.setPower(-power);
            bl.setPower(-power);
            br.setPower(power);
            fl.setPower(power);
        }
        fr.setPower(0);
        bl.setPower(0);
        br.setPower(0);
        fl.setPower(0);
    }
    public void simpleDrive(double power){
        fr.setPower(power);
        bl.setPower(power);
        br.setPower(power);
        fl.setPower(power);
    }
    public double[] llcheck(){
        try{
            LLResult result = limelight.getLatestResult();
            double tx = result.getTx();
            double ty = result.getTy();
            if(Math.abs(tx) < 6 && Math.abs(ty) < 6 && result.isValid()){
                String classname = result.getDetectorResults().get(0).getClassName();
                double colorIndex = 0;
                if(classname.equals("red")){
                    colorIndex = 1;
                }else if(classname.equals("blue")){
                    colorIndex = 2;
                }else if(classname.equals("yellow")){
                    colorIndex = 0;
                }
                double arr[] = {0.35, colorIndex};
                return arr;
            }else if(Math.abs(tx) < 20 && result.isValid()){
                double arr[] = {0.2, 3};
                return arr;
            }
        }catch (Exception e){
        }
        double arr[] = {0, -1};
        return arr;
    }


}
