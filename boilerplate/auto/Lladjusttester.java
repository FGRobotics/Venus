package org.firstinspires.ftc.teamcode.boilerplate.auto;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.boilerplate.BoilerBot;

@Disabled
public class Lladjusttester extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BoilerBot robot = new BoilerBot(hardwareMap, telemetry);
        while(opModeInInit()){
            robot.deliverySub.setOUTTAKE_MECH_TRANSFER();
            robot.clawSub.setINTAKE_Middle();

            telemetry.addData("heading: ", Math.toDegrees(robot.chassis.otos.otos.getPosition().h));
            telemetry.addData("rotPid: ", robot.odo.rotationalPID(0,1,0,0));
            telemetry.addData("l: ", robot.chassis.leftX.getCurrentPosition());
            telemetry.addData("m: ", robot.chassis.middle.getCurrentPosition());
            telemetry.addData("r: ", robot.chassis.rightX.getCurrentPosition());
            telemetry.addData("x: ", robot.chassis.otos.otos.getPosition().x);
            telemetry.addData("y: ", robot.chassis.otos.otos.getPosition().y);
            LLResult result = robot.chassis.limelight.getLatestResult();
            if (result != null && result.isValid()) {
                double tx = result.getTx(); // How far left or right the target is (degrees)
                double ty = result.getTy(); // How far up or down the target is (degrees)
                double ta = result.getTa(); // How big the target looks (0%-100% of the image)

                telemetry.addData("Target X", tx);
                telemetry.addData("Target Y", ty);
                telemetry.addData("Target Area", ta);
            } else {
                telemetry.addData("Limelight", "No Targets");
            }
            if(robot.chassis.limelight == null){
                telemetry.addLine("null");
            }else{
                telemetry.addLine("not null");
            }
            try {
                if (robot.chassis.limelight.isRunning()){
                    telemetry.addLine(robot.chassis.limelight.getStatus().toString());
                    telemetry.addData("pl: ", robot.chassis.limelight.getLatestResult().getPipelineIndex());
                    telemetry.addData("degreesAway: ", robot.odo.exceptionHandledLLResult());
                    telemetry.update();
                }else{
                    telemetry.addData("limelight: ", false);
                    telemetry.update();
                }
            }catch (Exception e) {
                telemetry.addData("limelight: ", "doesnt exist");
                telemetry.addData("e: ", e.toString());
                telemetry.update();
            }



        }


        waitForStart();

        robot.odo.llAdjustLeft(()->true);
        sleep(2000);


    }
}
