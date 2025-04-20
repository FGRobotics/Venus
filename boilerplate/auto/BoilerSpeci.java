package org.firstinspires.ftc.teamcode.boilerplate.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.boilerplate.BoilerBot;

@Autonomous(name = "speci")
public class BoilerSpeci extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        BoilerBot robot = new BoilerBot(hardwareMap, telemetry);
        while(opModeInInit()){
            robot.deliverySub.setOUTTAKE_MECH_TRANSFER();
            telemetry.addData("heading: ", Math.toDegrees(robot.chassis.otos.otos.getPosition().h));
            telemetry.addData("rotPid: ", robot.odo.rotationalPID(0,1,0,0));
            telemetry.addData("l: ", robot.chassis.leftX.getCurrentPosition());
            telemetry.addData("m: ", robot.chassis.middle.getCurrentPosition());
            telemetry.addData("r: ", robot.chassis.rightX.getCurrentPosition());
            telemetry.update();
        }
        waitForStart();
        robot.chassis.otos.resetIMU();


        robot.deliverySub.setOUTTAKEFULLSPECI();
        robot.clawSub.specimenFloorUp();
        robot.odo.profilemove(8, -32, 0.5);
        robot.chassis.simpleDrive(-0.4);
        sleep(300);
        while(!robot.chassis.leftX.isStopped()){}
        sleep(200);
        robot.odo.whileController(()->robot.oSlides.setSpeci());
        robot.odo.whileController(()->robot.oSlides.homeOuttake());
        robot.deliverySub.setOUTTAKE_MECH_TRANSFER();


        //push one in
        robot.odo.profilemove(-32, -15, 0.5);
        robot.odo.profilemove(-32, -50, 0.5);
        robot.odo.profilemove(-45, -50, 0.5);
        robot.odo.profilemove(-45, -11, 0.5);
        robot.odo.profilemove(-45, -19, 0.5);
        sleep(500);

        //pickup drop 1
        robot.clawSub.speciGroundPickup();
        robot.odo.whileController(()->robot.stateBasedTransfer(false,true), 0, ()->robot.deliverySub.setOUTTAKEFULLSPECI());
        robot.odo.profilemove(10.7, -32, 0.5);
        robot.chassis.simpleDrive(-0.4);
        sleep(300);
        while(!robot.chassis.leftX.isStopped()){}
        sleep(200);
        robot.odo.whileController(()->robot.oSlides.setSpeci());
        robot.odo.whileController(()->robot.oSlides.homeOuttake());
        robot.deliverySub.setOUTTAKE_MECH_TRANSFER();

        //pickup drp 2
        robot.clawSub.specimenFloorUp();
        robot.odo.profilemove(10.7, -18, 0.5);
        robot.odo.profilemove(-45, -18, 0.5);
        robot.clawSub.speciGroundPickup();
        robot.odo.whileController(()->robot.stateBasedTransfer(false,true), 0, ()->robot.deliverySub.setOUTTAKEFULLSPECI());
        robot.odo.profilemove(4, -32, 0.5);
        robot.odo.profilemove(-3, -32, 0.5);
        robot.chassis.simpleDrive(-0.4);
        sleep(300);
        while(!robot.chassis.leftX.isStopped()){}
        sleep(200);
        robot.odo.whileController(()->robot.oSlides.setSpeci());
        robot.odo.whileController(()->robot.oSlides.homeOuttake());
        robot.deliverySub.setOUTTAKE_MECH_TRANSFER();

        robot.odo.profilemove(-1, -20, 0.5);
        robot.odo.profilemove(-50, -5, 0.5);















    }
}
