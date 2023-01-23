package org.firstinspires.ftc.teamcode.Olaf;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
    Motor Config
    0 - Lmotor white
    1 - RMotor black
    2 - FMotor red
    3 - BMotor blue
    4 - LiftMotor

    Servo Config
    0 - Claw

    Digital Config
    0 - Home

    I2C Config
    0 - 0- IMU
    0 - 1 - leftDist
    1 - 0 - backDist
    2 - 0 - rightDist
    3 - 0 - V3color
*/

@Autonomous

//sets program name to EaglebotAuto_v5 and includes linearOpMode
public class EaglebotAuto_Right extends LinearOpMode {

    //Lets this program call functions inside of Eagle anConfig
    EaglebotConfig Eagle = new EaglebotConfig(this);

    public void runOpMode() {
        Eagle.init();// initialize hardware and sensors

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Autonomous:", "Ready");
        telemetry.update();

        waitForStart();// Waits until start is pressed
        Eagle.claw.setPosition(0.0);// Makes sure claw is all the way open to start
        Eagle.liftHome();// Homes the lift

        Eagle.liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Eagle.claw.setPosition(1.0);// Grabs preloaded cone
        sleep(500);

        // Move to cone to get color
        while (opModeIsActive() && Eagle.Distance.getDistance(DistanceUnit.INCH) > 2 && Eagle.backDist.getDistance(DistanceUnit.INCH) < 48) {// Moves up to near cone while staying straight
            double strafePower = -Eagle.leftDist.getDistance(DistanceUnit.INCH) - 27;// Calculates power to strafe back to correct position
            double turnPower = Eagle.getHeading();// Keeps robot straight

            Eagle.move(-0.3, strafePower / 4, turnPower / 50, true);
            sleep(100);// Sets how often it loops
            Eagle.checkData();
        }

        int color = Eagle.colorSense();// Puts cone color into memory for later use

        // Moves to the top of B2
        while (opModeIsActive() && Eagle.backDist.getDistance(DistanceUnit.INCH) < 25){
            double turnPower = Eagle.getHeading();// Keeps robot straight

            Eagle.move(0.3, 0, turnPower / 20, true);
            sleep(100);// Sets how often it loops
        }

        // Lift motor to medium pole
        Eagle.liftMotor.setTargetPosition(2000);
        Eagle.liftMotor.setPower(1);
        while (Eagle.liftMotor.isBusy()){
            idle();
        }

        // Moves to the pole at the top left of E2
        while (opModeIsActive() && Eagle.rightDist.getDistance(DistanceUnit.INCH) < 40){
            double turnPower = Eagle.getHeading();// Keeps robot straight

            Eagle.move(0, -0.3, turnPower / 20, true);
            sleep(100);// Sets how often it loops
        }
        Eagle.stopDrive();

        Eagle.claw.setPosition(0.0);// Drops cone onto pole
        sleep(500);

        // Moves to the top center of E2
        while (opModeIsActive() && Eagle.rightDist.getDistance(DistanceUnit.INCH) > 26){
            double turnPower = Eagle.getHeading();// Keeps robot straight

            Eagle.move(0, 0.3, turnPower, true);
            sleep(100);// Sets how often it loops
        }

        Eagle.liftMotor.setTargetPosition(250);

        // Moves to the center of E3
        while (opModeIsActive() && Eagle.backDist.getDistance(DistanceUnit.INCH) < 48){
            double turnPower = Eagle.getHeading();// Keeps robot straight

            Eagle.move(-0.3, 0, turnPower / 20, true);
            sleep(100);// Sets how often it loops
        }

        // Turns to the right
        while (opModeIsActive()){
            double turnPower = Eagle.getHeading() + 90;// Turns robot to be facing 90 degrees to the right

            Eagle.move(0, 0, turnPower / 20, true);
            sleep(100);// Sets how often it loops
        }

        // Moves to the cone stack
        while (opModeIsActive() && Eagle.Distance.getDistance(DistanceUnit.INCH) > 2){
            double drivePower = Eagle.Distance.getDistance(DistanceUnit.INCH);
            double turnPower = Eagle.getHeading();// Keeps robot straight

            Eagle.move(-drivePower / 10, 0, turnPower / 20, true);
            sleep(100);// Sets how often it loops
        }

        Eagle.liftMotor.setTargetPosition(1000);// Lifts claw to correct height
        sleep(500);
        Eagle.claw.setPosition(1.0);// Grabs cone
        sleep(500);

        Eagle.colorMove(1, color);// Side 1 is left, color is what the cone was set to
    }// end runOpMode function
}//end EagleAuto class