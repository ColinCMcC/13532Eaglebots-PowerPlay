package org.firstinspires.ftc.teamcode.Olaf;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

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
        Eagle.liftHome();// Brings lift back to the home to reset it
        Eagle.claw.setPosition(0.0);// Makes sure claw is all the way open to start

        while (opModeIsActive() && Eagle.Distance.getDistance(DistanceUnit.INCH) > 2 && Eagle.backDist.getDistance(DistanceUnit.INCH) < 48) {// Moves up to near cone while staying straight
            double strafePower = Eagle.rightDist.getDistance(DistanceUnit.INCH) - 27;// Calculates power to strafe back to correct position
            double turnPower = Eagle.getHeading();// Calculates power to spin back to straight

            Eagle.move(-0.3, strafePower / 4, turnPower / 50, true);
            sleep(100);// Sets how often it loops
            Eagle.checkData();
        }
        int color = Eagle.colorSense();

        Eagle.colorMove(2, color);
    }// end runOpMode function
}//end EagleAuto class