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
public class EaglebotAuto_Left extends LinearOpMode {

    //Lets this program call functions inside of Eagle anConfig
    EaglebotConfig_v5 Eagle = new EaglebotConfig_v5(this);


    public void runOpMode() {
        Eagle.init();// Initialize hardware and sensors

        telemetry.addData("Autonomous:", "Ready");// Send telemetry message to signify robot waiting;
        telemetry.update();

        waitForStart();// Waits until start is pressed
        Eagle.liftHome();
        Eagle.claw.setPosition(0.0);// Makes sure claw is all the way open to start

        while (opModeIsActive() && Eagle.Distance.getDistance(DistanceUnit.INCH) > 2 && Eagle.backDist.getDistance(DistanceUnit.INCH) < 48) {// Moves up to near cone while staying straight

            double strafePower = -Eagle.leftDist.getDistance(DistanceUnit.INCH) - 27;// Calculates power to strafe back to correct position
            double turnPower = Eagle.getHeading();// Calculates power to spin back to straight
            Eagle.move(-0.3, strafePower / 4, turnPower / 50, false);
            sleep(100);
            Eagle.checkData();
        }
        Eagle.colorMove(1);
    }// end runOpMode function
}//end EagleAuto class