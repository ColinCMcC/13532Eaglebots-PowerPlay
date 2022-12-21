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
    1 - Home
*/

@Autonomous

//sets program name to EaglebotAuto_v5 and includes linearOpMode
public class EaglebotAuto_Left extends LinearOpMode {

    //Lets this program call functions inside of Eagle anConfig
    EaglebotConfig_v5 Eagle = new EaglebotConfig_v5(this);

    IMUDrive IMUDrive = new IMUDrive();


    public void runOpMode() {
        Eagle.init();// initialize hardware and sensors

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Autonomous:", "Ready");
        telemetry.update();

        // Waits until start is pressed
        waitForStart();
        Eagle.liftHome();
        Eagle.claw.setPosition(0.0);

        boolean autoRun = false;
        while (opModeIsActive() && !autoRun && Eagle.backDist.getDistance(DistanceUnit.INCH) < 35) {
            Eagle.rotateToZero();
            Eagle.rideLeftWall(24.5);
            sleep(200);// Delays before the next run of rotateToZero

            if (Eagle.Distance.getDistance(DistanceUnit.INCH) < 2) {
                autoRun = true;
            }
        }

        autoRun = false;//resets autoRun for another iteration if necessary
        Eagle.claw.setPosition(1);//grabs cone to not run over it
        Eagle.colorMoveDistLeft();
    }// end runOpMode function
}//end EagleAuto class