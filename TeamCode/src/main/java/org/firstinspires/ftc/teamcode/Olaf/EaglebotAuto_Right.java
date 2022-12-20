package org.firstinspires.ftc.teamcode.Olaf;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
public class EaglebotAuto_Right extends LinearOpMode {

    //Lets this program call functions inside of Eagle anConfig
    EaglebotConfig_v5 Eagle = new EaglebotConfig_v5(this);

    IMUDrive IMUDrive = new IMUDrive();

    ElapsedTime runtime = new ElapsedTime();

    public void runOpMode()
    {
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
            Eagle.rideRightWall(24.5);
            sleep(200);

            if (Eagle.Distance.getDistance(DistanceUnit.INCH) < 2) {autoRun = true;}
        }
        Eagle.claw.setPosition(1);
        autoRun = false;
        Eagle.colorMoveDistRight();
    }// end runOpMode function
}//end EagleAuto class