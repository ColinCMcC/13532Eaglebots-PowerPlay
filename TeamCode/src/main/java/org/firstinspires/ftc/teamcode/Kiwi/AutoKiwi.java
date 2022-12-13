package org.firstinspires.ftc.teamcode.Kiwi;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Kiwi.KiwiConfig;

@Autonomous(name="AutoKiwi", group="Kiwi")

public class AutoKiwi extends LinearOpMode {

    // Create a RobotHardware object to be used to access robot hardware.
    // Prefix any hardware functions with "Kiwi." to access this class.
    KiwiConfig   Kiwi  = new KiwiConfig(this);
    @Override
    public void runOpMode() {

        // initialize all the hardware, using the hardware class. See how clean and simple this is?
        Kiwi.init();

        // Send telemetry message to signify robot waiting;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        if (opModeIsActive()) {

            // Send gamepad to KiwiDrive to calculate power to wheels
            //Kiwi.driveRotate(false);
            //Kiwi.driveRotate(true);
            /* 
            speed -1.0 to 1.0 double,  direction in radians
            0    3.142   0 to 180 use positive; 180 0.000 to 360 use negative
            15   2.880  105 1.309
            30   2.618  120 1.047
            45   2.356  135 0.785
            60   2.094  150 0.524
            75   1.833  165 0.262
            90   1.571  180 0.000
            */
            //Kiwi.drivePower(0.5,0.0);
            //sleep(700);
            Kiwi.drivePower(0.5, 1.571);
            sleep(700);
            //Kiwi.drivePower(0.5, 3.142);
            //sleep(700);
            Kiwi.drivePower(.5, -1.571);
            sleep(700);
            
            // Send telemetry messages to controls and show robot status
            telemetry.addData("- ", "-------");

            telemetry.addData("rho", "%.2f", Kiwi.rho(gamepad1.left_stick_y, gamepad1.left_stick_x));
            telemetry.addData("phi ", "%.2f", Kiwi.phi(gamepad1.left_stick_y, gamepad1.left_stick_x));
            telemetry.update();
            // Pace this loop so hands move at a reasonable speed.
            sleep(50);
        }
    }
}
