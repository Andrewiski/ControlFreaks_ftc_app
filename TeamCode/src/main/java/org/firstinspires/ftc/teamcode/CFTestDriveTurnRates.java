package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.teamcode.ControlFreaks.CFPushBotHardware;

/**
 * Created by adevries on 11/6/2015.
 */
@TeleOp(name="Mech Test Run To Position", group="Manual")  // @Autonomous(...) is the other common choice
@Disabled
public class CFTestDriveTurnRates extends LinearOpMode {

    /* Declare OpMode members. */
    CFPushBotHardware robot;   // Use a Pushbot's hardware



    //--------------------------------------------------------------------------
    //
    // loop
    //
    /**
     * Implement a state machine that controls the robot during
     * manual-operation.  The state machine uses gamepad input to transition
     * between states.
     *
     * The system calls this member repeatedly while the OpMode is running.
     */
    float stickdeadzone = .1f;
    boolean isFirstTimeButtonPress = true;
    @Override
    public void runOpMode() throws InterruptedException {

        try {
            /* Declare OpMode members. */
            robot           = new CFPushBotHardware();   // Use a Pushbot's hardware
            robot.isDriveAndyMark20 = false;
            robot.isMechDrive = true;
            robot.debugOn();
            robot.init(this);
            robot.set_message("left stick button for manual, dpad 36 inch, bumper strife");
            robot.setupManualDrive();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();
            boolean isDriveInchesInProgress = false;
            boolean isTurnInProgress = false;
            boolean isManualMode = true;

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                robot.hardware_loop();

                if(isDriveInchesInProgress ){
                  if(robot.drive_inches_complete()){
                      isDriveInchesInProgress = false;

                  }
                }else if(isTurnInProgress){
                    if(robot.turn_complete()){
                        isTurnInProgress = false;
                    }
                }else{
                    if(isManualMode == true){
                        robot.drive_set_power(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
                    }else {
                        //robot.update_gamepad_telemetry();
                        if (gamepad1.dpad_up) {
                            robot.setupAutoDrive();
                            robot.drive_inches( 36, true);
                            isDriveInchesInProgress = true;
                        }
                        if (gamepad1.dpad_left) {
                            robot.setupAutoDrive();
                            robot.turn_degrees(-90, gamepad1.right_bumper, true);
                            isTurnInProgress = true;
                        }
                        if (gamepad1.dpad_right) {
                            robot.setupAutoDrive();
                            robot.turn_degrees(90, gamepad1.right_bumper, true);
                            isTurnInProgress = true;
                        }
                        if (gamepad1.dpad_down) {
                            robot.setupAutoDrive();
                            robot.drive_inches( -36, true);
                            isDriveInchesInProgress = true;
                        }
                        if(gamepad1.left_bumper){
                            robot.setupAutoDrive();
                            robot.drive_inches_strife( -36, true);
                            isDriveInchesInProgress = true;
                        }
                        if(gamepad1.right_bumper){
                            robot.setupAutoDrive();
                            robot.drive_inches_strife( 36, true);
                            isDriveInchesInProgress = true;
                        }
                    }


                }
                if(gamepad1.left_stick_button){
                    if(isManualMode == false) {
                        robot.hardware_stop();
                        robot.setupManualDrive();
                        isTurnInProgress = false;
                        isDriveInchesInProgress = false;
                        isManualMode = true;
                        robot.set_message("in Manual Mode");
                    }else {
                        robot.setupAutoDrive();
                        isManualMode = false;
                        robot.set_message("in Auto Mode");

                    }
                }
                //robot.waitForTick(5);
            } // loop

        }catch(Exception ex){
            robot.set_error_message("Fatal Error "  + ex.toString());
        }
    }


}
