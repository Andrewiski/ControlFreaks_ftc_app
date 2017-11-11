package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ControlFreaks.CFPushBotHardware;

/**
 * Created by adevries on 10/7/2017.
 */
@TeleOp(name="Servo Test", group="Manual" )  // @Autonomous(...) is the other common choice
//@Disabled
public class CFPushBotManualServoTest extends LinearOpMode {

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
    boolean buttonXReleased = true;
    boolean buttonBReleased = true;
    boolean buttonAReleased = true;
    boolean buttonDpadUpReleased = true;
    boolean buttonDpadDownReleased = true;
    boolean buttonDpadRightReleased = true;
    boolean buttonDpadLeftReleased = true;
    boolean buttonLeftBumperReleased = true;
    boolean buttonRightBumperReleased = true;
    boolean buttonG2XReleased = true;
    boolean buttonG2YReleased = true;
    boolean buttonG2AReleased = true;
    boolean buttonG2BReleased = true;
    ElapsedTime myFliperRetractElapsedTime;
    @Override
    public void runOpMode() throws InterruptedException {

        try {
            /* Declare OpMode members. */
            robot           = new CFPushBotHardware();   // Use a Pushbot's hardware
            robot.init(this);

            robot.setupManualDrive();
            //robot.vuforia_Init();
            // Wait for the game to start (driver presses PLAY)
            waitForStart();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                try {
                    robot.hardware_loop();

                    if(gamepad1.left_trigger > .2) {
                        robot.wrist_setposition(-gamepad1.left_stick_y);
                    }
                    if(gamepad1.left_bumper){
                        if(buttonLeftBumperReleased == true) {
                            robot.wrist_retract();
                            buttonLeftBumperReleased = false;
                        }
                    }else{
                        buttonLeftBumperReleased = true;
                    }
                    if(gamepad1.right_bumper){
                        if(buttonRightBumperReleased == true) {
                            robot.wrist_extend();
                            buttonRightBumperReleased = false;
                        }
                    }else{
                        buttonRightBumperReleased = true;
                    }
                    if(gamepad1.dpad_up) {
                        robot.wrist_step(.05);
                    }
                    if(gamepad1.dpad_down){
                        robot.wrist_step(-.05);
                    }

                    if(gamepad1.dpad_right){
                        if(buttonDpadRightReleased == true) {
                            robot.wrist_step(-.05);
                            buttonDpadRightReleased = false;
                        }
                    }else{
                        buttonDpadRightReleased = true;
                    }
                    if(gamepad1.dpad_left){
                        if(buttonDpadLeftReleased == true) {
                            robot.wrist_step(.05);
                            buttonDpadLeftReleased = false;
                        }
                    }else{
                        buttonDpadLeftReleased = true;
                    }

                    if(gamepad1.x){
                        if(buttonXReleased == true){
                            buttonXReleased = false;
                            robot.sensor_range_enable(true);
                        }
                        robot.set_message("Range " +  robot.sensor_range_get_distance());
                    }else{
                        if(buttonXReleased == false){
                            buttonXReleased = true;
                            robot.sensor_range_enable(false);
                        }
                    }



                    //robot.waitForTick(2);
                }catch(Exception ex){
                    robot.set_error_message("Fatal Error "  + ex.toString());
                }
            } // loop

        }catch(Exception ex){
            robot.set_error_message("Fatal Error "  + ex.toString());
        }
    }


}
