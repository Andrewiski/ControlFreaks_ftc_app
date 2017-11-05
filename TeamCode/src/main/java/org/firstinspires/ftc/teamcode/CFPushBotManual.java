package org.firstinspires.ftc.teamcode;

import android.os.CountDownTimer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.ControlFreaks.*;
/**
 * Created by adevries on 11/6/2015.
 */
@TeleOp(name="CF Tank", group="Manual")  // @Autonomous(...) is the other common choice
//@Disabled
public class CFPushBotManual extends LinearOpMode {

    /* Declare OpMode members. */
    CFPushBotHardware robot;   // Use a Pushbot's hardware

    private static boolean bothControllersEnabled = false;
    private byte v_neopixels_mode = 0;


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
    boolean buttonDpadleftReleased = true;
    boolean buttonDpadrightReleased = true;
    boolean buttonDpadUpReleased = true;
    boolean buttonDpadDownReleased = true;
    boolean buttonLeftBumperReleased = true;
    boolean buttonRightBumperReleased = true;

    boolean buttonG2DpadleftReleased = true;
    boolean buttonG2DpadrightReleased = true;
    boolean buttonG2DpadUpReleased = true;
    boolean buttonG2DpadDownReleased = true;
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
            //robot.led7seg_timer_init(120);
            robot.run_without_encoders();

            waitForStart();
            //robot.debugOff();
            //robot.led7seg_timer_start(120);


            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                try {
                    robot.hardware_loop();
                    if(gamepad1.left_trigger > .2 || gamepad1.right_trigger > .2 ) {
                        robot.set_drive_power(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
                    }else{
                        robot.set_drive_power(-gamepad1.left_stick_y/1.5f, -gamepad1.right_stick_y/1.5f);
                    }

                    if(gamepad2.left_stick_y > .2 ) {
                        robot.shoulder_step(.02);
                    }else if(gamepad2.left_stick_y < -.2)
                    {
                        robot.shoulder_step(-.02);
                    }

                    if(gamepad2.right_stick_y > .2 ) {
                        robot.slider_step(100);
                    }else if(gamepad2.right_stick_y < -.2)
                    {
                        robot.slider_step(-100);
                    }


                    if(gamepad1.left_bumper){
                        if(buttonLeftBumperReleased == true) {
                            robot.jewel_extend();
                            buttonLeftBumperReleased = false;
                        }
                    }else{
                        buttonLeftBumperReleased = true;
                    }
                    if(gamepad1.right_bumper){
                        if(buttonRightBumperReleased == true) {
                            robot.jewel_retract();
                            buttonRightBumperReleased = false;
                        }
                    }else{
                        buttonRightBumperReleased = true;
                    }

                    if(gamepad1.dpad_up){
                        robot.hand_open();
                        if(buttonDpadUpReleased == true) {
                            buttonDpadUpReleased = false;
                        }
                    }else{
                        buttonDpadUpReleased = true;
                    }
                    if(gamepad1.dpad_down){
                        robot.hand_close();
                        if(buttonDpadDownReleased == true) {
                            buttonDpadDownReleased = false;
                        }
                    }else{
                        buttonDpadDownReleased = true;
                    }
                    if(gamepad1.dpad_left){
                        if(buttonDpadleftReleased == true) {

                            buttonDpadleftReleased = false;
                        }
                        robot.slider_step(100);
                    }else {
                        buttonDpadleftReleased = true;
                    }
                    if(gamepad1.dpad_right){
                        if(buttonDpadrightReleased == true) {

                            buttonDpadrightReleased = false;
                        }
                        robot.slider_step(-100);
                    }else {
                        buttonDpadrightReleased = true;
                    }


                    if(gamepad2.dpad_up){
                        robot.lifter_up();
                        if(buttonG2DpadUpReleased == true) {
                            buttonG2DpadUpReleased = false;
                        }
                    }else{
                        if(buttonG2DpadUpReleased == false){
                            robot.lifter_off();
                        }
                        buttonG2DpadUpReleased = true;

                    }
                    if(gamepad2.dpad_down){
                        robot.lifter_down();
                        if(buttonG2DpadDownReleased == true) {

                            buttonG2DpadDownReleased = false;
                        }
                    }else{
                        if(buttonG2DpadDownReleased == false){
                            robot.lifter_off();
                        }
                        buttonG2DpadDownReleased = true;
                    }

                    if(gamepad2.dpad_left){
                        if(buttonG2DpadleftReleased == true) {
                            robot.blockslide_left();
                            buttonG2DpadleftReleased = false;
                        }
                    }else {
                        buttonG2DpadleftReleased = true;
                    }
                    if(gamepad2.dpad_right){
                        if(buttonG2DpadrightReleased == true) {
                            robot.blockslide_right();
                            buttonG2DpadrightReleased = false;
                        }
                    }else {
                        buttonG2DpadrightReleased = true;
                    }
                    if(gamepad1.x ){
                        if(buttonXReleased == true) {
                            robot.blueled_toggle();
                            buttonXReleased = false;
                        }
                    }else{
                        buttonXReleased = true;
                    }
                    if(gamepad1.b){
                        if(buttonBReleased == true) {
                            robot.redled_toggle();
                            buttonBReleased = false;
                        }
                    }else{
                        buttonBReleased = true;
                    }
                    if(gamepad2.a){
                            if(buttonG2AReleased == true) {
                                robot.blockgrabber_toggle();
                                buttonG2AReleased = false;
                            }
                    }else{
                        buttonG2AReleased = true;
                    }
                    if(gamepad2.x){
                        if(buttonG2XReleased == true) {
                            //robot.rackpinion_load();
                            buttonG2XReleased = false;
                        }
                    }else{
                        if(buttonG2XReleased == false) {
                            //if (robot.catapult_load_complete()) {
                                buttonG2XReleased = true;
                            //}
                        }
                    }
                    if(gamepad2.b){
                        if(buttonG2BReleased == true) {
                            robot.hand_toggle();
                            buttonG2BReleased = false;
                        }
                    }else{
                        if(buttonG2BReleased == false) {
                            buttonG2BReleased = true;
                        }
                    }
                    if(gamepad2.y){
                        if(buttonG2YReleased == true) {
                            robot.sensor_color_enable(true);
                            robot.sensor_color_led(true);

                            //myFliperRetractElapsedTime = null;
                            buttonG2YReleased = false;
                        }else {
                            int myColor = robot.sensor_color_GreatestColor();
                            if (myColor == 0) {
                                robot.redled_on();
                                robot.blueled_off();
                            } else if (myColor == 2) {

                                robot.redled_off();
                                robot.blueled_on();
                            }else {
                                robot.redled_off();
                                robot.blueled_off();
                            }
                        }
                    }else{
                        buttonG2YReleased = true;
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
