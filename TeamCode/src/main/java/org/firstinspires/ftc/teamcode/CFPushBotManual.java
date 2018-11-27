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
    boolean buttonYReleased = true;
    boolean buttonBReleased = true;
    boolean buttonAReleased = true;
    boolean buttonStartReleased = true;
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
    boolean button_rightstick_deadzone = true;

    boolean highspeedmode = false;
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
            robot.run_using_encoders();
           // robot.lifter_step(-500); //reset the min on lifter as automonmouse moved us up and reinit reset min

            waitForStart();
            //robot.debugOff();
            //robot.led7seg_timer_start(120);
            //robot.blockgrabber_open();

            // run until the end of the match (driver presses STOP)
            while (opModeIsActive()) {
                try {
                    robot.hardware_loop();
//
                    if(highspeedmode || gamepad1.left_trigger > .2 || gamepad1.right_trigger > .2 ) {
                        robot.drive_set_power(-gamepad1.left_stick_y, -gamepad1.right_stick_y);
                    }else{
                        //robot.set_drive_power(-gamepad1.left_stick_y/2.7f, -gamepad1.right_stick_y/2.7f);
                        float left = -gamepad1.left_stick_y ;
                        float right = -gamepad1.right_stick_y;
                        if((left > 0 && right < 0)
                                || (right > 0 && left < 0)
                                || ((left > 0 || left < 0) && right == 0)
                                || ((right > 0 || right < 0) && left == 0)
                                ){
                            //we are turning so slow down the motors
                            left = left/3.2f;
                            right = right/3.2f;
                        }else{
                            left = left/2.5f;
                            right = right/2.5f;
                        }
                        robot.drive_set_power(left, right);
                    }

                    if(gamepad2.left_stick_y > .2 ) {
                        //robot.shoulder_step(-.001);
                    }else if(gamepad2.left_stick_y < -.2)
                    {
                        //robot.shoulder_step(.001);
                    }

                    if(gamepad2.right_stick_y > .2 ) {
                        //robot.slider_step(-100, gamepad2.left_bumper);
                        button_rightstick_deadzone = false;
                    }else if(gamepad2.right_stick_y < -.2)
                    {
                        //robot.slider_step(100, false);
                        button_rightstick_deadzone = false;
                    }else{
                        if(button_rightstick_deadzone == false) {
                            //robot.slider_stop(gamepad2.left_bumper);
                            button_rightstick_deadzone = true;
                        }else{
                            //robot.slider_hold();
                        }
                    }


                    if(gamepad1.left_bumper){
                        if(buttonLeftBumperReleased == true) {
                            if (robot.debugMode()){
                                //robot.jewel_toggle();
                            }

                            buttonLeftBumperReleased = false;
                        }
                    }else{
                        buttonLeftBumperReleased = true;
                    }
                    if(gamepad1.right_bumper){
                        if(buttonRightBumperReleased == true) {
                            //highspeedmode = !highspeedmode;
                            buttonRightBumperReleased = false;
                        }
                    }else{
                        buttonRightBumperReleased = true;
                    }

                    if(gamepad1.dpad_up){
                        //robot.hand_open();
                        if(buttonDpadUpReleased == true) {
                            buttonDpadUpReleased = false;
                        }
                    }else{
                        buttonDpadUpReleased = true;
                    }
                    if(gamepad1.dpad_down){
                        //robot.hand_close();
                        if(buttonDpadDownReleased == true) {
                            buttonDpadDownReleased = false;
                        }
                    }else{
                        buttonDpadDownReleased = true;
                    }
//                    if(gamepad1.dpad_left){
//                        if(buttonDpadleftReleased == true) {
//
//                            buttonDpadleftReleased = false;
//                        }
//                        robot.slider_step(100);
//                    }else {
//                        buttonDpadleftReleased = true;
//                    }
//                    if(gamepad1.dpad_right){
//                        if(buttonDpadrightReleased == true) {
//
//                            buttonDpadrightReleased = false;
//                        }
//                        robot.slider_step(-100);
//                    }else {
//                        buttonDpadrightReleased = true;
//                    }


                    if(gamepad2.dpad_up){

                        if(buttonG2DpadUpReleased == true) {
                            buttonG2DpadUpReleased = false;
                        }
                        //robot.lifter_step(100);
                    }else{
                        if(buttonG2DpadUpReleased == false){
                            //robot.lifter_stop();
                        }
                        buttonG2DpadUpReleased = true;

                    }
                    if(gamepad2.dpad_down){

                        if(buttonG2DpadDownReleased == true) {

                            buttonG2DpadDownReleased = false;
                        }
                        //robot.lifter_step(-100);
                    }else{
                        if(buttonG2DpadDownReleased == false){
                            //robot.lifter_stop();
                        }
                        buttonG2DpadDownReleased = true;
                    }

                    if(gamepad2.dpad_left){
                        if(buttonG2DpadleftReleased == true) {
                            //robot.blockslide_left();
                            buttonG2DpadleftReleased = false;
                        }
                    }else {
                        buttonG2DpadleftReleased = true;
                    }
                    if(gamepad2.dpad_right){
                        if(buttonG2DpadrightReleased == true) {
                            //robot.blockslide_right();
                            buttonG2DpadrightReleased = false;
                        }
                    }else {
                        buttonG2DpadrightReleased = true;
                    }
                    if(gamepad1.x ){
                        if(buttonXReleased == true) {
                            robot.blueled_toggle();
                            //highspeedmode = false;
                            buttonXReleased = false;
                        }
                    }else{
                        buttonXReleased = true;
                    }
                    if(gamepad1.b){
                        if(buttonBReleased == true) {
                            robot.redled_toggle();
                            //highspeedmode = true;
                            buttonBReleased = false;
                        }
                    }else{
                        buttonBReleased = true;
                    }
                    if(gamepad1.start && gamepad1.left_trigger > .2 ){
                        if(buttonStartReleased == true) {
                            if(robot.debugMode()){
                                robot.debugOff();
                            }else{
                                robot.debugOn();
                            }
                            buttonStartReleased = false;
                        }
                    }else{
                        buttonStartReleased = true;
                    }

                    if(gamepad2.left_trigger > .2 ){
                            //robot.bgtilt_step(30);

                    }else if(gamepad2.right_trigger > .2 ){
                        //robot.bgtilt_step(-30);

                    }else if(gamepad2.left_trigger > .1 || gamepad2.left_trigger > .1  ){
                        //robot.bgtilt_stop();

                    }

                    if(gamepad2.a){
                            if(buttonG2AReleased == true) {
                                //robot.blockgrabber_toggle();
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
                            //robot.hand_toggle();
                            buttonG2BReleased = false;
                        }
                    }else{
                        if(buttonG2BReleased == false) {
                            buttonG2BReleased = true;
                        }
                    }
                    if(gamepad1.y){
                        if(buttonYReleased == true) {
                            //robot.sensor_color_enable(true);
                            //robot.sensor_color_led(true);

                            //myFliperRetractElapsedTime = null;
                            buttonYReleased = false;
                        }else {
                            int myColor = 0; //robot.sensor_color_GreatestColor();
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
                        if(buttonYReleased == false){
                            //robot.sensor_color_led(false);
                            //robot.sensor_color_enable(false);
                            robot.redled_off();
                            robot.blueled_off();
                        }
                        buttonYReleased = true;

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
