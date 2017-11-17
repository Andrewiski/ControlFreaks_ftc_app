package org.firstinspires.ftc.teamcode.ControlFreaks;

//import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Calendar;

/**
 * Created by adevries on 11/13/2017.
 */
public class PixyCamera {
    static final String logId = "Pixy:";       // Tag identifier in FtcRobotController.LogCat
    private String v_deviceName = "Not Set";
    private byte I2CAddress = 0x54;
    private Wire v_pixy;

    private static final int PIXY_START_WORD = 0xaa55;
    private static final int PIXY_START_WORD_CC = 0xaa56;
    private static final int PIXY_START_WORDX = 0x55aa;

    private static final byte PIXY_SERVO_SYNC = (byte) 0xff;
    private static final byte PIXY_CAM_BRIGHTNESS_SYNC = (byte) 0xfe;
    private static final byte PIXY_LED_SYNC = (byte) 0xfd;

    public enum BlockType {
        NORMAL_BLOCK,
        CC_BLOCK
    }

    private BlockType g_blockType;
    private boolean v_pixy_enabled = false;

    public PixyCamera(HardwareMap hardwareMap, String deviceName, byte RealI2CAddress) throws Exception {
        try {
            I2CAddress = RealI2CAddress;
            v_deviceName = deviceName;
            v_pixy = new Wire(hardwareMap, deviceName, I2CAddress * 2);
            init();
        } catch (Exception p_exeception) {
            debugLogException("Error Creating Wire", p_exeception);
            v_pixy = null;
            throw p_exeception;
        }
    }

    public PixyCamera(HardwareMap hardwareMap, String deviceName) throws Exception

    {
        //
        // Initialize base classes.
        //
        // All via self-construction.

        //
        // Initialize class members.
        //
        // All via self-construction.
        try {
            v_deviceName = deviceName;
            v_pixy = new Wire(hardwareMap, deviceName, I2CAddress * 2);
            init();

        } catch (Exception p_exeception) {
            debugLogException("Error Creating Wire", p_exeception);
            v_pixy = null;
        }

    }

    private void init() throws Exception {
        try {
            if (v_pixy != null) {
//                v_ledseg.write(0x21, 0);
//                v_ledseg.write(0x81, 0);
//                v_ledseg.beginWrite(0x00);
//                v_ledseg.write(numbertable[0]);
//                v_ledseg.write(0x00);
//                v_ledseg.write(numbertable[0]);
//                v_ledseg.write(0x00);
//                v_ledseg.write(0x2);
//                v_ledseg.write(0x00);
//                v_ledseg.write(numbertable[0]);
//                v_ledseg.write(0x00);
//                v_ledseg.write(numbertable[0]);
//                v_ledseg.write(0x00);
//                v_ledseg.endWrite();
                v_pixy_enabled = true;
            }
        } catch (Exception p_exeception) {
            debugLogException("Error Init", p_exeception);
            throw p_exeception;
        }
    }


    /**
     * put this in your loop so its called over and over in the loop
     */
    boolean readingData = false;
    byte[] buffer;
    public void loop() {
        if(v_pixy_enabled){
            if(readingData == false) {
                v_pixy.beginWrite(0x00);
                v_pixy.endWrite();
                readingData = true;
                v_pixy.getResponse();
            }else{

            }
        }
    }
/*
int setServos(uint16_t s0, uint16_t s1)
{
  uint8_t outBuf[6];

  outBuf[0] = 0x00;
  outBuf[1] = PIXY_SERVO_SYNC;
  *(uint16_t *)(outBuf + 2) = s0;
  *(uint16_t *)(outBuf + 4) = s1;

  return send(outBuf, 6);
}

 */

    public boolean set_servos(int s0, int s1){
        if(v_pixy!=null) {
            v_pixy.beginWrite(0x00);
            v_pixy.write(PIXY_SERVO_SYNC);
            v_pixy.write(s0); //two bytes here may need shift
            v_pixy.write(s1);
            v_pixy.endWrite();
            return true;
        }else{
            return false;
        }
    }

    public boolean set_led(byte red, byte green, byte blue){
        if(v_pixy!=null) {
            v_pixy.beginWrite(0x00);
            v_pixy.write(PIXY_LED_SYNC);
            v_pixy.write(red);
            v_pixy.write(green);
            v_pixy.write(blue);
            v_pixy.endWrite();
            return true;
        }else{
            return false;
        }
    }

    public boolean set_brightness(byte brightness) {
        if(v_pixy!=null) {
            v_pixy.beginWrite(0x00);
            v_pixy.write(PIXY_CAM_BRIGHTNESS_SYNC);
            v_pixy.write(brightness);
            v_pixy.endWrite();
            return true;
        }else{
            return false;
        }
    }



    public boolean isEnabled(){
        return v_pixy_enabled;
    }
    public boolean enabled(boolean enable){
        try{
            if (v_pixy_enabled != enable) {
                v_pixy_enabled = enable;
            }
            return true;
        }catch (Exception p_exeception)
        {
            debugLogException("Error Init", p_exeception);
            return false;
        }
    }

    public void stop() {
        if(v_pixy != null) {
            v_pixy.close();
        }
    }
    void debugLogException( String msg, Exception ex){

        String debugMessage = logId + msg;
        if (ex != null) {
            String errMsg = ex.getLocalizedMessage();
            if (errMsg != null) {
                debugMessage = debugMessage + errMsg;
            }else{
                debugMessage = debugMessage + " error. is null";
            }
        }else{
            debugMessage = debugMessage + " error is null";
        }
        android.util.Log.d("pixyCamera",debugMessage);
        //DbgLog.msg(debugMessage);
        //telemetry.addData(line, debugMessage);
    }
}
