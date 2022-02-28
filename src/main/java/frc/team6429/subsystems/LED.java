// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team6429.subsystems;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;
import com.ctre.phoenix.led.TwinkleOffAnimation.TwinkleOffPercent;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.RgbFadeAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.StrobeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.TwinkleOffAnimation;
import com.ctre.phoenix.led.FireAnimation;

import frc.team6429.robot.Constants;
import frc.team6429.robot.RobotData.AnimationTypes;

/** Add your docs here. */
public class LED {

    private static LED mInstance = new LED();

    public static LED getInstance(){
        return mInstance;
    }

    //CANdle
    public CANdle candle;
    public CANdleConfiguration config;

    //Animation
    public RainbowAnimation rainbow;
    public ColorFlowAnimation colorFlow;
    public LarsonAnimation larson;
    public RgbFadeAnimation rgbFade;
    public SingleFadeAnimation singleFade;
    public StrobeAnimation strobe;
    public TwinkleAnimation twinkle;
    public TwinkleOffAnimation twinkleOff;
    public FireAnimation fire;
    public Direction _direction;
    public Animation _animation;
    public AnimationTypes animationTypes;


    private LED(){
      candle = new CANdle(Constants.candleID);
      config = new CANdleConfiguration();
      candle.configAllSettings(config);
      candle.getAllConfigs(config);
      config.stripType = LEDStripType.RGB; //set the strip type to RGB
    }

    /**
     * select and change animation type
     * @param option
     */
    public void 
    select(AnimationTypes option) {
        switch (option) {
          case COLORFLOW:
            setColorFlow(0, 0, 255, 0, 10, 68, Direction.Forward);
            break;
          case COLORFLOWOPP:
            setColorFlow(0, 0, 255, 0, 10, 68, Direction.Backward);
            break;
          case GREENFLOW:
            setColorFlow(0, 255, 0, 0, 10, 68, Direction.Forward);
            break;
          case GREENFLOWOPP:
            setColorFlow(0, 255, 0, 0, 10, 68, Direction.Backward);
          case FIRE:
            setFireAnim(0, 0, 0, 1, 1, 69, 1, 1);
            break;
          case LARSON:
            setLarson(0, 0, 100, 0, 1, 68, BounceMode.Back, 5);
            break;
          case RAINBOW:
            setRainbow(10, 0.7, 68);
            break;
          case RGBFADE:
            setRgbFade(10, 1, 68);
            break;
          case SINGLEFADE:
            setSingleFade(0, 0, 200, 200, 10, 68);
            break;
          case STROBERED:
            setStrobe(255, 0, 0, 0, 10, 68);
            break;
          case STROBEBLUE:
            setStrobe(0, 0, 255, 0, 10, 68);
            break;
          case STROBEGREEN:
            setStrobe(0, 255, 0, 0, 10, 68);
            break;
          case TWINKLE:
            setTwinkle(0, 0, 0, 0, 0, 68, TwinklePercent.Percent100);
            break;
          case TWINKLEOFF:
            setTwinkleOff(0, 0, 0, 0, 0, 68, TwinkleOffPercent.Percent100);
            break;
          case CUSTOM:
            setCustom();
            break;
          case OFF:
            candle.setLEDs(0, 0, 0);
            break;
        }
    }
    /**
     * Set Rainbow Animation
     * @param brightness The brightness of the LEDs [0, 1]
     * @param speed How fast the rainbow travels through the leds [0, 1]
     * @param numLed How many LEDs are controlled by the CANdle
     */
    public void setRainbow(double speed, double brightness, int numLed){
        candle.animate(new RainbowAnimation(brightness, speed, numLed));
    }

    /**
     * Set RGB Fade Animation
     * @param brightness How bright the LEDs are [0, 1]
     * @param speed How fast the LEDs fade between Red, Green, and Blue [0, 1]
     * @param numLed How many LEDs are controlled by the CANdle
     */
    public void setRgbFade(double speed, double brightness, int numLed){
        candle.animate(new RgbFadeAnimation(speed,brightness,numLed));
    }

    /**
     * Set Colorflow Animation
     * @param r How much red should the color have [0, 255]
     * @param g How much green should the color have [0, 255]
     * @param b How much blue should the color have [0, 255]
     * @param w How much white should the color have [0, 255]
     * @param speed How fast should the color travel the strip [0, 1]
     * @param numLed How many LEDs is the CANdle controlling
     * @param direction What direction should the color move in
     */
    public void setColorFlow(int r, int g, int b, int w, double speed, int numLed, Direction direction){
        candle.animate(new ColorFlowAnimation(r, g, b, w, speed, numLed, direction));
    }

    /**
     * Set Larson Animation
     * @param r How much red should the color have [0, 255]
     * @param g How much green should the color have [0, 255]
     * @param b How much blue should the color have [0, 255]
     * @param w How much white should the color have [0, 255]
     * @param speed How fast should the color travel the strip [0, 1]
     * @param numLed The number of LEDs the CANdle will control
     * @param mode How the pocket of LEDs will behave once it reaches the end of the strip
     * @param size How large the pocket of LEDs are [0, 7]
     */
    public void setLarson(int r, int g, int b, int w, double speed, int numLed, BounceMode mode, int size){
        candle.animate(new LarsonAnimation(r, g, b, w, speed, numLed, mode, size));
    }

    /**
     * Set Single Fade Animation
     * @param r How much red should the color have [0, 255]
     * @param g How much green should the color have [0, 255]
     * @param b How much blue should the color have [0, 255]
     * @param w How much white should the color have [0, 255]
     * @param speed How fast should the color travel the strip [0, 1]
     * @param numLed How many LEDs the CANdle controls
     */
    public void setSingleFade(int r, int g, int b, int w, double speed, int numLed){
        candle.animate(new SingleFadeAnimation(r, g, b, w, speed, numLed));
    }

    /**
     * Set Strobe Animation
     * @param r How much red should the color have [0, 255]
     * @param g How much green should the color have [0, 255]
     * @param b How much blue should the color have [0, 255]
     * @param w How much white should the color have [0, 255]
     * @param speed How fast should the color travel the strip [0, 1]
     * @param numLed How many LEDs the CANdle controls
     */
    public void setStrobe(int r ,int g, int b, int w, double speed, int numLed){
        candle.animate(new StrobeAnimation(r, g, b, w, speed, numLed));
    }

    /**
     * Set Twinkle Animation
     * @param r How much red should the color have [0, 255]
     * @param g How much green should the color have [0, 255]
     * @param b How much blue should the color have [0, 255]
     * @param w How much white should the color have [0, 255]
     * @param speed How fast should the color travel the strip [0, 1]
     * @param numLed How many LEDs the CANdle controls
     * @param divider What percentage of LEDs can be on at any point
     */
    public void setTwinkle(int r, int g, int b, int w, double speed, int numLed, TwinklePercent divider){
        candle.animate(new TwinkleAnimation(r, g, b, w, speed, numLed, divider));
    }
    
    /**
     * Set Twinkle Off Animation
     * @param r How much red should the color have [0, 255]
     * @param g How much green should the color have [0, 255]
     * @param b How much blue should the color have [0, 255]
     * @param w How much white should the color have [0, 255]
     * @param speed How fast should the color travel the strip [0, 1]
     * @param numLed How many LEDs the CANdle controls
     * @param divider What percentage of LEDs can be on at any point
     */
    public void setTwinkleOff(int r, int g, int b, int w, double speed, int numLed, TwinkleOffPercent divider){
        candle.animate(new TwinkleOffAnimation(r, g, b, w, speed, numLed, divider));
    }

    /**
     * Set Fire Animation
     * @param brightness How bright should the animation be [0, 1]
     * @param speed How fast will the flame be processed at [0, 1]
     * @param numLed How many LEDs is the CANdle controlling
     * @param sparking The rate at which the Fire "Sparks" [0, 1]
     * @param cooling The rate at which the Fire "Cools" along the travel [0, 1]
     */
    public void setFireAnim(int r, int g, int b, double brightness, double speed, int numLed, double sparking, double cooling){
        candle.animate(new FireAnimation(brightness, speed, numLed, sparking, cooling));
    }

    //Custom led color
    public void setCustom(){
    }
}
