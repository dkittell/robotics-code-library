package frc.robot;

public class Constants {

  public static final class LEDConstants {

    public static final int ADDRESSABLE_LED = 0;
    public static final int LED_COUNT = 288;

    /*
     * Fixed Palette Pattern
     */
    public static final double RAINBOW_RAINBOW_PALETTE = -0.99;
    public static final double RAINBOW_PARTY_PALETTE = -0.97;
    public static final double RAINBOW_OCEAN_PALETTE = -0.95;
    public static final double RAINBOW_LAVA_PALETTE = -0.93;
    public static final double RAINBOW_FOREST_PALETTE = -0.91;
    public static final double RAINBOW_WITH_GLITTER = -0.89;
    public static final double CONFETTI = -0.87;
    public static final double SHOT_RED = -0.85;
    public static final double SHOT_BLUE = -0.83; // 2024-07-02 Only half of the LEDs
    public static final double SHOT_WHITE = -0.81;
    public static final double SINELON_RAINBOW_PALETTE = -0.79;
    public static final double SINELON_PARTY_PALETTE = -0.77;
    public static final double SINELON_OCEAN_PALETTE = -0.75;
    public static final double SINELON_LAVA_PALETTE = -0.73;
    public static final double SINELON_FOREST_PALETTE = -0.71;
    public static final double BEATS_PER_MINUTE_RAINBOW_PALETTE = -0.69;
    public static final double BEATS_PER_MINUTE_PARTY_PALETTE = -0.67;
    public static final double BEATS_PER_MINUTE_OCEAN_PALETTE = -0.65;
    public static final double BEATS_PER_MINUTE_LAVA_PALETTE = -0.63;
    public static final double BEATS_PER_MINUTE_FOREST_PALETTE = -0.61;
    public static final double FIRE_MEDIUM = -0.59;
    public static final double FIRE_LARGE = -0.57;
    public static final double TWINKLES_RAINBOW_PALETTE = -0.55;
    public static final double TWINKLES_PARTY_PALETTE = -0.53;
    public static final double TWINKLES_OCEAN_PALETTE = -0.51;
    public static final double TWINKLES_LAVA_PALETTE = -0.49;
    public static final double TWINKLES_FOREST_PALETTE = -0.47;
    public static final double COLOR_WAVES_RAINBOW_PALETTE = -0.45;
    public static final double COLOR_WAVES_PARTY_PALETTE = -0.43;
    public static final double COLOR_WAVES_OCEAN_PALETTE = -0.41;
    public static final double COLOR_WAVES_LAVA_PALETTE = -0.39;
    public static final double COLOR_WAVES_FOREST_PALETTE = -0.37;
    public static final double LARSON_SCANNER_RED = -0.35;
    public static final double LARSON_SCANNER_GRAY = -0.33;
    public static final double LIGHT_CHASE_RED = -0.31;
    public static final double LIGHT_CHASE_BLUE = -0.29; // 2024-07-02 Only half of the LEDs
    public static final double LIGHT_CHASE_GRAY = -0.27;
    public static final double HEARTBEAT_RED = -0.25;
    public static final double HEARTBEAT_BLUE = -0.23; // 2024-07-02 Only half of the LEDs
    public static final double HEARTBEAT_WHITE = -0.21;
    public static final double HEARTBEAT_GRAY = -0.19;
    public static final double BREATH_RED = -0.17;
    public static final double BREATH_BLUE = -0.15; // 2024-07-02 Only half of the LEDs
    public static final double BREATH_GRAY = -0.13;
    public static final double STROBE_RED = -0.11;
    public static final double STROBE_BLUE = -0.09; // 2024-07-02 Only half of the LEDs
    public static final double STROBE_GOLD = -0.07;
    public static final double STROBE_WHITE = -0.05;
    /*
     * CP1: Color 1 Pattern
     */
    public static final double CP1_END_TO_END_BLEND_TO_BLACK = -0.03;
    public static final double CP1_LARSON_SCANNER = -0.01;
    public static final double CP1_LIGHT_CHASE = +0.01;
    public static final double CP1_HEARTBEAT_SLOW = +0.03;
    public static final double CP1_HEARTBEAT_MEDIUM = +0.05;
    public static final double CP1_HEARTBEAT_FAST = +0.07;
    public static final double CP1_BREATH_SLOW = +0.09;
    public static final double CP1_BREATH_FAST = +0.11;
    public static final double CP1_SHOT = +0.13;
    public static final double CP1_STROBE = +0.15;
    /*
     * CP2: Color 2 Pattern
     */
    public static final double CP2_END_TO_END_BLEND_TO_BLACK = +0.17;
    public static final double CP2_LARSON_SCANNER = +0.19;
    public static final double CP2_LIGHT_CHASE = +0.21;
    public static final double CP2_HEARTBEAT_SLOW = +0.23;
    public static final double CP2_HEARTBEAT_MEDIUM = +0.25;
    public static final double CP2_HEARTBEAT_FAST = +0.27;
    public static final double CP2_BREATH_SLOW = +0.29;
    public static final double CP2_BREATH_FAST = +0.31;
    public static final double CP2_SHOT = +0.33;
    public static final double CP2_STROBE = +0.35;
    /*
     * CP1_2: Color 1 and 2 Pattern
     */
    public static final double CP1_2_SPARKLE_1_ON_2 = +0.37;
    public static final double CP1_2_SPARKLE_2_ON_1 = +0.39;
    public static final double CP1_2_COLOR_GRADIENT = +0.41;
    public static final double CP1_2_BEATS_PER_MINUTE = +0.43;
    public static final double CP1_2_END_TO_END_BLEND_1_TO_2 = +0.45;
    public static final double CP1_2_END_TO_END_BLEND = +0.47;
    public static final double CP1_2_NO_BLENDING = +0.49;
    public static final double CP1_2_TWINKLES = +0.51;
    public static final double CP1_2_COLOR_WAVES = +0.53;
    public static final double CP1_2_SINELON = +0.55;
    /*
     * Solid color
     */
    public static final double HOT_PINK = +0.57;
    public static final double DARK_RED = +0.59;
    public static final double RED = +0.61;
    public static final double RED_ORANGE = +0.63;
    public static final double ORANGE = +0.65;
    public static final double GOLD = +0.67;
    public static final double YELLOW = +0.69;
    public static final double LAWN_GREEN = +0.71;
    public static final double LIME = +0.73;
    public static final double DARK_GREEN = +0.75; // 2024-07-02 Lights up All LEDs
    public static final double GREEN = +0.77;
    public static final double BLUE_GREEN = +0.79; // 2024-07-02 Half is blue, half is green
    public static final double AQUA = +0.81;  // 2024-07-02 Does not light up All LEDs
    public static final double SKY_BLUE = +0.83; // 2024-07-02 Does not light up All LEDs
    public static final double DARK_BLUE = +0.85;
    public static final double BLUE = +0.87;
    public static final double BLUE_VIOLET = +0.89;
    public static final double VIOLET = +0.91;
    public static final double WHITE = +0.93;
    public static final double GRAY = +0.95;
    public static final double DARK_GRAY = +0.97;
    public static final double BLACK = +0.99;
  }
}
