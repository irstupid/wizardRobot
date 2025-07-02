package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {

    final int LED_LENGTH = 150;

    public enum Pattern {
        RED,
        ORANGE,
        YELLOW,
        GREEN,
        BLUE,
        PURPLE,
        PINK,
        WHITE,
        RAINBOW,
        FAST,
        SLOW,
        BLINK,
        PULSE,
        FLASH,
        MARCH,
        WAVE,
        SCATTER,
        GLITTER,
    }

    Pattern[] patterns;
    private AddressableLED controller = null;
    private AddressableLEDBuffer buffer = null;
    Timer t;
    boolean fast;
    boolean slow;

    Lights() {
        controller = new AddressableLED(1);
        buffer = new AddressableLEDBuffer(LED_LENGTH + 1);
        controller.setLength(buffer.getLength());
        controller.setData(buffer);
        controller.start();

        t = new Timer();
        t.reset();
        t.start();
    }

    private Color red(Color color, int i) {
        return new Color(255, 0, 0);
    }

    private Color orange(Color color, int i) {
        return new Color(255, 100, 0);
    }

    private Color yellow(Color color, int i) {
        return new Color(255, 255, 0);
    }

    private Color green(Color color, int i) {
        return new Color(0, 255, 0);
    }

    private Color blue(Color color, int i) {
        return new Color(0, 0, 255);
    }

    private Color purple(Color color, int i) {
        return new Color(150, 0, 255);
    }

    private Color pink(Color color, int i) {
        return new Color(255, 0, 255);
    }

    private Color white(Color color, int i) {
        return new Color(254, 254, 254);
    }

    private Color blink(Color color, int i) {
        double delay = 0;
        if (fast) {
            delay = 0.2;
        } else if (slow) {
            delay = 1;
        } else {
            delay = 0.4;
        }
        if (t.get() > delay * 2) {
            t.reset();
        }
        if (t.get() > delay) {
            return color;
        } else {
            return new Color();
        }
    }

    private Color patternLookup(Color color, int i, Pattern pattern) {
        switch (pattern) {
            case RED:
                return red(color, i);
            case ORANGE:
                return orange(color, i);
            case YELLOW:
                return yellow(color, i);
            case GREEN:
                return green(color, i);
            case BLUE:
                return blue(color, i);
            case PURPLE:
                return purple(color, i);
            case PINK:
                return pink(color, i);
            case WHITE:
                return white(color, i);
            default:
                return color;
        }
    }

    private void update() {
        for (int i = 0; i < LED_LENGTH; i++) {
            Color color = new Color();
            for (int n = 0; n < patterns.length; n++) {
                if (patterns[n] == Pattern.FAST) {
                    fast = true;
                    slow = false;
                } else if (patterns[n] == Pattern.SLOW) {
                    fast = false;
                    slow = true;
                } else {
                    color = patternLookup(color, i, patterns[n]);
                    fast = false;
                    slow = false;
                }
            }
        }
    }

    class Color {
        int r;
        int g;
        int b;

        Color() {
            r = 0;
            g = 0;
            b = 0;
        }

        Color(int r, int g, int b) {
            this.r = r;
            this.g = g;
            this.b = b;
        }
    }
}
