import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

/**
 * ColorsParser Class - Parse color definition and return the specified color.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-06-11
 */
public class ColorsParser {
    /**
     * There are some options to represents color in our block:
     * color(RGB(244,248,129)) or color(blue)
     */

    /**
     * parse color definition and return the specified color.
     *
     * @param s - the string to get the color from.
     * @return the color.
     */
    public java.awt.Color colorFromString(String s) {
        /**
         * There are some options to represents color in our block:
         * color(RGB(244,248,129)) or color(colorName)
         */
        if (s.contains("RGB")) {
            return this.getColorFromRGB(s);
        } else {
            return this.getRegularColor(s);
        }
    }

    /**
     * @return list of optional input colors in our file. color(colorname),
     * where color name is one of: black, blue, cyan, gray, lightGray,
     * green, orange, pink, red, white + yellow.
     */
    private List<Color> getOptionalColorsList() {
        List<Color> optionalColorsList = new ArrayList<Color>();
        optionalColorsList.add(Color.black);
        optionalColorsList.add(Color.blue);
        optionalColorsList.add(Color.cyan);
        optionalColorsList.add(Color.gray);
        optionalColorsList.add(Color.lightGray);
        optionalColorsList.add(Color.green);
        optionalColorsList.add(Color.orange);
        optionalColorsList.add(Color.pink);
        optionalColorsList.add(Color.red);
        optionalColorsList.add(Color.white);
        optionalColorsList.add(Color.yellow);
        return optionalColorsList;
    }

    /**
     * @param s - a string to parse to colors.
     * @return the color of this string.
     */
    public Color getRegularColor(String s) {
        // get the colorName from color(colorName)
        List<Color> optionalColorsList = this.getOptionalColorsList();

        int i = 0;
        if (s.contains("blue")) {
            i = 1;
        } else if (s.contains("cyan")) {
            i = 2;
        } else if (s.contains("gray")) {
            i = 3;
        } else if (s.contains("lightGray")) {
            i = 4;
        } else if (s.contains("green")) {
            i = 5;
        } else if (s.contains("orange")) {
            i = 6;
        } else if (s.contains("pink")) {
            i = 7;
        } else if (s.contains("red")) {
            i = 8;
        } else if (s.contains("white")) {
            i = 9;
        } else if (s.contains("yellow")) {
            i = 10;
        }

        return optionalColorsList.get(i);
    }

    /**
     *
     * @param s - string contains color from rgb definition to parse.
     * @return the Color.
     */
    public Color getColorFromRGB(String s) {
        // For example from: color(RGB(244,248,129))
        String[] t = s.split("RGB", 2);
        String str = t[1].replace('(', ' ');
        String finalNumbers = str.replace(')', ' ');
        String st = finalNumbers.trim();
        String[] numbers = st.split(",");
        double[] rgbArray = new double[3];
        try {
            for (int i = 0; i < numbers.length; i += 1) {
                rgbArray[i] = Double.parseDouble(numbers[i]);
            }
        } catch (NumberFormatException n) {
            System.out.println("NumberFormatException in colorsParser, in getColorFromRGB method");
        }

        int x = 255;
        Color c = new Color((float) rgbArray[0] / x, (float) rgbArray[1] / x, (float) rgbArray[2] / x);
        return c;
    }

}
