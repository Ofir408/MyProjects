import java.awt.Color;
import java.util.ArrayList;
import java.util.List;

import biuoop.DrawSurface;
import biuoop.KeyboardSensor;

/**
 * This program is a MenuAnimation class.
 *
 * @param <T>
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-06-09
 */
public class MenuAnimation<T> implements Menu<T>, Task<Void> {

    private String title;
    private List<String> menuKeys;
    private List<String> menuMessages;
    private List<T> menuReturnValues;
    private KeyboardSensor keyboard;
    private List<String> subMenuKeys = new ArrayList<String>();
    private List<String> subMenuMessages = new ArrayList<String>();
    private List<Menu<T>> subMenusList = new ArrayList<Menu<T>>();
    private AnimationRunner animationRun;

    /**
     *
     * @param newTitle - name to show.
     * @param newKeyboard - KeyboardSensor.
     * @param anRunner - AnimationRunner.
     */
    public MenuAnimation(String newTitle, KeyboardSensor newKeyboard, AnimationRunner anRunner) {
        this.title = newTitle;
        // this.animationRunner = aniRun;
        this.keyboard = newKeyboard;
        this.menuKeys = new ArrayList<String>();
        this.menuMessages = new ArrayList<String>();
        this.menuReturnValues = new ArrayList<T>();
        this.animationRun = anRunner;
    }

    /**
     * constructor.
     */
    public MenuAnimation() {
        this.menuKeys = new ArrayList<String>();
        this.menuMessages = new ArrayList<String>();
        this.menuReturnValues = new ArrayList<T>();
    }

    @Override
    public void doOneFrame(DrawSurface d, double dt) {
        d.setColor(Color.BLUE);
        d.fillRectangle(0, 0, 800, 600);
        d.setColor(Color.RED);

        d.setColor(Color.YELLOW);
        d.drawText(300, 50, this.title, 30);
        // display options to press..
        d.setColor(Color.GREEN);
        for (int i = 0; i < this.menuKeys.size(); i += 1) {
            String s = this.menuMessages.get(i);
            d.drawText(180 + 150, 230 + 40 * i, this.menuKeys.get(i) + ") " + s, 20);
        }

    }

    @Override
    public boolean shouldStop() {
        for (String currentKey : this.menuKeys) {
            if (this.keyboard.isPressed(currentKey)) {
                return true;
            }
        }
        for (String key : this.subMenuKeys) {
            if (this.keyboard.isPressed(key)) {
                return true;
            }
        }
        return false;
    }

    @Override
    public void addSelection(String key, String message, T returnVal) {
        this.menuKeys.add(key);
        this.menuMessages.add(message);
        this.menuReturnValues.add(returnVal);
    }

    @Override
    public T getStatus() {
        for (String currentKey : this.menuKeys) {
            if (this.keyboard.isPressed(currentKey)) {
                int pressedIndex = this.menuKeys.indexOf(currentKey);
                return this.menuReturnValues.get(pressedIndex);
            }
        }
        return null; // otherwise - didn't find any pressed key, return null.
    }

    @Override
    public void addSubMenu(String key, String message, Menu<T> subMenu) {
        this.subMenuKeys.add(key);
        this.subMenuMessages.add(message);
        this.subMenusList.add(subMenu);
    }

    /**
     * @return Menu<T> - the SubMenu of the current Menu.
     */
    public Menu<T> nextSubMenu() {
        System.out.println("here");
        for (String key : this.subMenuKeys) {
            if (this.keyboard.isPressed(key)) {
                return this.subMenusList.get(this.subMenuKeys.indexOf(key));
            }
        }
        return null;
    }

    @Override
    public Void run() {
        if (this.nextSubMenu() != null) {
            this.animationRun.run(this.nextSubMenu());
        }
        return null;
    }

}
