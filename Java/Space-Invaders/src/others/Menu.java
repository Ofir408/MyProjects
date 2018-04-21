
/**
 * Menu<T> interface. Our Menu will need to be displayed on screen, so it will
 * be an Animation. By using generics, we allow the menu to be used with
 * different return types. For example, if we want the selection to result in a
 * java.awt.Color object instead of a String
 *
 * @param <T> - By using generics, we allow the menu to be used with different
 *            return types.
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-06-09
 */
public interface Menu<T> extends Animation {

    /**
     * @param key       - key to wait for, the key that needed to press in order to
     *                  get this option (T) - the returnVal.
     * @param message   - line to print.
     * @param returnVal - what to return if the key was pressed.
     */
    void addSelection(String key, String message, T returnVal);

    /**
     * @return the option that was selected.
     */
    T getStatus();

    /**
     * add SubMenu to this Menu.
     *
     * @param key     - key to wait for, the key that needed to press in order to
     *                get this option (T) - the returnVal.
     * @param message - line to print.
     * @param subMenu - the subMenu to add to this Manu.
     */
    void addSubMenu(String key, String message, Menu<T> subMenu);

}
