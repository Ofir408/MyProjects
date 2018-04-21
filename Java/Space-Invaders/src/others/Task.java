
/**
 * This program is a Task<T> interface.
 *
 * @param <T>
 * @author Ofir Ben Shoham.
 * @version 1.0
 * @since 2017-06-09
 */
public interface Task<T> {
    /**
     * Run this assigment.
     *
     * @return T.
     */
    T run();
}
