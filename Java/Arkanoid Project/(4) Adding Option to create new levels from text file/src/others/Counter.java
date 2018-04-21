
/**
 * a counter use the notifier to keep track of the remaining number of blocks,
 * so that we could recognize when no more blocks are available.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-23
 */
public class Counter {
    /**
     * current number of blocks that still remain.
     */
    private int remainBlocksNumber;

    /**
     * constructor function to Counter object.
     *
     * @param number - the number to assign as that
     *               current number of blocks that still remain.
     */
    public Counter(int number) {
        this.remainBlocksNumber = number;
    }

    /**
     * add number to current count.
     *
     * @param number - the number to add to the current counter block remain number.
     */
    void increase(int number) {
        this.remainBlocksNumber += number;
    }

    /**
     * subtract number from current count.
     *
     * @param number - the number to subtract from the current counter block remain number.
     */
    void decrease(int number) {
        this.remainBlocksNumber -= number;
    }

    /**
     * add ONE to current counter.
     */
    void increaseOne() {
        this.remainBlocksNumber += 1;
    }

    /**
     * subtract ONE to current counter.
     */
    void decreaseOne() {
        this.remainBlocksNumber -= 1;
    }

    /**
     * get current count.
     *
     * @return current the number of the blocks that still remain.
     */
    int getValue() {
        return this.remainBlocksNumber;
    }
}
