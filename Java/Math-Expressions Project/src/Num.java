import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * This program is a Num object.
 *
 * @author Ofir Ben Shoham
 * @since 2017-05-11
 */
public class Num implements Expression {
    /**
     * the value of the number, double type.
     */

    private double value;

    /**
     * constructor of Num object.
     * Num should have a constructor accepting a double
     *
     * @param newValue - the value to put in the Num object.
     */
    public Num(final double newValue) {
        this.value = newValue;
    }

    /**
     * @return the value (a double number) of the Num object.
     */
    public double getNumebr() {
        return this.value;
    }

    @Override

    public double evaluate(Map<String,
            Double> assignment) throws Exception {
        return this.value;
    }

    @Override
    public double evaluate() throws Exception {
        return this.value;
    }

    @Override
    public List<String> getVariables() {

        List<String> l = new ArrayList<String>();
        return l;
        // empty list - because there is no variables in Number.
    }

    @Override
    public Expression assign(String var, Expression expression) {
        // no variable to assign in, therefore return this object -num Expression
        return this;
    }

    @Override
    public String toString() {
        String s = Double.toString(this.value);
        return s;
    }

    @Override
    public Expression differentiate(final String var) {
        // (num)' = 0.
        return new Num(0.0);
    }

    @Override
    public Expression simplify() {
        return this;
    }
}
