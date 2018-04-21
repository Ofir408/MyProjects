import java.util.Map;

/**
 * This program is a Neg object.
 *
 * @author Ofir Ben Shoham
 * @since 2017-05-12
 */


public class Neg extends UnaryExpression implements Expression {

    /**
     * Neg Object has an Expression.
     */
    private Expression negExpression;

    /**
     * Neg has a constructor accepting an Expression.
     *
     * @param newNegExpression - new expression for the neg.
     */
    public Neg(final Expression newNegExpression) {
        super(newNegExpression);
    }

    /**
     * @param newString - the String.
     */
    public Neg(final String newString) {
        super(newString);
    }

    /**
     * @param doubleNumber - the double number.
     */
    public Neg(final double doubleNumber) {
        super(doubleNumber);
    }

    @Override
    public double evaluate(final Map<String,
            Double> assignment) throws Exception {
        if (this.checkException(assignment)) {
            throw new Exception("can't evaluate this "
                    + "Neg because"
                    + " there is least one variable"
                    + " without value in the map");
        }
        // can to evaluate
        try {
            double d = (Double.parseDouble(this.
                    getTheExp().toString()));
            return -1 * (d);
        } catch (Exception e) {
            return -1 * (this.getTheExp().evaluate(assignment));
        }

    }


    @Override
    public double evaluate() throws Exception {
        try {
            double d = (Double.parseDouble(this.
                    getTheExp().toString()));
            return -1 * (d);
        } catch (Exception e) {
            return -(this.getTheExp().evaluate());
        }
    }

    @Override
    public String toString() {
        return "(" + "-" + this.getTheExp().toString() + ")";
    }

    @Override
    public Expression differentiate(String var) {
        // TODO Auto-generated method stub
        return new Neg(this.getTheExp().differentiate(var));
    }

    @Override
    public Expression simplify() {
        try {
            return new Num(this.evaluate());
        } catch (Exception e) {
            if (new Num(0).toString().equals(this.getTheExp().toString())) {
                return new Num(0);
            } else {
                return new Neg(this.getTheExp().simplify());  // recursion.
            }
        }
    }
}
