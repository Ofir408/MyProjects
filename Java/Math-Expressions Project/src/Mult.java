import java.util.Map;

/**
 * This program is a Mult object.
 *
 * @author Ofir Ben Shoham.
 * @since 2017-05-13
 */

public class Mult extends BinaryExpression implements Expression {

    /**
     * @param firstExp  - the first Expression to add
     * @param secondExp - the second Expression to add
     */
    public Mult(final Expression firstExp, final Expression secondExp) {
        super(firstExp, secondExp);
    }

    /**
     * @param exp    - Expression to add.
     * @param number - a double number.
     */
    public Mult(final Expression exp, final double number) {
        super(exp, number);
    }

    /**
     * @param exp - Expression to add.
     * @param str - String
     */
    public Mult(final Expression exp, final String str) {
        super(exp, str);
    }

    /**
     * @param str - String
     * @param exp - Expression to add.
     */
    public Mult(final String str, final Expression exp) {
        super(str, exp);
    }

    /**
     * @param str1 - first String
     * @param str2 - second String.
     */
    public Mult(final String str1, final String str2) {
        super(str1, str2);
    }

    /**
     * @param str    - String.
     * @param number - a double number.
     */
    public Mult(final String str, final double number) {
        super(str, number);
    }

    /**
     * @param numberOne - first double number.
     * @param numberTwo - second double number.
     */
    public Mult(final double numberOne, final double numberTwo) {
        super(numberOne, numberTwo);
    }

    /**
     * @param number - a double number.
     * @param str    - String.
     */
    public Mult(final double number, final String str) {
        super(number, str);
    }

    /**
     * @param number - a double number.
     * @param exp    - Expression.
     */
    public Mult(final double number, final Expression exp) {
        super(number, exp);
    }

    @Override
    public double evaluate(final Map<String,
            Double> assignment) throws Exception {
        if (this.checkException(assignment)) {
            throw new Exception("can't evaluate this "
                    + "Mult because"
                    + " there is least one variable"
                    + " without value in the map");
        }
        // can to evaluate
        return this.getFirstExp().evaluate(assignment)
                * this.getSecondExp().
                evaluate(assignment);
    }

    @Override
    public double evaluate() throws Exception {
        if (this.getVariables().isEmpty()) {
            // can compute the result
            return this.getFirstExp().evaluate()
                    * this.getSecondExp().evaluate();
        } else {
            // can not compute, there is a exception
            throw new Exception(" can't evaluate this"
                    + " Mult because "
                    + "there is at least one variable");
        }
    }

    @Override
    public String toString() {
        return "(" + this.getFirstExp().toString() + " * "
                + this.getSecondExp().toString() + ")";
    }

    @Override
    public Expression differentiate(String var) {
        //( f(x)*g(x) )' = f(x)'g(x) + g(x)' f(x).
        Expression f = this.getFirstExp();
        Expression g = this.getSecondExp();
        Mult m1 = new Mult(f.differentiate(var), g);
        Mult m2 = new Mult(g.differentiate(var), f);
        return new Plus(m1, m2);
    }

    @Override
    public Expression simplify() {
        // x * 1 = 1
        // x * 0 = 1
        Num one = new Num(1);
        Num zero = new Num(0);
        try {
            Num n = new Num(this.evaluate());
            return n;
        } catch (Exception e) {
            Expression firstSimplify = this.getFirstExp().simplify();
            Expression secondSimplify = this.getSecondExp().simplify();
            // check x * 1 or 1 * x
            String firstString = firstSimplify.toString();
            String secondString = secondSimplify.toString();
            if (firstString.toString().equals(one.toString())
                    || firstString.toString().equals(zero.toString())) {
                if (firstString.toString().equals(one.toString())) {
                    // 1 * x
                    return (secondSimplify);
                } else {
                    // 0 * x
                    return zero;
                }
            }
            if (secondString.toString().equals(one.toString())
                    || secondString.toString().equals(zero.toString())) {
                if (secondString.toString().equals(one.toString())) {
                    // x * 1
                    return (firstSimplify);
                } else {
                    // 0 * x
                    return zero;
                }
            }
            // for the bonus for (x*y) * (y*x) return (x*y)^2
            if (this.checkEqualsAlphabetical(firstString, secondString)) {
                return new Pow(firstSimplify, 2);
            }
            return new Mult(firstSimplify, secondSimplify);
        }
    }
}
