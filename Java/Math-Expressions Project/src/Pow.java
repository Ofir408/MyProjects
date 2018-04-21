import java.util.Map;

/**
 * This program is a Pow object.
 *
 * @author Ofir Ben Shoham.
 * @since 2017-05-13
 */

// Pow(x,y) indicating raising x to the y power.

public class Pow extends BinaryExpression implements Expression {

    /**
     * first expression to add.
     */
    private Expression expressionToRaise;

    /**
     * second expression to add.
     */
    private Expression thePowerExpression;

    /**
     * @param firstExp  - the first Expression to add
     * @param secondExp - the second Expression to add
     */
    public Pow(final Expression firstExp, final Expression secondExp) {
        super(firstExp, secondExp);
    }

    /**
     * @param exp    - Expression to add.
     * @param number - a double number.
     */
    public Pow(final Expression exp, final double number) {
        super(exp, number);
    }

    /**
     * @param exp - Expression to add.
     * @param str - String
     */
    public Pow(final Expression exp, final String str) {
        super(exp, str);
    }

    /**
     * @param str - String
     * @param exp - Expression to add.
     */
    public Pow(final String str, final Expression exp) {
        super(str, exp);
    }

    /**
     * @param str1 - first String
     * @param str2 - second String.
     */
    public Pow(final String str1, final String str2) {
        super(str1, str2);
    }

    /**
     * @param str    - String.
     * @param number - a double number.
     */
    public Pow(final String str, final double number) {
        super(str, number);
    }

    /**
     * @param numberOne - first double number.
     * @param numberTwo - second double number.
     */
    public Pow(final double numberOne, final double numberTwo) {
        super(numberOne, numberTwo);
    }

    /**
     * @param number - a double number.
     * @param str    - String.
     */
    public Pow(final double number, final String str) {
        super(number, str);
    }

    /**
     * @param number - a double number.
     * @param exp    - Expression.
     */
    public Pow(final double number, final Expression exp) {
        super(number, exp);
    }

    @Override
    public double evaluate(final Map<String,
            Double> assignment) throws Exception {
        if (this.checkException(assignment)) {
            throw new Exception("can't evaluate this "
                    + "Pow because"
                    + " there is least one variable"
                    + " without value in the map");
        }
        // can to evaluate
        return Math.pow(this.getFirstExp().evaluate(assignment),
                this.getSecondExp().
                        evaluate(assignment));
    }

    @Override
    public double evaluate() throws Exception {
        if (this.getVariables().isEmpty()) {
            // can compute the result
            return Math.pow(this.getFirstExp().evaluate(),
                    this.getSecondExp().
                            evaluate());
        } else {
            // can not compute, there is a exception
            throw new Exception(" can't evaluate this"
                    + " Pow because "
                    + "there is at least one variable");
        }
    }

    @Override
    public String toString() {
        return "(" + this.getFirstExp().toString() + "^"
                + this.getSecondExp().toString() + ")";
    }

    @Override
    public Expression differentiate(String var) {
        // f(x)^g(x) = e^ ( g(x) * lnf(x) ). - removed.
        Expression f = this.getFirstExp();
        Expression g = this.getSecondExp();
        Const e = new Const(Math.E, "e");
        Pow fPowG = new Pow(f, g);
        Div m1 = new Div(g, f);
        Mult partOne = new Mult(f.differentiate(var), m1);
        Mult partTwo = new Mult(g.differentiate(var), new Log(e, f));
        Plus together = new Plus(partOne, partTwo);
        return new Mult(fPowG, together);
    }

    @Override
    public Expression simplify() {
        try {
            return new Num(this.evaluate());
        } catch (Exception e) {
            if (this.getSecondExp().toString().equals(new Num(0).toString())) {
                // x ^ 0 = 1.
                return new Num(1);
            }
            // Bonus part:
            if (this.getFirstExp() instanceof Pow) {
                Pow one = (Pow) this.getFirstExp();
                Mult powersResult = new Mult(one.getSecondExp(),
                        this.getSecondExp());
                return new Pow(one.getFirstExp(), powersResult);
            }
            return new Pow(this.getFirstExp().simplify(),
                    this.getSecondExp().simplify());
        }
    }
}

