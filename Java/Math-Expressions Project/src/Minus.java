import java.util.Map;

/**
 * This program is a Minus object.
 *
 * @author Ofir Ben Shoham.
 * @since 2017-05-12
 */
public class Minus extends BinaryExpression implements Expression {

    /**
     * first expression to sub.
     */
    private Expression firstExpression;

    /**
     * second expression to sub.
     */
    private Expression secondExpression;

    /**
     * @param firstExp  - the first Expression to add
     * @param secondExp - the second Expression to add
     */
    public Minus(final Expression firstExp, final Expression secondExp) {
        super(firstExp, secondExp);
    }

    /**
     * @param exp    - Expression to add.
     * @param number - a double number.
     */
    public Minus(final Expression exp, final double number) {
        super(exp, number);
    }

    /**
     * @param exp - Expression to add.
     * @param str - String
     */
    public Minus(final Expression exp, final String str) {
        super(exp, str);
    }

    /**
     * @param str - String
     * @param exp - Expression to add.
     */
    public Minus(final String str, final Expression exp) {
        super(str, exp);
    }

    /**
     * @param str1 - first String
     * @param str2 - second String.
     */
    public Minus(final String str1, final String str2) {
        super(str1, str2);
    }

    /**
     * @param str    - String.
     * @param number - a double number.
     */
    public Minus(final String str, final double number) {
        super(str, number);
    }

    /**
     * @param numberOne - first double number.
     * @param numberTwo - second double number.
     */
    public Minus(final double numberOne, final double numberTwo) {
        super(numberOne, numberTwo);
    }

    /**
     * @param number - a double number.
     * @param str    - String.
     */
    public Minus(final double number, final String str) {
        super(number, str);
    }

    /**
     * @param number - a double number.
     * @param exp    - Expression.
     */
    public Minus(final double number, final Expression exp) {
        super(number, exp);
    }

    @Override
    public double evaluate(final Map<String,
            Double> assignment) throws Exception {

        if (this.checkException(assignment)) {
            throw new Exception(" can't evaluate this "
                    + "minus because"
                    + " there is least one variable"
                    + " without value in the map");
        }
        // can to evaluate
        return this.getFirstExp().evaluate(assignment)
                - this.getSecondExp().
                evaluate(assignment);
    }

    @Override
    public double evaluate() throws Exception {
        if (this.getVariables().isEmpty()) {
            // can to compute the result
            return this.getFirstExp().evaluate()
                    - this.getSecondExp().evaluate();
        } else {
            // can not compute, there is a exception
            throw new Exception(" can't evaluate this"
                    + " minus because "
                    + "there is at least one variable");
        }
    }

    @Override
    public String toString() {
        return "(" + this.getFirstExp().toString() + " - "
                + this.getSecondExp().toString() + ")";
    }

    @Override
    public Expression differentiate(String var) {
        // ( f(x) - g(x) ) ' = (f(x))' - (g(x))'
        return new Minus(this.getFirstExp().differentiate(var),
                this.getSecondExp().differentiate(var));
    }

    @Override
    public Expression simplify() {
        try {
            Num n = new Num(this.evaluate());
            return n;
        } catch (Exception e) {
            Expression firstSimplify = this.getFirstExp().simplify();
            Expression secondSimplify = this.getSecondExp().simplify();
            if (firstSimplify.toString().equals(new Num(0).toString())) {
                return new Neg(secondSimplify);
            } else if (secondSimplify.toString().equals(new Num(0).toString())) {
                return firstSimplify;
            } else if (this.checkEqualsAlphabetical(firstSimplify.toString(),
                    secondSimplify.toString())) {
                return new Num(0);
            } else {
                return new Minus(firstSimplify, secondSimplify);
            }
        }
    }

}
