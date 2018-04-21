import java.util.List;
import java.util.Map;

/**
 * This program is a Plus object.
 *
 * @author Ofir Ben Shoham
 * @since 2017-05-12
 */

public class Plus extends BinaryExpression implements Expression {

    /**
     * first expression to add.
     */
    private Expression firstExpression;

    /**
     * second expression to add.
     */
    private Expression secondExpression;

    /**
     * @param firstExp  - the first Expression to add
     * @param secondExp - the second Expression to add
     */
    public Plus(final Expression firstExp, final Expression secondExp) {
        super(firstExp, secondExp);
    }

    /**
     * @param exp    - Expression to add.
     * @param number - a double number.
     */
    public Plus(final Expression exp, final double number) {
        super(exp, number);
    }

    /**
     * @param exp - Expression to add.
     * @param str - String
     */
    public Plus(final Expression exp, final String str) {
        super(exp, str);
    }

    /**
     * @param str - String
     * @param exp - Expression to add.
     */
    public Plus(final String str, final Expression exp) {
        super(str, exp);
    }

    /**
     * @param str1 - first String
     * @param str2 - second String.
     */
    public Plus(final String str1, final String str2) {
        super(str1, str2);
    }

    /**
     * @param str    - String.
     * @param number - a double number.
     */
    public Plus(final String str, final double number) {
        super(str, number);
    }

    /**
     * @param numberOne - first double number.
     * @param numberTwo - second double number.
     */
    public Plus(final double numberOne, final double numberTwo) {
        super(numberOne, numberTwo);
    }

    /**
     * @param number - a double number.
     * @param str    - String.
     */
    public Plus(final double number, final String str) {
        super(number, str);
    }

    /**
     * @param number - a double number.
     * @param exp    - Expression.
     */
    public Plus(final double number, final Expression exp) {
        super(number, exp);
    }

    @Override
    public double evaluate(final Map<String,
            Double> assignment) throws Exception {

        if (this.checkException(assignment)) {
            throw new Exception("can't evaluate this "
                    + "plus because"
                    + " there is least one variable"
                    + " without value in the map");
        }
        // can to evaluate
        return this.getFirstExp().evaluate(assignment)
                + this.getSecondExp().
                evaluate(assignment);
    }

    @Override
    public double evaluate() throws Exception {
        if (this.getVariables().isEmpty()) {
            // can to compute the result
            return this.getFirstExp().evaluate()
                    + this.getSecondExp().evaluate();
        } else {
            // can not compute, there is a exception
            throw new Exception(" can't evaluate this Plus because "
                    + "there is at least one variable");
        }
    }

    @Override
    public String toString() {
        return "(" + this.getFirstExp().toString() + " + "
                + this.getSecondExp().toString() + ")";
    }

    @Override
    public Expression differentiate(String var) {
        // ( f(x) + g(x) ) ' = (f(x))' + (g(x))'
        return new Plus(this.getFirstExp().differentiate(var),
                this.getSecondExp().differentiate(var));
    }

    @Override
    public Expression simplify() {
        //  x + 0 = x
        String strOne, strTwo;
        try {
            Num n = new Num(this.evaluate());
            return n;
        } catch (Exception e) {
            Expression firstSimplify = this.getFirstExp().simplify();
            Expression secondSimplify = this.getSecondExp().simplify();
            strOne = firstSimplify.toString().replace("(", "");
            strOne = strOne.toString().replace(")", "");
            strTwo = secondSimplify.toString().replace("(", "");
            strTwo = strTwo.toString().replace(")", "");

            if (firstSimplify.toString().equals(new Num(0).toString())) {
                return secondSimplify;
            } else if (secondSimplify.toString().equals(new Num(0).toString())) {
                return firstSimplify;
            } else if (this.checkEqualsAlphabetical(strOne,
                    strTwo)) {
                return new Mult(2, firstSimplify);
                // also this for the bonus for ((2x) + (4x)) = 6x
            } else if ((firstSimplify instanceof Mult || firstSimplify instanceof Num)
                    && (secondSimplify instanceof Mult
                    || secondSimplify instanceof Num)) {
                return this.commonFactor();
            }
            return new Plus(firstSimplify, secondSimplify);
        }
    }

    /**
     *
     * @return the common factor expression of the first exp and
     * the second exp of this plus object.
     */
    public Expression commonFactor() {
        if (this instanceof Plus) {
            List<String> firstVars = this.getFirstExp().getVariables();
            List<String> secondVars = this.getSecondExp().getVariables();
            if (firstVars.containsAll(secondVars) && secondVars.containsAll(firstVars)) {
                try {
                    double number = this.gcd(this.getNumberFromExp(this.getFirstExp()),
                            this.getNumberFromExp(this.getSecondExp()));
                    Mult m1 = new Mult(number, firstVars.get(0));
                    Div resultOne = new Div(this.getFirstExp().simplify(), m1);
                    Div resultTwo = new Div(this.getSecondExp().simplify(), m1);
                    Expression resultO = resultOne.simplify();
                    Expression resultT = resultTwo.simplify();
                    Mult finalResult = new Mult(new Plus(resultO.simplify(),
                            resultT.simplify()), m1);
                    return (finalResult.simplify());
                } catch (Exception e) {
                    return new Plus(this.getFirstExp(), this.getSecondExp());
                }

            }
        }
        return new Plus(this.getFirstExp(), this.getSecondExp());
    }


    /**
     * @param exp - the expression for this checking.
     * @return the number from Exp,
     * For example for 3x - return 3.
     */
    private double getNumberFromExp(Expression exp) {
        double number;
        if (exp instanceof Num) {
            Num expIsNum = (Num) exp;
            number = expIsNum.getNumebr();
        } else {
            // Mult expression, according our if
            // Statement that calls to this function.
            Mult expIsMult = (Mult) exp;
            // the number appears from left (firstExp).
            Num num = (Num) expIsMult.getFirstExp();
            number = num.getNumebr();
        }
        return number;
    }
}
