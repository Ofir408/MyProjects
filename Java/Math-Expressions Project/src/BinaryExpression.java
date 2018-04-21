import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Map;
import java.util.TreeMap;

/**
 * This program is a BinaryExpression object.
 *
 * @author Ofir Ben Shoham.
 * @since 2017-05-12
 */
public abstract class BinaryExpression extends BaseExpression implements Expression {

    /**
     * first expression to sub.
     */
    private Expression firstExpression;

    /**
     * second expression to sub.
     */
    private Expression secondExpression;

    // Let's define the constructor methods that the classes that inherit
    // from this class have in common.


    /**
     * @param first  - the first Expression.
     * @param second - the second Expression.
     */
    public BinaryExpression(final Expression first,
                            final Expression second) {
        this.firstExpression = first;
        this.secondExpression = second;
    }

    /**
     * @param firstString  - the first String.
     * @param secondString - the second String.
     */
    public BinaryExpression(final String firstString,
                            final String secondString) {
        this.firstExpression = new Var(firstString);
        this.secondExpression = new Var(secondString);
    }

    /**
     * @param firstNumber  - the first number (double number).
     * @param secondNumber - the second number (also double number).
     */
    public BinaryExpression(final double firstNumber,
                            final double secondNumber) {
        this.firstExpression = new Num(firstNumber);
        this.secondExpression = new Num(secondNumber);
    }

    /**
     * @param exp - Expression.
     * @param str - a String.
     */
    public BinaryExpression(final Expression exp,
                            final String str) {
        this.firstExpression = exp;
        this.secondExpression = new Var(str);
    }

    /**
     * @param exp    - Expression.
     * @param number - a double number.
     */
    public BinaryExpression(final Expression exp,
                            final double number) {
        this.firstExpression = exp;
        this.secondExpression = new Num(number);
    }

    /**
     * @param str    - string
     * @param number - a double number.
     */
    public BinaryExpression(final String str,
                            final double number) {
        this.firstExpression = new Var(str);
        this.secondExpression = new Num(number);
    }

    /**
     * @param str - string.
     * @param exp - Expression.
     */
    public BinaryExpression(final String str,
                            final Expression exp) {
        this.firstExpression = new Var(str);
        this.secondExpression = exp;
    }

    /**
     * @param number - a double number.
     * @param str    - string
     */
    public BinaryExpression(final double number,
                            final String str) {
        this.firstExpression = new Num(number);
        this.secondExpression = new Var(str);
    }

    /**
     * @param number - a double number.
     * @param exp    - Expression.
     */
    public BinaryExpression(final double number,
                            final Expression exp) {
        this.firstExpression = new Num(number);
        this.secondExpression = exp;
    }


    @Override
    public Expression assign(String var,
                             Expression expression) {
        Expression ex = expression.simplify();
        if (this.getFirstExp().getVariables().indexOf(var) != -1) {
            //this.setFirstExp(this.firstExpression.assign(var, ex));
            this.firstExpression =
                    this.getFirstExp().
                            assign(var, ex);
        }
        if (this.getSecondExp().getVariables().indexOf(var) != -1) {
            //this.setSecondExp(this.secondExpression.assign(var, ex));
            this.secondExpression =
                    this.getSecondExp().
                            assign(var, ex);
        }
        Expression e = this;
        return e;
    }

    @Override
    public List<String> getVariables() {
        List<String> variablesList =
                this.firstExpression.getVariables();
        variablesList.addAll(this.getSecondExp().
                getVariables());
        return variablesList;
    }

    /**
     * @return firstExpression - Expression.
     */
    public Expression getFirstExp() {
        return this.firstExpression;
    }

    /**
     * @return secondExpression - Expression.
     */
    public Expression getSecondExp() {
        return this.secondExpression;
    }

    /**
     * @param newFirstExp - the new Expression to set.
     */
    public void setFirstExp(Expression newFirstExp) {
        this.firstExpression = newFirstExp;
    }

    /**
     * @param newFirstExp - the new Expression to set.
     */
    public void setSecondExp(Expression newFirstExp) {
        this.secondExpression = newFirstExp;
    }

    /**
     * @param stringToSort - the string that we want to sort
     *                     according ASCII table.
     * @return the sorted string.
     */
    private String sortedString(String stringToSort) {
        char[] chars = stringToSort.toCharArray();
        Arrays.sort(chars);
        String sorted = new String(chars);
        return (sorted);
    }

    /**
     * @param firstString  - the first string.
     * @param secondString - the second string.
     * @return true if the strings equals  after sorting alphabetical.
     * Otherwise - false.
     */
    public boolean checkEqualsAlphabetical(String firstString,
                                           String secondString) {
        String firstSorted, secondSorted;
        //String saverOne = firstString.toString(), saverTwo = secondString.toString();
        if (firstString.length() > 1 && secondString.length() > 1) {
            char c1 = firstString.charAt(0), c2 = secondString.charAt(0);

            if (c1 == '('
                    && firstString.charAt(firstString.length() - 1) == ')') {
                firstString = firstString.substring(1,
                        firstString.length() - 1);
            }
            if (c2 == '('
                    && secondString.charAt(secondString.length() - 1) == ')') {
                secondString = secondString.substring(1,
                        secondString.length() - 1);
            }
        }

        firstSorted = this.sortedString(firstString);
        secondSorted = this.sortedString(secondString);
        if (firstSorted.equals(secondSorted)) {
            List<String> l1 = this.getFirstExp().getVariables();
            List<String> l2 = this.getSecondExp().getVariables();
            if (l1.containsAll(l2) && l2.containsAll(l1)) {
                this.placingChecker(this.getFirstExp().toString(),
                        this.getSecondExp().toString(), this.getVariables());
                return true;
            }
        }
        return false;
    }

    /**
     * @param first  - the first string.
     * @param second - the second string.
     * @param var    - list of variables in the Expressions.
     * @return true if the result is equal. Otherwise, false.
     */
    private boolean placingChecker(String first, String second,
                                   List<String> var) {
        Collections.sort(var, String.CASE_INSENSITIVE_ORDER);
        double t = 0.23;
        int x = 1;
        Map<String, Double> assignment = new TreeMap<String, Double>();
        for (String element : var) {
            assignment.put(element, x * t);
            x += 1;
        }
        try {
            if (this.getFirstExp().evaluate(assignment)
                    == this.getSecondExp().evaluate(assignment)) {
                return true;
            }
        } catch (Exception e) {
            return false;
        }
        return false;
    }

    /**
     * @param first  - first number
     * @param second - second number
     * @return - their gcd.
     */
    public double gcd(double first, double second) {
        if (second == 0) {
            return first;
        }
        return gcd(second, first % second);
    }
}


