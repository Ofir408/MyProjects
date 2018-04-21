import java.util.List;
import java.util.Map;

/**
 * This program is an Expression interface.
 *
 * @author Ofir Ben Shoham
 * @version 1.0
 * @since 2017-05-11
 */

public interface Expression {

    /**
     * Evaluate the expression using the variable values provided.
     *
     * @param assignment - a Maps <String, Double> that represents the
     *                   Assignment to evaluate.
     * @return Evaluate the expression using the variable
     * values provided.
     * @throws Exception - If the expression
     *                   contains a variable which is not in the assignment,
     *                   an exception  is thrown.
     */
    double evaluate(Map<String,
            Double> assignment) throws Exception;

    /**
     * A convenience method. Like the `evaluate(assignment)`
     * method above, but uses an empty assignment.
     * It can be useful for expressions without any free variables,
     * for example (1+7) or (2 * 9).
     *
     * @return the result, double number.
     * after Evaluating the expression using the variable
     * values provided.
     * @throws Exception - If the expression
     *                   contains a variable which is not in the assignment,
     *                   an exception  is thrown.
     */

    double evaluate() throws Exception;

    /**
     * @return a list of the variables in the expression.
     */
    List<String> getVariables();

    /**
     * @return a nice string representation of the expression.
     */
    @Override
    String toString();

    /**
     * Returns a new expression in which all occurrences of the variable
     * var are replaced with the provided expression (Does not modify
     * the current expression).
     *
     * @param var        - the variable to replaced with a Expression.
     * @param expression - the expression to assign instead of
     *                   the variable input.
     * @return the Expression: new expression in which all occurrences
     * of the variable. var are replaced with the provided expression
     */
    Expression assign(String var, Expression expression);

    /**
     * Returns the expression tree resulting from differentiating
     * the current expression relative to variable `var`.
     *
     * @param var - the variable to make the derivative according him.
     * @return Returns the expression tree resulting
     * from differentiating.
     */
    Expression differentiate(String var);

    /**
     * @return a simplified version of the current expression.
     */
    Expression simplify();

}
