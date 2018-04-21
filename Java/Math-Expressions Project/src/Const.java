import java.util.ArrayList;
import java.util.List;
import java.util.Map;

/**
 * This program is a Const object.
 *
 * @author Ofir Ben Shoham.
 * @since 2017-05-14.
 */
public class Const extends BaseExpression implements Expression {

    /**
     * double number, the value of this Const.
     */
    private double num;
    /**
     * the Var of this const.
     */
    private Var var;

    /**
     * @param newNum - double number, value of const.
     * @param newVar - String, that I will change to Var,
     *               the var of this const.
     */
    public Const(double newNum, String newVar) {
        this.num = newNum;
        this.var = new Var(newVar);
    }

    /**
     * @return the double number of this const.
     */
    public double getNum() {
        return this.num;
    }

    @Override
    public double evaluate(Map<String, Double> assignment) throws Exception {
        return this.getNum();
    }

    @Override
    public double evaluate() throws Exception {
        return this.getNum();
    }

    @Override
    public Expression simplify() {
        return this;
    }

    @Override
    public String toString() {
        return this.var.toString();
    }

    @Override
    public List<String> getVariables() {

        //The expression is just a number.
        List<String> l = new ArrayList<String>();
        l.add(this.var.toString());
        return new ArrayList<String>();
    }

    @Override
    public Expression differentiate(String newVar) {
        Num n = new Num(0);
        return n;
    }

    @Override
    public Expression assign(String newVar, Expression expression) {
        return this;
    }


}
