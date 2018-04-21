import java.util.Map;
import java.util.TreeMap;

/**
 * This program is a ExpressionsTest.
 *
 * @author Ofir Ben Shoham.
 * @since 2017-05-19
 */
public class ExpressionsTest {

    /**
     * @param args - none.
     * @throws Exception - if there is a problem with the values.
     */
    public static void main(String[] args) throws Exception {

        Mult m1 = new Mult(new Num(2), new Var("x"));
        Sin s1 = new Sin(new Mult(new Num(4), new Var("y")));
        Pow p1 = new Pow(new Var("e"), new Var("x"));
        Plus all = new Plus(m1, new Plus(s1, p1));
        // Print the expression.
        System.out.println(all);
        // Print the value of the expression with (x=2,y=0.25,e=2.71)
        Map<String, Double> assignment = new TreeMap<String, Double>();
        assignment.put("x", (double) 2); // x=2
        assignment.put("y", 0.25); // y=0.25
        assignment.put("e", 2.71);
        try {
            System.out.println(all.evaluate(assignment));
        } catch (Exception ex) {
            System.out.println("there is a problem with the values.");
        }

        // Print the differentiated expression according to x.
        System.out.println(all.differentiate("x"));

        // Print the value of the differentiated expression
        // according to x with the assignment above.
        try {
            System.out.println(all.differentiate("x").evaluate(assignment));
        } catch (Exception ex) {
            System.out.println("there is a problem with the values.");
        }

        // Print the simplified differentiated expression.
        System.out.println(all.differentiate("x").simplify());

    }
}
