
/**
 * This program is a BounusTester.
 *
 * @author Ofir Ben Shoham.
 * @since 2017-05-18
 */
public class SimplificationDemo {

    /**
     * BounusTester - Ofir Ben Shoham.
     *
     * @param args - none.
     */
    public static void main(String[] args) {

        // lets check if (x^y)^z => x^(y*z)
        Pow p1 = new Pow(new Pow(new Var("x"), new Var("y")), new Var("z"));
        System.out.println("\n---------------- (1) ----------------");
        System.out.println("Before my simplify the Expression was: " + p1);
        System.out.println("After simplify the expression is:  " + p1.simplify());
        Pow p2 = new Pow(new Pow(new Var("x"), new Var("y")),
                new Log(new Var("z"), new Var("t")));
        System.out.println("\n---------------- (2) ----------------");
        System.out.println("Before my simplify the Expression was: " + p2);
        System.out.println("After simplify the expression is:  " + p2.simplify());

        Mult m1 = new Mult(new Var("x"), new Var("y"));
        Mult m2 = new Mult(m1, m1);

        // (x * y) * (x * y) = (x * y)^2
        System.out.println("\n---------------- (3) ----------------");
        System.out.println("Before my simplify the Expression was: " + m2);
        System.out.println("After simplify the expression is:  " + m2.simplify());

        // lets check if ((2x) + (4x)) => 6*x
        Plus pl = new Plus(new Mult(new Num(2), new Var("x")),
                new Mult(new Num(4), new Var("x")));
        System.out.println("\n---------------- (4) -------------");
        System.out.println("Before my simplify the Expression was: " + pl);
        System.out.println("After simplify the expression is:  " + pl.simplify());

        Mult p3 = new Mult(2, new Mult(new Pow(new Var("z"), 2), new Pow(new Var("t"), 3)));
        Mult p4 = new Mult(3, new Mult(new Pow(new Var("t"), 3), new Pow(new Var("z"), 2)));
        Div d = new Div(p4, p3);

        System.out.println("\n---------------- (5) -------------");
        System.out.println("Before my simplify the Expression was: " + d);
        System.out.println("After simplify the expression is:  " + d.simplify().simplify());

        System.out.println("\n---------------- (6) -------------");
        Plus p5 = new Plus(new Mult(new Log(new Var("x"), new Mult(new Div(2, new Sin(new Cos(0))),
                new Mult(new Pow(new Var("z"), 2),
                new Pow(new Var("t"), 3)))), new Var("t")),
                new Plus(new Var("z"), new Num(8)));
        Plus p6 = new Plus(new Mult(new Var("t"), new Log(new Var("x"), p3)),
                new Plus(new Num(8), new Var("z")));
        Div d1 = new Div(p5, p6);
        System.out.println("Before my simplify the Expression was:\n " + d1);
        System.out.println("After simplify the expression is:  " + d1.simplify());
        System.out.println("\n---------------- (7) -------------");
        Plus pp = new Plus(new Mult(3, new Var("x")), new Mult(4, new Var("x")));
        System.out.println("Before simplify the expression was: " + pp);
        System.out.println("After simplify the expression is: " + pp.simplify());

        System.out.println("\n---------------- (8) -------------");

        Mult mgt = new Mult(5, new Sin(90));
        Mult t3 = new Mult(new Pow(mgt, 2), new Mult(new Sin(new Cos(new Sin(50))), new Var("x")));
        Mult t4 = new Mult(5, new Mult(new Sin(new Cos(new Sin(50))), new Var("x")));
        Div d4 = new Div(t3, t4);
        System.out.println("Before simplify the expression was: " + d4);
        System.out.println("After simplify the expression is: " + d4.simplify().simplify());

    }

}
