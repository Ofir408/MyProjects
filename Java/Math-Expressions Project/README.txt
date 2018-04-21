Explanation for the Bonus part.
Name: Ofir Ben Shoham.
Id number: 208642496.

I simplified some expressions, at this text I will describe the simplifications types
that I did and the approach that I took to do them. 

* Simplify that (x^y)^z = x^(y*z)
How did I approach this? 

# I added new if statment for the simplify function in "Pow" Class, that checks 
if the first expression is instanceOf Pow. For example: (x^y)^z -> the first expression here
is (x^y) and it is instanceOf Pow (There is a power between them). After this checking, 
I can cast the first expression in Pow, because it is instance of Pow. 
Than, I return new Power object, that it first expression is the first expression in the first expression.
For example, in the example of (x^y)^z -> (x^y), the first expression and x is the first expression
 in the first expression. So, return new Pow of x at this example, and the second expression 
is the multiplication (y*z). This way we get the simplify expression x^(y*z). 

* Simplify that (x*y) * (y*x) = (x*y)^2, same as (x*y) * (x*y) = (x*y)^2
How did I approach this?

# I added new if statment to the simplify function in "Mult" class.
In this statment, I used in the function that checks if two expressions are equal.
This function appers in BinaryExpression class, that "Mult" Inherits from BinaryExpression,
in order not to duplicate code. 
This function return true if two expression are equal. If they are, return new Pow, 
that it first expression is one of the equal expression. The second expression of this pow 
will be 2, because there is mult between the equal expression. 
In that way we return (x*y)^2 in this example. 

* Simplify that ((4.0 * ((y * z) * x)) + (4.0 * ((y * z) * x))) = (2.0 * (4.0 * ((y * z) * x)))
How did I approach this? 

# First of all, I created two new helper functinos in Plus class. 
The first function, called getNumberFromExp. As it derives from his name - this function
returns us the number from the expression. For example, let's define an expression " 8x ", 
when we will call to this function with the current expression, we will get 8.
The second function that I wrote, called commonFactor that returns an expression that 
represents the max common factor between two current expressions(because this function 
in plus class that has two expressions).
Finally, I added new if statment to simplify function in plus class, that checks 
if one of the expressions of plus is Num or mult, and then I called to the function "CommonFactor"
that used in the second helper function "getNumberFromExp". In that way I got the maximum common factor
between the expression, and then arrange the full expression after exits the shared expression outside parentheses.
For example - in our example ((4.0 * ((y * z) * x)) + (4.0 * ((y * z) * x))), the maximum common factor of them 
is (4*((y *z) * x)). Therefore, after exit him we get (2.0 * (4.0 * ((y * z) * x))).

* Simplify that ((3.0 * x) + (4.0 * x)) = (7.0 * x)
How did I approach this? 

# I used in the function that I mentioned above (commonFactor) and in this way I checked what is the common Factor
of the expression. In this example, out common factor is "x". Then, we get x(3.0 + 4.0). 
However, I wanted to get (7.0 * x) as expected. So in order to get the same result, the recursive function simplify
compute how much is 3.0 + 4.0, or in other words try to simplify that expression (3.0 + 4.0). Afterwards, return 7.0.
In this situation I get (x * 7.0). However, I wanted to get (7.0 * x). Therefore, I switch between the first
expression and the second. For example: return new Mult (this.getSecondExpression, this.getFirstExperssion).
Finally, I get that ((3.0 * x) + (4.0 * x)) = (7.0 * x), as needed.

I would be happy to get the 25 points of the bonus. I worked hard for the code and in particular on the bonus part,
and his explanation :)