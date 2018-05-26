# Ofir Ben Shoham.

import numpy
import matplotlib.pyplot as plt


def main():
    W, B  = train(generateTrainingSet(), 1000, 0.01, 0.01)
    test = []
    for x in numpy.linspace(0, 10, 100):
        test.append(softMax(W, x, B)[0])
    plotFunc(numpy.linspace(0, 10, 100),generateTrueDistibution(),test)



# return tuple of 300 point..
# Each point has (float number, class tag). class tag = 1/2/3.
def generateTrainingSet():
    Xvalues = []
    Xvalues.extend(numpy.random.normal(2, 1, 100))
    Xvalues.extend(numpy.random.normal(4, 1, 100))
    Xvalues.extend(numpy.random.normal(6, 1, 100))
    # now Xvalues contains 300 values.


    Ytags = []
    for classNum in range(1, 4):
        Ytags.extend([classNum for i in range(100)])
    # now Ytags contians {1..1 (100 times), 2...2 (100 times), and then 3..3 (100 times also).

    # return list that each element is a tuple of a value and a class tag (1/2/3).
    trainingSetToReturn = zip(Xvalues, Ytags)
    return trainingSetToReturn

# getting W, B, num (current float num), and a tag.
# return the der -> dw, db.
def gettingDer(W, B, num, tag, j):
    updTag = tag - 1 # value between 0-3 as the indexes are.
    smVal = softMax(W, num, B)
    if updTag == j:
        # it means i = j
        dw = smVal * num - num
        db = smVal - 1
    else:
        # it means i != j
        dw = smVal * num
        db = smVal
    return dw, db



# getting the training set which compose from points(number, tag), and then train it.
def train(trainingSet, epochNumber, learningRate, bRate):
    W = numpy.random.random((3, 1)) # initialized by random.
    B = numpy.random.random((3, 1)) # the same.

    for i in range(epochNumber):
        numpy.random.shuffle(trainingSet)
        for num,tag in trainingSet:
            # into predictVec we have list that each element predict what is the probability that
            # we into his class(element into index 0 = changes to be in class 1).


            for inx in range(0, 3):
                dw, db = gettingDer(W, B, num, tag, inx)
                # update W and B:
                W[inx] = W[inx] - learningRate * dw[inx]
                B[inx] = B[inx] - bRate * db[inx]

    return W, B


def generateTrueDistibution():
    finalTrueProb = []

    for x in numpy.linspace(0, 10, 100):
        temp = [normalCalc(x, currentMiu) for currentMiu in range(2, 8, 2)]
        finalTrueProb.append(temp[0] / (temp[0] + temp[1] + temp[2]))

    return finalTrueProb

# function that returns the result of softmax function.
"""
W is a vector of K x 1, each row is a float number for class (since we have K calsses).
X is the input vector -> size of D rows, it's a vector. 
B is a vector.
return a float number which is the result of the softmax (= the probability).
"""
def softMax(W, X, B):
    x = W * X + B
    e_x = numpy.exp(x - numpy.max(x))
    return e_x / e_x.sum(axis=0)


# comupte the prob of the normal distribution.
def normalCalc(x, miu):
    return 1 / numpy.sqrt(2 * numpy.pi) * numpy.exp((-(x - miu) ** 2) / 2)


# plot the true distribution graph.
def plotFunc(xToDraw, truthProb, trainedProb):
    plt.xlabel('X')
    plt.ylabel('p(y = 1 | x)')
    plt.plot(xToDraw, truthProb, label='true distribution')
    plt.plot(xToDraw, trainedProb, label='trained prob')
    plt.legend()
    plt.show()

main()


