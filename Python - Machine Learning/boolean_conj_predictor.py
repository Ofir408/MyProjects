
# Name: Ofir Ben-Shoham

import sys
import numpy as np


filePath = sys.argv[1]
training_examples = np.loadtxt(filePath).tolist()

# load x, y from the file.
def loadDataFromFile():
    Xexamples = []
    YexamplesTags = []
    numberOfVar = len(training_examples[0]) - 1 # decrease one (tag value). numberOfVar = 4 in our code.

    # convert data from to Xexamples & YexamplesTags
    for current in training_examples:

        Xexamples.append(current[0:numberOfVar]) # update xExamples
        YexamplesTags.append(current[-1])

    # convert lists to integers.
    index = 0
    for currentExample in Xexamples:
        other = [int(i) for i in currentExample]
        Xexamples[index] = other
        index += 1
    YexamplesTags = [int(i) for i in YexamplesTags]

    return Xexamples, YexamplesTags, numberOfVar


    # for example if finalListResult = " [1, 2, -3], sizeOfLIst = 3,
    #  it means that we should write x1,x2,not(x3) into our text
def covertToText(finalListResult):

    strToWrite = ""
    for currentNum in finalListResult:
        j = abs(currentNum)
        if currentNum != j:
            # negative number, write not(i)
            strToWrite += "not(x" + str(j) + "),"
        else:
            strToWrite += "x" +(str(currentNum) + ",")

    withOutlast = strToWrite[0:len(strToWrite) - 1]  # without last ","
    with open("output.txt", "w") as f:
        f.write(withOutlast)


def ConsistencyAlgorithm(X, Y, d, numberOfExamples):
    # initialize all_negative_hypothesis
    h = []
    for i in range(1, d + 1):
        h.append(i)
        h.append(-i) # since we want the negative of each element in all_negative_hypothesis.
    hPrev = h
    # finish to build all_negative

    for instanceIndex in range(0, numberOfExamples):
        h = hPrev
        if Y[instanceIndex] == 1 and checkIfTrue(X[instanceIndex], h) == 0:
            for index in range(0, len(X[instanceIndex])):
                currentIndexVal = X[instanceIndex][index]

                if currentIndexVal == 1:
                    if (-(index+1)) in h:
                        h.remove(-(index+1))
                if currentIndexVal == 0:
                    if (index+1) in h:
                        h.remove(index + 1)

        hPrev = h
    return h

    # write the result to text file



# for example getting (-1, 2, 1) = not(x1) & x2 & x1.
# This method replace the not to normal values, then checks if the final result is true or false.
def checkIfTrue(xListFromFile, v):
    zerosAndOnesList = [convertToVal(xListFromFile, current) for current in v]
    # now we have list with zeros and ones, return true if and between them returns true. Otherwise false.
    for i in zerosAndOnesList:
        if i == 0:
            return False # since 0 &&.... && = 0 always.
    return True



# helper function that gets current list and an integer (>0 or <0)
# Returns his value in our expression (0 or 1)
# for example if we got -3 = not(x3)
# and if we got 4 = x4
def convertToVal(xListFromFile, x):
    if (x > 0):
        toReturn = xListFromFile[x - 1]  # since the first value in list is for x1 and not x0.
    else:
        toReturn = (xListFromFile[-x - 1] + 1) % 2  # if 0 now it's 1, if 0 now it's 2.
    return toReturn

def main():
    X, Y, d =  loadDataFromFile()
    h = ConsistencyAlgorithm(X, Y, d, len(Y))
    covertToText(h)



# main function here..
main()
