# Ofir Ben Shoham

import numpy as np

sigmoid = lambda x: 1 / (1 + np.exp(-x))
sigmoidDer = lambda x: (sigmoid(x) * (1 - sigmoid(x)))

def main():
    trainingSet_x, trainingSet_y, validation_x, validation_y = getTrainingAndValidation()
    test_set = np.loadtxt("test_x")

    # initialize the params W1, W2, B1, B2...
    H_Size = 100 # need to adjust this parameter to get better result.
    classes_Number = 10

    W1 = np.random.uniform(-0.08, 0.08, ( H_Size, 784))
    B1 = np.random.uniform(0,0,(H_Size, 1))
    W2 = np.random.uniform(-0.08, 0.08, (classes_Number, H_Size))
    B2 = np.random.uniform(0,0, (classes_Number, 1))

    # Train the parameters.
    W1, W2, B1, B2 = train(trainingSet_x, trainingSet_y, W1, B1, W2, B2, 6, 0.001)
    writePredictions(test_set, W1, B1, W2, B2, sigmoid)


# train the paramaters on out training set.
def train(trainingSet_x, trainingSet_y, W1, B1, W2, B2, epochNum, learningRate):

    for inx in range(0, epochNum):

        # shuffle x, y into our training set.
        np.random.seed(0)
        np.random.shuffle(trainingSet_x)
        np.random.seed(0)
        np.random.shuffle(trainingSet_y)



        for inputPic, tag in zip(trainingSet_x, trainingSet_y):
            prodVec, h, z1, z2 = fowardFunc(W1, B1, W2, B2, sigmoid, inputPic)

            # return the follows when will have backFunc function to update the parameters.
            dW1, dB1, dW2, dB2 = backFunc(prodVec, inputPic, z1, h, tag, W2)

            # get the gradients from backFunc.
            # update the params according SGD.
            
            W1 = W1 - learningRate * dW1
            W2 = W2 - learningRate * dW2
            B1 = B1 - learningRate * dB1
            B2 = B2 - learningRate * dB2

    return W1, W2, B1, B2


# function that returns the training set & validation set
# returns trainingSet_x, trainingSet_y, validation_x, validation_y
def getTrainingAndValidation():
    x_data =  np.loadtxt("train_x")
    y_data = np.loadtxt("train_y")
    common_length = len(x_data)

    # seperate the data to training set & validation set (80% training, 20& validation).
    dev_size = round(common_length * 0.2)
    validation_x = np.dot(x_data[common_length - dev_size : ], 1/255.0)
    validation_y = y_data[common_length - dev_size : ]
    trainingSet_x = np.dot(x_data[0 : common_length - dev_size], 1/255.0)
    trainingSet_y = y_data[0 : common_length - dev_size]
    return trainingSet_x, trainingSet_y, validation_x, validation_y


# write predictions on the test set into test.pred file.

def writePredictions(test_set_x, W1, B1, W2, B2, active_func):
    f = open("test.pred", "w")
    temp = ""
    for x in test_set_x:
        probVec, j2, j3, z2 = fowardFunc(W1, B1, W2, B2, active_func, x)
        tag = np.argmax(probVec)
        temp += (str(tag) + '\n') # save it into a string to imporve cache time since we work with a file.

    temp = temp[:-1] # remove last '\n'
    f.write(temp)
    f.close()


# return vector of probabilities that each index(i) is the prob that x
# is class number i. (i is from 0 to 9 since we have 10 classes).
# It means, return vector that his size is: 10 x 1.
# Moreover, return h, z1 since we will need them for backFunc.
def fowardFunc(W1, B1, W2, B2, activeFunc, x):

    temp = np.dot(W1, x)
    z1 = np.add(temp.reshape(100, 1), B1)

    # convert z1 to be in the range (0-1) by dividing with 255
    # (since the max value of each element into z1 is 255)
    Z1AfterDiv = z1
    h = activeFunc(Z1AfterDiv)
    probVec = softmax(W2, h, B2)
    z2 = np.add(np.dot(W2, h ) ,B2 )

    return probVec, h, Z1AfterDiv, z2


# return dW1, db1, dW2, db2 to update according SGD.
def backFunc(predVec, currentX, z1, h1, tag, W2):

    currentX.shape = [784, 1]

    y_z = np.zeros(10)
    y_z.shape = [10, 1]
    y_z[int(tag)] = 1
    db2 = predVec - y_z
    dW2 = np.dot(db2, h1.T)
    db1 = np.dot(W2.T, db2) * sigmoidDer(z1)
    dW1 = np.dot(db1, currentX.T)

    return dW1, db1, dW2, db2


# softmax function to get the probabilities.
def softmax(W, X, B):
    x = np.add(np.dot(W, X ) ,B )
    e_x = np.exp(x - np.max(x))
    return e_x / e_x.sum(axis=0)

main()
