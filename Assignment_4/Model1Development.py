import tensorflow as tf
import numpy as np

from tensorflow.keras import datasets
import matplotlib.pyplot as plt

def printIMG(x,y, idx):
    plt.imshow(x[idx])
    plt.xlabel(y[idx])
# plt.imshow(x[0])

def preprocessing():
    (x_train, y_train), (x_test, y_test) = datasets.fashion_mnist.load_data() #tf.keras.datasets.mnist.load_data(path="mnist.npz") # 
    x_train =x_train/255  #np.reshape(x_train, (x_train.shape[0], 784))/255.
    x_test = x_test/255 #np.reshape(x_test, (x_test.shape[0], 784))/255.
    y_train = tf.keras.utils.to_categorical(y_train)
    y_test = tf.keras.utils.to_categorical(y_test)
    x_train=  tf.cast( tf.expand_dims(x_train,-1), dtype= tf.float32, name=None) 
    x_test=  tf.cast( tf.expand_dims(tf.expand_dims(x_test,-1),-1), dtype= tf.float32, name=None) 
    x_train= tf.image.grayscale_to_rgb(x_train, name=None)
    x_test= tf.image.grayscale_to_rgb(x_test, name=None)

    print(x_train.shape)
    print(x_train.shape)
    
    return (x_train, y_train, x_test, y_test)

class CNN:
    def __init__(self, layers, lr, dropout):
        self.hiddenlayers =layers
        self.num_features = layers[0]
        self.num_classes = layers[-1]
        self.L =  len(layers) #len(self.Weights)+1  # len(layers)

        self.lr = lr
        self.drouput = dropout

        # paramerters
        self.Weights = {}
        self.biases = {}
        self.dw = {}
        self.db = {}

        # thins that we can ply with are ###
        # pooling
        # stride
        #perhaps more
        # ###


        # should happen at the beginning when initlizing the 
        self.initilizeWeight()

    def initilizeWeight(self):
        # print("initilizeWeight")
        # for i in range(1, self.L):
        #     self.Weights[i] = tf.Variable(tf.random.normal(shape =(self.hiddenlayers[i], self.hiddenlayers[i-1]))) #, initializer =  tf.contrib.layers.xavier_initializer(seed = 0)))
        #     self.biases[i] = tf.Variable(tf.random.normal(shape = (self.hiddenlayers[i], 1)))

            self.Weights[1] = tf.compat.v1.get_variable("W1", [4, 4, 3, 8], initializer =  tf.keras.initializers.glorot_normal(seed = 0)) # tf.nn.conv2d
            self.Weights[2] = tf.compat.v1.get_variable("W2", [2, 2, 8, 10], initializer =  tf.keras.initializers.glorot_normal(seed = 0)) # , 8, 16

            # self.biases[1] = tf.Variable(tf.random.normal(shape = (self.hiddenlayers[i], 1)))
            # self.biases[2] = tf.Variable(tf.random.normal(shape = (self.hiddenlayers[i], 1)))



    def compute_loss(self, A, Y):
        # print("compute_loss")
        loss = tf.nn.softmax_cross_entropy_with_logits(Y,A)
        # why reduce mean
        return tf.reduce_mean(loss)

    def forwardpass(self, X):
        # print("forwardpass")

        # A = tf.convert_to_tensor(X, dtype=tf.float32)

        # for i in range(1, len(self.hiddenlayers)-1):
        s = [8,4]
        f =[8,4]
        A = tf.convert_to_tensor(X, dtype=tf.float32)
        for i in range(1, len(self.Weights)+1):

            # Z = tf.matmul(A,tf.transpose(self.Weights[i])) + tf.transpose(self.biases[i])
            Z = tf.nn.conv2d(A, self.Weights[i], strides = [1,1,1,1], padding = 'SAME')
            if i != self.L-1:
                A = tf.nn.relu(Z)
 
                A = tf.nn.max_pool(A, ksize = [1,f[i-1], f[i-1],1], strides = [1,s[i-1] ,s[i-1],1], padding = 'SAME')
            else:
                # input()
                input()
                F = tf.contrib.layers.flatten(Z)
                # FULLY-CONNECTED without non-linear activation function (not not call softmax).
                # 6 neurons in output layer. Hint: one of the arguments should be "activation_fn=None" 
                num_outputs = 10
                Z3 = tf.contrib.layers.fully_connected(F, num_outputs, activation_fn=None)
                input()
                A = Z3
        return A


    def updateParams(self):
        """ 
        We have all the weights and biases. now we need to update the weights with gd
        The formula is 
        """
        # print("updateParams")

        # check also with assignning sub
        for i in range(1, len(self.Weights)+1):
            self.Weights[i].assign_sub(self.lr * self.dw[i])
            #self.biases[i].assign_sub(self.lr * self.db[i]) 


    def printInfoModel(self):
        print(f"Number of features: {self.num_features}")
        print(f"Number of classes: {self.num_classes}")
        for i in range(1, len(self.hiddenlayers)):
            print(f"Hidden Layer {i}: {self.hiddenlayers[i]}")


    def computeLoss(self, Y, Z):
        loss = tf.nn.softmax_cross_entropy_with_logits(Y, Z) # try to use softmax
        return (tf.reduce_mean(loss))


    def train(self, x_train, y_train, x_test, y_test, epochs, steps_per_epoch, batch_size):

        history = {
            'val_loss':[],
            'train_loss':[],
            'val_acc':[]
        }
        
        for e in range(0, epochs):
            epoch_train_loss = 0.
            print('Epoch{}'.format(e), end='.')
            for i in range(0, steps_per_epoch):
                x_batch = x_train[i*batch_size:(i+1)*batch_size]
                y_batch = y_train[i*batch_size:(i+1)*batch_size]
                
                batch_loss = self.trainOnBatch(x_batch, y_batch)
                epoch_train_loss += batch_loss
                
                if i%int(steps_per_epoch/10) == 0:
                    print(end='.')
                    
            history['train_loss'].append(epoch_train_loss/steps_per_epoch)
            val_A = self.forwardpass(x_test)
            # print("val_A")
            # input()
            val_loss = self.compute_loss(val_A, y_test).numpy()
            history['val_loss'].append(val_loss)
            val_preds = self.predict(x_test)
            val_acc =    np.mean(np.argmax(y_test, axis=1) == val_preds.numpy())
            history['val_acc'].append(val_acc)
            print('Val acc:',val_acc)
        return history

    def dropout(self, X):
        a = tf.random.uniform((X.shape[0], X.shape[1]), dtype=tf.dtypes.float32)
        b = tf.where(a<self.drouput , 0,1)
        b = tf.cast(b, tf.dtypes.float32)
        return (tf.math.multiply(b,X))


    def trainOnBatch(self, X, Y):

        X = tf.convert_to_tensor(X, dtype = tf.float32)
        Y = tf.convert_to_tensor(Y,dtype = tf.float32)

        with tf.GradientTape(persistent=True) as tape:
            for i in range(1, len(self.Weights)+1):
                #X = self.dropout(X)
                Z = self.forwardpass(X)
                
                loss = self.computeLoss(Y, Z)
                self.dw[i] = tape.gradient(loss, self.Weights[i])
                #self.db[i] = tape.gradient(loss, self.biases[i])
        del tape
        self.updateParams()
        return loss.numpy()

    def predict(self, X):
        A = self.forwardpass(X)
        return tf.argmax(tf.nn.softmax(A), axis=1)
        

net = CNN([784,128,128,10],3e-3, 0.3)
net.printInfoModel()

(x_train, y_train, x_test, y_test) = preprocessing()

batch_size = 120
epochs = 25
steps_per_epoch = int(x_train.shape[0]/batch_size)
lr = 3e-3
print('Steps per epoch', steps_per_epoch)





history = net.train(
    x_train,y_train,
    x_test, y_test,
    epochs, steps_per_epoch,
    batch_size)
