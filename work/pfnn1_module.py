# -*- coding: utf-8 -*
"""
Example of using tensorflow to implement a neural network (PFNN) for timeseries prediction.
PFNN Implemented by Taesoo Kwon.
"""

from __future__ import print_function, division

import numpy as np
import os.path
import pdb
import tensorflow as tf

# PFNN implemented by taesoo
def psiToCoefs(batch_psi1):
    nitem=batch_psi1.shape[0]
    batch_coefs=np.zeros((nitem, 4))
    for j in range(nitem):
        psi=batch_psi1[j,0]
        w=np.mod(psi*4,1)
        coefs=np.zeros(4)
        coefs[int(np.mod(np.floor(psi*4)-1,4))]=(-0.5*w +w**2 -0.5*w**3)
        coefs[int(np.mod(np.floor(psi*4)+0,4))]=(-5.0/2.0*w**2 +3.0/2.0*w**3+1.0)
        coefs[int(np.mod(np.floor(psi*4)+1,4))]=(0.5*w +2.0*w**2 -3.0/2.0*w**3)
        coefs[int(np.mod(np.floor(psi*4)+2,4))]=(-0.5*w** 2 +0.5*w**3)
        batch_coefs[j,:]=coefs

    return batch_coefs
def mlp(x, weights, biases,  postfix):
    layer_1 = tf.add( tf.matmul( x, weights['h1']), biases['b1'])
    layer_1 = tf.nn.elu(layer_1)
    # Hidden layer with elu activation
    layer_2 = tf.add( tf.matmul( layer_1, weights['h2']), biases['b2'])
    layer_2 = tf.nn.elu(layer_2)
    # Hidden layer with elu activation
    layer_3_1 = tf.add( tf.matmul( layer_2, weights['h3']), biases['b3'])
    layer_3_1 = tf.nn.elu(layer_3_1)
    layer_3_2 = tf.add( tf.matmul( layer_2, weights['h4']), biases['b4'])
    layer_3_2 = tf.nn.elu(layer_3_2)
    layer_3_3 = tf.add( tf.matmul( layer_2, weights['h5']), biases['b5'])
    layer_3_3 = tf.nn.elu(layer_3_3)
    # Output layer with linear activation
    out_layer1 = tf.add( tf.matmul( layer_3_1, weights['out1'+postfix]), biases['out1'+postfix])
    return out_layer1


def multilayer_perceptron(x, coefs_a, coefs_b, coefs_c, coefs_d, weights, biases):
    layer_a=mlp(x, weights, biases,  '_a')
    layer_b=mlp(x, weights, biases,  '_b')
    layer_c=mlp(x, weights, biases,  '_c')
    layer_d=mlp(x, weights, biases,  '_d')
    out_layer = layer_a*coefs_a +layer_b*coefs_b + layer_c*coefs_c + layer_d*coefs_d
    return out_layer

class Model(object):
    def __init__(self, model_name, n_input, nb_outputs):
        self.input_shape=n_input
        self.output_shape=nb_outputs
        learning_rate = 0.0001
        #n_hidden_1=66
        #n_hidden_2=26
        #n_hidden_3=36 #-> epoch 7: 0.28
        #n_hidden_1=100
        #n_hidden_2=50
        #n_hidden_3=50 #-> epoch 7: 0.27
        #n_hidden_1=200
        #n_hidden_2=100
        #n_hidden_3=100 #-> epoch 7: 0.21, epoch 60:0.12, epoch 130: 0.08, epoch 200: 0.07
        n_hidden_1=20
        n_hidden_2=50
        n_hidden_3=20 #-> epoch 7: 0.32, epoch 60: 0.19, epoch 130: 0.17, epoch 200: 
        self.x=tf.placeholder("float", [None, n_input])
        self.y=tf.placeholder("float", [None, nb_outputs])
        self.coefs_a=tf.placeholder("float", [None,1])
        self.coefs_b=tf.placeholder("float", [None,1])
        self.coefs_c=tf.placeholder("float", [None,1])
        self.coefs_d=tf.placeholder("float", [None,1])
        self.model_name=model_name
        print('# of parameters for each layer:', n_input*n_hidden_1,n_hidden_1*n_hidden_2,n_hidden_2*n_hidden_3*3, n_hidden_3*nb_outputs)
        with tf.name_scope(model_name+'L'):
            # Store layers weight & bias
            self.weightsL = {
                'h1': tf.Variable(tf.random_uniform([n_input, n_hidden_1], -0.05, 0.05)),
                'h2': tf.Variable(tf.random_uniform([n_hidden_1, n_hidden_2], -0.05, 0.05)),
                'h3': tf.Variable(tf.random_uniform([n_hidden_2, n_hidden_3],-0.05, 0.05)),
                'h4': tf.Variable(tf.random_uniform([n_hidden_2, n_hidden_3],-0.05, 0.05)),
                'h5': tf.Variable(tf.random_uniform([n_hidden_2, n_hidden_3],-0.05, 0.05)),
                'out1_a': tf.Variable(tf.random_uniform([n_hidden_3, nb_outputs],-0.05, 0.05)),
                'out1_b': tf.Variable(tf.random_uniform([n_hidden_3, nb_outputs],-0.05, 0.05)),
                'out1_c': tf.Variable(tf.random_uniform([n_hidden_3, nb_outputs],-0.05, 0.05)),
                'out1_d': tf.Variable(tf.random_uniform([n_hidden_3, nb_outputs],-0.05, 0.05)),
            }
            self.biasesL = {
                'b1': tf.Variable(tf.random_uniform([n_hidden_1], -0.01, 0.01)),
                'b2': tf.Variable(tf.random_uniform([n_hidden_2], -0.01, 0.01)),
                'b3': tf.Variable(tf.random_uniform([n_hidden_3], -0.01, 0.01)),
                'b4': tf.Variable(tf.random_uniform([n_hidden_3], -0.01, 0.01)),
                'b5': tf.Variable(tf.random_uniform([n_hidden_3], -0.01, 0.01)),
                'out1_a': tf.Variable(tf.random_uniform([nb_outputs], -0.01, 0.01)),
                'out1_b': tf.Variable(tf.random_uniform([nb_outputs], -0.01, 0.01)),
                'out1_c': tf.Variable(tf.random_uniform([nb_outputs], -0.01, 0.01)),
                'out1_d': tf.Variable(tf.random_uniform([nb_outputs], -0.01, 0.01)),
            }


        # Construct model
        self.pred = multilayer_perceptron(self.x, self.coefs_a, self.coefs_b, self.coefs_c, self.coefs_d, self.weightsL, self.biasesL)

        # Define loss and optimizer
        #self.cost = tf.reduce_mean(tf.nn.softmax_cross_entropy_with_logits(logits=self.pred, labels=self.y))
        self.cost = tf.reduce_mean(tf.square(self.pred-self.y))
        #self.cost = tf.reduce_mean(tf.square(self.pred-self.y))+0.0001*(
        #        tf.reduce_mean(tf.square(self.weightsL['h1_a']))+
        #        tf.reduce_mean(tf.square(self.weightsL['h1_b']))+
        #        tf.reduce_mean(tf.square(self.weightsL['h1_c']))+
        #        tf.reduce_mean(tf.square(self.weightsL['h1_d']))+
        #        tf.reduce_mean(tf.square(self.weightsL['h2_a']))+
        #        tf.reduce_mean(tf.square(self.weightsL['h2_b']))+
        #        tf.reduce_mean(tf.square(self.weightsL['h2_c']))+
        #        tf.reduce_mean(tf.square(self.weightsL['h2_d']))+
        #        tf.reduce_mean(tf.square(self.weightsL['h3_a']))+
        #        tf.reduce_mean(tf.square(self.weightsL['h3_b']))+
        #        tf.reduce_mean(tf.square(self.weightsL['h3_c']))+
        #        tf.reduce_mean(tf.square(self.weightsL['h3_d']))+
        #        tf.reduce_mean(tf.square(self.weightsL['out_a']))+
        #        tf.reduce_mean(tf.square(self.weightsL['out_b']))+
        #        tf.reduce_mean(tf.square(self.weightsL['out_c']))+
        #        tf.reduce_mean(tf.square(self.weightsL['out_d']))+
        #self.optimizer =tf.train.GradientDescentOptimizer(learning_rate=0.1).minimize(self.cost)
        self.optimizer = tf.train.AdamOptimizer(learning_rate=learning_rate).minimize(self.cost)
        self.init = tf.global_variables_initializer()

        #config = tf.ConfigProto()
        #config.graph_options.optimizer_options.global_jit_level = tf.OptimizerOptions.ON_1
        self.sess=tf.compat.v1.Session()
        self.saver=tf.compat.v1.train.Saver([
            self.weightsL[  'h1'], 
            self.weightsL[  'h2'], 
            self.weightsL[  'h3'], 
            self.weightsL[  'h4'], 
            self.weightsL[  'h5'], 
            self.weightsL['out1_a'], self.weightsL['out1_b'],self.weightsL['out1_c'],self.weightsL['out1_d'],
            self. biasesL[  'b1'], 
            self. biasesL[  'b2'], 
            self. biasesL[  'b3'], 
            self. biasesL[  'b4'], 
            self. biasesL[  'b5'], 
            self. biasesL['out1_a'], self. biasesL['out1_b'],self. biasesL['out1_c'],self. biasesL['out1_d'],
            ])


    def fit(self, x_train, y_train, psi1_train, nb_epoch, batch_size):
        # Launch the graph
        display_step = 1
        num_examples=x_train.shape[0]
        sess=self.sess
        sess.run(self.init)

        if len(psi1_train.shape)==1:
            psi1_train=psi1_train.reshape(psi1_train.shape[0],1)

        # Training cycle
        for epoch in range(nb_epoch):
            avg_cost = 0.
            total_batch = int(num_examples/batch_size)+1
            # Loop over all batches
            for i in range(total_batch):
                batch_x=x_train[i*batch_size: (i+1)*batch_size]
                batch_y=y_train[i*batch_size: (i+1)*batch_size]
                batch_psi1=psi1_train[i*batch_size: (i+1)*batch_size]
                batch_coefsL=psiToCoefs(np.atleast_2d(batch_psi1[:,0]).T)
                batch_coefsR=psiToCoefs(np.atleast_2d(batch_psi1[:,1]).T)

                # Run optimization op (backprop) and cost op (to get loss value)
                _, c = sess.run([self.optimizer, self.cost], feed_dict={
                    self.x: batch_x, 
                    self.coefs_a: np.atleast_2d(batch_coefsL[:,0]).T, 
                    self.coefs_b: np.atleast_2d(batch_coefsL[:,1]).T, 
                    self.coefs_c: np.atleast_2d(batch_coefsL[:,2]).T, 
                    self.coefs_d: np.atleast_2d(batch_coefsL[:,3]).T, 
                    self.y: batch_y})
                # Compute average loss
                avg_cost += c / total_batch
            # Display logs per epoch step
            if epoch % display_step == 0:
                print("Epoch:", '%04d' % (epoch+1), "cost=", \
                    "{:.9f}".format(avg_cost))
        print("Optimization Finished!")

        #vals=sess.run(self.pred, feed_dict={self.x:x_test})


    def predict(self, x_test, psi1_test):
        sess=self.sess
        if len(psi1_test.shape)==1:
            psi1_test=psi1_test.reshape(1, psi1_test.shape[0])
        coefsL=psiToCoefs(np.atleast_2d(psi1_test[:,0]).T)
        coefsR=psiToCoefs(np.atleast_2d(psi1_test[:,1]).T)

        vals=sess.run(self.pred, feed_dict={
            self.x: x_test, 
            self.coefs_a: np.atleast_2d(coefsL[:,0]).T, 
            self.coefs_b: np.atleast_2d(coefsL[:,1]).T, 
            self.coefs_c: np.atleast_2d(coefsL[:,2]).T, 
            self.coefs_d: np.atleast_2d(coefsL[:,3]).T, 
            })
        return vals
    def save_weights(self, file_name):
        self.saver.save(self.sess, file_name)

    def load_weights(self,file_name):
        sess=self.sess
        sess.run(self.init)
        with tf.name_scope(self.model_name):
            self.saver.restore(sess, file_name)

def make_regressor(model_name, nb_inputs=1, nb_outputs=1):
    model=Model( model_name, nb_inputs, nb_outputs)
    return model


def to2D(hypermat):
    s=hypermat.shape
    return hypermat.reshape(s[0], s[1]*s[2])


global models
models={}

def createModel( name, nb_inputs, nb_outputs):
    global models
    # lua uses float. so converting to int
    nb_inputs=int(nb_inputs) 
    nb_outputs=int(nb_outputs) 
    model = make_regressor(name, nb_inputs=nb_inputs, nb_outputs=nb_outputs)
    print('\n\nModel with input size {}, output size {}'.format(model.input_shape, model.output_shape ))
    models[name]=[model]

def normalizeData(model_name, XO, yo):
    global models
    model=models[model_name]
    assert(len(model)==1)
    X_mean=XO.mean(axis=0)
    X_std=XO.std(axis=0)
    y_mean=yo.mean(axis=0)
    y_std=yo.std(axis=0)

    X_std[X_std<1e-6]=1e-6
    y_std[y_std<1e-6]=1e-6
    model.append([X_mean, X_std, y_mean, y_std])
    X = (XO - X_mean) / X_std
    y = (yo - y_mean) / y_std
    return X,y

def fitModel(model_name, XO,psi1, yo, numEpoch=25):

    global models
    X,y=normalizeData(model_name, XO, yo)
    np.savez(model_name+'_info.npz', models[model_name][1:])
    # you can temporaily override numEpoch here. 
    #numEpoch=1
    model=models[model_name]
    nb_samples=X.shape[0]
    nb_inputs=X.shape[1]


    print('\n\n', model_name, ' Input features:', X, '\n\nOutput labels:', y, '\n\n')
    test_size = int(0.01 * nb_samples)           # In real life you'd want to use 0.2 - 0.5
    X_train, X_test, y_train, y_test = X[:-test_size], X[-test_size:], y[:-test_size], y[-test_size:]
    psi1_train, psi1_test=psi1[:-test_size], psi1[-test_size:]
    batch_size=32
    model[0].fit(X_train, y_train, psi1_train, numEpoch, batch_size)
    model[0].save_weights(model_name+'.weights')

    pred = model[0].predict(X_test, psi1_test)
    print('\n\nactual', 'predicted', sep='\t')
    for actual, predicted in zip(y_test, pred.squeeze()):
        print(actual.squeeze(), predicted, sep='\t')


def predict(model_name, XO, psi, yo):
    global models
    model=models[model_name]
    X_mean, X_std, y_mean, y_std=model[1]
    X=(XO-X_mean)/X_std
    pred = model[0].predict(np.array([X]), psi).squeeze()
    yo[:]=pred*y_std+y_mean
    #print(y[0:5])

def testModel( model_name, X,y, psi):
    global models
    model=models[model_name][0]
    pred = model.predict(X,psi)
    print('\n\nactual', 'predicted', sep='\t')
    for actual, predicted in zip(y, pred.squeeze()):
        print(actual.squeeze(), predicted, sep='\t')

def loadModel( model_name):
    global models
    model=models[model_name][0]
    model.load_weights('./'+model_name+'.weights')

def loadModelFromData(name, XO, yo):
    global models
    normalizeData(name, XO, yo)
    loadModel(name)

def fitOrLoadModel(name, XO, phase, yo, nb_epoch=250):
    global models
    if os.path.exists(name+'.weights.meta'):
        if os.path.exists(name+'_info.npz'):
            data=np.load(name+'_info.npz')
            model=models[name]
            model.append(data['arr_0'][0])
        else:
            normalizeData(name, XO, yo)
            np.savez(name+'_info.npz', models[name][1:])

        loadModel(name)
    else:
        fitModel(name,XO,phase, yo, int(nb_epoch))
