# -*- coding: utf-8 -*-
"""
Created on Mon Mar 14 10:56:55 2016

@author: lauren
"""
import numpy as np

discount = 0.95
values = 'reward'
states = ['beginner', 'expert']
actions = ['high-autonomy', 'low-autonomy']
observations = ['no-hit', 'hit']
transitionProb = [np.array([[0.8, 0.2],[0.8, 0.2]]), np.array([[0.8, 0.2],[0.8, 0.2]])]
obsProb = np.array([[0.3, 0.7],[0.99, 0.01]])
rewards = [1, 2, 3, 4,5,6,7,8]


filename = 'test.pomdp'
target = open(filename, 'w')
target.write('#This is the test POMDP \n \n')

target.write('discount: ' + str(discount) + '\n')
target.write('values: ' + str(values) + '\n')

target.write('states: ')
for s in states:
  target.write(s + ' ')
target.write('\n')

target.write('actions: ')
for a in actions:
  target.write(a + ' ')
target.write('\n')

target.write('observations: ')
for o in observations:
  target.write(o + ' ')
target.write('\n')

target.write('start: uniform \n \n')

#Write transition probabilites
it = 0
for a in actions:
  target.write('T:' + str(actions[it]) + '\n')
  for j in range(len(transitionProb[it])):
    for i in range(len(transitionProb[it][j])):
      target.write(str(transitionProb[it][j][i]) + ' ')
    target.write('\n')
  target.write('\n')
  it+=1

  
#Write observation probabilites
it = 0
for a in actions:
  target.write('O:' + str(actions[it]) + '\n')
  for j in range(len(obsProb[it])):
   target.write(str(obsProb[it][j]) + ' ')
  target.write('\n \n')
  it+=1
  
#Rewards
it = 0
for a in actions:
  for s in states:
    for o in observations:
      target.write('R:' + str(a) + ' : ' + str(s) + ' : ' + ' * : ' +str(o) + ' ' + str(rewards[it]) + '\n')
      it += 1


target.close()