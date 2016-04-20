from pomdp import *



def main():
  #POMDP files
  filename_env =  'POMDPu/userlevel2.pomdp'
  filename_policy = 'POMDPu/out.policy'
  pomdp = POMDP(filename_env, filename_policy, np.array([[0.25],[0.25],[0.25],[0.25]]))
  action = 1
  while True:
    observation = input('Give Observation 0 many hit, 1 some hit, 2 no hit: ')
    mapLevel = input('Give map level 0 easy 1 hard: ')
    mapLevel *= 3
    observation = observation + mapLevel
    pomdp.update_belief(action,observation)
    action = pomdp.get_best_action()[0] 
    print 'Most likely state: ' + pomdp.pomdpenv.states[np.argmax(pomdp.belief)]
    print 'Observation: ' +  pomdp.pomdpenv.observations[observation]
    print 'Action: ' + pomdp.pomdpenv.actions[action]
    print pomdp.belief
    print 'Reward: ' + str(pomdp.get_best_action()[1] )
    
if __name__ == '__main__':
    main()