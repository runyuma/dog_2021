# -*-coding:utf-8-*-
class action():
    def __init__(self,name,prioity,actionlenth,action_initfun,actionfun,action_des):
        # name just name
        # priority higher less necessary 0 is most necessary
        # actionlenth the total time index
        # action_initfun prepare for action
        # action_fun main part of action
        self.name = name
        self.priority = prioity
        self.actionlenth = actionlenth
        self.action_initfun = action_initfun
        self.action_fun = actionfun
        self.action_des = action_des