#!/usr/bin/env python
class StateMachine:
    def __init__(self, initalState):
        self.currentState = initalState
        self.previousState = initalState
        self.inputs = []
        self.running = True
        
    def runAll(self):
        self.currentState.run()
        while self.running:
            self.previousState = self.currentState
            self.currentState = self.currentState.next(self.inputs.pop())
            self.currentState.run()

    def addInput(self, input):
        self.inputs.append(input)

