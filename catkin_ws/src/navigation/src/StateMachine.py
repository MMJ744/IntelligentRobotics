#!/usr/bin/env python
class StateMachine:
    def __init__(self, initial_state, model):
        self.model = model
        self.currentState = initial_state
        self.previousState = initial_state
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

