#!/usr/bin/env python
class StateMachine:
    def __init__(self, initial_state, model):
        self.model = model
        self.currentState = initial_state
        self.previousState = initial_state
        self.inputs = []
        self.running = True
        
    def run_all(self):
        self.currentState.run(instance=self)
        while self.running:
            self.previousState = self.currentState
            self.currentState = self.currentState.next(self, self.inputs.pop())
            self.currentState.run(instance=self)

    def addInput(self, input):
        self.inputs.append(input)

