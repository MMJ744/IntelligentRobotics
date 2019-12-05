#!/usr/bin/env python
class StateMachine:
    def __init__(self, initial_state, model):
        self.model = model
        self.currentState = initial_state
        self.previousState = None
        self.inputs = []
        self.running = True
        
    def run_all(self):
        self.currentState.run(instance=self)
        while self.running:
            
            input = ""
            try:
                input = self.inputs.pop()
                if input is None:
                    input = ""
            except:
                print("sm | inputs_empty")
            

            self.nextState = self.currentState.next(instance=self, input=input)
            self.previousState = self.currentState
            self.currentState = self.nextState
            self.nextState.run(instance=self)

    def addInput(self, input):
        self.inputs.append(input)

