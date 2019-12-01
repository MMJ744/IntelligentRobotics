class StateMachine:
    def __init__(self, initalState):
        self.currentState = initalState
        self.previousState = initalState
        self.running = True
        
    def runAll(self, inputs):
        self.currentState.run()
        while self.running:
            self.previousState = self.currentState
            self.currentState = self.currentState.next(inputs.pop())
            print(self.currentState)
            self.currentState.run()

