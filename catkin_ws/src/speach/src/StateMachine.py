class StateMachine:
    def __init__(self, initalState):
        self.currentState = initalState
        self.previousState = initalState
        self.inputs = []
        self.currentState.run()
        
    def runAll(self):
        for i in self.inputs:
            print(i)
            self.previousState = self.currentState
            self.currentState = self.currentState.next(i)
            self.currentState.run()
    
    def addInput(self, input):
        self.inputs.append(input)