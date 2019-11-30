class StateMachine:
    def __init__(self, initalState):
        self.currentState = initalState
        self.previousState = initalState
        self.inputs = []
        self.currentState.run()
        self.running = True
        
    def runAll(self):
        while self.running:
            self.previousState = self.currentState
            print('dfgidifg')
            self.currentState = self.currentState.next(self.inputs.pop())
            print(self.currentState)
            self.currentState.run()
    
    def addInput(self, input):
        self.inputs.append(input)