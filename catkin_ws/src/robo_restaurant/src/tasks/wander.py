from task_base import TaskBase, Priority


class Wander(TaskBase):
    def __init__(self):
        """lorum ipsum"""

    def run(self):

        def __init__(self):
            """
            Superconstructor, handles timestamp and TODO
            """
            super().__init__()

        def init_priority_level(self):
            self.priority = Priority.BASE
            raise NotImplementedError()

        def start(self):
            """
            Starts the task running either from fresh or resuming

            end with super(Task, self).start()
            """
            super().start()
            raise NotImplementedError()

        def interrupt(self):
            """
            Interrupts the current task cleanly so it can resume
            """
            raise NotImplementedError()