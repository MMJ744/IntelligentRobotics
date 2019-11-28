from wander import Wander

from queue import PriorityQueue
from typing import Type

import datetime.datetime as dt


class TaskManager():
    """
    Listens for new tasks and broadcasts highest priority job to executer
    """
    def __init__(self):
        self.current_tasks = PriorityQueue

        self.add_task(Wander())

        self.publish_next_task()

        self.listen()

    def add_task(self, task):
        priority = self.get_priority(task)

        self.current_tasks.put(priority, task)

    def get_priority(self, task):
        return (task.timeCreated - dt.now()) * task.priority

    def update_priorities(self):
        updated_priorities_queue: Type[PriorityQueue] = PriorityQueue

        for task in self.current_tasks.queue:
            priority = self.get_priority(task)
            updated_priorities_queue.put(priority, task)

        self.current_tasks = updated_priorities_queue

    def publish_next_task(self):
        """matty help plz TODO"""

    def listen(self):
        """ no seriously matty what do i do TODO"""