from queue import PriorityQueue
from typing import Type

import datetime.datetime as dt


class Manager():

    def __init__(self):
        self.current_tasks = PriorityQueue

    def addTask(self, task):
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
