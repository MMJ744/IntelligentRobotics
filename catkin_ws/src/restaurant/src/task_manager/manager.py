from task_base import Wander, GreetCustomer

from queue import PriorityQueue
from typing import Type

import datetime.datetime as dt

import rospy
from restaurant.msg import Task


def get_priority(task):
    return (task.timeCreated - dt.now()) * task.priority

class TaskManager():
    """
    Listens for new tasks and broadcasts highest priority job to executer
    """
    def __init__(self):
        self.current_tasks = PriorityQueue

        self.add_task(Wander())

        rospy.Subscriber("/task", Task, self.subscriber)
        self.pub = rospy.Publisher('/task', Task, queue_size=1)
        rospy.init_node('Manager', anonymous=True)

        self.publish_next_task()

    def add_task(self, task):
        priority = get_priority(task)

        self.current_tasks.put(priority, task)

    def update_priorities(self, remove=None):
        updated_priorities_queue = PriorityQueue

        for task in self.current_tasks.queue:
            if not task == remove:                              # dubious, logic error?
                priority = get_priority(task)
                updated_priorities_queue.put(priority, task)
        self.current_tasks = updated_priorities_queue

    def publish_next_task(self):
        next = self.current_tasks.get()
        t = Task()
        t.task_type = next.type
        t.created_at = next.timeCreated
        t.finished = False
        self.pub.publish(t)

    def subscriber(self, task):
        if task.finished:
            if task.task_type == "Wander":
                t = Wander(task.created_at)
            elif task.task_type == "GreetCustomer":
                t = GreetCustomer(task.created_at)
            else:
                raise NotImplementedError

        self.update_priorities(remove=t)
        self.publish_next_task()
