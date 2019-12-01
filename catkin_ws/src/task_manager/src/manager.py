from queue import PriorityQueue
import TaskType as tt

import datetime.datetime as dt

import rospy
from restaurant.msg import Task




class TaskManager:
    """
    Listens for new tasks and broadcasts highest priority job to executer
    """
    def __init__(self):



        rospy.Subscriber("/task", Task, self.subscriber)
        self.pub = rospy.Publisher('/task', Task, queue_size=1)
        rospy.init_node('Manager', anonymous=True)

        self.current_tasks = PriorityQueue

        self.add_task(tt.Wander)

        self.publish_next_task()

    def add_task(self, task):
        self.current_tasks.put(task.priority, task)

    def update_priorities(self, remove=None):
        updated_priorities_queue = PriorityQueue

        for task in self.current_tasks.queue:
            if not task == remove:                              # dubious, logic error?
                task.update_priority()
                updated_priorities_queue.put(task.priority, task)
        self.current_tasks = updated_priorities_queue

    def publish_next_task(self):
        if self.current_tasks.empty():
            next = tt.Wander
        else:
            next = self.current_tasks.get()
        t = Task()
        t.task_type = next.type
        t.created_at = next.time_created
        t.table_number = next.table_number
        t.finished = False
        self.pub.publish(t)

    def subscriber(self, task):
        if task.finished:
            if task.task_type == "Wander":
                t = tt.Wander(task.created_at)
            elif task.task_type == "GreetCustomer":
                t = tt.Waiting(task.created_at)
            else:
                raise NotImplementedError
            self.update_priorities(remove=t)
            self.publish_next_task()
