import TaskType as tt
from WanderTask import WanderTask
from WaitingTask import WaitingTask

import rospy
from restaurant.msg import Task


class Model:
    def __init__(self):

        self.locations = ["Kitchen", "Table1", "Table2", "FrontDesk"]

        self.tables = [
            {
                "places": 6,
                "available": False,
                'id': 1
            },
            {
                "places": 4,
                "available": True,
                'id': 2
            }
        ]


class TaskExecuter:
    """
    Listens to TaskManager, executes jobs, and broadcasts once done
    """

    def __init__(self):
        rospy.Subscriber("/task", Task, self.subscriber)
        self.pub = rospy.Publisher('/task', Task, queue_size=1)
        rospy.init_node('Executer', anonymous=True)

    def run_task(self, task):
        """
        creates and runs task, broadcasting when done
        :param task:
        """
        self.current_task = task

        t = None

        if self.current_task.type == "Wander":
            t = WanderTask
        elif self.current_task.type == "Waiting":
            t = WaitingTask

        if t is None:
            raise NotImplementedError

        t.run_all()

        self.publish_done(self.current_task)

    def publish_done(self, task):
        t = Task()
        t.task_type = task.type
        t.created_at = task.timeCreated
        t.finished = True
        self.pub.publish(t)

    def subscriber(self, task):
        if not task.finished:
            if task.task_type == "Wander":
                priority_task = tt.Wander(task.created_at)
            elif task.task_type == "Waiting":
                priority_task = tt.Waiting(task.created_at)
            else:
                raise NotImplementedError
            self.run_task(priority_task)
