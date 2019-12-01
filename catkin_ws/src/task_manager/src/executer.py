from task_base import Wander, GreetCustomer
import rospy
from restaurant.msg import Task


class Model:
    def __init__(self):
        self.priorityLevels = {
            "IMMEDIATE": 0,
            "HIGH": 1,
            "MID": 2,
            "LOW": 3,
            "BASE": 4
        }

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
        if task.task_type == "Wander":
            t = Wander(time=task.created_at)
        elif task.task_type == "GreetCustomer":
            t = GreetCustomer(time=task.created_at)
        else:
            raise NotImplementedError

        self.current_task = t

        # rate = rospy.Rate(10)  # 10hz
        t.run()
        # while self.run:
        #   rate.sleep()

        self.publish_done(t)

    def publish_done(self, task):
        t = Task()
        t.task_type = task.type
        t.created_at = task.timeCreated
        t.finished = True
        self.pub.publish(t)

    def subscriber(self, task):
        if not task.finished:
            self.run_task(task)
        """help TODO"""