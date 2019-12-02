import taskTypes as tt
from task_CheckUp import CheckUpTask
from task_CollectPayment import CollectPaymentTask
from task_TakeOrder import TakeOrderTask
from task_Wander import WanderTask
from task_NewCustomer import NewCustomerTask

import rospy
from navigation.msg import Task


class Model:
    def __init__(self):

        self.locations = ["Kitchen", "Table1", "Table2", "Table3", "FrontDesk"]

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

    def run_task(self, task_info):
        """
        creates and runs task, broadcasting when done
        :param task_info:
        """
        self.current_task = task_info

        task_executable = None

        if self.current_task.type == "CheckUp":
            task_executable = CheckUpTask()
        elif self.current_task.type == "CollectPayment":
            task_executable = CollectPaymentTask()
        elif self.current_task.type == "NewCustomer":
            task_executable = NewCustomerTask()
        elif self.current_task.type == "TakeOrder":
            task_executable = TakeOrderTask()
        elif self.current_task.type == "Wander":
            task_executable = WanderTask()

        if task_executable is None:
            raise NotImplementedError

        task_executable.run_all()

        self.publish_done(self.current_task)

    def publish_done(self, task):
        t = Task()
        t.task_type = task.type
        t.created_at = task.timeCreated
        t.finished = True
        self.pub.publish(t)

    def subscriber(self, task):
        if not task.finished:
            priority_task = tt.create(task)
            self.run_task(priority_task)
