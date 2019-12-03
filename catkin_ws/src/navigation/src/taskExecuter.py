#!/usr/bin/env python
from task_Checkup import CheckupTask
from task_CollectPayment import CollectPaymentTask
from task_TakeOrder import TakeOrderTask
from task_Wander import WanderTask
from task_NewCustomer import NewCustomerTask

import rospy
from navigation.msg import Task

te = None


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
        self.model = Model()

        rospy.Subscriber("task", Task, self.subscriber)
        self.pub = rospy.Publisher('task', Task, queue_size=1)
        rospy.init_node('Executer', anonymous=True)

        self.task_msg = None
        self.current_task = None

    def run_task(self):
        """
        creates and runs task, broadcasting when done
        """
        task_executable = None

        if self.task_msg.task_type == "Checkup":
            task_executable = CheckupTask(self.model)
        elif self.task_msg.task_type == "CollectPayment":
            task_executable = CollectPaymentTask(self.model, self.task_msg.table_number)
        elif self.task_msg.task_type == "NewCustomer":
            task_executable = NewCustomerTask(self.model)
        elif self.task_msg.task_type == "TakeOrder":
            task_executable = TakeOrderTask(self.model, self.task_msg.table_number)
        elif self.task_msg.task_type == "Wander":
            task_executable = WanderTask(self.model)

        if task_executable is None:
            raise NotImplementedError

        task_executable.run_all()

        self.publish_done()

    def publish_done(self):
        print("_te finished task " + str(self.task_msg.task_type))
        self.task_msg.finished = True
        self.pub.publish(self.task_msg)

    def subscriber(self, task):
        if not task.finished:
            print("_te received new task: " + str(task.task_type))
            self.task_msg = task
            self.run_task()


def main():
    global te
    te = TaskExecuter()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("err task executer")
