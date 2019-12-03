#!/usr/bin/env python
import taskTypes as tt
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

    def run_task(self, task_info):
        """
        creates and runs task, broadcasting when done
        :param task_info:
        """
        self.current_task = task_info

        task_executable = None

        if self.current_task.type == "CheckUp":
            task_executable = CheckupTask(self.model)
        elif self.current_task.type == "CollectPayment":
            task_executable = CollectPaymentTask(self.model, task_info.table_number)
        elif self.current_task.type == "NewCustomer":
            task_executable = NewCustomerTask(self.model)
        elif self.current_task.type == "TakeOrder":
            task_executable = TakeOrderTask(self.model, task_info.table_number)
        elif self.current_task.type == "Wander":
            task_executable = WanderTask(self.model)

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
        print("err task management")
