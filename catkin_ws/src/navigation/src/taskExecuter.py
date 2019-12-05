#!/usr/bin/env python

import rospy
from navigation.msg import Task


def messages(channel):
    global model
    return model.messages[channel]


class Model:
    def __init__(self):

        self.locations = ["kitchen", "table1", "table2", "table3", "frontdesk"]

        self.tables = [
            {
                "places": 6,
                "available": True,
                'id': 1
            },
            {
                "places": 4,
                "available": True,
                'id': 2
            },
            {
                "places": 3,
                "available": True,
                'id': 3
            }
        ]

        self.messages = {
            "kitchen": "",
            "staff": ""
        }

    def prepend_message(self, channel, msg):
        self.messages[channel] = "[" + str(rospy.Time.now()) + "]\t" + msg + '\n' + self.messages[channel]


class TaskExecuter:
    """
    Listens to TaskManager, executes jobs, and broadcasts once done
    """

    def __init__(self):
        global model

        print("new TaskExecuter")
        self.model = Model()
        model = self.model

        rospy.Subscriber("task_e", Task, self.subscriber)
        self.pub = rospy.Publisher('task_m', Task, queue_size=1)
        rospy.init_node('Executer', anonymous=True)

        self.task_msg = None
        self.current_task = None

    def run_task(self):

        import task_Wander
        import task_Checkup
        import task_CollectPayment
        import task_TakeOrder
        import task_NewCustomer
        import task_Deliver

        
        """
        creates and runs task, broadcasting when done
        """
        task_executable = None
        
        if self.task_msg.task_type == "Checkup":
            task_executable = task_Checkup.CheckupTask(self.model, self.task_msg.table_number)
        if self.task_msg.task_type == "CollectPayment":
            task_executable = task_CollectPayment.CollectPaymentTask(self.model, self.task_msg.table_number)
        elif self.task_msg.task_type == "NewCust":
            task_executable = task_NewCustomer.NewCustomerTask(self.model)
        elif self.task_msg.task_type == "TakeOrder":
            task_executable = task_TakeOrder.TakeOrderTask(self.model, self.task_msg.table_number)
        elif self.task_msg.task_type == "Deliver":
            task_executable = task_Deliver.DeliverTask(self.model, self.task_msg.table_number)
        elif self.task_msg.task_type == "Wander":
            task_executable = task_Wander.WanderTask(self.model)
        
        if task_executable is None:
            raise NotImplementedError

        print("te:" + str(task_executable.model))
        task_executable.run_all()

        self.publish_done()

    def publish_done(self):
        print("te | task_done\t" + str(self.task_msg.task_type))
        self.task_msg.finished = True
        self.pub.publish(self.task_msg)

    def subscriber(self, task):
        print("_te: heard task " + str(task))
        if not task.finished:
            print("te | new_task\t" + str(task.task_type))
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