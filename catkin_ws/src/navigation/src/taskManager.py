#!/usr/bin/env python

import sys

if sys.version[0] == "2":
    from Queue import PriorityQueue
else:
    from queue import PriorityQueue

import taskTypes as tt
from std_msgs.msg import String
import rospy
from navigation.msg import Task


pub = None


def new_task(task_type, table_number=None, delay=0, customerID=-1):
    """
    adds task to priority queue, and will be executed once it becomes the highest priority job
    :param task_type: name of task to be executed
    :param table_number: relevant table number, defaults to None
    :param delay: minimum delay time on task execution in minutes, defaults to 0
    :return:
    """
    global pub

    print("tm new_task/:" + task_type)

    SEQUENCES_SHORTENED = False

    if SEQUENCES_SHORTENED:
        delay = 0

    if pub is None:
        pub = rospy.Publisher('task_m', Task, queue_size=1)

    pub.publish(tt.new(task_type, table_number, delay, customerID).to_msg())

    print("tm new_task\\:" + task_type)



class TaskManager:
    """
    Listens for new tasks and broadcasts highest priority job to executer
    """
    def __init__(self):
        print("new TaskManager")
        global pub

        rospy.init_node('Manager', anonymous=True)
        rospy.Subscriber("task_m", Task, self.subscriber)
        if pub is None:
            pub = rospy.Publisher('task_e', Task, queue_size=1)

        self.current_tasks = PriorityQueue()
        self.current_task = None

        self.publish_next_task()


    def __str__(self):
        o = "\nTask Manager:\n|\tCurrent Task -\n|\t\t" + str(self.current_task) + "\n|\tOther Tasks -"
        for task in self.current_tasks.queue:
            o = o + "\n|\t\t" + str(task)

        return o + "\n"


    def add_task(self, task):
        print("tm | add_task\t" + str(task) + "\n")
        self.current_tasks.put(task)
        print(str(self))


    def update_priorities(self):
        updated_priorities_queue = PriorityQueue()

        for task in self.current_tasks.queue:
            if task is not None:
                task.update_priority()
                updated_priorities_queue.put(task)
        self.current_tasks = updated_priorities_queue


    def publish_next_task(self):
        # if self.current_task is not None:
        #    self.add_task(self.current_task)

        if self.current_tasks.empty():
            self.current_task = tt.Wander()
        else:
            self.current_task = self.current_tasks.get()

        print(self.current_task.priority)
        if self.current_task.priority > 0:
            self.current_tasks.put(self.current_task)
            self.current_task = tt.Wander()
            rospy.logwarn("positive priority, replacing with wander")

        t = self.current_task.to_msg()
        r = rospy.Rate(0.2)
        r.sleep()
        pub.publish(t)
        print(self)


    def subscriber(self, task_msg):
        priority_task = tt.from_msg(task_msg)
        if task_msg.finished:
            print("tm | subscriber\tfinished")
            if priority_task == self.current_task:
                self.current_task = None
            self.update_priorities()
            self.publish_next_task()
        else:
            self.add_task(priority_task)


def main():
    TaskManager()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("err task management")
