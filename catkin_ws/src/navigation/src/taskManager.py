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


tm = None


def new_task(task_type, table_number=1, delay=0):
    """
    adds task to priority queue, and will be executed once it becomes the highest priority job
    :param task_type: name of task to be executed
    :param table_number: relevant table number, defaults to None
    :param delay: minimum delay time on task execution in minutes, defaults to 0
    :return:
    """
    global tm
    print("tm.new_task on " + str(tm))
    tm.add_task(tt.new(task_type, table_number, delay))


class TaskManager:
    """
    Listens for new tasks and broadcasts highest priority job to executer
    """
    def __init__(self):
        print("new TaskManager")

        rospy.init_node('Manager', anonymous=True)
        rospy.Subscriber("task_", Task, self.subscriber)
        self.pub = rospy.Publisher('task', Task, queue_size=1)

        self.current_tasks = PriorityQueue()
        self.current_task = None

        self.publish_next_task()


    def __str__(self):
        o = "Task Manager:\n\tCurrent Task -\n\t\t" + str(self.current_task) + "\n\tOther Tasks -"
        for task in self.current_tasks.queue:
            o = o + "\n\t\t " + str(task) + "\n"

        return o


    def add_task(self, task):
        self.current_tasks.put(task.update_priority())


    def update_priorities(self):
        updated_priorities_queue = PriorityQueue()

        for task in self.current_tasks.queue:
            task.update_priority()
            updated_priorities_queue.put(task)
        self.current_tasks = updated_priorities_queue


    def publish_next_task(self):
        if self.current_task is not None:
            self.add_task(self.current_task)

        if self.current_tasks.empty():
            self.current_task = tt.Wander()
        else:
            self.current_task = self.current_tasks.get()
        t = Task()
        t.task_type = self.current_task.type
        t.created_at = self.current_task.time_created
        #t.table_number = self.current_task.table_number
        t.table_number = 1
        t.finished = False
        r = rospy.Rate(0.2)
        r.sleep()
        self.pub.publish(t)
        print(self)


    def subscriber(self, task):
        if task.finished:
            print("_tm received task done: publish next task")
            finished_task = tt.create(task)
            if finished_task == self.current_task:
                self.current_task = None
            self.update_priorities()
            self.publish_next_task()


def main():
    global tm

    tm = TaskManager()
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print("err task management")
