#!/usr/bin/env python3
from queue import PriorityQueue

import taskTypes as tt

import rospy
from navigation.msg import Task


tm = None


def new_task(task_type, table_number=-1, delay=0):
    global tm

    tm.add_task(tt.new(task_type, table_number, delay))


class TaskManager:
    """
    Listens for new tasks and broadcasts highest priority job to executer
    """
    def __init__(self):
        print("new TaskManager")

        rospy.init_node('Manager', anonymous=True)
        rospy.Subscriber("task", Task, self.subscriber)
        self.pub = rospy.Publisher('task', Task, queue_size=1)

        self.current_tasks = PriorityQueue()

        self.add_task(tt.Wander())

        self.publish_next_task()

    def add_task(self, task):
        self.current_tasks.put(task)

    def update_priorities(self, remove=None):
        updated_priorities_queue = PriorityQueue()

        for task in self.current_tasks.queue:
            if not task == remove:                              # dubious, logic error?
                task.update_priority()
                updated_priorities_queue.put(task)
        self.current_tasks = updated_priorities_queue

    def publish_next_task(self):
        if self.current_tasks.empty():
            next_task = tt.Wander()
        else:
            next_task = self.current_tasks.get()
        t = Task()
        t.task_type = next_task.type
        t.created_at = next_task.time_created
        t.table_number = next_task.table_number
        t.finished = False
        self.pub.publish(t)

    def subscriber(self, task):
        if task.finished:
            finished_task = tt.create(task)
            #  self.update_priorities(remove=finished_task)     No :'(
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
