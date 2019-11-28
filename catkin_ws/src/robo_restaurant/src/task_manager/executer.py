from task_base import Wander, GreetCustomer
# import rospy

class TaskExecuter():
    """
    Listens to TaskManager, executes jobs, and broadcasts once done
    """



    def run_task(self, message):
        """
        creates and runs task, broadcasting when done
        :param message:
        """
        if (message == "Wander"):
            t = Wander
        elif (message == "GreetCustomer"):
            t = GreetCustomer
        else:
            raise NotImplementedError

        # rate = rospy.Rate(10)  # 10hz
        t.run()
        # while self.run:
        #   rate.sleep()

        self.publish_done(t)

    def publish_done(self, task):
        """ok but how do you broadcast TODO"""

    def listener(self):
        """help TODO"""