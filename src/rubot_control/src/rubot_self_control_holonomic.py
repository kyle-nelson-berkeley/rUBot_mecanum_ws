#!/usr/bin/env python3

import rospy
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class rUBot:

    def __init__(self):

        rospy.init_node("rubot_nav", anonymous=False)
        self._distanceLaser = rospy.get_param("~distance_laser")
        self._speedFactor = rospy.get_param("~speed_factor")
        self._forwardSpeed = rospy.get_param("~forward_speed")
        self._backwardSpeed = rospy.get_param("~backward_speed")
        # EDIT
        self._leftSpeed = rospy.get_param("~left_speed")
        self._rightSpeed = rospy.get_param("~right_speed")
        self._rotationSpeed = rospy.get_param("~rotation_speed")
        self._angleTolerance = rospy.get_param("~angleTolerance")

        self._msg = Twist()
        self._cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.callbackLaser)
        rospy.on_shutdown(self.shutdown)

        self._r = rospy.Rate(5)
        
        # Propiedades secundarias

        # Tenemos operando dos versiones de Lidar que devuelven 360 0 720 0 1080 puntos.
        # Para que el codigo sea compatible con cualquiera de los dos, aplicaremos
        # este factor de correccion en los angulos/indices de scan.ranges.
        # Se debe de calcular en la primera ejecucion de __callbackLaser(). Esta
        # variable sirve para asegurar que solo se ejecuta este calculo del
        # factor de correccion una sola vez.
        self.__isScanRangesLengthCorrectionFactorCalculated = False
        self.__scanRangesLengthCorrectionFactor = 1

    def start(self):

        while not rospy.is_shutdown():
            self._cmdVel.publish(self._msg)
            self._r.sleep()

    def callbackLaser(self, scan):
        """Funcion ejecutada cada vez que se recibe un mensaje en /scan."""
        # En la primera ejecucion, calculamos el factor de correcion
        if not self.__isScanRangesLengthCorrectionFactorCalculated:
            self.__scanRangesLengthCorrectionFactor = len(scan.ranges) / 360
            self.__isScanRangesLengthCorrectionFactorCalculated = True
            rospy.loginfo("Scan Ranges correction %5.2f we have,%5.2f points.", self.__scanRangesLengthCorrectionFactor, len(scan.ranges))
        
        def update_closestDistance(scan):
            closestDistance, elementIndex, angleClosestDistance = 0, 0, 0
            closestDistance, elementIndex = min(
                (val, idx) for (idx, val) in enumerate(scan.ranges)
                if scan.range_min < val < scan.range_max
            )
            angleClosestDistance = (elementIndex / self.__scanRangesLengthCorrectionFactor)
            rospy.loginfo("Degree div factor %5.2f ", angleClosestDistance)

            angleClosestDistance = self.__wrapAngle(angleClosestDistance)
            rospy.loginfo("Degree wrapped %5.2f ", angleClosestDistance)

            if angleClosestDistance > 0:
                angleClosestDistance -= 180
            else:
                angleClosestDistance += 180

            rospy.loginfo("Closest distance of %5.2f m at %5.1f degrees.", closestDistance, angleClosestDistance)
            return closestDistance, angleClosestDistance

        # Call update_closestDistance() and assign its returned values
        closestDistance, angleClosestDistance = update_closestDistance(scan)

        distance = 0.45
        distanceTolerance = 0.3

        # EDIT 
        if closestDistance < distance:
            if -45 < angleClosestDistance < 45:
                # Rotate to align with the obstacle's angle
                if not (0 - self._angleTolerance <= angleClosestDistance <= 0 + self._angleTolerance):
                    self._msg.linear.x = 0
                    self._msg.linear.y = 0
                    self._msg.angular.z = -self._rotationSpeed * self._speedFactor
                    self._cmdVel.publish(self._msg)

                if not (closestDistance < distance - distanceTolerance): # move away from object
                    self._msg.linear.y = 0
                    self._msg.angular.z = 0
                    self._msg.linear.x = self._forwardSpeed * self._speedFactor
                    self._cmdVel.publish(self._msg)

                else:
                    # Move while obstacle is close
                    self._msg.linear.x = 0
                    self._msg.angular.z = 0
                    self._msg.linear.y = self._leftSpeed * self._speedFactor
                    self._cmdVel.publish(self._msg)

                # Dynamically update angleClosestDistance
                update_closestDistance(scan)


            elif 45 < angleClosestDistance < 135:
                # Rotate to align with the obstacle's angle
                if not (90 - self._angleTolerance <= angleClosestDistance <= 90 + self._angleTolerance):
                    self._msg.linear.x = 0
                    self._msg.linear.y = 0
                    self._msg.angular.z = self._rotationSpeed * self._speedFactor
                    self._cmdVel.publish(self._msg)

                if not (closestDistance < distance - distanceTolerance): # move away from object
                    self._msg.angular.z = 0
                    self._msg.linear.x = 0
                    self._msg.linear.y = self._rightSpeed * self._speedFactor
                    self._cmdVel.publish(self._msg)
                
                else:
                    # Move while obstacle is close
                    self._msg.angular.z = 0
                    self._msg.linear.y = 0
                    self._msg.linear.x = self._forwardSpeed * self._speedFactor
                    self._cmdVel.publish(self._msg)

                # Dynamically update angleClosestDistance
                update_closestDistance(scan)

            elif 135 < angleClosestDistance or angleClosestDistance < -135:
                # Rotate to face away from the obstacle
                if not (180 - self._angleTolerance <= angleClosestDistance <= -1 * (180 - self._angleTolerance)):
                    self._msg.linear.x = 0
                    self._msg.linear.y = 0
                    self._msg.angular.z = self._rotationSpeed * self._speedFactor
                    self._cmdVel.publish(self._msg)

                if not (closestDistance < distance - distanceTolerance): # move away from object
                    self._msg.angular.z = 0 
                    self._msg.linear.y = 0
                    self._msg.linear.x = self._backwardSpeed * self._speedFactor
                    self._cmdVel.publish(self._msg)
                else:
                    # Move left while obstacle is close
                    self._msg.linear.x = 0
                    self._msg.angular.z = 0
                    self._msg.linear.y = self._rightSpeed * self._speedFactor
                    self._cmdVel.publish(self._msg)

                # Dynamically update angleClosestDistance
                update_closestDistance(scan)

            elif -135 < angleClosestDistance < -45:
                # Rotate to align with the obstacle's angle
                if not (-90 - self._angleTolerance <= angleClosestDistance <= -90 + self._angleTolerance):
                    self._msg.linear.x = 0
                    self._msg.linear.y = 0
                    self._msg.angular.z = -self._rotationSpeed * self._speedFactor
                    self._cmdVel.publish(self._msg)

                if not (closestDistance < distance - distanceTolerance): # move away from object
                    self._msg.angular.z = 0
                    self._msg.linear.x = 0
                    self._msg.linear.y = self._leftSpeed * self._speedFactor
                    self._cmdVel.publish(self._msg)
                else:
                    # Move while obstacle is close
                    self._msg.angular.z = 0
                    self._msg.linear.y = 0
                    self._msg.linear.x = self._backwardSpeed * self._speedFactor
                    self._cmdVel.publish(self._msg)

                # Dynamically update angleClosestDistance
                update_closestDistance(scan)

        else:
            # Default behavior: Move forward
            self._msg.linear.x = self._backwardSpeed * self._speedFactor
            self._msg.linear.y = 0
            self._msg.angular.z = 0
            self._cmdVel.publish(self._msg)



    def __sign(self, val):

        if val >= 0:
            return 1
        else:
            return -1

    def __wrapAngle(self, angle):
        if 0 <= angle <= 180:
            return angle
        else:
            return angle - 360

    def shutdown(self):
        self._msg.linear.x = 0
        self._msg.linear.y = 0
        self._msg.angular.z = 0
        self._cmdVel.publish(self._msg)
        rospy.loginfo("Stop RVIZ")

if __name__ == '__main__':
    try:
        rUBot1 = rUBot()
        rUBot1.start()
        rospy.spin()
        rUBot1.shutdown()
    except rospy.ROSInterruptException: rUBot1.shutdown()#pass
