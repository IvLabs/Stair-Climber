import rospy
from geometry_msgs.msg import Twist
PI = 3.1415926535897
def initialise():
	rospy.init_node('robot_cleaner', anonymous=True)
	velocity_publisher = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
	vel_msg = Twist()
	trace_path(vel_msg,velocity_publisher)

def trace_path(vel_msg,velocity_publisher):
    ch=1
    while(ch==1):
        turn=input("enter True if you want to turn")
        if turn:
            rotate(vel_msg,velocity_publisher)
            ch=int(input("type 1 to continue"))
        else:
		    move(vel_msg,velocity_publisher)
		    ch=int(input("type 1 to continue"))	
            
        
def move(vel_msg,velocity_publisher):
	print("Let's move your robot")
	speed = int(input("Input your speed:"))
	distance = int(input("Type distance to be moved"))
	isForward = input("Forward?:")
	if(isForward):
		vel_msg.linear.x = abs(speed)
	else:
		vel_msg.linear.x = -abs(speed)
	vel_msg.linear.y = 0
	vel_msg.linear.z = 0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0
	vel_msg.angular.z = 0
	t0 = rospy.Time.now().to_sec()
	current_distance = 0
	while(current_distance < distance):
		velocity_publisher.publish(vel_msg)
		t1=rospy.Time.now().to_sec()
		current_distance= speed*(t1-t0)
	vel_msg.linear.x = 0
	velocity_publisher.publish(vel_msg)


def rotate(vel_msg,velocity_publisher):
	speed=int(input("input your speed"))
	angle=int(input("Type your angle of rotation"))
	clockwise=int(input("Clockwise"))
	angular_speed = speed*2*PI/360
	relative_angle = angle*2*PI/360
	vel_msg.linear.x=0
	vel_msg.linear.y=0
	vel_msg.linear.z=0
	vel_msg.angular.x = 0
	vel_msg.angular.y = 0

	if(clockwise):
		vel_msg.angular.z=-abs(angular_speed)
	else:
		vel_msg.angular.z=abs(angular_speed)
	t0 = rospy.Time.now().to_sec()
	current_angle = 0
	while(current_angle < relative_angle):
		velocity_publisher.publish(vel_msg)
		t1=rospy.Time.now().to_sec()
		current_angle= angular_speed*(t1-t0)
	vel_msg.angular.z = 0
	velocity_publisher.publish(vel_msg)
	
   
if __name__ == '__main__':
      try:
           #Testing our function
           initialise()
      except rospy.ROSInterruptException: pass
