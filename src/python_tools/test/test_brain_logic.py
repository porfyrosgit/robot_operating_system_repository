
import pytest
import rclpy
from std_msgs.msg import String
from sensor_msgs.msg import Imu
import time

def test_brain_stabilization_logic():

    rclpy.init()   # allows this script to create "Nodes" that can talk to the C++ Brain.

    tester_node = rclpy.create_node('tester_node')   #create a tester node that will test the robot logic
    #the tester node needs to simulate the HW (the IMU) and listen to the software (the brain)


    # 1. Verifying Topic Names (Implicit "Timeout" Assertion)
    # There isn't a line that says assert topic_name == "/imu_mock". Instead, the verification is baked into the Communication Loop.
    # The Logic:
    # The Test Node shouts on /imu_mock.
    # The Brain Node listens on /imu_mock.
    # The Failure: If the C++ developer changed the Brain to listen on /imu_data, the Brain receives nothing. It never sends a response.
    # The Result: The captured_commands list stays empty. Your test will fail at the final line because "STABILIZE" never arrived.
    # The verification: We verify topic integrity through an end-to-end timeout. 
    # If the topic names don't match, the message 'handshake' fails, the list remains empty, and the final assertion fails by default.
    #If Topic Name is wrong: The captured_commands list stays empty because the Brain never "heard" the message.
    Imu_publisher = tester_node.create_publisher(Imu,'/imu_mock',10)   #publisher to inject fake imu data to the brain decision logic

    captured_commands = []  #save here what the brain says so to inspect it later

    tester_node.create_subscription(     #subscriber to listen to the brain's decisions on '/posture_command'
        String,
        '/posture_command',    #brain publishes its decisions in this default topic
        lambda msg: captured_commands.append(msg.data),   #callback function takes incoming msg and adds its text data to 'captured_commands' list
        10  #DDS queue, QoS-controlled, If more than 10 string msgs come, the older ones get dropped as new ones are added
    )

    #Executing the Stimulus--------------------------------------------------------------------
    #Now we perform the "poking." We send data that should trigger a reaction.


    #3.Verification of the Math (C++ Logic)This is the "Input vs. Output" check. 
    # We provide an input that must trigger the C++ if statement ($0.8 > 0.5$) and then check if the output matches.
    # 2. Verification of Data Types
    # This ensures the Brain understands the specific "language" (the data structure) we are using. 
    # Using the official library import is the key here.
    # 2.If Data Type is wrong: The code will throw an AttributeError or the ROS 2 middleware will refuse to send the data.
    test_msg=Imu()  #prepare the trigger message
    test_msg.orientation.x=0.8  #we set a x > 0.5 threshold

    #the spin loop.
    #because ros2 is async the brain might not hear us the very first time
    end_time = time.time() + 2.0         #we send the message repeatedly for 2secs to ensure the system processes it
    while time.time() < end_time:
    Imu_publisher.publish(test_msg)  #injection

    #stop and listen for any incoming messages
    #the heartbeat of your test script. Without it, your test is "deaf."
    #we use spin_once here because this script is a Single-Threaded Executor.
    #Since the test and the subscriber live in the same thread, 
    #we must explicitly yield control to the executor so it can process the callback queue.
    #When the code hits rclpy.spin_once, Python pauses its loop, looks at the buffer, 
    #sees the "OK" message, and triggers the callback function (the lambda that appends the message to your list).
    #If we remove the below line, the loop will run until the recovery_deadline, but captured_commands will remain [] (empty). 
    # The test will fail with an error saying the Brain never responded, even if the Brain was actually shouting "OK" the whole time!
    rclpy.spin_once(tester_node, timeout_sec=0.1)   #from the brain for 0.1 secs

    #3.If the Math is wrong: The captured_commands list will contain "OK" instead of "STABILIZE", causing the assert to fail.
    if "STABILIZE" in captured_commands:  #if we already heard "STABILIZE" we don't need to wait the full 2secs
        break

        #The Assertion (The Moment of Truth)--------------------------------
        #The most important part.The final Assertion : is a hard "Stop" or "Go" check.

    assert "STABILIZE" in captured_commands, f"TEST FAILED: brain did not stabilize for tilt 0.8 Received: {captured_commands}"

    #EXTRA VERIFICATION:
    # Since the previous test only checks a "High" tilt, we add a second test case to verify 
    # that the Brain correctly returns to an "OK" state when the tilt is small (e.g., 0.1)

    #Prepare the "Safe" sensor data 
    # We create a new IMU message and set the x-axis tilt to 0.1. 
    # Since 0.1 is less than 0.5, the C++ Brain should decide the robot is "OK".
    safe_msg = Imu() 
    safe_msg.orientation.x=0.1
    #Define the Safety Deadline
    # We give the Brain a 2-second window to react to the new "Safe" data.
    recovery_deadline = time.time() + 2.0
    #The Recovery Execution Loop (The part I missed!) 
    # We keep telling the Brain "You are level" until it acknowledges it.
    while time.time() <recovery_deadline:
    #THE FINAL ASSERTION: This line mathematically proves the "Return to OK" happened.
    # We look at index [-1], which is the absolute latest command sent.
    # If it's still "STABILIZE", the Brain logic is 'stuck' or 'latched'.
    assert captured_commands[-1] == "OK", f"RECOVERY FAILED: Brain is stuck in {captured_commands[-1]}"


    #Final Clean up
tester_node.destroy_node()   #close the nodes
rclpy.shutdown()             #shutdown to prevent memory leaks

#VERIFICATIONS:
# The topic names: does the brain actually listen to /imu_mock ? if someone renamed it to /imu_data this test fails
# The data types: does the brain understand the sensor_msgs/Imu format ?
# The Math: does the C++ if(std::abs(tilt)>0.5) logic actually work when compiled ?
# The Latency: does the brain respond within our 2-second window ?
# Plus: final extra verification




