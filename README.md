An inexpensive DIY electromyographic trans-radial Prosthetic arm! 

# Overview

According to 2005 census data, there are over 540,000 people in the US that are classified as upper limb amputees, with the expectation that this number will double by 2050, as it increases by more than 10,000 every year. It is estimated that 5.2% or more of these amputations are trans-radial. To regain the functionality of this lost limb usually costs the patient hundreds of thousands of US dollars. Due to this overwhelming price barrier, many children and young teenagers go without prosthesis because they would quickly outgrow any arm that they would receive.

The overarching goal of this design is to create an affordable alternative to commercially available prothesis to allow young amputees to regain the functionality of their arm while still being visually appealing. To achieve this goal our team decided to design a 3D-printed arm to keep production costs low. The fingers of the arm are actuated by servo motors and the patients are able to control the grip of the hand with the use of electromyographic sensors.

Electromyography (EMG) is a technique for evaluating a biomedical signal that measures electrical currents generated in muscles during its contraction representing neuromuscular activities. By placing a surface EMG sensor on the patient's skin over targeted muscle groups we are able to read their muscle contractions. With two sensors we are able to map the patient's contractions to upwards of 8 unique hand positions restoring some of their original arm's functionality. 

This repository serves as the main repository for this project and contains the source code and general project. A separate repository is used to maintain and track the development of our printed circuit board and our signal processing. Both repositories are linked below. 


# Initial Testing Video

https://user-images.githubusercontent.com/49044136/208213866-cb7916af-ef06-4852-877d-61d72dd4ef12.mp4

Above is a video of our team's initial test of both the hardware and software. Since the test, we have migrated the code an RTOS architecture and designed a printed circuit board (Linked Below) to downsize the bulky prototype board shown in the video. Our biggest focus moving forward is to clean up the signal processing, and add 6 additional hand positions.

# Links to relevant repositories

Printed Circuit Board: https://github.com/ZIRoberts/Prosthetic-Arm-Control-Board

Signal Processor: https://github.com/ZIRoberts/Prosthetic-Arm-Myoware-Signal-Proccessor
