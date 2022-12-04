# Prosthetic-Arm-Control-ESP32-RTOS

An inexpensive DIY electromographic transradial Prothestic arm! 

According to 2005 census data, there are over 540,000 people in the US that are classified as upper limb amputees, with the expectation that this number will double by 2050, as it increases by more than 10,000 every year. It is estimated that 5.2% or more of these amputations are transradial. To regain the functionality of this lost limb usually costs the patient hundreds of thousands of US dollars. Due to this overwhelming price barrier many children and young teenagers go without prothesis because they would quickly outgrow any arm that they would receive.

The overarching goal of this design is to create an afforable alterntaive to commercially avaialable prothesis to allow young amputees to regain the fucntionaility of their arm while still being visually appealing. To achive this goal our team decided decided to deisng a 3D printed arm to keep production costs low. The fingers of the arm are actuated by servo motors and the patients are able to control the grip of the hand with the use of electromyographic sensors

Electrmyography (EMG) is a technique for evaluating a biomedical signal that measures electrical currents generated in muscles during its contraction representing neuromuscular activities. By placing an surface EMG sensor on the patient's skin over targeted muscle groups we are able read their muscle contrations. With two sensors we are able to map the patient's contractions to upwards of 8 unique hand position restoring some of their origional arm's functionality. 

This repository serves as the main repository for this project and contains the source code and general project information including the Prosethic Arm Control Manual (P.A.C. Man.). A seperate repository is used to maintain and track the development our printing circuit board and our signal processing. Both repositories are linked below. 

Printed Circuit Board: https://github.com/ZIRoberts/Prosthetic-Arm-Control-Board

Signal Processor: https://github.com/ZIRoberts/Prosthetic-Arm-Myoware-Signal-Proccessor
