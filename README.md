# hexapodFABRIK
Inverse kinematics for hexapod robot using a hybrid version of the forward and backwards reaching inverse kinematics (FABRIK) 
This is a proof of concept written in C++ it uses the EIGEN3 library for the matrix manipulation. It is very buggy at the moment however, the 2 dimensional angle solver does work
and is very accurate. The current issue is expanding it to three dimensions without changing the angles of the previous joints. This was created because I was curious as to how accurate it
is compared to Jacobian methods. NOTE: THIS IS A WORK IN PROGRESS AND WAS MORE TO SEE HOW THE ALGORITHM WORKED IN PRACTICE. 
