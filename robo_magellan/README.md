# Table of Contents  
[Eclipse setup](#link1)

[Executable build](#link2)

[Test build](#link3)


<a name="link1"/>
## Eclipse setup

<a name="link2"/>
### Executable build configuration
#### Source Tree:
++ bbahrs, imu, qpcpp

#### Include folders
+ src/bbahrs, src/imu, src/qpcpp (various folders)

<a name="link3"/>
### Test Build Configuration 

#### Source Tree changes from the executable configuration
- src/*/main.cpp (Exclude the main.cpp or other files that have a main method)
+ tests

#### Include folders changes from executable configuration
+ testcases (for various cpp files). These cpp are included in the test suites in the 'tests' folder.

