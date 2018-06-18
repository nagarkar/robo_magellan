##### Table of Contents  
[Eclipse setup](#eclipse_setup)



## Eclipse setup
### Executable build configuration
#### Source Tree:
++ bbahrs, imu, qpcpp

#### Include folders
+ src/bbahrs, src/imu, src/qpcpp (various folders)

### Test Build Configuration 

#### Source Tree changes from the executable configuration
- src/*/main.cpp (Exclude the main.cpp or other files that have a main method)
+ tests

<a name="eclipse_setup"/>
#### Include folders changes from executable configuration
+ testcases (for various cpp files). These cpp are included in the test suites in the 'tests' folder.

