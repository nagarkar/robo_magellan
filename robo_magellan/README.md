=  Eclipse setup +
== Executable build configuration ==
Source Tree:
+ src
++ bbahrs, imu, qpcpp

Include folders:
+ src/bbahrs, src/imu, src/qpcpp (various folders)

== Test Build Configuration == 
Source Tree changes from the executable configuration
- src/*/main.cpp (Exclude the main.cpp or other files that have a main method)
+ tests

Include folders changes from executable configuration
+ testcases (for various cpp files). These cpp are included in the test suites in the 'tests' folder.

