# Table of Contents  
[Eclipse Configurations](#link1)

[- Executable build](#link1_1)

[- Test build](#link1_2)

[Running Tests](#link2)


## Symbols Required
Q_EVT_CTOR		To enable constructors for events


# Eclipse Setup

<a name="link1_1"/>

## Executable build configuration
### Source Tree (src/):
\+ bbahrs

\+ imu

\+ qpcpp

\+ util

### Include folders
\+ src

\+ src/qpcpp (various folders)

<a name="link1_2"/>

## Test Build Configuration 

### Source Tree changes from the executable configuration
\- src/*/main.cpp (Exclude the main.cpp or other files that have a main method)

\+ tests

### Include folders changes from executable configuration
\+ testcases (for various cpp files). These cpp are included in the test suites in the 'tests' folder.

<a name="link2"/>

# Running Tests

TBD


For help on how to edit this file, see https://github.com/adam-p/markdown-here/wiki/Markdown-Cheatsheet#lists
