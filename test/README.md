# Test interfaces for phoxi_camera node
## Requirements
* Installed PhoXi Control

## How to run tests
Before run of tests set scanner ID or file camera ID in the **interfaces/config.py**, 
it is needed for connect to camera. Default camera ID for testing is "InstalledExamples-PhoXi-example".

Run test: 
```bash
rostest -t phoxi_camera phoxi_camera_interfaces.test 
```
Find more options of rostest:
```bash
rostest -h
```

or you can run it with all tests by command:
```
catkin_make run_tests
```

## Output of test
After run of a test, base information from the test are written to console.
For addition information check files:

Test result:
```bash
~/.ros/test_results/phoxi_camera/rostest-tests_phoxi_camera_interfaces.xml
```

More detailed test log:
```bash
~/.ros/log/rostest-X-Y.log
```

## How it function
These tests are realized via launch files with extension .test.
In this special launch file is included tested launch file which runs
target ros node and load parameters. The special launch file also launch
testing node which consist of python unittests, this node interact with
tested node and perform tests.