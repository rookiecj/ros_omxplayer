# ros_omxplayer 
This is a package for playing an mp3 via omxplayer

## prerequisites

```
$ sudo apt-get install omxplayer
```

## workspace

```
$ cd catkin_ws
$ mkdir -p src
$ git clone https://github.com/rookiecj/ros_omxplayer.git src/ros_omxplayer
$ catkin_make
```

## run

```
$ source devel/setup.bash
$ roscore
$ rosrun ros_omxplayer ros_omxplayer
```

## test

```
$ rostopic pub -1 /play_sound_file std_msgs/String '/home/travis/hello.m4a'
```

