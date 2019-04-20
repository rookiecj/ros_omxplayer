#include <signal.h> // kill
#include <sys/types.h>
#include <sys/wait.h> // waitpid
#include <ros/ros.h>
#include <std_msgs/String.h>

pid_t g_play_pid = -1;
std::string g_sound_file_path = "";
std::string g_sound_output = "hdmi"; // hdmi/local/both/alsa
ros::Publisher g_done_msg_pub;

static void wait_for_child(int child)
{
  int status;
  while (-1 == waitpid(child, &status, 0))
    ;
  if (!WIFEXITED(status) || WEXITSTATUS(status) != 0) {
    printf( "Process %d failed\n", child);
  }
}

void play_sound_callback(const std_msgs::String::ConstPtr &msg)
{
  std_msgs::String done_msg;

  if (msg->data == "")
  {
    if (g_play_pid != -1)
      kill(g_play_pid, SIGKILL);

    g_play_pid = -1;
    done_msg.data = "play_sound_fail";
    g_done_msg_pub.publish(done_msg);
    return;
  }

  if (g_play_pid != -1) {
    kill(g_play_pid, SIGKILL);
  }

  g_play_pid = fork();

  switch (g_play_pid)
  {
  case -1:
    fprintf(stderr, "Fork Failed!! \n");
    done_msg.data = "play_sound_fail";
    g_done_msg_pub.publish(done_msg);
    break;
  case 0:
    execl("/usr/bin/omxplayer", "omxplayer", "-o", g_sound_output.c_str(), (g_sound_file_path + msg->data).c_str(), (char*)0);
    //execl("/usr/bin/omxplayer", "omxplayer", (g_sound_file_path + msg->data).c_str(), (char *)0);
    break;
  default:
    wait_for_child(g_play_pid);
    g_play_pid = -1;
    done_msg.data = "play_sound";
    g_done_msg_pub.publish(done_msg);
    break;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ros_omxplayer");
  ros::NodeHandle nh("~");

  g_sound_file_path = nh.param<std::string>("sound_file_path", "");
  if (g_sound_file_path != "" && g_sound_file_path.compare(g_sound_file_path.size() - 1, 1, "/") != 0)
    g_sound_file_path += "/";
  // private param
  g_sound_output = nh.param<std::string>("sound_output", "hdmi");
  printf("sound_output= %s\n", g_sound_output.c_str());

  ros::Subscriber play_mp3_sub = nh.subscribe("/play_sound_file", 10, &play_sound_callback);
  g_done_msg_pub = nh.advertise<std_msgs::String>("/ros_omxplayer/play_done", 5);

  ros::spin();
  return 0;
}
