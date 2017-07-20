#include <boost/format.hpp>
// #include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>

#include <fcntl.h>
#include <unistd.h>
#include <sys/inotify.h>
#include <sys/stat.h>
#include <sys/types.h>

int main(int argc, char *argv[]){

    if (argc != 2) {
      std::cerr << boost::format("Usage: %s <wakeup_directory>") % argv[0] << std::endl;
      exit(1);
    }

    // wait for wake up file from tx before proceeding
    {
      const std::string wakeup_dir = argv[1];
      const std::string wakeup_file = "wakeup.txt";
      
      bool is_created = false;
      
      const std::string fullpath = wakeup_dir + "/" + wakeup_file;
      int fd = inotify_init();
      int watch = inotify_add_watch(fd, wakeup_dir.c_str(), IN_MODIFY|IN_CREATE|IN_MOVED_TO);

      if (access(fullpath.c_str(), F_OK) == 0) {
	is_created = true;
      }

      char buf[1024*(sizeof(inotify_event)+16)];
      ssize_t len;

      while (!is_created) {
	len = read(fd, buf, sizeof(buf));
	if (len < 0) {
	  break;
	}
	inotify_event *event;
	for (size_t i = 0; i < static_cast<size_t>(len); i += sizeof(inotify_event)+event->len) {
	  event = reinterpret_cast<inotify_event *>(&buf[i]);
	  if ((event->len > 0) && (wakeup_file == event->name)) {
	    is_created = true;
	    break;
	  }
	}
      }

      inotify_rm_watch(fd, watch);
      close(fd);

      // std::cout << boost::format("%s: %s triggered") 
      // 	% boost::posix_time::to_simple_string(boost::posix_time::microsec_clock::universal_time())
      // 	% fullpath.c_str() << std::endl;
    }
    
    return 0;
}
