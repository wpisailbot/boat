#include "pgn.h"
#include <linux/can.h>
#include <stdio.h>
#include <inttypes.h>
#include <sys/ioctl.h>
#include <string.h>
#include <unistd.h>
#include <net/if.h>
#include <thread>


void writer(int fd) {
  struct can_frame frame;
  frame.can_dlc = 8;
  CANID id;
  id.priority = 3;
  id.dataA = ; // PGN top
  id.dataB = ; // PGN bottom
  id.source = 1011;
  frame.can_id = ConstructID(id);
  for (int i = 0; i < 8; ++i) {
    frame.data[i] = i;
  }
  while(true) {
    usleep(1000000);
    frame.data[0] += 1;
    write(fd, &frame, sizeof(struct can_frame));
  }
}

int main() {
  // NMEA2000 is 250kbit/s
  int s;
  struct sockaddr_can addr;
  struct ifreq ifr;
  s = socket(PF_CAN, SOCK_RAW, CAN_RAW);

  strcpy(ifr.ifr_name, "can0");
  ioctl(s, SIOCGIFINDEX, &ifr);

  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  bind(s, (struct sockaddr *)&addr, sizeof(addr));
  // Now we can just read() and write()
  struct can_frame frame;

  while (true) {
    int d = read(s, &frame, sizeof(struct can_frame));
    if (d <= 0) continue;
    CANID id = RetrieveID(frame.can_id);
    int len = frame.can_dlc;
    printf("id: prio: %d, dataA: %d, dataB: %d, src: %d\n", id.priority,
           id.dataA, id.dataB, id.source);
    printf("data: ");
    for (int i = 0; i < len; ++i) {
      printf("%d ", frame.data[i]);
    }
    printf("\n");
  }
}
