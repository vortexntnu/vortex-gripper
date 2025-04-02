#include "canfd.h"
#include <errno.h>
#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/bcm.h>
#include <linux/can/error.h>
#include <linux/can/gw.h>
#include <linux/can/isotp.h>
#include <linux/can/j1939.h>
#include <linux/can/netlink.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

/* CAN Message recieve enum */

static int sock = -1;

// Initialize CAN FD Interface
int canfd_init(const char* interface) {
  struct sockaddr_can addr;
  struct ifreq ifr;

  // Create CAN socket
  sock = socket(PF_CAN, SOCK_RAW, CAN_RAW);
  if (sock < 0) {
    perror("Error creating CAN socket");
    return -1;
  }

  // Set interface name
  strncpy(ifr.ifr_name, interface, IFNAMSIZ - 1);
  if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
    perror("Error getting CAN interface index");
    close(sock);
    return -1;
  }

  // Bind socket to CAN interface
  addr.can_family = AF_CAN;
  addr.can_ifindex = ifr.ifr_ifindex;

  if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
    perror("Error binding CAN socket");
    close(sock);
    return -1;
  }

  return 0;
}
void set_can_filter(uint16_t start_id, uint16_t id_mask) {
  struct can_filter filter[1];
  filter[0].can_id = start_id;
  filter[0].can_mask = id_mask;

  if (setsockopt(sock, SOL_CAN_RAW, CAN_RAW_FILTER, &filter, sizeof(filter)) <
      0) {
    perror("Error setting CAN filter");
  }
}

// Send CAN FD Message
int canfd_send(const CANFD_Message *msg) {
  struct canfd_frame frame;
  memset(&frame, 0, sizeof(frame));

  // Set CAN ID and Frame Type
  frame.can_id = msg->id;
  if (msg->is_extended) {
    frame.can_id |= CAN_EFF_FLAG;
  }
  if (msg->is_fd) {
    frame.len = msg->length; // CAN FD allows up to 64 bytes
  } else {
    if (msg->length > 8) {
      perror("CAN 2.0 only supports messages a maximum of 8 bytes payload");
      return -1;
    }
    frame.len = msg->length;
    /*frame.len = msg->length > CAN_MAX_DLEN ? CAN_MAX_DLEN : msg->length;*/
  }

  // Copy data
  memcpy(frame.data, msg->data, frame.len);

  // Send CAN frame
  if (write(sock, &frame, sizeof(frame)) != sizeof(frame)) {
    perror("Error sending CAN FD message");
    return -1;
  }

  return 0;
}

// Receive CAN FD Message
int canfd_receive(CANFD_Message *msg, int timeout_ms) {
  struct canfd_frame frame;
  struct timeval timeout;
  fd_set read_fds;

  FD_ZERO(&read_fds);
  FD_SET(sock, &read_fds);

  timeout.tv_sec = timeout_ms / 1000;
  timeout.tv_usec = (timeout_ms % 1000) * 1000;

  int ret = select(sock + 1, &read_fds, NULL, NULL, &timeout);
  if (ret < 0) {
    perror("Error in select()");
    return -1;
  } else if (ret == 0) {
    return -1; // Timeout
  }

  if (read(sock, &frame, sizeof(frame)) != sizeof(frame)) {
    perror("Error receiving CAN FD message");
    return -1;
  }

  // Copy data to user structure
  msg->id = frame.can_id & CAN_EFF_MASK;
  msg->is_extended = (frame.can_id & CAN_EFF_FLAG) ? true : false;
  msg->is_fd = true;
  msg->length = frame.len;
  memcpy(msg->data, frame.data, frame.len);

  return 0;
}

// Takes in all data sent from MCU
// Encoder angles
// Sensor data
void canfd_recieve_handler(CANFD_Message *msg) {
  switch (msg->id) {
  case 0x0:
    break;
  case 0x150:
    break;
  case ENCODER_ANGLES:
    break;
  case TEMP:
    break;
  case PRESSURE:
    break;
  default:
    break;
  }
}

// Close CAN FD Socket
void canfd_close() {
  if (sock >= 0) {
    close(sock);
    sock = -1;
  }
}
