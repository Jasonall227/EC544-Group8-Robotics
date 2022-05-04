#include <arpa/inet.h>
#include <netinet/in.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>
#include <unistd.h>

#define DEFAULT_PORT 54400
#define BUFFER_SIZE 512

int make_socket(uint16_t portNum);
int read_from_client(int fd);
