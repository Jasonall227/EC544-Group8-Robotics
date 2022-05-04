#include "server.h"

int make_socket(uint16_t portNum) {
    int sock;
    struct sockaddr_in name;

    sock = socket(PF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("socket\n");
        return -1;
    }

    name.sin_family = AF_INET;
    name.sin_port = htons(portNum);
    name.sin_addr.s_addr = htonl(INADDR_ANY);
    if (bind(sock, (struct sockaddr*)&name, sizeof(name)) < 0) {
        perror("bind\n");
        return -1;
    }

    return sock;
}

int read_from_client(int fd) {
    char buffer[BUFFER_SIZE];
    int nbytes;

    nbytes = read(fd, buffer, BUFFER_SIZE);
    if (nbytes < 0) {
        // read error
        perror("read\n");
        exit(EXIT_FAILURE);
    } else if (nbytes == 0) {
        // EOF
        return -1;
    } else {
        // data read
        fprintf(stdout, "Msg: %s", buffer);
        return 0;
    }
}
