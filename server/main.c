#include <string.h>

#include "server.h"
#include "controller.h"

#define SAFECALL(CMD, ERR)  \
    if (CMD) {              \
        perror(ERR);        \
        exit(EXIT_FAILURE); \
    }

void printHelp();

int main(int argc, char* argv[]) {
    // print help if user passes "-h"
    if (argc > 1 && strncmp("-h", argv[1], 3) == 0) {
        printHelp();
        return 0;
    }

    int socket, new_sockfd;
    struct sockaddr_in client;
    socklen_t cli_len = sizeof(client);
    char buffer[BUFFER_SIZE];

    init();
    SAFECALL((socket = make_socket((argc < 2) ? 54400 : (uint16_t)atoi(argv[1]))) == -1,
             "make_socket")
    SAFECALL(listen(socket, 1) == -1, "listen")
    while ((new_sockfd = accept(socket, (struct sockaddr*)&client, &cli_len)) > 0) {
        while (1) {
            SAFECALL(recv(new_sockfd, buffer, BUFFER_SIZE, 0) == -1, "recv")

            // echo request back to client
            if (fork() == 0) {
                close(socket);
                SAFECALL(send(new_sockfd, buffer, strlen(buffer), 0) == -1, "send")
                close(new_sockfd);
                exit(EXIT_SUCCESS);
            }
            printf(">>> %s\n", buffer);

            // disconnect if request is "disconnect"
            if (strncmp("disconnect", buffer, 10) == 0) {
                printf("disconnect...\n");
                close(new_sockfd);
                break;
            }

            // handle request
            if (handleInput(buffer) == -1) {
                perror("handle input\n");
            }

            bzero(buffer, BUFFER_SIZE);
        }
    }
}

void printHelp() {
    printf("Usage: controller [Port number]\n");
    printf("\t-Port: the port this program binds to, 54400 by default.\n");
}
