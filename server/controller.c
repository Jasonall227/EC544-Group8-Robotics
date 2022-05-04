#include "controller.h"

/* init wiringPi */
void init() {
    wiringPiSetupGpio();

    // init motor selection pins
    pinMode(P1, OUTPUT);
    pinMode(P2, OUTPUT);
    pinMode(P3, OUTPUT);

    // init operation pins
    softPwmCreate(P4, 50, 100);
    pinMode(P5, OUTPUT);
}

/* select motor */
int selectMotor(int motor) {
    switch (motor) {
        case 1:
            printf("select motor 1\n");
            digitalWrite(P1, LOW);
            digitalWrite(P2, LOW);
            digitalWrite(P3, HIGH);
            break;
        case 2:
            printf("select motor 2\n");
            digitalWrite(P1, LOW);
            digitalWrite(P2, HIGH);
            digitalWrite(P3, LOW);
            break;
        case 3:
            printf("select motor 3\n");
            digitalWrite(P1, LOW);
            digitalWrite(P2, HIGH);
            digitalWrite(P3, HIGH);
            break;
        case 4:
            printf("select motor 4\n");
            digitalWrite(P1, HIGH);
            digitalWrite(P2, LOW);
            digitalWrite(P3, LOW);
            break;
        case 5:
            printf("select motor 5\n");
            digitalWrite(P1, HIGH);
            digitalWrite(P2, LOW);
            digitalWrite(P3, HIGH);
            break;
        case 6:
            printf("select motor 6\n");
            digitalWrite(P1, HIGH);
            digitalWrite(P2, HIGH);
            digitalWrite(P3, LOW);
            break;
        default:
            perror("invalid motor number\n");
            return -1;
    }

    return 0;
}

int operate(int pos) {
    printf("set position to %d\n", pos);
    softPwmWrite(P4, pos);

    // activate
    digitalWrite(P5, HIGH);
    digitalWrite(P5, LOW);

    return 0;
}

int handleInput(char* input) {
    int motor;
    char* temp;
    int pos;

    motor = input[3] - '0';
    if (motor < 1 || motor > 6) {
        perror("motor number\n");
        return -1;
    }

    temp = strtok(input, "-");
    pos = atoi(temp);
    if (pos < 0 || pos > 100) {
        perror("position\n");
        return -1;
    }

    selectMotor(motor);
    operate(pos);

    return 0;
}
